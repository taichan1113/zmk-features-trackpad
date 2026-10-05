/* Two-finger transform gesture primary recognizer.
 * 2本指pinch/rotation primary recognizer。 */

#define DT_DRV_COMPAT zmk_travelers_pad_iqs9150_transform_gesture

#include <errno.h>

#include <zephyr/device.h>
#include <zephyr/dt-bindings/input/input-event-codes.h>
#include <zephyr/drivers/kscan.h>
#include <zephyr/input/input.h>
#include <zephyr/logging/log.h>

#include <travelers_pad/contact_arbiter.h>
#include <travelers_pad/contact_output_broker.h>
#include <travelers_pad/multitouch.h>

LOG_MODULE_REGISTER(travelers_pad_transform_gesture, LOG_LEVEL_INF);

enum transform_mode {
	TRANSFORM_NONE,
	TRANSFORM_PINCH,
	TRANSFORM_ROTATE,
	TRANSFORM_PINCH_ROTATE,
};

enum transform_action {
	TRANSFORM_ACTION_PINCH_IN,
	TRANSFORM_ACTION_PINCH_OUT,
	TRANSFORM_ACTION_ROTATE_COUNTER_CLOCKWISE,
	TRANSFORM_ACTION_ROTATE_CLOCKWISE,
};

struct transform_config {
	bool pinch_enabled;
	bool rotation_enabled;
	uint32_t distance_threshold;
	uint32_t rotation_threshold;
	uint32_t pinch_step_threshold;
	uint32_t rotation_step_threshold;
};

struct transform_data {
	const struct device *dev;
	kscan_callback_t callback;
	uint16_t first_x;
	uint16_t first_y;
	uint16_t second_x;
	uint16_t second_y;
	bool second_valid;
	uint8_t fingers;
	bool baseline_valid;
	int32_t baseline_x;
	int32_t baseline_y;
	uint32_t baseline_distance_squared;
	int32_t rotation_step_x;
	int32_t rotation_step_y;
	uint32_t pinch_step_distance_squared;
	bool claim_submitted;
	bool active;
	enum transform_mode mode;
	bool pending_pinch;
	bool pending_rotation;
	enum transform_action pending_pinch_action;
	enum transform_action pending_rotation_action;
	uint32_t ownership_frame_seq;
	bool sensor_reset;
	uint32_t arbiter_frame_seq;
};

static int transform_configure(const struct device *dev, kscan_callback_t callback)
{
	if (callback == NULL) {
		return -EINVAL;
	}
	((struct transform_data *)dev->data)->callback = callback;
	return 0;
}

static const struct kscan_driver_api transform_api = {
	.config = transform_configure,
};

static int32_t abs32(int32_t value)
{
	return value < 0 ? -value : value;
}

static uint32_t squared_distance(int32_t x, int32_t y)
{
	return (uint32_t)(x * x + y * y);
}

static const char *mode_name(enum transform_mode mode)
{
	switch (mode) {
	case TRANSFORM_PINCH:
		return "PINCH";
	case TRANSFORM_ROTATE:
		return "ROTATE";
	case TRANSFORM_PINCH_ROTATE:
		return "PINCH_ROTATE";
	case TRANSFORM_NONE:
	default:
		return "NONE";
	}
}

static void reset_transform(struct transform_data *data)
{
	data->baseline_valid = false;
	data->claim_submitted = false;
	data->active = false;
	data->mode = TRANSFORM_NONE;
	data->pending_pinch = false;
	data->pending_rotation = false;
	data->ownership_frame_seq = 0;
}

static void update_mode(struct transform_data *data, bool pinch, bool rotation)
{
	if ((pinch && (rotation || data->mode == TRANSFORM_ROTATE)) ||
	    (rotation && data->mode == TRANSFORM_PINCH)) {
		data->mode = TRANSFORM_PINCH_ROTATE;
	} else if (pinch && data->mode == TRANSFORM_NONE) {
		data->mode = TRANSFORM_PINCH;
	} else if (rotation && data->mode == TRANSFORM_NONE) {
		data->mode = TRANSFORM_ROTATE;
	}
}

static void queue_transform_actions(struct transform_data *data, int32_t distance_delta,
				    int32_t rotation_cross, bool pinch, bool rotation)
{
	if (pinch) {
		data->pending_pinch = true;
		data->pending_pinch_action = distance_delta < 0 ? TRANSFORM_ACTION_PINCH_IN :
								     TRANSFORM_ACTION_PINCH_OUT;
		data->pinch_step_distance_squared = squared_distance(
			(int32_t)data->second_x - data->first_x,
			(int32_t)data->second_y - data->first_y);
	}
	if (rotation) {
		data->pending_rotation = true;
		/* IQS9150 Y increases downwards, so a positive 2-D cross product is
		 * clockwise on the physical pad. */
		data->pending_rotation_action = rotation_cross < 0 ?
						    TRANSFORM_ACTION_ROTATE_COUNTER_CLOCKWISE :
						    TRANSFORM_ACTION_ROTATE_CLOCKWISE;
		data->rotation_step_x = (int32_t)data->second_x - data->first_x;
		data->rotation_step_y = (int32_t)data->second_y - data->first_y;
	}
	update_mode(data, pinch, rotation);
}

static void emit_pending_actions(struct transform_data *data)
{
	if (data->callback == NULL) {
		LOG_ERR("transform KSCAN callback is not configured");
		data->pending_pinch = false;
		data->pending_rotation = false;
		return;
	}
	if (data->pending_pinch) {
		enum transform_action action = data->pending_pinch_action;

		data->pending_pinch = false;
		if (travelers_pad_output_behavior_primary(
			data->dev, data->callback, data->ownership_frame_seq,
			TRAVELERS_PAD_RECOGNIZER_TRANSFORM_GESTURE,
			TRAVELERS_PAD_OWNER_TRANSFORM_GESTURE, 0, action,
			"transform-pinch") == 0) {
			LOG_INF("2F transform action=%s", action == TRANSFORM_ACTION_PINCH_IN ?
						       "PINCH_IN" : "PINCH_OUT");
		}
	}
	if (data->pending_rotation) {
		enum transform_action action = data->pending_rotation_action;

		data->pending_rotation = false;
		if (travelers_pad_output_behavior_primary(
			data->dev, data->callback, data->ownership_frame_seq,
			TRAVELERS_PAD_RECOGNIZER_TRANSFORM_GESTURE,
			TRAVELERS_PAD_OWNER_TRANSFORM_GESTURE, 0, action,
			"transform-rotation") == 0) {
			LOG_INF("2F transform action=%s",
				action == TRANSFORM_ACTION_ROTATE_COUNTER_CLOCKWISE ?
					"ROTATE_COUNTER_CLOCKWISE" : "ROTATE_CLOCKWISE");
		}
	}
}

static void recognize_transform(const struct device *dev)
{
	const struct transform_config *cfg = dev->config;
	struct transform_data *data = dev->data;
	int32_t geometry_x;
	int32_t geometry_y;
	uint32_t distance_squared;
	int32_t distance_delta;
	int32_t rotation_cross;
	bool pinch;
	bool rotation;

	if (data->sensor_reset || data->fingers != 2 || !data->second_valid) {
		/* Preserve an accepted claim through the normal zero-contact frame so
		 * the resolved-frame handler can log completion and clear pending state. */
		if (data->fingers != 2 && !(data->fingers == 0 && data->claim_submitted)) {
			reset_transform(data);
		}
		return;
	}

	geometry_x = (int32_t)data->second_x - data->first_x;
	geometry_y = (int32_t)data->second_y - data->first_y;
	if (!data->baseline_valid) {
		data->baseline_x = geometry_x;
		data->baseline_y = geometry_y;
		data->baseline_distance_squared = squared_distance(geometry_x, geometry_y);
		data->pinch_step_distance_squared = data->baseline_distance_squared;
		data->rotation_step_x = geometry_x;
		data->rotation_step_y = geometry_y;
		data->baseline_valid = true;
		return;
	}
	if (data->claim_submitted && !data->active) {
		return;
	}

	distance_squared = squared_distance(geometry_x, geometry_y);
	if (data->active) {
		distance_delta = (int32_t)distance_squared - data->pinch_step_distance_squared;
		rotation_cross = data->rotation_step_x * geometry_y -
				 data->rotation_step_y * geometry_x;
		pinch = cfg->pinch_enabled && abs32(distance_delta) >= cfg->pinch_step_threshold;
		rotation = cfg->rotation_enabled && abs32(rotation_cross) >= cfg->rotation_step_threshold;
		if (pinch || rotation) {
			queue_transform_actions(data, distance_delta, rotation_cross, pinch, rotation);
		}
		return;
	}

	distance_delta = (int32_t)distance_squared - data->baseline_distance_squared;
	rotation_cross = data->baseline_x * geometry_y - data->baseline_y * geometry_x;
	pinch = cfg->pinch_enabled && abs32(distance_delta) >= cfg->distance_threshold;
	rotation = cfg->rotation_enabled && abs32(rotation_cross) >= cfg->rotation_threshold;
	if (!pinch && !rotation) {
		return;
	}

	if (!travelers_pad_arbiter_submit(&(struct travelers_pad_arbiter_proposal){
			.frame_seq = data->arbiter_frame_seq,
			.recognizer = TRAVELERS_PAD_RECOGNIZER_TRANSFORM_GESTURE,
			.intent = TRAVELERS_PAD_PROPOSAL_CLAIM,
			.requested_owner = TRAVELERS_PAD_OWNER_TRANSFORM_GESTURE,
			.capabilities = TRAVELERS_PAD_CAP_BEHAVIOR_ACTION,
			.contact_count = data->fingers,
			.evidence = ((uint32_t)(pinch && rotation ? TRANSFORM_PINCH_ROTATE :
					     pinch ? TRANSFORM_PINCH : TRANSFORM_ROTATE) << 30) |
				    (abs32(distance_delta) & 0x3fffffff),
	})) {
		LOG_WRN("2F transform proposal rejected");
		return;
	}
	data->claim_submitted = true;
	data->ownership_frame_seq = data->arbiter_frame_seq;
	queue_transform_actions(data, distance_delta, rotation_cross, pinch, rotation);
	LOG_INF("2F transform=%s proposed distance=%d rotation=%d", mode_name(data->mode),
		distance_delta, rotation_cross);
}

static void transform_deliver_resolved_frame(
	void *context, const struct travelers_pad_session_snapshot *snapshot,
	const struct travelers_pad_owner_token *token)
{
	struct transform_data *data = context;

	if (snapshot->cancelled || data->sensor_reset) {
		reset_transform(data);
		data->sensor_reset = false;
		return;
	}
	if (data->claim_submitted && !data->active && token != NULL && token->valid &&
	    token->owner == TRAVELERS_PAD_OWNER_TRANSFORM_GESTURE) {
		data->active = true;
	}
	if (data->claim_submitted && !data->active && snapshot->contact_count != 0 &&
	    token != NULL && token->valid && token->owner != TRAVELERS_PAD_OWNER_TRANSFORM_GESTURE) {
		data->pending_pinch = false;
		data->pending_rotation = false;
		LOG_INF("2F transform suppressed by owner=%u", token->owner);
	}
	if (data->active && token != NULL && token->valid &&
	    token->owner == TRAVELERS_PAD_OWNER_TRANSFORM_GESTURE && snapshot->contact_count != 0) {
		emit_pending_actions(data);
	}
	if (snapshot->contact_count == 0 && data->claim_submitted) {
		if (token != NULL && token->valid &&
		    token->owner == TRAVELERS_PAD_OWNER_TRANSFORM_GESTURE) {
			LOG_INF("2F transform=%s completed", mode_name(data->mode));
		}
		reset_transform(data);
	}
}

static void transform_handle_input(const struct device *dev, struct input_event *evt)
{
	struct transform_data *data = dev->data;

	if (evt->type == INPUT_EV_ABS) {
		switch (evt->code) {
		case INPUT_ABS_X:
			data->first_x = evt->value;
			break;
		case INPUT_ABS_Y:
			data->first_y = evt->value;
			break;
		case INPUT_ABS_RX:
			data->second_x = evt->value;
			break;
		case INPUT_ABS_RY:
			data->second_y = evt->value;
			break;
		case INPUT_ABS_RUDDER:
			data->second_valid = evt->value;
			break;
		case INPUT_ABS_Z:
			data->fingers = evt->value;
			break;
		case INPUT_ABS_RZ:
			if (evt->value) {
				data->sensor_reset = true;
			}
			break;
		default:
			break;
		}
	}
	if (evt->sync) {
		data->arbiter_frame_seq =
			travelers_pad_arbiter_next_frame(TRAVELERS_PAD_RECOGNIZER_TRANSFORM_GESTURE);
		recognize_transform(dev);
		travelers_pad_arbiter_observed(TRAVELERS_PAD_RECOGNIZER_TRANSFORM_GESTURE,
					       data->arbiter_frame_seq);
	}
}

static int transform_init(const struct device *dev)
{
	(void)travelers_pad_arbiter_register(TRAVELERS_PAD_RECOGNIZER_TRANSFORM_GESTURE);
	(void)travelers_pad_arbiter_bind_frame_handler(
		TRAVELERS_PAD_RECOGNIZER_TRANSFORM_GESTURE, transform_deliver_resolved_frame, dev->data);
	return 0;
}

#define TRANSFORM_INST(n)                                                                       \
	static struct transform_data transform_data_##n = {.dev = DEVICE_DT_INST_GET(n)};         \
	static const struct transform_config transform_config_##n = {                              \
		.pinch_enabled = !DT_INST_PROP(n, pinch_disabled),                                  \
		.rotation_enabled = !DT_INST_PROP(n, rotation_disabled),                            \
		.distance_threshold = DT_INST_PROP_OR(n, distance_threshold, 30000),                 \
		.rotation_threshold = DT_INST_PROP_OR(n, rotation_threshold, 30000),                 \
		.pinch_step_threshold = DT_INST_PROP_OR(n, pinch_step_threshold, 30000),             \
		.rotation_step_threshold = DT_INST_PROP_OR(n, rotation_step_threshold, 30000),       \
	};                                                                                           \
	static void transform_input_handler_##n(struct input_event *evt)                           \
	{                                                                                            \
		transform_handle_input(DEVICE_DT_INST_GET(n), evt);                                    \
	}                                                                                            \
	INPUT_CALLBACK_DEFINE(DEVICE_DT_GET(DT_INST_PHANDLE(n, input_device)),                     \
			      transform_input_handler_##n);                                                 \
	DEVICE_DT_INST_DEFINE(n, transform_init, NULL, &transform_data_##n, &transform_config_##n, \
			      POST_KERNEL, CONFIG_TRAVELERS_PAD_IQS9150_FUNCTION_INIT_PRIORITY,       \
			      &transform_api);

DT_INST_FOREACH_STATUS_OKAY(TRANSFORM_INST)
