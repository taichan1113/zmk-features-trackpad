/*
 * Travelers Pad IQS9150 multi-finger router.
 *
 * Normalized session contact data is classified before it reaches the ordinary
 * pointer function. This makes gesture ownership explicit: pinch/rotation and
 * 3--5 finger sessions never leak into two-finger scroll output.
 *
 * 正規化済みsession接触データを通常ポインタ機能の前で分類する。ピンチ・回転・3--5本指
 * 操作を二本指スクロールへ漏らさないため、操作の所有権をここで明示的に決める。
 */

#define DT_DRV_COMPAT zmk_travelers_pad_iqs9150_multifinger_router

#include <zephyr/device.h>
#include <zephyr/dt-bindings/input/input-event-codes.h>
#include <zephyr/input/input.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

#include <travelers_pad/contact_arbiter.h>
#include <travelers_pad/multitouch.h>

LOG_MODULE_REGISTER(travelers_pad_router, LOG_LEVEL_INF);

enum router_state {
	ROUTER_IDLE,
	ROUTER_PASS_ONE,
	ROUTER_AWAIT_TWO,
	ROUTER_SCROLL_TWO,
	ROUTER_CLAIM_GEOMETRY,
	ROUTER_CLAIM_MULTIFINGER,
	ROUTER_TAP_DRAG,
	ROUTER_EDGE_DIAL,
};

struct router_config {
	bool pinch_geometry_enabled;
	bool rotation_geometry_enabled;
	uint16_t scroll_classify_move;
	uint32_t geometry_distance_threshold;
	uint32_t geometry_rotation_threshold;
};

struct router_data {
	const struct device *dev;
	enum router_state state;
	uint16_t x;
	uint16_t y;
	uint16_t finger2_x;
	uint16_t finger2_y;
	bool finger2_valid;
	uint8_t fingers;
	uint8_t max_fingers;
	uint16_t start_x;
	uint16_t start_y;
	uint16_t start_finger2_x;
	uint16_t start_finger2_y;
	bool have_geometry;
	int32_t start_geometry_x;
	int32_t start_geometry_y;
	uint32_t start_distance_squared;
	bool sensor_reset;
	uint32_t arbiter_frame_seq;
	bool scroll_extra_contact;
	/* Router contact output is deferred until the arbiter has observed every
	 * primary recognizer for this frame. Track whether the pointer adapter has
	 * an open logical contact so an ownership hand-off can cancel it exactly
	 * once. / routerの接触出力は、このframeを全primary recognizerが観測した後まで
	 * 保留する。pointer adapterが開いた論理接触を追跡し、所有権移行時に一度だけ
	 * cancelできるようにする。 */
	bool pointer_stream_open;
};

static int32_t abs32(int32_t value)
{
	return value < 0 ? -value : value;
}

static uint32_t squared_distance(int32_t x, int32_t y)
{
	return (uint32_t)(x * x + y * y);
}

static void router_emit(const struct device *dev, const struct router_data *data,
			uint32_t frame_seq, uint8_t fingers)
{
	int rc = input_report(dev, INPUT_EV_MSC, TRAVELERS_PAD_INPUT_MSC_ROUTED_FRAME_SEQ,
			      frame_seq, false, K_NO_WAIT);

	rc |= input_report_abs(dev, INPUT_ABS_X, data->x, false, K_NO_WAIT);

	rc |= input_report_abs(dev, INPUT_ABS_Y, data->y, false, K_NO_WAIT);
	rc |= input_report_abs(dev, INPUT_ABS_RX, data->finger2_x, false, K_NO_WAIT);
	rc |= input_report_abs(dev, INPUT_ABS_RY, data->finger2_y, false, K_NO_WAIT);
	rc |= input_report_abs(dev, INPUT_ABS_RUDDER, data->finger2_valid, false, K_NO_WAIT);
	rc |= input_report_abs(dev, INPUT_ABS_Z, fingers, false, K_NO_WAIT);
	rc |= input_report_key(dev, INPUT_BTN_TOUCH, fingers != 0, true, K_NO_WAIT);
	if (rc != 0) {
		LOG_ERR("routed contact frame report failed: %d", rc);
	}
}

static void router_cancel_pointer(const struct device *dev)
{
	int rc = input_report_abs(dev, INPUT_ABS_RZ, 1, false, K_NO_WAIT);

	rc |= input_report_abs(dev, INPUT_ABS_Z, 0, false, K_NO_WAIT);
	rc |= input_report_key(dev, INPUT_BTN_TOUCH, 0, true, K_NO_WAIT);
	if (rc != 0) {
		LOG_ERR("pointer cancellation report failed: %d", rc);
	}
}

static void router_begin(struct router_data *data)
{
	data->max_fingers = data->fingers;
	data->start_x = data->x;
	data->start_y = data->y;
	data->start_finger2_x = data->finger2_x;
	data->start_finger2_y = data->finger2_y;
	data->have_geometry = data->finger2_valid;
	if (data->have_geometry) {
		data->start_geometry_x = (int32_t)data->finger2_x - data->x;
		data->start_geometry_y = (int32_t)data->finger2_y - data->y;
		data->start_distance_squared = squared_distance(data->start_geometry_x,
							data->start_geometry_y);
	}
	data->scroll_extra_contact = false;
}

static const char *swipe_direction(const struct router_data *data)
{
	int32_t dx = (int32_t)data->x - data->start_x;
	int32_t dy = (int32_t)data->y - data->start_y;

	if (abs32(dx) >= abs32(dy)) {
		return dx >= 0 ? "right" : "left";
	}
	return dy >= 0 ? "down" : "up";
}

/* Phase 1: observe and propose only. Input reports are deferred until every
 * primary recognizer has observed the frame. / 第1段階は観測とproposalだけを
 * 行い、input reportは全primary recognizerの観測完了まで保留する。 */
static void router_recognize_frame(const struct device *dev)
{
	const struct router_config *cfg = dev->config;
	struct router_data *data = dev->data;
	int32_t geometry_x;
	int32_t geometry_y;
	uint32_t distance_squared;
	uint32_t distance_delta;
	uint32_t rotation_cross;
	int32_t first_dx;
	int32_t first_dy;
	int32_t second_dx;
	int32_t second_dy;
	int32_t direction_dot;
	uint32_t first_move;
	uint32_t second_move;
	/* Ownership and all host-facing contact output are resolved after every
	 * primary recognizer has observed this frame. */
	if (data->sensor_reset || data->fingers == 0 || data->state == ROUTER_TAP_DRAG ||
	    data->state == ROUTER_EDGE_DIAL || data->state == ROUTER_CLAIM_GEOMETRY ||
	    data->state == ROUTER_CLAIM_MULTIFINGER) {
		return;
	}

	if (data->state == ROUTER_IDLE) {
		router_begin(data);
		if (data->fingers == 1) {
			data->state = ROUTER_PASS_ONE;
		} else if (data->fingers == 2) {
			data->state = ROUTER_AWAIT_TWO;
			LOG_INF("2F route=awaiting scroll-or-geometry classification");
		} else {
			data->state = ROUTER_CLAIM_MULTIFINGER;
			if (data->fingers == 5) {
				LOG_INF("5F route=Grasp/Release claimed");
			} else {
				LOG_INF("%uF route=multifinger claimed", data->fingers);
			}
		}
		return;
	}

	data->max_fingers = MAX(data->max_fingers, data->fingers);
	if (data->state == ROUTER_PASS_ONE && data->fingers >= 2) {
		router_begin(data);
		data->state = data->fingers == 2 ? ROUTER_AWAIT_TWO : ROUTER_CLAIM_MULTIFINGER;
		return;
	}
	if (data->state == ROUTER_SCROLL_TWO && data->fingers >= 3) {
		/* A committed scroll owns this session until all contacts are released.
		 * Promoting a third contact to MULTI_GESTURE here would leave the arbiter
		 * owner as SCROLL while this router stops producing scroll frames. Keep a
		 * stable logical two-contact stream instead; a new 3--5F gesture requires
		 * a fresh contact session.
		 * commit済みscrollは全接触が離れるまでこのsessionを所有する。ここで3本目を
		 * MULTI_GESTUREへ昇格すると、arbiter ownerはSCROLLのままrouterだけが
		 * scroll frameを止める。安定した論理2点streamを維持し、3〜5本指gestureは
		 * 新しいcontact sessionから始める。 */
		if (!data->scroll_extra_contact) {
			LOG_INF("%uF added during committed scroll; retaining scroll owner", data->fingers);
			data->scroll_extra_contact = true;
		}
	}
	if (data->state == ROUTER_AWAIT_TWO && data->fingers >= 3) {
		data->state = ROUTER_CLAIM_MULTIFINGER;
		if (data->fingers == 5) {
			LOG_INF("5F route=Grasp/Release claimed");
		} else {
			LOG_INF("%uF route=multifinger claimed", data->fingers);
		}
		return;
	}
	if (data->state == ROUTER_PASS_ONE) {
		return;
	}
	if (data->state == ROUTER_SCROLL_TWO) {
		if (data->fingers < 3) {
			data->scroll_extra_contact = false;
		}
		return;
	}
	if (data->state != ROUTER_AWAIT_TWO || data->fingers != 2) {
		return;
	}

	if (data->finger2_valid && data->have_geometry) {
		geometry_x = (int32_t)data->finger2_x - data->x;
		geometry_y = (int32_t)data->finger2_y - data->y;
		distance_squared = squared_distance(geometry_x, geometry_y);
		distance_delta = abs32((int32_t)distance_squared - data->start_distance_squared);
		rotation_cross = abs32(data->start_geometry_x * geometry_y -
				       data->start_geometry_y * geometry_x);
		if ((cfg->pinch_geometry_enabled && distance_delta >= cfg->geometry_distance_threshold) ||
		    (cfg->rotation_geometry_enabled && rotation_cross >= cfg->geometry_rotation_threshold)) {
			data->state = ROUTER_CLAIM_GEOMETRY;
			LOG_INF("2F route=geometry claimed distance-delta=%u rotation=%u", distance_delta,
				rotation_cross);
			return;
		}
	}
	if (!data->finger2_valid || !data->have_geometry) {
		return;
	}
	first_dx = (int32_t)data->x - data->start_x;
	first_dy = (int32_t)data->y - data->start_y;
	second_dx = (int32_t)data->finger2_x - data->start_finger2_x;
	second_dy = (int32_t)data->finger2_y - data->start_finger2_y;
	first_move = abs32(first_dx) + abs32(first_dy);
	second_move = abs32(second_dx) + abs32(second_dy);
	direction_dot = first_dx * second_dx + first_dy * second_dy;
	/* A scroll is a translation of both contacts in the same general
	 * direction. Requiring both displacement vectors prevents a slow pinch or
	 * rotation from committing SCROLL merely because contact 1 moved first. */
	if (first_move >= cfg->scroll_classify_move &&
	    second_move >= cfg->scroll_classify_move && direction_dot > 0) {
		/* The frame callback emits only after every recognizer has observed this
		 * proposal and the arbiter has made its immutable decision. */
		if (!travelers_pad_arbiter_submit(&(struct travelers_pad_arbiter_proposal){
			.frame_seq = data->arbiter_frame_seq,
			.recognizer = TRAVELERS_PAD_RECOGNIZER_ROUTER,
			.intent = TRAVELERS_PAD_PROPOSAL_CLAIM,
			.requested_owner = TRAVELERS_PAD_OWNER_SCROLL,
			.capabilities = TRAVELERS_PAD_CAP_SCROLL_DELTA,
			.contact_count = data->fingers,
			.evidence = MIN(first_move, second_move),
		})) {
			LOG_WRN("2F scroll proposal rejected");
			return;
		}
		data->state = ROUTER_SCROLL_TWO;
		LOG_INF("2F route=scroll proposed move=(%u,%u) dot=%d; awaiting frame decision",
			first_move, second_move, direction_dot);
	}
}

static void router_suppress_pointer(const struct device *dev, struct router_data *data)
{
	if (data->pointer_stream_open) {
		router_cancel_pointer(dev);
		data->pointer_stream_open = false;
	}
}

/* Phase 2: invoked once after all primary recognizers observed this frame.
 * This is the router's only contact-reporting path. / 第2段階は全primary
 * recognizerの観測後に一度だけ呼ばれる。routerで唯一接触reportを送れる経路である。 */
static void router_deliver_resolved_frame(
	void *context, const struct travelers_pad_session_snapshot *snapshot,
	const struct travelers_pad_owner_token *token)
{
	struct router_data *data = context;
	const struct device *dev = data->dev;
	enum travelers_pad_owner owner = token != NULL ? token->owner : TRAVELERS_PAD_OWNER_NONE;

	if (snapshot->cancelled || data->sensor_reset) {
		router_suppress_pointer(dev, data);
		data->state = ROUTER_IDLE;
		data->max_fingers = 0;
		data->sensor_reset = false;
		LOG_WRN("router cancelled session after sensor reset");
		return;
	}

	if (owner == TRAVELERS_PAD_OWNER_DRAG) {
		router_suppress_pointer(dev, data);
		data->state = ROUTER_TAP_DRAG;
		if (snapshot->contact_count == 0) {
			data->state = ROUTER_IDLE;
			data->max_fingers = 0;
		}
		return;
	}
	if (owner == TRAVELERS_PAD_OWNER_EDGE_DIAL) {
		router_suppress_pointer(dev, data);
		data->state = ROUTER_EDGE_DIAL;
		if (snapshot->contact_count == 0) {
			data->state = ROUTER_IDLE;
			data->max_fingers = 0;
		}
		return;
	}
	if (owner == TRAVELERS_PAD_OWNER_TIP_TAP || owner == TRAVELERS_PAD_OWNER_TRANSFORM_GESTURE ||
	    owner == TRAVELERS_PAD_OWNER_MULTI_GESTURE) {
		router_suppress_pointer(dev, data);
		data->state = ROUTER_CLAIM_MULTIFINGER;
		if (snapshot->contact_count == 0) {
			data->state = ROUTER_IDLE;
			data->max_fingers = 0;
		}
		return;
	}
	if (owner == TRAVELERS_PAD_OWNER_SCROLL) {
		data->state = ROUTER_SCROLL_TWO;
		if (snapshot->contact_count == 0) {
			router_emit(dev, data, snapshot->frame_seq, 0);
			data->pointer_stream_open = false;
			data->state = ROUTER_IDLE;
			data->max_fingers = 0;
			return;
		}
		if (!travelers_pad_arbiter_prepare_routed_output(
			snapshot->frame_seq, TRAVELERS_PAD_OWNER_SCROLL,
			TRAVELERS_PAD_CAP_SCROLL_DELTA)) {
			return;
		}
		/* The pointer adapter consumes a logical two-contact scroll stream, even
		 * when a third physical contact arrives during an owned scroll. */
		router_emit(dev, data, snapshot->frame_seq,
			snapshot->contact_count >= 2 ? 2 : snapshot->contact_count);
		data->pointer_stream_open = snapshot->contact_count != 0;
		return;
	}

	if (snapshot->contact_count == 0) {
		if (data->state == ROUTER_PASS_ONE || data->state == ROUTER_SCROLL_TWO) {
			router_emit(dev, data, snapshot->frame_seq, 0);
			data->pointer_stream_open = false;
		} else if (data->state == ROUTER_AWAIT_TWO) {
			/* A held-still 2F session becomes a right-click candidate only on
			 * release, after every same-frame competing claim was resolved. */
			data->x = data->start_x;
			data->y = data->start_y;
			data->finger2_x = data->start_finger2_x;
			data->finger2_y = data->start_finger2_y;
			data->finger2_valid = data->have_geometry;
			router_emit(dev, data, snapshot->frame_seq, 2);
			router_emit(dev, data, snapshot->frame_seq, 0);
			LOG_INF("2F route=tap candidate");
		} else if (data->state == ROUTER_CLAIM_GEOMETRY) {
			LOG_INF("2F route=geometry candidate complete");
		} else if (data->state == ROUTER_CLAIM_MULTIFINGER) {
			if (data->max_fingers == 5) {
				LOG_INF("5F route=Grasp/Release candidate complete");
			} else {
				LOG_INF("%uF route=swipe candidate direction=%s", data->max_fingers,
					swipe_direction(data));
			}
		}
		data->state = ROUTER_IDLE;
		data->max_fingers = 0;
		return;
	}

	if (data->state == ROUTER_PASS_ONE) {
		if (!travelers_pad_arbiter_prepare_routed_output(
			snapshot->frame_seq, TRAVELERS_PAD_OWNER_NONE,
			TRAVELERS_PAD_CAP_POINTER_DELTA)) {
			return;
		}
		router_emit(dev, data, snapshot->frame_seq, 1);
		data->pointer_stream_open = true;
	} else {
		/* Awaiting a two-finger classification must not leak a pointer frame. */
		router_suppress_pointer(dev, data);
	}
}

static void router_handle_input(const struct device *dev, struct input_event *evt)
{
	struct router_data *data = dev->data;

	if (evt->type == INPUT_EV_ABS) {
		if (evt->code == INPUT_ABS_X) {
			data->x = evt->value;
		} else if (evt->code == INPUT_ABS_Y) {
			data->y = evt->value;
		} else if (evt->code == INPUT_ABS_RX) {
			data->finger2_x = evt->value;
		} else if (evt->code == INPUT_ABS_RY) {
			data->finger2_y = evt->value;
		} else if (evt->code == INPUT_ABS_RUDDER) {
			data->finger2_valid = evt->value;
		} else if (evt->code == INPUT_ABS_Z) {
			data->fingers = evt->value;
		} else if (evt->code == INPUT_ABS_RZ && evt->value) {
			data->sensor_reset = true;
		}
	}
	if (evt->sync) {
		data->arbiter_frame_seq =
			travelers_pad_arbiter_next_frame(TRAVELERS_PAD_RECOGNIZER_ROUTER);
		router_recognize_frame(dev);
		travelers_pad_arbiter_observed(TRAVELERS_PAD_RECOGNIZER_ROUTER,
					       data->arbiter_frame_seq);
	}
}

static int router_init(const struct device *dev)
{
	(void)travelers_pad_arbiter_register(TRAVELERS_PAD_RECOGNIZER_ROUTER);
	(void)travelers_pad_arbiter_bind_frame_handler(
		TRAVELERS_PAD_RECOGNIZER_ROUTER, router_deliver_resolved_frame, dev->data);
	return 0;
}

/* Read configuration only; do not call into a competing recognizer.
 * 設定だけを共有し、競合recognizerへのruntime呼出しは追加しない。 */
#define ROUTER_GEOMETRY_ENABLED(n, disabled_prop)                                               \
	COND_CODE_1(DT_INST_NODE_HAS_PROP(n, transform_device),                                  \
		(DT_NODE_HAS_STATUS(DT_INST_PHANDLE(n, transform_device), okay) &&                 \
		 IS_ENABLED(CONFIG_TRAVELERS_PAD_IQS9150_TRANSFORM_GESTURE) &&                     \
		 !DT_PROP_OR(DT_INST_PHANDLE(n, transform_device), disabled_prop, false)), (true))

#define ROUTER_INST(n)                                                                        \
	static struct router_data router_data_##n = {.dev = DEVICE_DT_INST_GET(n)};                \
	static const struct router_config router_config_##n = {                                    \
		.pinch_geometry_enabled = ROUTER_GEOMETRY_ENABLED(n, pinch_disabled),              \
		.rotation_geometry_enabled = ROUTER_GEOMETRY_ENABLED(n, rotation_disabled),        \
		.scroll_classify_move = DT_INST_PROP_OR(n, scroll_classify_move, 35),               \
		.geometry_distance_threshold = DT_INST_PROP_OR(n, geometry_distance_threshold, 30000), \
		.geometry_rotation_threshold = DT_INST_PROP_OR(n, geometry_rotation_threshold, 30000), \
	};                                                                                           \
	static void router_input_handler_##n(struct input_event *evt) {                             \
		router_handle_input(DEVICE_DT_INST_GET(n), evt);                                      \
	}                                                                                            \
	INPUT_CALLBACK_DEFINE(DEVICE_DT_GET(DT_INST_PHANDLE(n, input_device)),                     \
			      router_input_handler_##n);                                                   \
	DEVICE_DT_INST_DEFINE(n, router_init, NULL, &router_data_##n, &router_config_##n, POST_KERNEL,   \
			      CONFIG_TRAVELERS_PAD_IQS9150_FUNCTION_INIT_PRIORITY, NULL);

DT_INST_FOREACH_STATUS_OKAY(ROUTER_INST)
