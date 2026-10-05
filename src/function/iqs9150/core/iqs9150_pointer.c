/*
 * Travelers Pad IQS9150 pointer function.
 *
 * This module consumes normalized contact frames from the sensor driver and
 * publishes ordinary ZMK relative-pointer events. It deliberately contains
 * pointer motion and scrolling policy, leaving tap/drag and the driver reusable.
 */

#define DT_DRV_COMPAT zmk_travelers_pad_iqs9150_pointer

#include <zephyr/device.h>
#include <zephyr/dt-bindings/input/input-event-codes.h>
#include <zephyr/input/input.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

#include <travelers_pad/contact_output_broker.h>
#include <travelers_pad/multitouch.h>

LOG_MODULE_REGISTER(travelers_pad_pointer, LOG_LEVEL_INF);

struct pointer_config {
	uint16_t scroll_axis_lock_min_move;
	uint16_t scroll_axis_lock_ratio;
	uint16_t scroll_max_frame_delta;
};

enum scroll_axis {
	SCROLL_AXIS_UNDECIDED,
	SCROLL_AXIS_HORIZONTAL,
	SCROLL_AXIS_VERTICAL,
};

struct pointer_data {
	const struct device *dev;
	uint16_t x;
	uint16_t y;
	uint8_t fingers;
	uint8_t previous_fingers;
	uint16_t previous_x;
	uint16_t previous_y;
	bool have_position;
	bool scroll_session;
	uint32_t routed_frame_seq;
	enum scroll_axis scroll_axis;
	int32_t scroll_probe_x;
	int32_t scroll_probe_y;
	bool reset_frame;
};

static int32_t absolute_int32(int32_t value)
{
	return value < 0 ? -value : value;
}

static void reset_scroll_state(struct pointer_data *data)
{
	data->scroll_axis = SCROLL_AXIS_UNDECIDED;
	data->scroll_probe_x = 0;
	data->scroll_probe_y = 0;
}

static void report_motion(const struct device *dev, const struct pointer_data *data,
			  int16_t dx, int16_t dy) {
	int rc;

	if (dx != 0) {
		rc = travelers_pad_output_routed_pointer(
			dev, data->routed_frame_seq, TRAVELERS_PAD_OWNER_NONE,
			TRAVELERS_PAD_CAP_POINTER_DELTA, INPUT_EV_REL, INPUT_REL_X, dx,
			dy == 0, "pointer-x");
		if (rc != 0) {
			LOG_ERR("pointer X report failed: %d", rc);
		}
	}
	if (dy != 0) {
		rc = travelers_pad_output_routed_pointer(
			dev, data->routed_frame_seq, TRAVELERS_PAD_OWNER_NONE,
			TRAVELERS_PAD_CAP_POINTER_DELTA, INPUT_EV_REL, INPUT_REL_Y, dy, true,
			"pointer-y");
		if (rc != 0) {
			LOG_ERR("pointer Y report failed: %d", rc);
		}
	}
}

static void report_scroll(const struct device *dev, const struct pointer_data *data,
			  int16_t dx, int16_t dy) {
	int rc;

	/* The input processor attached to this device owns user-facing inversion
	 * and scaling. Keep this function's output in the canonical ZMK wheel
	 * orientation so it remains a pure contact-to-relative adapter.
	 * ユーザー向けの反転と倍率は、このデバイスのinput processorが担当する。
	 * この機能は接触フレームを標準的なZMKホイール方向へ変換するだけにする。 */
	int16_t horizontal = dx;
	int16_t vertical = -dy;

	if (horizontal != 0) {
		rc = travelers_pad_output_routed_pointer(
			dev, data->routed_frame_seq, TRAVELERS_PAD_OWNER_SCROLL,
			TRAVELERS_PAD_CAP_SCROLL_DELTA, INPUT_EV_REL, INPUT_REL_HWHEEL, horizontal,
			vertical == 0, "pointer-hwheel");
		if (rc != 0) {
			LOG_ERR("horizontal scroll report failed: %d", rc);
		}
	}
	if (vertical != 0) {
		rc = travelers_pad_output_routed_pointer(
			dev, data->routed_frame_seq, TRAVELERS_PAD_OWNER_SCROLL,
			TRAVELERS_PAD_CAP_SCROLL_DELTA, INPUT_EV_REL, INPUT_REL_WHEEL, -vertical,
			true, "pointer-wheel");
		if (rc != 0) {
			LOG_ERR("vertical scroll report failed: %d", rc);
		}
	}
}

static void process_scroll_delta(const struct device *dev, const struct pointer_config *cfg,
				 int16_t dx, int16_t dy)
{
	struct pointer_data *data = dev->data;
	int32_t x = dx;
	int32_t y = dy;
	int32_t x_abs;
	int32_t y_abs;
	uint16_t ratio = MAX(cfg->scroll_axis_lock_ratio, 1);

	/* Contact tracking can occasionally hand us a one-frame coordinate
	 * discontinuity. Never convert it to a large wheel jump; rebase and wait
	 * for the next coherent delta instead.
	 * 接触追跡が一時的に不連続な座標を返すことがある。これを大きなホイール
	 * ジャンプへ変換せず、基準だけ更新して次の整合した差分を待つ。 */
	if (absolute_int32(x) > cfg->scroll_max_frame_delta ||
	    absolute_int32(y) > cfg->scroll_max_frame_delta) {
		LOG_WRN("discarded implausible scroll delta %d/%d", dx, dy);
		reset_scroll_state(data);
		return;
	}

	if (data->scroll_axis == SCROLL_AXIS_UNDECIDED) {
		data->scroll_probe_x += x;
		data->scroll_probe_y += y;
		x_abs = absolute_int32(data->scroll_probe_x);
		y_abs = absolute_int32(data->scroll_probe_y);
		if (x_abs + y_abs < cfg->scroll_axis_lock_min_move) {
			return;
		}

		if (x_abs >= y_abs * ratio) {
			data->scroll_axis = SCROLL_AXIS_HORIZONTAL;
			LOG_INF("scroll axis locked: horizontal");
		} else {
			/* Equal or ambiguous diagonal movement resolves to vertical. This
			 * keeps the usual two-finger up/down gesture free of HWHEEL noise.
			 * 同程度または曖昧な斜め移動は縦に決定する。一般的な2本指の上下
			 * スクロールにHWHEELノイズを混ぜないためである。 */
			data->scroll_axis = SCROLL_AXIS_VERTICAL;
			LOG_INF("scroll axis locked: vertical");
		}
		x = data->scroll_probe_x;
		y = data->scroll_probe_y;
		data->scroll_probe_x = 0;
		data->scroll_probe_y = 0;
	}

	if (data->scroll_axis == SCROLL_AXIS_HORIZONTAL) {
		y = 0;
	} else {
		x = 0;
	}
	if (x != 0 || y != 0) {
		report_scroll(dev, data, (int16_t)x, (int16_t)y);
	}
}

static void process_frame(const struct device *dev) {
	const struct pointer_config *cfg = dev->config;
	struct pointer_data *data = dev->data;
	int16_t dx = 0;
	int16_t dy = 0;

	if (data->reset_frame) {
		data->fingers = 0;
		data->previous_fingers = 0;
		data->have_position = false;
		data->scroll_session = false;
		reset_scroll_state(data);
		data->reset_frame = false;
		data->routed_frame_seq = 0;
		/* The router uses this internal marker to end provisional pointer
		 * processing before it hands a contact to multi-finger logic. This is
		 * normal routing, not necessarily a physical sensor reset.
		 * ルーターは多指ロジックへ渡す前に、この内部マーカーで仮ポインタ処理を
		 * 終了する。通常の経路切替であり、必ずしもセンサーリセットではない。 */
		LOG_DBG("input session cleared by routed frame marker");
		return;
	}

	if (data->fingers != 0) {
		if (!data->scroll_session && data->fingers >= 2) {
			/* The 1F-to-2F centroid transition is not a scroll movement. */
			data->scroll_session = true;
			reset_scroll_state(data);
			data->previous_x = data->x;
			data->previous_y = data->y;
			data->have_position = true;
		} else if (data->scroll_session &&
			   (data->fingers < 2 || data->previous_fingers < 2)) {
			/* A partial contact frame during acquisition/release must only
			 * rebase. It cannot be used for either pointer or wheel movement. */
			data->previous_x = data->x;
			data->previous_y = data->y;
			data->have_position = true;
		} else if (data->have_position && data->previous_fingers != 0) {
			dx = (int16_t)((int)data->x - (int)data->previous_x);
			dy = (int16_t)((int)data->y - (int)data->previous_y);
			if (data->scroll_session) {
				process_scroll_delta(dev, cfg, dx, dy);
			} else {
				report_motion(dev, data, dx, dy);
			}
		}
		data->previous_x = data->x;
		data->previous_y = data->y;
		data->have_position = true;
	} else if (data->previous_fingers != 0) {
		data->have_position = false;
		data->scroll_session = false;
		reset_scroll_state(data);
	}

	data->previous_fingers = data->fingers;
}

static int pointer_init(const struct device *dev)
{
	const struct pointer_config *cfg = dev->config;

	LOG_INF("pointer active: axis-min=%u ratio=%u max-delta=%u; scroll transform/scaler are listener-owned",
		cfg->scroll_axis_lock_min_move, cfg->scroll_axis_lock_ratio, cfg->scroll_max_frame_delta);
	return 0;
}

static void pointer_handle_input(const struct device *dev, struct input_event *evt) {
	struct pointer_data *data = dev->data;

	switch (evt->type) {
	case INPUT_EV_MSC:
		if (evt->code == TRAVELERS_PAD_INPUT_MSC_ROUTED_FRAME_SEQ) {
			data->routed_frame_seq = evt->value;
		}
		break;
	case INPUT_EV_ABS:
		if (evt->code == INPUT_ABS_X) {
			data->x = evt->value;
		} else if (evt->code == INPUT_ABS_Y) {
			data->y = evt->value;
		} else if (evt->code == INPUT_ABS_Z) {
			data->fingers = evt->value;
		} else if (evt->code == INPUT_ABS_RZ && evt->value) {
			data->reset_frame = true;
		}
		break;
	default:
		break;
	}

	if (evt->sync) {
		process_frame(dev);
	}
}

#define POINTER_INST(n)                                                                          \
	static struct pointer_data pointer_data_##n = {.dev = DEVICE_DT_INST_GET(n)};                 \
	static const struct pointer_config pointer_config_##n = {                                     \
		.scroll_axis_lock_min_move = DT_INST_PROP_OR(n, scroll_axis_lock_min_move, 12),         \
		.scroll_axis_lock_ratio = DT_INST_PROP_OR(n, scroll_axis_lock_ratio, 2),                \
		.scroll_max_frame_delta = DT_INST_PROP_OR(n, scroll_max_frame_delta, 128),              \
	};                                                                                            \
	static void pointer_input_handler_##n(struct input_event *evt) {                              \
		pointer_handle_input(DEVICE_DT_INST_GET(n), evt);                                       \
	}                                                                                             \
	INPUT_CALLBACK_DEFINE(DEVICE_DT_GET(DT_INST_PHANDLE(n, input_device)),                        \
			      pointer_input_handler_##n);                                                      \
	DEVICE_DT_INST_DEFINE(n, pointer_init, NULL, &pointer_data_##n, &pointer_config_##n, POST_KERNEL, \
			      CONFIG_TRAVELERS_PAD_IQS9150_FUNCTION_INIT_PRIORITY, NULL);

DT_INST_FOREACH_STATUS_OKAY(POINTER_INST)
