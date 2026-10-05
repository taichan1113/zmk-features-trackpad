/*
 * Travelers Pad IQS9150 edge-hold continuation detector.
 *
 * A committed drag and router-committed two-finger scroll can continue beyond
 * the physical pad edge. Raw coordinates detect an edge candidate, but each
 * delayed output is authorized by the live arbiter owner token. Scroll
 * direction is latched from the pointer function's axis-locked wheel output.
 *
 * 確定済みドラッグとrouter確定済み2本指スクロールはパッド端の外まで継続する。
 * 生座標で辺候補を検出するが、遅延出力ごとにlive arbiter owner tokenで許可する。
 * スクロール方向は、ポインタ機能がすでに軸ロックして出力した標準ホイール値から
 * ラッチする。
 */

#define DT_DRV_COMPAT zmk_travelers_pad_iqs9150_edge_hold_continue

#include <errno.h>

#include <zephyr/device.h>
#include <zephyr/dt-bindings/input/input-event-codes.h>
#include <zephyr/input/input.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <travelers_pad/contact_arbiter.h>
#include <travelers_pad/contact_output_broker.h>
#include <travelers_pad/multitouch.h>

LOG_MODULE_REGISTER(travelers_pad_edge_hold, LOG_LEVEL_INF);

enum edge_hold_edge {
	EDGE_HOLD_NONE,
	EDGE_HOLD_LEFT,
	EDGE_HOLD_RIGHT,
	EDGE_HOLD_TOP,
	EDGE_HOLD_BOTTOM,
};

struct edge_hold_config {
	uint16_t surface_x;
	uint16_t surface_y;
	uint16_t edge_size;
	uint16_t drag_start_delay_ms;
	uint16_t drag_repeat_ms;
	int16_t drag_repeat_delta;
	const struct device *scroll_output_device;
	uint16_t scroll_start_delay_ms;
	uint16_t scroll_repeat_ms;
	int16_t scroll_repeat_delta;
};

struct edge_hold_data {
	const struct device *dev;
	uint8_t slot;
	uint16_t slot_x[TRAVELERS_PAD_MAX_CONTACTS];
	uint16_t slot_y[TRAVELERS_PAD_MAX_CONTACTS];
	bool slot_valid[TRAVELERS_PAD_MAX_CONTACTS];
	uint8_t drag_fingers;
	uint16_t scroll_x;
	uint16_t scroll_y;
	uint16_t scroll_finger2_x;
	uint16_t scroll_finger2_y;
	bool scroll_finger2_valid;
	uint8_t scroll_fingers;
	enum edge_hold_edge drag_edge;
	enum edge_hold_edge scroll_edge;
	bool drag_repeat_armed;
	bool scroll_repeat_armed;
	struct k_work_delayable drag_repeat_work;
	struct k_work_delayable scroll_repeat_work;
	struct k_work_sync drag_repeat_sync;
	struct k_work_sync scroll_repeat_sync;
	struct travelers_pad_auxiliary_lease drag_lease;
	struct travelers_pad_auxiliary_lease scroll_lease;
	int16_t scroll_horizontal;
	int16_t scroll_vertical;
	int16_t scroll_direction_horizontal;
	int16_t scroll_direction_vertical;
	uint32_t arbiter_frame_seq;
	uint32_t drag_work_generation;
	uint32_t scroll_work_generation;
};

static const char *edge_name(enum edge_hold_edge edge)
{
	switch (edge) {
	case EDGE_HOLD_LEFT:
		return "left";
	case EDGE_HOLD_RIGHT:
		return "right";
	case EDGE_HOLD_TOP:
		return "top";
	case EDGE_HOLD_BOTTOM:
		return "bottom";
	default:
		return "none";
	}
}

static enum edge_hold_edge edge_at(const struct edge_hold_config *cfg, uint16_t x, uint16_t y)
{
	/* Pick the nearest edge in a corner so a single contact has one stable
	 * continuation direction. The final output stage will latch that direction
	 * until release; this detector verifies the same choice now.
	 * 角では最も近い辺を選び、1接触に対する継続方向を一意にする。最終出力段階は
	 * 解放まで方向をラッチするため、検出段階でも同じ選択を検証する。 */
	uint16_t left = x;
	uint16_t right = x >= cfg->surface_x ? 0 : cfg->surface_x - x;
	uint16_t top = y;
	uint16_t bottom = y >= cfg->surface_y ? 0 : cfg->surface_y - y;
	uint16_t minimum = left;
	enum edge_hold_edge edge = EDGE_HOLD_LEFT;

	if (right < minimum) {
		minimum = right;
		edge = EDGE_HOLD_RIGHT;
	}
	if (top < minimum) {
		minimum = top;
		edge = EDGE_HOLD_TOP;
	}
	if (bottom < minimum) {
		minimum = bottom;
		edge = EDGE_HOLD_BOTTOM;
	}
	return minimum <= cfg->edge_size ? edge : EDGE_HOLD_NONE;
}

static bool drag_centroid(const struct edge_hold_data *data, uint16_t *x, uint16_t *y)
{
	uint32_t sum_x = 0;
	uint32_t sum_y = 0;
	uint8_t count = 0;

	for (uint8_t index = 0; index < TRAVELERS_PAD_MAX_CONTACTS; index++) {
		if (!data->slot_valid[index]) {
			continue;
		}
		sum_x += data->slot_x[index];
		sum_y += data->slot_y[index];
		count++;
	}
	if (count == 0) {
		return false;
	}
	*x = sum_x / count;
	*y = sum_y / count;
	return true;
}

static void stop_drag_repeat(const struct device *dev)
{
	struct edge_hold_data *data = dev->data;

	/* State and lease are replaced only after the running callback has exited;
	 * asynchronous cancel alone would let an old callback observe a new lease. */
	(void)k_work_cancel_delayable_sync(&data->drag_repeat_work, &data->drag_repeat_sync);
	data->drag_work_generation++;
	data->drag_repeat_armed = false;
	data->drag_lease.valid = false;
}

static void drag_repeat_work_handler(struct k_work *work)
{
	struct edge_hold_data *data = CONTAINER_OF(work, struct edge_hold_data,
						     drag_repeat_work.work);
	const struct edge_hold_config *cfg = data->dev->config;
	int16_t dx = 0;
	int16_t dy = 0;
	int rc;
	uint32_t generation = data->drag_work_generation;
	struct travelers_pad_auxiliary_lease lease = data->drag_lease;

	/* The tap/drag module owns the button. This module only adds motion while
	 * the arbiter still holds the matching drag token.
	 * ボタン状態はtap/dragモジュールが所有する。本モジュールはarbiterが同じ
	 * drag tokenを保持する間だけ移動を追加する。 */
	if (data->drag_fingers == 0 || data->drag_edge == EDGE_HOLD_NONE ||
	    generation != data->drag_work_generation ||
	    !travelers_pad_arbiter_authorize_auxiliary_output(&lease,
		TRAVELERS_PAD_AUXILIARY_EDGE_HOLD, TRAVELERS_PAD_OWNER_DRAG,
		TRAVELERS_PAD_CAP_POINTER_DELTA, "edge-hold-drag")) {
		data->drag_repeat_armed = false;
		return;
	}

	switch (data->drag_edge) {
	case EDGE_HOLD_LEFT:
		dx = -cfg->drag_repeat_delta;
		break;
	case EDGE_HOLD_RIGHT:
		dx = cfg->drag_repeat_delta;
		break;
	case EDGE_HOLD_TOP:
		dy = -cfg->drag_repeat_delta;
		break;
	case EDGE_HOLD_BOTTOM:
		dy = cfg->drag_repeat_delta;
		break;
	default:
		return;
	}

	if (dx != 0) {
			rc = travelers_pad_output_auxiliary(
			data->dev, &lease, TRAVELERS_PAD_AUXILIARY_EDGE_HOLD, TRAVELERS_PAD_OWNER_DRAG,
			TRAVELERS_PAD_CAP_POINTER_DELTA, INPUT_EV_REL, INPUT_REL_X, dx, true,
			"edge-hold-drag-x");
	} else {
			rc = travelers_pad_output_auxiliary(
			data->dev, &lease, TRAVELERS_PAD_AUXILIARY_EDGE_HOLD, TRAVELERS_PAD_OWNER_DRAG,
			TRAVELERS_PAD_CAP_POINTER_DELTA, INPUT_EV_REL, INPUT_REL_Y, dy, true,
			"edge-hold-drag-y");
	}
	if (rc != 0) {
		if (rc == -EACCES) {
			data->drag_repeat_armed = false;
			return;
		}
		LOG_ERR("drag continuation report failed: %d", rc);
	}
	if (generation == data->drag_work_generation) {
		k_work_schedule(&data->drag_repeat_work, K_MSEC(cfg->drag_repeat_ms));
	}
}

static void start_drag_repeat(const struct device *dev)
{
	const struct edge_hold_config *cfg = dev->config;
	struct edge_hold_data *data = dev->data;

	if (data->drag_repeat_armed ||
	    !travelers_pad_arbiter_acquire_auxiliary_lease(
		TRAVELERS_PAD_AUXILIARY_EDGE_HOLD, TRAVELERS_PAD_OWNER_DRAG,
		TRAVELERS_PAD_CAP_POINTER_DELTA, &data->drag_lease)) {
		return;
	}
	(void)k_work_cancel_delayable_sync(&data->drag_repeat_work, &data->drag_repeat_sync);
	data->drag_work_generation++;
	data->drag_repeat_armed = true;
	k_work_schedule(&data->drag_repeat_work, K_MSEC(cfg->drag_start_delay_ms));
	LOG_INF("drag continuation started edge=%s delay=%ums period=%ums delta=%d",
		edge_name(data->drag_edge), cfg->drag_start_delay_ms, cfg->drag_repeat_ms,
		cfg->drag_repeat_delta);
}

static void stop_scroll_repeat(const struct device *dev)
{
	struct edge_hold_data *data = dev->data;

	(void)k_work_cancel_delayable_sync(&data->scroll_repeat_work, &data->scroll_repeat_sync);
	data->scroll_work_generation++;
	data->scroll_repeat_armed = false;
	data->scroll_horizontal = 0;
	data->scroll_vertical = 0;
	data->scroll_lease.valid = false;
}

static void scroll_repeat_work_handler(struct k_work *work)
{
	struct edge_hold_data *data = CONTAINER_OF(work, struct edge_hold_data,
						     scroll_repeat_work.work);
	const struct edge_hold_config *cfg = data->dev->config;
	int rc;
	uint32_t generation = data->scroll_work_generation;
	struct travelers_pad_auxiliary_lease lease = data->scroll_lease;

	if (data->scroll_fingers != 2 || data->scroll_edge == EDGE_HOLD_NONE ||
	    (data->scroll_horizontal == 0 && data->scroll_vertical == 0)) {
		data->scroll_repeat_armed = false;
		return;
	}
	if (generation != data->scroll_work_generation ||
	    !travelers_pad_arbiter_authorize_auxiliary_output(&lease,
		TRAVELERS_PAD_AUXILIARY_EDGE_HOLD, TRAVELERS_PAD_OWNER_SCROLL,
		TRAVELERS_PAD_CAP_SCROLL_DELTA, "edge-hold-scroll")) {
		data->scroll_repeat_armed = false;
		return;
	}

	if (data->scroll_horizontal != 0) {
			rc = travelers_pad_output_auxiliary(
			cfg->scroll_output_device, &lease, TRAVELERS_PAD_AUXILIARY_EDGE_HOLD,
			TRAVELERS_PAD_OWNER_SCROLL, TRAVELERS_PAD_CAP_SCROLL_DELTA, INPUT_EV_REL,
			INPUT_REL_HWHEEL, data->scroll_horizontal, true, "edge-hold-scroll-hwheel");
	} else {
			rc = travelers_pad_output_auxiliary(
			cfg->scroll_output_device, &lease, TRAVELERS_PAD_AUXILIARY_EDGE_HOLD,
			TRAVELERS_PAD_OWNER_SCROLL, TRAVELERS_PAD_CAP_SCROLL_DELTA, INPUT_EV_REL,
			INPUT_REL_WHEEL, data->scroll_vertical, true, "edge-hold-scroll-wheel");
	}
	if (rc != 0) {
		if (rc == -EACCES) {
			data->scroll_repeat_armed = false;
			return;
		}
		LOG_ERR("scroll continuation report failed: %d", rc);
	}
	if (generation == data->scroll_work_generation) {
		k_work_schedule(&data->scroll_repeat_work, K_MSEC(cfg->scroll_repeat_ms));
	}
}

static void start_scroll_repeat(const struct device *dev, int16_t horizontal, int16_t vertical)
{
	const struct edge_hold_config *cfg = dev->config;
	struct edge_hold_data *data = dev->data;

	if ((horizontal == 0 && vertical == 0) || data->scroll_edge == EDGE_HOLD_NONE ||
	    data->scroll_fingers != 2 || data->scroll_repeat_armed ||
	    !travelers_pad_arbiter_acquire_auxiliary_lease(
		TRAVELERS_PAD_AUXILIARY_EDGE_HOLD, TRAVELERS_PAD_OWNER_SCROLL,
		TRAVELERS_PAD_CAP_SCROLL_DELTA, &data->scroll_lease)) {
		return;
	}

	/* Latch the direction for the entire edge hold. Changing it requires leaving
	 * the boundary and deliberately entering again, matching drag continuation.
	 * 辺の保持中は方向をラッチする。方向変更には辺から離れ、意図的に再進入する。
	 * これはドラッグ継続と同じ操作規則である。 */
	data->scroll_horizontal = horizontal == 0 ? 0 :
				  (horizontal > 0 ? cfg->scroll_repeat_delta : -cfg->scroll_repeat_delta);
	data->scroll_vertical = vertical == 0 ? 0 :
				(vertical > 0 ? cfg->scroll_repeat_delta : -cfg->scroll_repeat_delta);
	(void)k_work_cancel_delayable_sync(&data->scroll_repeat_work, &data->scroll_repeat_sync);
	data->scroll_work_generation++;
	data->scroll_repeat_armed = true;
	k_work_schedule(&data->scroll_repeat_work, K_MSEC(cfg->scroll_start_delay_ms));
	LOG_INF("scroll continuation started edge=%s delay=%ums period=%ums delta=%d axis=%s",
		edge_name(data->scroll_edge), cfg->scroll_start_delay_ms, cfg->scroll_repeat_ms,
		cfg->scroll_repeat_delta, horizontal != 0 ? "horizontal" : "vertical");
}

static void detect_drag_edge(const struct device *dev)
{
	const struct edge_hold_config *cfg = dev->config;
	struct edge_hold_data *data = dev->data;
	uint16_t x;
	uint16_t y;
	enum edge_hold_edge edge = EDGE_HOLD_NONE;

	/* Edge position is only a candidate. The delayed work handler checks the
	 * arbiter token, which also avoids depending on callback order on the first
	 * double-tap-drag movement frame.
	 * 辺位置は候補にすぎない。遅延work handlerがarbiter tokenを確認するため、
	 * double-tap-drag最初の移動frameでcallback順に依存しない。 */
	if (data->drag_fingers != 0 && drag_centroid(data, &x, &y)) {
		edge = edge_at(cfg, x, y);
	}
	if (edge != data->drag_edge) {
		stop_drag_repeat(dev);
		if (edge != EDGE_HOLD_NONE) {
			LOG_INF("drag continuation candidate edge=%s centroid=(%u,%u) fingers=%u",
				edge_name(edge), x, y, data->drag_fingers);
		} else if (data->drag_edge != EDGE_HOLD_NONE) {
			LOG_INF("drag continuation disarmed edge=%s", edge_name(data->drag_edge));
		}
		data->drag_edge = edge;
	}
	/* The owner may have been committed after this detector first saw the edge.
	 * Retry arm-on-entry without requiring the finger to leave and re-enter. */
	if (data->drag_edge != EDGE_HOLD_NONE && !data->drag_repeat_armed) {
		start_drag_repeat(dev);
	}
}

static bool scroll_edge_reached(const struct edge_hold_config *cfg, const struct edge_hold_data *data,
				enum edge_hold_edge edge)
{
	if (edge == EDGE_HOLD_LEFT) {
		return data->scroll_x <= cfg->edge_size ||
		       (data->scroll_finger2_valid && data->scroll_finger2_x <= cfg->edge_size);
	}
	if (edge == EDGE_HOLD_RIGHT) {
		return data->scroll_x >= cfg->surface_x - cfg->edge_size ||
		       (data->scroll_finger2_valid &&
			data->scroll_finger2_x >= cfg->surface_x - cfg->edge_size);
	}
	if (edge == EDGE_HOLD_TOP) {
		return data->scroll_y <= cfg->edge_size ||
		       (data->scroll_finger2_valid && data->scroll_finger2_y <= cfg->edge_size);
	}
	return data->scroll_y >= cfg->surface_y - cfg->edge_size ||
	       (data->scroll_finger2_valid && data->scroll_finger2_y >= cfg->surface_y - cfg->edge_size);
}

static uint8_t scroll_edge_finger(const struct edge_hold_config *cfg,
				  const struct edge_hold_data *data, enum edge_hold_edge edge)
{
	if ((edge == EDGE_HOLD_LEFT && data->scroll_x <= cfg->edge_size) ||
	    (edge == EDGE_HOLD_RIGHT && data->scroll_x >= cfg->surface_x - cfg->edge_size) ||
	    (edge == EDGE_HOLD_TOP && data->scroll_y <= cfg->edge_size) ||
	    (edge == EDGE_HOLD_BOTTOM && data->scroll_y >= cfg->surface_y - cfg->edge_size)) {
		return 1;
	}
	return 2;
}

static void detect_scroll_edge(const struct device *dev)
{
	const struct edge_hold_config *cfg = dev->config;
	struct edge_hold_data *data = dev->data;
	enum edge_hold_edge edge = EDGE_HOLD_NONE;

	/* A scroll edge is meaningful only in the axis which the pointer function
	 * has already committed. Either contact may reach the forward boundary:
	 * this avoids placement-dependent failures while ignoring perpendicular
	 * edges that are natural during a two-finger gesture.
	 * スクロールの辺は、ポインタ機能がすでに決定した軸だけで意味を持つ。2本の
	 * どちらが進行方向の境界に到達してもよい。これにより指の置き方依存をなくし、
	 * 2本指操作で自然に触れ得る垂直方向の辺は無視する。 */
	if (data->scroll_fingers == 2) {
		if (data->scroll_direction_horizontal > 0 &&
		    scroll_edge_reached(cfg, data, EDGE_HOLD_RIGHT)) {
			edge = EDGE_HOLD_RIGHT;
		} else if (data->scroll_direction_horizontal < 0 &&
			   scroll_edge_reached(cfg, data, EDGE_HOLD_LEFT)) {
			edge = EDGE_HOLD_LEFT;
		} else if (data->scroll_direction_vertical > 0 &&
			   scroll_edge_reached(cfg, data, EDGE_HOLD_BOTTOM)) {
			edge = EDGE_HOLD_BOTTOM;
		} else if (data->scroll_direction_vertical < 0 &&
			   scroll_edge_reached(cfg, data, EDGE_HOLD_TOP)) {
			edge = EDGE_HOLD_TOP;
		}
	}

	if (edge != data->scroll_edge) {
		stop_scroll_repeat(dev);
		if (edge != EDGE_HOLD_NONE) {
			LOG_INF("scroll continuation eligible edge=%s finger=%u f1=(%u,%u) f2=(%u,%u)",
				edge_name(edge), scroll_edge_finger(cfg, data, edge), data->scroll_x,
				data->scroll_y, data->scroll_finger2_x, data->scroll_finger2_y);
		} else if (data->scroll_edge != EDGE_HOLD_NONE) {
			LOG_INF("scroll continuation disarmed edge=%s", edge_name(data->scroll_edge));
		}
		data->scroll_edge = edge;
	}
}

static void edge_hold_scroll_motion_input(const struct device *dev, struct input_event *evt)
{
	struct edge_hold_data *data = dev->data;
	int16_t horizontal = 0;
	int16_t vertical = 0;

	if (evt->type == INPUT_EV_REL) {
		if (evt->code == INPUT_REL_HWHEEL) {
			horizontal = evt->value;
		} else if (evt->code == INPUT_REL_WHEEL) {
			vertical = evt->value;
		}
	}
	/* Once continuation has started, preserve its direction until edge exit or
	 * contact release even when small physical reverse motion is reported.
	 * 継続開始後は、微小な逆方向の物理移動があっても辺から出るか接触を離すまで
	 * 方向を保持する。 */
	if ((horizontal != 0 || vertical != 0) && data->scroll_horizontal == 0 &&
	    data->scroll_vertical == 0 && horizontal != 0) {
		data->scroll_direction_horizontal = horizontal;
		data->scroll_direction_vertical = 0;
	} else if ((horizontal != 0 || vertical != 0) && data->scroll_horizontal == 0 &&
		   data->scroll_vertical == 0 && vertical != 0) {
		data->scroll_direction_horizontal = 0;
		data->scroll_direction_vertical = vertical;
	}
	/* Router coordinates arrive before this pointer output on the same physical
	 * frame. Re-evaluate now, so an edge reached on the first locked wheel
	 * movement can arm without waiting for another touch frame.
	 * router座標は同じ物理フレームでこのポインタ出力より先に到着する。そのため、
	 * 初回の軸ロック済みホイール移動で到達した辺も次フレームを待たずに有効化する。 */
	if ((horizontal != 0 || vertical != 0) && data->scroll_horizontal == 0 &&
	    data->scroll_vertical == 0) {
		detect_scroll_edge(dev);
	}
	if ((horizontal != 0 || vertical != 0) && data->scroll_edge != EDGE_HOLD_NONE &&
	    data->scroll_horizontal == 0 && data->scroll_vertical == 0) {
		start_scroll_repeat(dev, horizontal, vertical);
	}
}

static void edge_hold_drag_input(const struct device *dev, struct input_event *evt)
{
	struct edge_hold_data *data = dev->data;

	if (evt->type == INPUT_EV_ABS) {
		if (evt->code == INPUT_ABS_MT_SLOT) {
			data->slot = evt->value;
		} else if (evt->code == TRAVELERS_PAD_INPUT_ABS_MT_TRACKING_ID &&
			   data->slot < TRAVELERS_PAD_MAX_CONTACTS) {
			data->slot_valid[data->slot] = evt->value >= 0;
		} else if (evt->code == TRAVELERS_PAD_INPUT_ABS_MT_POSITION_X &&
			   data->slot < TRAVELERS_PAD_MAX_CONTACTS) {
			data->slot_x[data->slot] = evt->value;
		} else if (evt->code == TRAVELERS_PAD_INPUT_ABS_MT_POSITION_Y &&
			   data->slot < TRAVELERS_PAD_MAX_CONTACTS) {
			data->slot_y[data->slot] = evt->value;
		} else if (evt->code == INPUT_ABS_Z) {
			data->drag_fingers = evt->value;
		}
	}
	if (evt->sync) {
		data->arbiter_frame_seq =
			travelers_pad_arbiter_next_frame(TRAVELERS_PAD_RECOGNIZER_EDGE_HOLD);
		travelers_pad_arbiter_observed(TRAVELERS_PAD_RECOGNIZER_EDGE_HOLD,
					       data->arbiter_frame_seq);
	}
}

static void edge_hold_deliver_resolved_frame(
	void *context, const struct travelers_pad_session_snapshot *snapshot,
	const struct travelers_pad_owner_token *token)
{
	struct edge_hold_data *data = context;

	ARG_UNUSED(token);
	if (snapshot->cancelled) {
		stop_drag_repeat(data->dev);
		stop_scroll_repeat(data->dev);
		data->drag_fingers = 0;
		data->drag_edge = EDGE_HOLD_NONE;
		data->scroll_edge = EDGE_HOLD_NONE;
		return;
	}
	/* This observer's raw slot data is complete by construction here. The
	 * primary decision has also completed, so arming an edge timer cannot depend
	 * on Zephyr listener order. / このobserverの生slot dataはここで必ず完成済みであり、
	 * primary裁定も完了している。従ってedge timerのarmはZephyr listener順に依存しない。 */
	detect_drag_edge(data->dev);
}

static void edge_hold_scroll_input(const struct device *dev, struct input_event *evt)
{
	struct edge_hold_data *data = dev->data;

	if (evt->type == INPUT_EV_ABS) {
		if (evt->code == INPUT_ABS_X) {
			data->scroll_x = evt->value;
		} else if (evt->code == INPUT_ABS_Y) {
			data->scroll_y = evt->value;
		} else if (evt->code == INPUT_ABS_RX) {
			data->scroll_finger2_x = evt->value;
		} else if (evt->code == INPUT_ABS_RY) {
			data->scroll_finger2_y = evt->value;
		} else if (evt->code == INPUT_ABS_RUDDER) {
			data->scroll_finger2_valid = evt->value;
		} else if (evt->code == INPUT_ABS_Z) {
			data->scroll_fingers = evt->value;
			if (data->scroll_fingers != 2) {
				data->scroll_direction_horizontal = 0;
				data->scroll_direction_vertical = 0;
			}
		}
	}
	if (evt->sync) {
		detect_scroll_edge(dev);
	}
}

static int edge_hold_init(const struct device *dev)
{
	const struct edge_hold_config *cfg = dev->config;

	k_work_init_delayable(&((struct edge_hold_data *)dev->data)->drag_repeat_work,
			     drag_repeat_work_handler);
	k_work_init_delayable(&((struct edge_hold_data *)dev->data)->scroll_repeat_work,
			     scroll_repeat_work_handler);
	(void)travelers_pad_arbiter_register(TRAVELERS_PAD_RECOGNIZER_EDGE_HOLD);
	(void)travelers_pad_arbiter_bind_frame_handler(
		TRAVELERS_PAD_RECOGNIZER_EDGE_HOLD, edge_hold_deliver_resolved_frame, dev->data);
	LOG_INF("edge hold active: surface=%ux%u edge=%u; drag delay=%ums period=%ums delta=%d; "
		"scroll delay=%ums period=%ums delta=%d", cfg->surface_x, cfg->surface_y, cfg->edge_size,
		cfg->drag_start_delay_ms, cfg->drag_repeat_ms, cfg->drag_repeat_delta,
		cfg->scroll_start_delay_ms, cfg->scroll_repeat_ms, cfg->scroll_repeat_delta);
	return 0;
}

#define EDGE_HOLD_INST(n)                                                                      \
	static struct edge_hold_data edge_hold_data_##n = {.dev = DEVICE_DT_INST_GET(n)};           \
	static const struct edge_hold_config edge_hold_config_##n = {                               \
		.surface_x = DT_INST_PROP_OR(n, surface_x, 2000),                                      \
		.surface_y = DT_INST_PROP_OR(n, surface_y, 1213),                                      \
		.edge_size = DT_INST_PROP_OR(n, edge_size, 80),                                         \
		.drag_start_delay_ms = DT_INST_PROP_OR(n, drag_start_delay_ms, 120),                    \
		.drag_repeat_ms = DT_INST_PROP_OR(n, drag_repeat_ms, 24),                               \
		.drag_repeat_delta = DT_INST_PROP_OR(n, drag_repeat_delta, 8),                          \
		.scroll_output_device = DEVICE_DT_GET(DT_INST_PHANDLE(n, scroll_output_device)),       \
		.scroll_start_delay_ms = DT_INST_PROP_OR(n, scroll_start_delay_ms, 120),                \
		.scroll_repeat_ms = DT_INST_PROP_OR(n, scroll_repeat_ms, 24),                           \
		.scroll_repeat_delta = DT_INST_PROP_OR(n, scroll_repeat_delta, 4),                      \
	};                                                                                           \
	static void edge_hold_drag_input_handler_##n(struct input_event *evt)                       \
	{                                                                                            \
		edge_hold_drag_input(DEVICE_DT_INST_GET(n), evt);                                      \
	}                                                                                            \
	static void edge_hold_scroll_input_handler_##n(struct input_event *evt)                     \
	{                                                                                            \
		edge_hold_scroll_input(DEVICE_DT_INST_GET(n), evt);                                    \
	}                                                                                            \
	static void edge_hold_scroll_motion_input_handler_##n(struct input_event *evt)              \
	{                                                                                            \
		edge_hold_scroll_motion_input(DEVICE_DT_INST_GET(n), evt);                              \
	}                                                                                            \
	INPUT_CALLBACK_DEFINE(DEVICE_DT_GET(DT_INST_PHANDLE(n, drag_input_device)),                 \
			      edge_hold_drag_input_handler_##n);                                             \
	INPUT_CALLBACK_DEFINE(DEVICE_DT_GET(DT_INST_PHANDLE(n, scroll_input_device)),               \
			      edge_hold_scroll_input_handler_##n);                                           \
	INPUT_CALLBACK_DEFINE(DEVICE_DT_GET(DT_INST_PHANDLE(n, scroll_motion_input_device)),        \
			      edge_hold_scroll_motion_input_handler_##n);                                    \
	DEVICE_DT_INST_DEFINE(n, edge_hold_init, NULL, &edge_hold_data_##n, &edge_hold_config_##n,  \
			      POST_KERNEL, CONFIG_TRAVELERS_PAD_IQS9150_FUNCTION_INIT_PRIORITY, NULL);

DT_INST_FOREACH_STATUS_OKAY(EDGE_HOLD_INST)
