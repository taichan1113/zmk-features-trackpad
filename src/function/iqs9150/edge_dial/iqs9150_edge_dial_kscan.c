/*
 * Travelers Pad IQS9150 continuous edge-trace virtual KSCAN.
 *
 * A single contact that starts on an enabled edge acquires that edge for the
 * rest of the contact. Motion parallel to the acquired edge emits repeated
 * KSCAN press/release pairs. It deliberately does not decide whether a pair
 * means scrolling, volume, or another action: that remains a ZMK keymap
 * decision.
 *
 * 有効な辺から始まる1本指接触を、その接触が続く間だけ辺操作として取得する。
 * 取得した辺と平行な移動は、繰り返しKSCAN押下/解放へ変換する。スクロール、音量
 * などの意味付けは、このモジュールではなくZMKキーマップが決める。
 */

#define DT_DRV_COMPAT zmk_travelers_pad_iqs9150_edge_dial_kscan

#include <errno.h>
#include <stdbool.h>
#include <stdint.h>

#include <zephyr/device.h>
#include <zephyr/dt-bindings/input/input-event-codes.h>
#include <zephyr/drivers/kscan.h>
#include <zephyr/input/input.h>
#include <zephyr/logging/log.h>

#include <travelers_pad/contact_arbiter.h>
#include <travelers_pad/contact_output_broker.h>

LOG_MODULE_REGISTER(travelers_pad_edge_dial, LOG_LEVEL_INF);

enum edge_dial_edge {
	EDGE_DIAL_LEFT,
	EDGE_DIAL_RIGHT,
	EDGE_DIAL_TOP,
	EDGE_DIAL_BOTTOM,
	EDGE_DIAL_NONE,
};

enum edge_dial_mode {
	EDGE_DIAL_LINEAR,
	EDGE_DIAL_CIRCLE,
};

struct edge_dial_config {
	const struct device *input_device;
	uint16_t surface_x;
	uint16_t surface_y;
	uint16_t edge_size;
	uint16_t step_move;
	uint16_t circle_entry_depth;
	uint16_t circle_entry_span;
	uint16_t circle_step_move;
	uint16_t circle_corner_inset_x;
	uint16_t circle_corner_inset_y;
	bool enabled[4];
};

struct edge_dial_data {
	kscan_callback_t callback;
	uint16_t x;
	uint16_t y;
	uint16_t last_x;
	uint16_t last_y;
	uint8_t fingers;
	enum edge_dial_edge edge;
	enum edge_dial_mode mode;
	int8_t edge_direction;
	int8_t circle_direction;
	bool active;
	int32_t accumulated_move;
	uint32_t arbiter_frame_seq;
	uint32_t ownership_frame_seq;
	bool claim_submitted;
	uint32_t start_session_id;
	bool start_seen;
	enum edge_dial_edge start_edge;
};

static int edge_dial_configure(const struct device *dev, kscan_callback_t callback)
{
	if (callback == NULL) {
		return -EINVAL;
	}
	((struct edge_dial_data *)dev->data)->callback = callback;
	return 0;
}

static const struct kscan_driver_api edge_dial_api = {
	.config = edge_dial_configure,
};

static enum edge_dial_edge edge_dial_edge_for(const struct edge_dial_config *cfg, uint16_t x,
						      uint16_t y)
{
	/* The driver uses 0xffff for an unavailable coordinate in a partial
	 * contact frame. It is never a physical edge coordinate. */
	if (x > cfg->surface_x || y > cfg->surface_y) {
		return EDGE_DIAL_NONE;
	}
	if (x <= cfg->edge_size) {
		return EDGE_DIAL_LEFT;
	}
	if (x + cfg->edge_size >= cfg->surface_x) {
		return EDGE_DIAL_RIGHT;
	}
	if (y <= cfg->edge_size) {
		return EDGE_DIAL_TOP;
	}
	if (y + cfg->edge_size >= cfg->surface_y) {
		return EDGE_DIAL_BOTTOM;
	}
	return EDGE_DIAL_NONE;
}

static int32_t edge_dial_parallel_delta(const struct edge_dial_data *data)
{
	if (data->edge == EDGE_DIAL_LEFT || data->edge == EDGE_DIAL_RIGHT) {
		return (int32_t)data->y - data->last_y;
	}
	return (int32_t)data->x - data->last_x;
}

static bool edge_dial_is_on_acquired_edge(const struct edge_dial_config *cfg,
					 const struct edge_dial_data *data)
{
	switch (data->edge) {
	case EDGE_DIAL_LEFT:
		return data->x <= cfg->edge_size;
	case EDGE_DIAL_RIGHT:
		return data->x + cfg->edge_size >= cfg->surface_x;
	case EDGE_DIAL_TOP:
		return data->y <= cfg->edge_size;
	case EDGE_DIAL_BOTTOM:
		return data->y + cfg->edge_size >= cfg->surface_y;
	default:
		return false;
	}
}

static int8_t edge_dial_circle_action_direction(const struct edge_dial_data *data)
{
	return data->mode == EDGE_DIAL_CIRCLE ? data->circle_direction : data->edge_direction;
}

static int32_t edge_dial_circle_center_x(const struct edge_dial_config *cfg,
					 const struct edge_dial_data *data)
{
	switch (data->edge) {
	case EDGE_DIAL_LEFT:
		return cfg->circle_corner_inset_x;
	case EDGE_DIAL_RIGHT:
		return cfg->surface_x - cfg->circle_corner_inset_x;
	case EDGE_DIAL_TOP:
	case EDGE_DIAL_BOTTOM:
		return edge_dial_circle_action_direction(data) > 0 ?
			       cfg->surface_x - cfg->circle_corner_inset_x :
					    cfg->circle_corner_inset_x;
	default:
		return cfg->surface_x / 2;
	}
}

static int32_t edge_dial_circle_center_y(const struct edge_dial_config *cfg,
					 const struct edge_dial_data *data)
{
	switch (data->edge) {
	case EDGE_DIAL_LEFT:
	case EDGE_DIAL_RIGHT:
		return edge_dial_circle_action_direction(data) > 0 ?
			       cfg->surface_y - cfg->circle_corner_inset_y :
					    cfg->circle_corner_inset_y;
	case EDGE_DIAL_TOP:
		return cfg->circle_corner_inset_y;
	case EDGE_DIAL_BOTTOM:
		return cfg->surface_y - cfg->circle_corner_inset_y;
	default:
		return cfg->surface_y / 2;
	}
}

static int32_t edge_dial_absolute(int32_t value) { return value < 0 ? -value : value; }

static void edge_dial_emit_step(const struct device *dev, bool positive)
{
	struct edge_dial_data *data = dev->data;
	uint8_t column = ((uint8_t)data->edge * 2) + (positive ? 0 : 1);
	/* Circular continuation is an alternate trajectory for the same edge
	 * action, not a separate action. Keep the virtual KSCAN position unchanged.
	 * 円形継続は別機能ではなく、同じ辺操作の代替軌道であるため、仮想KSCAN位置を
	 * 直線時と同一に保つ。 */
	uint8_t row = 0;

	if (travelers_pad_output_behavior_primary(
		dev, data->callback, data->ownership_frame_seq,
		TRAVELERS_PAD_RECOGNIZER_EDGE_DIAL, TRAVELERS_PAD_OWNER_EDGE_DIAL, row, column,
		"edge-dial-step") != 0) {
		return;
	}
	LOG_INF("edge dial mode=%s edge=%u row=%u column=%u at=(%u,%u)",
		data->mode == EDGE_DIAL_CIRCLE ? "circle" : "linear", data->edge, row, column,
		data->x, data->y);
}

static bool edge_dial_has_entered_circle(const struct edge_dial_config *cfg,
					  const struct edge_dial_data *data)
{
	int32_t parallel_distance;

	if (data->edge_direction == 0) {
		return false;
	}

	switch (data->edge) {
	case EDGE_DIAL_LEFT:
		parallel_distance = (int32_t)data->y - edge_dial_circle_center_y(cfg, data);
		return data->x >= cfg->edge_size + cfg->circle_entry_depth &&
		       edge_dial_absolute(parallel_distance) <= cfg->circle_entry_span;
	case EDGE_DIAL_RIGHT:
		parallel_distance = (int32_t)data->y - edge_dial_circle_center_y(cfg, data);
		return data->x + cfg->edge_size + cfg->circle_entry_depth <= cfg->surface_x &&
		       edge_dial_absolute(parallel_distance) <= cfg->circle_entry_span;
	case EDGE_DIAL_TOP:
		parallel_distance = (int32_t)data->x - edge_dial_circle_center_x(cfg, data);
		return data->y >= cfg->edge_size + cfg->circle_entry_depth &&
		       edge_dial_absolute(parallel_distance) <= cfg->circle_entry_span;
	case EDGE_DIAL_BOTTOM:
		parallel_distance = (int32_t)data->x - edge_dial_circle_center_x(cfg, data);
		return data->y + cfg->edge_size + cfg->circle_entry_depth <= cfg->surface_y &&
		       edge_dial_absolute(parallel_distance) <= cfg->circle_entry_span;
	default:
		return false;
	}
}

static int32_t edge_dial_circular_delta(const struct edge_dial_config *cfg,
					 const struct edge_dial_data *data)
{
	int32_t center_x = edge_dial_circle_center_x(cfg, data);
	int32_t center_y = edge_dial_circle_center_y(cfg, data);
	int32_t previous_x = (int32_t)data->last_x - center_x;
	int32_t previous_y = (int32_t)data->last_y - center_y;
	int32_t current_x = (int32_t)data->x - center_x;
	int32_t current_y = (int32_t)data->y - center_y;
	int32_t radius = previous_x < 0 ? -previous_x : previous_x;
	int64_t cross;

	radius += previous_y < 0 ? -previous_y : previous_y;
	if (radius == 0) {
		return 0;
	}
	/* cross/radius approximates the travelled tangential distance and avoids
	 * floating point or atan2 on the nRF52840. 正接方向の移動量を整数だけで近似する。 */
	cross = (int64_t)previous_x * current_y - (int64_t)previous_y * current_x;
	return cross / radius;
}

static int32_t edge_dial_logical_circle_delta(struct edge_dial_data *data, int32_t raw_delta)
{
	if (raw_delta == 0) {
		return 0;
	}
	/* A circle is a continuation region, not a bidirectional rotary control.
	 * Its angular movement supplies only the amount of repeated input; its sign
	 * is deliberately ignored. The edge direction selected before entering the
	 * circle owns the key for this complete touch sequence. This rejects small
	 * coordinate reversals caused by a non-perfect circle or sensor noise.
	 *
	 * 円形領域は双方向ロータリー操作ではなく、辺操作の継続領域です。角度移動は反復
	 * 入力の量だけを与え、符号は意図的に無視します。円へ入る前に確定した辺方向が、
	 * このタッチシーケンス全体のキーを決定します。完全な円でない操作やセンサー
	 * ノイズによる微小な逆行で、キーが反転することを防ぎます。 */
	return data->circle_direction * edge_dial_absolute(raw_delta);
}

static void edge_dial_process_frame(const struct device *dev)
{
	const struct edge_dial_config *cfg = dev->config;
	struct edge_dial_data *data = dev->data;
	struct travelers_pad_session_snapshot snapshot;
	int32_t delta;
	uint16_t step_move;

	/* Decide eligibility once, from the first frame of the physical session.
	 * Interior starts (including multi-contact/invalid-coordinate starts) must
	 * never turn into an edge dial later, even after a temporary finger change.
	 * 接触sessionの最初のframeだけで辺操作の資格を判定する。内側開始・多指開始・
	 * 無効座標開始は、途中の移動や指本数変化で辺操作へ切り替えない。 */
	if (!travelers_pad_arbiter_get_frame_snapshot(data->arbiter_frame_seq, &snapshot)) {
		return;
	}
	if (snapshot.cancelled || snapshot.contact_count == 0) {
		data->start_seen = false;
		data->start_edge = EDGE_DIAL_NONE;
		data->active = false;
		data->claim_submitted = false;
		data->edge = EDGE_DIAL_NONE;
		data->mode = EDGE_DIAL_LINEAR;
		data->edge_direction = 0;
		data->circle_direction = 0;
		data->accumulated_move = 0;
		return;
	}
	if (!data->start_seen || data->start_session_id != snapshot.session_id) {
		data->start_seen = true;
		data->start_session_id = snapshot.session_id;
		data->active = false;
		data->claim_submitted = false;
		data->start_edge = data->fingers == 1 ?
			edge_dial_edge_for(cfg, data->x, data->y) : EDGE_DIAL_NONE;
		if (data->start_edge < EDGE_DIAL_NONE && !cfg->enabled[data->start_edge]) {
			data->start_edge = EDGE_DIAL_NONE;
		}
	}

	if (data->fingers != 1) {
		data->active = false;
		data->edge = EDGE_DIAL_NONE;
		data->mode = EDGE_DIAL_LINEAR;
		data->edge_direction = 0;
		data->circle_direction = 0;
		/* Preserve the one arbiter claim while a temporary extra contact is
		 * present. The dial may re-acquire after the contact count returns
		 * to one, but it is still the same logical session. / 一時的に接触が増えても
		 * arbiter claimは維持する。接触数が1へ戻るとdialは再取得し得るが、
		 * 論理sessionは同一だからである。 */
		if (data->fingers == 0) {
			data->claim_submitted = false;
		}
		return;
	}

	if (!data->active) {
		enum travelers_pad_owner owner;

		if (data->start_edge == EDGE_DIAL_NONE) {
			return;
		}
		data->edge = edge_dial_edge_for(cfg, data->x, data->y);
		/* Re-acquisition is permitted only on the original starting edge.
		 * 再取得時も接触開始時の辺に限定する。 */
		if (data->edge == data->start_edge) {
			owner = travelers_pad_arbiter_owner_for_frame(data->arbiter_frame_seq);
			/* Edge dial has the highest primary priority, but it must not revive
			 * after a previously committed incompatible owner. EDGE_DIAL itself is
			 * allowed when a temporary extra contact disappeared and this same
			 * logical session is being re-acquired.
			 * edge dialはprimary中で最高priorityだが、先にcommit済みの非互換owner
			 * の後に復帰してはならない。一時的な追加接触が消えて同じlogical sessionを
			 * 再取得する時だけ、EDGE_DIAL自身を許可する。 */
			if (owner != TRAVELERS_PAD_OWNER_NONE && owner != TRAVELERS_PAD_OWNER_EDGE_DIAL) {
				return;
			}
			if (!data->claim_submitted) {
				/* Edge ownership is requested on acquisition, before the first
				 * detent. The arbiter resolves this first claim; subsequent detents
				 * and re-acquisition in the same contact session share its token.
				 * edge所有権は最初のdetentではなく取得時に要求する。arbiterがこの最初の
				 * claimを解決し、同一contact sessionの以後のdetentと再取得はそのtokenを
				 * 共有する。 */
				if (!travelers_pad_arbiter_submit(&(struct travelers_pad_arbiter_proposal){
					.frame_seq = data->arbiter_frame_seq,
					.recognizer = TRAVELERS_PAD_RECOGNIZER_EDGE_DIAL,
					.intent = TRAVELERS_PAD_PROPOSAL_CLAIM,
					.requested_owner = TRAVELERS_PAD_OWNER_EDGE_DIAL,
					.capabilities = TRAVELERS_PAD_CAP_BEHAVIOR_ACTION,
					.contact_count = data->fingers,
					.evidence = data->edge,
				})) {
					LOG_WRN("edge dial claim proposal rejected edge=%u", data->edge);
					return;
				}
				data->ownership_frame_seq = data->arbiter_frame_seq;
				data->claim_submitted = true;
			}
			data->active = true;
			data->last_x = data->x;
			data->last_y = data->y;
			data->accumulated_move = 0;
			data->mode = EDGE_DIAL_LINEAR;
			data->edge_direction = 0;
			data->circle_direction = 0;
			/* Claim on acquisition, not on the first detent. Otherwise the
			 * initial edge movement leaks into ordinary pointer motion until the
			 * step threshold is crossed.
			 * 最初のステップ発火時ではなく、辺を取得した時点で所有権を得る。
			 * そうしないと閾値へ達するまでの最初の辺移動が通常ポインタへ漏れる。 */
			LOG_INF("edge dial acquired edge=%u at=(%u,%u)", data->edge, data->x, data->y);
		}
		return;
	}

	if (data->mode == EDGE_DIAL_LINEAR) {
		delta = edge_dial_parallel_delta(data);
		if (edge_dial_is_on_acquired_edge(cfg, data)) {
			/* While the finger remains on the acquired edge, normal direction
			 * changes are valid. Record this as the direction to continue if the
			 * finger then leaves the edge for the local circle.
			 * 指が取得した辺の帯域内にある間は、通常の方向変更を許可する。その後
			 * 局所円へ移る時に継続する方向として記録する。 */
			if (delta != 0) {
				data->edge_direction = delta > 0 ? 1 : -1;
			}
		} else if (data->edge_direction != 0) {
			/* Once the finger has left the edge, a bend toward the local circle
			 * must retain its last edge direction rather than emit an opposite key.
			 * 辺を離れた後は、局所円への曲線を逆方向キーへ変換せず、最後の辺方向を
			 * 継続する。 */
			delta = data->edge_direction * edge_dial_absolute(delta);
		}
		if (edge_dial_has_entered_circle(cfg, data)) {
			data->mode = EDGE_DIAL_CIRCLE;
			data->last_x = data->x;
			data->last_y = data->y;
			data->accumulated_move = 0;
			data->circle_direction = data->edge_direction;
			LOG_INF("edge dial circle acquired edge=%u direction=%d at=(%u,%u) center=(%u,%u)",
				data->edge, data->circle_direction, data->x, data->y,
				edge_dial_circle_center_x(cfg, data), edge_dial_circle_center_y(cfg, data));
			return;
		}
	} else {
		delta = edge_dial_logical_circle_delta(data, edge_dial_circular_delta(cfg, data));
	}
	data->last_x = data->x;
	data->last_y = data->y;
	data->accumulated_move += delta;
	step_move = data->mode == EDGE_DIAL_CIRCLE ? cfg->circle_step_move : cfg->step_move;

	while (data->accumulated_move >= step_move || data->accumulated_move <= -((int32_t)step_move)) {
		bool positive = data->accumulated_move > 0;

		edge_dial_emit_step(dev, positive);
		data->accumulated_move += positive ? -step_move : step_move;
	}
}

static void edge_dial_handle_input(const struct device *dev, struct input_event *evt)
{
	struct edge_dial_data *data = dev->data;

	if (evt->type == INPUT_EV_ABS) {
		if (evt->code == INPUT_ABS_X) {
			data->x = evt->value;
		} else if (evt->code == INPUT_ABS_Y) {
			data->y = evt->value;
		} else if (evt->code == INPUT_ABS_Z) {
			data->fingers = evt->value;
		}
	}
	if (evt->sync) {
		data->arbiter_frame_seq =
			travelers_pad_arbiter_next_frame(TRAVELERS_PAD_RECOGNIZER_EDGE_DIAL);
		edge_dial_process_frame(dev);
		travelers_pad_arbiter_observed(TRAVELERS_PAD_RECOGNIZER_EDGE_DIAL,
					       data->arbiter_frame_seq);
	}
}

static int edge_dial_init(const struct device *dev)
{
	ARG_UNUSED(dev);
	(void)travelers_pad_arbiter_register(TRAVELERS_PAD_RECOGNIZER_EDGE_DIAL);
	return 0;
}

#define EDGE_DIAL_INST(n)                                                                       \
	static struct edge_dial_data edge_dial_data_##n;                                          \
	static const struct edge_dial_config edge_dial_config_##n = {                             \
		.input_device = DEVICE_DT_GET(DT_INST_PHANDLE(n, input_device)),                    \
		.surface_x = DT_INST_PROP_OR(n, surface_x, 2000),                                    \
		.surface_y = DT_INST_PROP_OR(n, surface_y, 1213),                                    \
		.edge_size = DT_INST_PROP_OR(n, edge_size, 140),                                     \
		.step_move = DT_INST_PROP_OR(n, step_move, 80),                                      \
		.circle_entry_depth = DT_INST_PROP_OR(n, circle_entry_depth, 80),                    \
		.circle_entry_span = DT_INST_PROP_OR(n, circle_entry_span, 250),                     \
		.circle_step_move = DT_INST_PROP_OR(n, circle_step_move, 80),                        \
		.circle_corner_inset_x = DT_INST_PROP_OR(n, circle_corner_inset_x, 360),            \
		.circle_corner_inset_y = DT_INST_PROP_OR(n, circle_corner_inset_y, 313),            \
		.enabled = {DT_INST_PROP_OR(n, edge_left, false),                                    \
			    DT_INST_PROP_OR(n, edge_right, false),                                       \
			    DT_INST_PROP_OR(n, edge_top, false),                                         \
			    DT_INST_PROP_OR(n, edge_bottom, false)},                                     \
	};                                                                                         \
	static void edge_dial_input_handler_##n(struct input_event *evt)                           \
	{                                                                                          \
		edge_dial_handle_input(DEVICE_DT_INST_GET(n), evt);                                  \
	}                                                                                          \
	INPUT_CALLBACK_DEFINE(DEVICE_DT_GET(DT_INST_PHANDLE(n, input_device)),                     \
			      edge_dial_input_handler_##n);                                            \
	DEVICE_DT_INST_DEFINE(n, edge_dial_init, NULL, &edge_dial_data_##n, &edge_dial_config_##n, \
			      POST_KERNEL, CONFIG_TRAVELERS_PAD_IQS9150_FUNCTION_INIT_PRIORITY,       \
			      &edge_dial_api);

DT_INST_FOREACH_STATUS_OKAY(EDGE_DIAL_INST)
