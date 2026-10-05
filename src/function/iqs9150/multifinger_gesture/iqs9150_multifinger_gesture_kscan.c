/*
 * Travelers Pad IQS9150 three-to-five-finger swipe function.
 *
 * A completed 3/4-finger swipe becomes one virtual KSCAN press. Five-finger
 * contact uses independent coordinates for Grasp/Release. A confirmed
 * candidate emits one virtual KSCAN event and is latched until release.
 *
 * 3/4本指スワイプ完了を仮想KSCANの一回の押下へ変換する。5本指では独立座標を
 * Grasp/Release用に使う。確定時に仮想KSCANを一度だけ発火し、全接触が離れる
 * までラッチする。実際の操作ポリシーはセンサードライバから分離する。
 */

#define DT_DRV_COMPAT zmk_travelers_pad_iqs9150_multifinger_gesture_kscan

#include <zephyr/device.h>
#include <zephyr/dt-bindings/input/input-event-codes.h>
#include <zephyr/drivers/kscan.h>
#include <zephyr/input/input.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

#include <travelers_pad/contact_arbiter.h>
#include <travelers_pad/contact_output_broker.h>
#include <travelers_pad/multitouch.h>

LOG_MODULE_REGISTER(travelers_pad_multifinger_gesture, LOG_LEVEL_INF);

enum gesture_column { GESTURE_LEFT, GESTURE_RIGHT, GESTURE_UP, GESTURE_DOWN };
enum five_finger_candidate { FIVE_FINGER_NONE, FIVE_FINGER_GRASP, FIVE_FINGER_RELEASE };

struct gesture_config {
	uint16_t swipe_min_move;
	uint16_t swipe_max_ms;
	uint16_t five_finger_radial_move;
	uint8_t five_finger_stable_frames;
};

struct gesture_data {
	const struct device *dev;
	kscan_callback_t callback;
	uint16_t slot_x[TRAVELERS_PAD_MAX_CONTACTS];
	uint16_t slot_y[TRAVELERS_PAD_MAX_CONTACTS];
	int32_t slot_tracking_id[TRAVELERS_PAD_MAX_CONTACTS];
	bool slot_valid[TRAVELERS_PAD_MAX_CONTACTS];
	uint8_t slot;
	uint16_t x;
	uint16_t y;
	uint16_t start_x;
	uint16_t start_y;
	uint8_t fingers;
	uint8_t previous_fingers;
	uint8_t max_fingers;
	int64_t started_ms;
	bool moved;
	bool cancelled_by_drag_owner;
	bool cancelled_by_owner;
	uint8_t five_finger_stable_frames;
	uint8_t five_finger_slot[5];
	int32_t five_finger_tracking_id[5];
	uint16_t five_finger_radius[5];
	bool five_finger_baseline_valid;
	enum five_finger_candidate five_finger_candidate;
	bool sensor_reset;
	uint32_t arbiter_frame_seq;
	uint32_t ownership_frame_seq;
	bool claim_submitted;
	bool pending_five_finger_action;
	enum five_finger_candidate pending_five_finger_candidate;
	uint32_t pending_five_finger_frame_seq;
};

static int absolute_int(int value) { return value < 0 ? -value : value; }

static void update_contact_centroid(struct gesture_data *data)
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
	if (count != 0) {
		data->x = sum_x / count;
		data->y = sum_y / count;
	}
}

static uint8_t active_contact_count(const struct gesture_data *data)
{
	uint8_t count = 0;

	for (uint8_t index = 0; index < TRAVELERS_PAD_MAX_CONTACTS; index++) {
		count += data->slot_valid[index];
	}
	return count;
}

static uint16_t radial_distance(const struct gesture_data *data, uint8_t slot)
{
	return absolute_int((int)data->slot_x[slot] - data->x) +
	       absolute_int((int)data->slot_y[slot] - data->y);
}

static const char *five_finger_candidate_name(enum five_finger_candidate candidate)
{
	return candidate == FIVE_FINGER_GRASP ? "Grasp" : "Release";
}

static void emit_five_finger_gesture(const struct device *dev,
				      enum five_finger_candidate candidate);

static void multifinger_resolve_action(void *context,
					const struct travelers_pad_owner_token *token,
					uint32_t ownership_frame_seq)
{
	struct gesture_data *data = context;
	uint16_t column;

	ARG_UNUSED(token);
	if (!data->pending_five_finger_action ||
	    data->pending_five_finger_frame_seq != ownership_frame_seq) {
		return;
	}
	data->pending_five_finger_action = false;
	column = data->pending_five_finger_candidate == FIVE_FINGER_GRASP ? 0 : 1;
	if (travelers_pad_output_behavior_primary(
			data->dev, data->callback, ownership_frame_seq,
			TRAVELERS_PAD_RECOGNIZER_MULTIFINGER_GESTURE,
			TRAVELERS_PAD_OWNER_MULTI_GESTURE, 2, column,
			"multifinger-geometry") == 0) {
		LOG_INF("5F %s action dispatched by arbiter",
			five_finger_candidate_name(data->pending_five_finger_candidate));
	}
}

static bool claim_multifinger_gesture(struct gesture_data *data, uint32_t evidence)
{
	enum travelers_pad_owner owner =
		travelers_pad_arbiter_owner_for_frame(data->arbiter_frame_seq);

	/* The arbiter is authoritative for this primary recognizer. A prior
	 * exclusive owner must prevent a competing gesture output. MULTI_GESTURE is
	 * permitted here only for continuation of this recognizer's proposal.
	 * このprimary recognizerではarbiterを正とする。先行する排他ownerは競合する
	 * gesture出力を阻止する。MULTI_GESTUREはこのrecognizerのproposal継続時だけ
	 * 許可する。 */
	if (owner != TRAVELERS_PAD_OWNER_NONE && owner != TRAVELERS_PAD_OWNER_MULTI_GESTURE) {
		return false;
	}

	if (!data->claim_submitted) {
		/* Submit exactly one claim for the session, regardless of whether 3/4F
		 * swipe or 5F radial geometry supplied the evidence. The resolver's
		 * deterministic decision—not callback-local state—now chooses whether
		 * this recognizer owns the operation.
		 * 3/4本指swipe／5本指radial geometryのいずれでもsessionごとに一つだけ
		 * claimを提出する。callback局所stateではなく、resolverの決定的な判断が
		 * このrecognizerの操作所有を決める。 */
		if (!travelers_pad_arbiter_submit(&(struct travelers_pad_arbiter_proposal){
			.frame_seq = data->arbiter_frame_seq,
			.recognizer = TRAVELERS_PAD_RECOGNIZER_MULTIFINGER_GESTURE,
			.intent = TRAVELERS_PAD_PROPOSAL_CLAIM,
			.requested_owner = TRAVELERS_PAD_OWNER_MULTI_GESTURE,
			.capabilities = TRAVELERS_PAD_CAP_BEHAVIOR_ACTION,
			.contact_count = data->fingers,
			.evidence = evidence,
		})) {
			LOG_WRN("%uF gesture claim proposal rejected", data->max_fingers);
			return false;
		}
		data->ownership_frame_seq = data->arbiter_frame_seq;
		data->claim_submitted = true;
	}

	return true;
}

static void capture_five_finger_baseline(struct gesture_data *data)
{
	uint8_t contact = 0;

	for (uint8_t slot = 0; slot < TRAVELERS_PAD_MAX_CONTACTS; slot++) {
		if (!data->slot_valid[slot]) {
			continue;
		}
		data->five_finger_slot[contact] = slot;
		data->five_finger_tracking_id[contact] = data->slot_tracking_id[slot];
		data->five_finger_radius[contact] = radial_distance(data, slot);
		contact++;
	}
	data->five_finger_baseline_valid = true;
	LOG_INF("5F geometry baseline slots=(%u,%u,%u,%u,%u) ids=(%d,%d,%d,%d,%d) radial=(%u,%u,%u,%u,%u)",
		data->five_finger_slot[0], data->five_finger_slot[1], data->five_finger_slot[2],
		data->five_finger_slot[3], data->five_finger_slot[4], data->five_finger_tracking_id[0],
		data->five_finger_tracking_id[1], data->five_finger_tracking_id[2],
		data->five_finger_tracking_id[3], data->five_finger_tracking_id[4], data->five_finger_radius[0],
		data->five_finger_radius[1], data->five_finger_radius[2], data->five_finger_radius[3],
		data->five_finger_radius[4]);
}

static void process_five_finger_geometry(const struct device *dev)
{
	const struct gesture_config *cfg = dev->config;
	struct gesture_data *data = dev->data;
	uint8_t inward = 0;
	uint8_t outward = 0;

	if (data->fingers != 5 || active_contact_count(data) != 5) {
		return;
	}
	if (!data->five_finger_baseline_valid) {
		data->five_finger_stable_frames++;
		if (data->five_finger_stable_frames >= cfg->five_finger_stable_frames) {
			capture_five_finger_baseline(data);
		}
		return;
	}
	for (uint8_t slot = 0; slot < 5; slot++) {
		uint8_t tracked_slot = data->five_finger_slot[slot];
		uint16_t current;

		if (!data->slot_valid[tracked_slot] ||
		    data->slot_tracking_id[tracked_slot] != data->five_finger_tracking_id[slot]) {
			data->five_finger_baseline_valid = false;
			data->five_finger_stable_frames = 0;
			LOG_WRN("5F geometry baseline invalidated by contact identity change");
			return;
		}
		current = radial_distance(data, tracked_slot);

		if (current + cfg->five_finger_radial_move <= data->five_finger_radius[slot]) {
			inward++;
		} else if (current >= data->five_finger_radius[slot] + cfg->five_finger_radial_move) {
			outward++;
		}
	}
	if (data->five_finger_candidate == FIVE_FINGER_NONE && (inward >= 4 || outward >= 4) &&
	    claim_multifinger_gesture(data, inward >= 4 ? inward : outward)) {
		data->five_finger_candidate = inward >= 4 ? FIVE_FINGER_GRASP : FIVE_FINGER_RELEASE;
		LOG_INF("5F %s candidate: inward=%u outward=%u threshold=%u",
			five_finger_candidate_name(data->five_finger_candidate), inward, outward,
			cfg->five_finger_radial_move);
		emit_five_finger_gesture(dev, data->five_finger_candidate);
	}
}

static int gesture_configure(const struct device *dev, kscan_callback_t callback)
{
	if (callback == NULL) {
		return -EINVAL;
	}
	((struct gesture_data *)dev->data)->callback = callback;
	return 0;
}

static const struct kscan_driver_api gesture_api = {.config = gesture_configure};

static enum gesture_column gesture_direction(const struct gesture_data *data)
{
	int dx = (int)data->x - data->start_x;
	int dy = (int)data->y - data->start_y;

	if (absolute_int(dx) >= absolute_int(dy)) {
		return dx >= 0 ? GESTURE_RIGHT : GESTURE_LEFT;
	}
	return dy >= 0 ? GESTURE_DOWN : GESTURE_UP;
}

static void emit_gesture(const struct device *dev)
{
	struct gesture_data *data = dev->data;
	uint16_t row = data->max_fingers - 3;
	uint16_t column = gesture_direction(data);

	if (travelers_pad_output_behavior_primary(
		data->dev, data->callback, data->ownership_frame_seq,
		TRAVELERS_PAD_RECOGNIZER_MULTIFINGER_GESTURE,
		TRAVELERS_PAD_OWNER_MULTI_GESTURE, row, column, "multifinger-swipe") != 0) {
		return;
	}
	LOG_INF("%uF swipe row=%u column=%u start=(%u,%u) end=(%u,%u)", data->max_fingers,
		row, column, data->start_x, data->start_y, data->x, data->y);
}

static void emit_five_finger_gesture(const struct device *dev,
				      enum five_finger_candidate candidate)
{
	struct gesture_data *data = dev->data;
	uint16_t column = candidate == FIVE_FINGER_GRASP ? 0 : 1;

	/* The 5F candidate claims and emits in the same logical input frame. Stage
	 * its first KSCAN pair until that frame has a committed arbiter token.
	 * 5本指candidateは同一論理input frameでclaimと出力を行う。その最初の
	 * KSCAN pairは、そのframeのarbiter tokenがcommitされるまで保留する。 */
	if (data->ownership_frame_seq == data->arbiter_frame_seq) {
		data->pending_five_finger_action = true;
		data->pending_five_finger_candidate = candidate;
		data->pending_five_finger_frame_seq = data->ownership_frame_seq;
		return;
	}
	if (travelers_pad_output_behavior_primary(
		data->dev, data->callback, data->ownership_frame_seq,
		TRAVELERS_PAD_RECOGNIZER_MULTIFINGER_GESTURE,
		TRAVELERS_PAD_OWNER_MULTI_GESTURE, 2, column,
		"multifinger-geometry") != 0) {
		return;
	}
	LOG_INF("5F %s emitted row=2 column=%u", five_finger_candidate_name(candidate), column);
}

static void reset_session(struct gesture_data *data)
{
	data->previous_fingers = 0;
	data->max_fingers = 0;
	data->moved = false;
	data->cancelled_by_drag_owner = false;
	data->cancelled_by_owner = false;
	data->five_finger_stable_frames = 0;
	data->five_finger_baseline_valid = false;
	data->five_finger_candidate = FIVE_FINGER_NONE;
	data->ownership_frame_seq = 0;
	data->claim_submitted = false;
	data->pending_five_finger_action = false;
	data->pending_five_finger_frame_seq = 0;
}

static void gesture_process_frame(const struct device *dev)
{
	const struct gesture_config *cfg = dev->config;
	struct gesture_data *data = dev->data;
	int64_t now = k_uptime_get();

	if (data->fingers != 0) {
		update_contact_centroid(data);
	}
	if (data->sensor_reset) {
		data->sensor_reset = false;
		reset_session(data);
		LOG_WRN("multifinger gesture cancelled after sensor reset");
		return;
	}
	if (data->fingers != 0 && data->previous_fingers == 0) {
		data->start_x = data->x;
		data->start_y = data->y;
		data->started_ms = now;
		data->max_fingers = data->fingers;
		data->moved = false;
		data->cancelled_by_drag_owner = false;
		data->cancelled_by_owner = false;
	}
	if (data->fingers != 0) {
		bool fingers_increased = data->fingers > data->max_fingers;

		data->max_fingers = MAX(data->max_fingers, data->fingers);
		/* Simultaneous contacts can arrive 1F, then 2F, then 3F. Rebase before
		 * movement starts so completing the set never becomes a false swipe. */
		if (fingers_increased && !data->moved) {
			data->start_x = data->x;
			data->start_y = data->y;
		}
		/* A committed DRAG owner exclusively interprets this
		 * contact session. Consult the arbiter rather than tap/drag's private
		 * tracking flag, so gesture cancellation follows the same owner source as
		 * router suppression.
		 * commit済みDRAG ownerは、この接触sessionを排他的に解釈する。
		 * routerの抑止と同じowner sourceに従うため、tap/drag固有のtracking flag
		 * ではなくarbiterを参照してgestureを取消す。 */
		if (travelers_pad_arbiter_owner_for_frame(data->arbiter_frame_seq) ==
	    TRAVELERS_PAD_OWNER_DRAG) {
			if (!data->cancelled_by_drag_owner) {
				LOG_INF("%uF gesture suppressed: arbiter drag owner", data->max_fingers);
			}
			data->cancelled_by_drag_owner = true;
		}
		/* A detected drag owner must prevent the gesture from proposing a
		 * competing owner. This guard is intentionally separate from terminal
		 * output suppression because proposal creation is itself a side effect.
		 * 検出済みのdrag ownerはgestureが競合ownerをproposalすること自体を阻止する。
		 * 終端出力の抑止とは別にする。proposal生成自体が副作用となるためである。 */
		if (data->max_fingers >= 3 && !data->moved && !data->cancelled_by_drag_owner &&
	    (absolute_int((int)data->x - data->start_x) >= cfg->swipe_min_move ||
	    absolute_int((int)data->y - data->start_y) >= cfg->swipe_min_move)) {
			uint32_t move = absolute_int((int)data->x - data->start_x) +
					absolute_int((int)data->y - data->start_y);

			if (claim_multifinger_gesture(data, move)) {
				data->moved = true;
			} else {
				data->cancelled_by_owner = true;
			}
		}
		if (data->max_fingers == 5 && !data->cancelled_by_drag_owner) {
			process_five_finger_geometry(dev);
		}
	} else if (data->previous_fingers != 0) {
		if (data->max_fingers == 5 && data->five_finger_candidate != FIVE_FINGER_NONE) {
			LOG_INF("5F %s gesture complete",
				five_finger_candidate_name(data->five_finger_candidate));
		} else if (!data->cancelled_by_drag_owner && !data->cancelled_by_owner &&
		    data->max_fingers >= 3 && data->max_fingers <= 4 &&
		    data->moved && now - data->started_ms <= cfg->swipe_max_ms) {
			emit_gesture(dev);
		} else if (data->max_fingers >= 3 && data->max_fingers <= 4 && data->moved) {
			LOG_INF("%uF swipe ignored: duration=%lld ms drag-owner=%u", data->max_fingers,
			now - data->started_ms, data->cancelled_by_drag_owner);
		}
		reset_session(data);
	}
	data->previous_fingers = data->fingers;
}

static void gesture_handle_input(const struct device *dev, struct input_event *evt)
{
	struct gesture_data *data = dev->data;

	if (evt->type == INPUT_EV_ABS) {
		if (evt->code == INPUT_ABS_MT_SLOT) {
			data->slot = evt->value;
		} else if (evt->code == TRAVELERS_PAD_INPUT_ABS_MT_TRACKING_ID &&
			   data->slot < TRAVELERS_PAD_MAX_CONTACTS) {
			data->slot_valid[data->slot] = evt->value >= 0;
			data->slot_tracking_id[data->slot] = evt->value;
		} else if (evt->code == TRAVELERS_PAD_INPUT_ABS_MT_POSITION_X &&
			   data->slot < TRAVELERS_PAD_MAX_CONTACTS) {
			data->slot_x[data->slot] = evt->value;
		} else if (evt->code == TRAVELERS_PAD_INPUT_ABS_MT_POSITION_Y &&
			   data->slot < TRAVELERS_PAD_MAX_CONTACTS) {
			data->slot_y[data->slot] = evt->value;
		} else if (evt->code == INPUT_ABS_Z) {
			data->fingers = evt->value;
		} else if (evt->code == INPUT_ABS_RZ && evt->value) {
			data->sensor_reset = true;
		}
	}
	if (evt->sync) {
		data->arbiter_frame_seq =
			travelers_pad_arbiter_next_frame(TRAVELERS_PAD_RECOGNIZER_MULTIFINGER_GESTURE);
		gesture_process_frame(dev);
		travelers_pad_arbiter_observed(TRAVELERS_PAD_RECOGNIZER_MULTIFINGER_GESTURE,
					       data->arbiter_frame_seq);
	}
}

static int gesture_init(const struct device *dev)
{
	((struct gesture_data *)dev->data)->dev = dev;
	(void)travelers_pad_arbiter_register(TRAVELERS_PAD_RECOGNIZER_MULTIFINGER_GESTURE);
	(void)travelers_pad_arbiter_bind_resolve_handler(
		TRAVELERS_PAD_RECOGNIZER_MULTIFINGER_GESTURE, multifinger_resolve_action,
		dev->data);
	return 0;
}

#define GESTURE_INST(n)                                                                        \
	static struct gesture_data gesture_data_##n;                                                 \
	static const struct gesture_config gesture_config_##n = {                                   \
		.swipe_min_move = DT_INST_PROP_OR(n, swipe_min_move, 180),                            \
		.swipe_max_ms = DT_INST_PROP_OR(n, swipe_max_ms, 700),                                \
		.five_finger_radial_move = DT_INST_PROP_OR(n, five_finger_radial_move, 80),          \
		.five_finger_stable_frames = DT_INST_PROP_OR(n, five_finger_stable_frames, 3),       \
	};                                                                                           \
	static void gesture_input_handler_##n(struct input_event *evt)                              \
	{                                                                                            \
		gesture_handle_input(DEVICE_DT_INST_GET(n), evt);                                      \
	}                                                                                            \
	INPUT_CALLBACK_DEFINE(DEVICE_DT_GET(DT_INST_PHANDLE(n, input_device)),                      \
			      gesture_input_handler_##n);                                               \
	DEVICE_DT_INST_DEFINE(n, gesture_init, NULL, &gesture_data_##n, &gesture_config_##n, POST_KERNEL,  \
			      CONFIG_TRAVELERS_PAD_IQS9150_FUNCTION_INIT_PRIORITY, &gesture_api);

DT_INST_FOREACH_STATUS_OKAY(GESTURE_INST)
