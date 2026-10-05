/*
 * Travelers Pad IQS9150 tap and double-tap drag function.
 *
 * 1/2/3-finger taps produce left/right/middle clicks. The first tap is held
 * briefly so that a nearby second tap with the same finger count can become
 * a hold-to-drag operation. It receives session-republished contact frames, so
 * router suppression for scrolling, geometry, and multi-finger swipes cannot
 * lose tap history while the arbiter controls exclusive ownership.
 *
 * 1/2/3本指タップを左/右/中クリックへ変換する。最初のタップを短時間保留し、
 * 近い位置・同本数の二回目タップをホールドするとドラッグへ移行する。session再配信の
 * 接触frameを監視するため、タップ履歴を失わず、排他所有権はarbiterが制御する。
 */

#define DT_DRV_COMPAT zmk_travelers_pad_iqs9150_tap_drag

#include <zephyr/device.h>
#include <zephyr/dt-bindings/input/input-event-codes.h>
#include <zephyr/input/input.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>
#include <string.h>

#include <travelers_pad/multitouch.h>
#include <travelers_pad/contact_arbiter.h>
#include <travelers_pad/contact_output_broker.h>

LOG_MODULE_REGISTER(travelers_pad_tap_drag, LOG_LEVEL_INF);

#define TRAVELERS_PAD_DRAG_MAX_FRAME_DELTA 256

struct tap_drag_config {
	uint16_t tap_max_ms;
	uint16_t tap_max_move;
	uint16_t multifinger_settle_ms;
	uint16_t double_tap_max_gap_ms;
	uint16_t double_tap_max_distance;
	uint16_t drag_start_move;
};

struct tap_drag_data {
	const struct device *dev;
	struct k_work_delayable deferred_click_work;
	struct k_work_sync deferred_click_sync;
	uint16_t x;
	uint16_t y;
	uint8_t slot;
	uint16_t slot_x[TRAVELERS_PAD_MAX_CONTACTS];
	uint16_t slot_y[TRAVELERS_PAD_MAX_CONTACTS];
	bool slot_valid[TRAVELERS_PAD_MAX_CONTACTS];
	uint16_t reference_slot_x[TRAVELERS_PAD_MAX_CONTACTS];
	uint16_t reference_slot_y[TRAVELERS_PAD_MAX_CONTACTS];
	bool reference_slot_valid[TRAVELERS_PAD_MAX_CONTACTS];
	uint8_t fingers;
	uint8_t previous_fingers;
	uint8_t max_fingers;
	uint16_t start_x;
	uint16_t start_y;
	int64_t started_ms;
	int64_t multifinger_settle_until_ms;
	int64_t first_released_ms;
	uint16_t first_x;
	uint16_t first_y;
	uint8_t first_fingers;
	uint8_t drag_fingers;
	uint16_t drag_previous_x;
	uint16_t drag_previous_y;
	bool drag_position_valid;
	bool moved;
	bool first_valid;
	struct travelers_pad_terminal_action_grant first_tap_grant;
	bool second_tap;
	bool button_down;
	/* The second tap owns the session but has not crossed drag_start_move yet.
	 * Keep this separate from button_down: edge continuation is delegated only
	 * after the arbiter has committed the drag token. / 二回目tapはsessionを所有
	 * するが、まだdrag_start_moveを越えていない。button_downとは分け、edge
	 * continuationはarbiterがdrag tokenを確定後にだけ委譲する。 */
	bool drag_motion_active;
	bool drag_claim_authorized;
	bool sensor_reset;
	bool session_suppressed;
	/* Store the arbiter owner that caused suppression. The terminal frame is
	 * processed after the live token can have been retired, so reporting the
	 * live query there could otherwise lose the reason for the suppression.
	 * 抑止の原因となったarbiter ownerを保持する。terminal frameはlive tokenが
	 * 失効した後に処理され得るため、その時点で問い合わせるだけでは原因を失う。 */
	enum travelers_pad_owner suppressed_owner;
	uint32_t arbiter_frame_seq;
	uint32_t drag_ownership_frame_seq;
	bool drag_start_pending;
	bool terminal_pending;
	uint32_t terminal_frame_seq;
	int64_t terminal_timestamp_ms;
	uint32_t tap_generation;
	uint32_t deferred_click_generation;
};

static int absolute_int(int value)
{
	return value < 0 ? -value : value;
}

static bool owner_excludes_tap_drag(enum travelers_pad_owner owner)
{
	/* DRAG is this recognizer's own ownership, while NONE means the primary
	 * decision has no owner. Every other committed owner excludes this session
	 * from tap/drag classification.
	 * DRAGはこのrecognizer自身の所有権であり、NONEはprimary裁定にownerが無いことを示す。
	 * それ以外のcommit済みownerはtap/drag分類からこのsessionを除外する。 */
	return owner != TRAVELERS_PAD_OWNER_NONE && owner != TRAVELERS_PAD_OWNER_DRAG;
}

static void update_contact_centroid(struct tap_drag_data *data)
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

static void snapshot_contact_slots(struct tap_drag_data *data)
{
	for (uint8_t index = 0; index < TRAVELERS_PAD_MAX_CONTACTS; index++) {
		data->reference_slot_x[index] = data->slot_x[index];
		data->reference_slot_y[index] = data->slot_y[index];
		data->reference_slot_valid[index] = data->slot_valid[index];
	}
}

static bool existing_contact_moved(const struct tap_drag_data *data, uint16_t threshold)
{
	/* Only compare contacts that existed before this report. A newly revealed
	 * second/third finger legitimately moves the centroid, but it must not hide
	 * movement of the fingers that were already down.
	 * 直前のレポートにも存在した接触だけを比較する。新たに検出された2本目/3本目は
	 * 重心を正当に動かすが、すでに触れていた指の移動を隠してはならない。 */
	for (uint8_t index = 0; index < TRAVELERS_PAD_MAX_CONTACTS; index++) {
		if (!data->slot_valid[index] || !data->reference_slot_valid[index]) {
			continue;
		}
		if (absolute_int((int)data->slot_x[index] - data->reference_slot_x[index]) > threshold ||
		    absolute_int((int)data->slot_y[index] - data->reference_slot_y[index]) > threshold) {
			return true;
		}
	}
	return false;
}

static uint16_t button_for_fingers(uint8_t fingers)
{
	return fingers == 1 ? INPUT_BTN_0 : fingers == 2 ? INPUT_BTN_1 : INPUT_BTN_2;
}

static bool report_click(struct tap_drag_data *data,
			 struct travelers_pad_terminal_action_grant *grant,
			 enum travelers_pad_owner owner, uint8_t fingers, uint8_t click_count)
{
	uint16_t button = button_for_fingers(fingers);
	int rc = travelers_pad_output_terminal_click(
		data->dev, grant, TRAVELERS_PAD_RECOGNIZER_TAP_DRAG, owner, button, click_count,
		"tap-drag-terminal-click");
	if (rc != 0) {
		LOG_ERR("tap click report failed: %d", rc);
	}
	return rc == 0;
}

static bool report_button(struct tap_drag_data *data, bool pressed)
{
	int rc;

	if (pressed) {
		rc = travelers_pad_output_primary(
			data->dev, data->drag_ownership_frame_seq, TRAVELERS_PAD_RECOGNIZER_TAP_DRAG,
			TRAVELERS_PAD_OWNER_DRAG, TRAVELERS_PAD_CAP_BUTTON, INPUT_EV_KEY,
			button_for_fingers(data->drag_fingers), 1, true, "tap-drag-button-down");
	} else {
		rc = travelers_pad_output_terminal(
			data->dev, data->drag_ownership_frame_seq, TRAVELERS_PAD_RECOGNIZER_TAP_DRAG,
			TRAVELERS_PAD_OWNER_DRAG, TRAVELERS_PAD_CAP_BUTTON, INPUT_EV_KEY,
			button_for_fingers(data->drag_fingers), 0, true, "tap-drag-button-up");
	}

	if (rc != 0) {
		LOG_ERR("drag button report failed: %d", rc);
	}
	return rc == 0;
}

static void report_drag_motion(struct tap_drag_data *data, int16_t dx, int16_t dy)
{
	int rc;

	/* A contact-count transition must never turn an invalid centroid into a
	 * screen-edge jump. Normal IQS9150 deltas at the report rate are much
	 * smaller than this guard.
	 * 接触本数の遷移によって不正な重心を画面端へのジャンプにしてはならない。
	 * 通常のIQS9150レポート周期での差分はこの上限より十分小さい。 */
	if (absolute_int(dx) > TRAVELERS_PAD_DRAG_MAX_FRAME_DELTA ||
	    absolute_int(dy) > TRAVELERS_PAD_DRAG_MAX_FRAME_DELTA) {
		LOG_WRN("discarded implausible drag delta %d/%d", dx, dy);
		return;
	}

	if (dx != 0) {
		rc = travelers_pad_output_primary(
			data->dev, data->drag_ownership_frame_seq, TRAVELERS_PAD_RECOGNIZER_TAP_DRAG,
			TRAVELERS_PAD_OWNER_DRAG, TRAVELERS_PAD_CAP_POINTER_DELTA, INPUT_EV_REL,
			INPUT_REL_X, dx, dy == 0, "tap-drag-motion-x");
		if (rc != 0) {
			LOG_ERR("drag X report failed: %d", rc);
		}
	}
	if (dy != 0) {
		rc = travelers_pad_output_primary(
			data->dev, data->drag_ownership_frame_seq, TRAVELERS_PAD_RECOGNIZER_TAP_DRAG,
			TRAVELERS_PAD_OWNER_DRAG, TRAVELERS_PAD_CAP_POINTER_DELTA, INPUT_EV_REL,
			INPUT_REL_Y, dy, true, "tap-drag-motion-y");
		if (rc != 0) {
			LOG_ERR("drag Y report failed: %d", rc);
		}
	}
}

static void start_drag_motion(struct tap_drag_data *data);
static void request_drag_claim(struct tap_drag_data *data);

static void tap_drag_resolve_action(void *context, const struct travelers_pad_owner_token *token,
				    uint32_t ownership_frame_seq)
{
	struct tap_drag_data *data = context;

	if (!data->drag_start_pending || data->drag_ownership_frame_seq != ownership_frame_seq) {
		return;
	}
	data->drag_start_pending = false;
	if (token == NULL || !token->valid || token->owner != TRAVELERS_PAD_OWNER_DRAG) {
		data->drag_position_valid = false;
		return;
	}
	/* Do not send a click yet. A host performs a double-click action on the
	 * second button-down, before it can observe whether the user moves. Hold
	 * the HID output until drag intent is spatially confirmed.
	 * ここではまだclickを送らない。hostはユーザーが移動したかを観測する前、二回目
	 * button-down時点でdouble-click actionを実行する。drag意思が距離で確定するまで
	 * HID出力を保留する。 */
	data->drag_claim_authorized = true;
	LOG_INF("%uF double-tap drag armed by arbiter; awaiting movement", data->drag_fingers);
}

static void deferred_click_work(struct k_work *work)
{
	struct tap_drag_data *data = CONTAINER_OF(work, struct tap_drag_data,
						     deferred_click_work.work);

	if (data->first_valid && data->deferred_click_generation == data->tap_generation) {
		LOG_INF("%uF single tap confirmed", data->first_fingers);
		(void)report_click(data, &data->first_tap_grant, TRAVELERS_PAD_OWNER_NONE,
				   data->first_fingers, 1);
		data->first_valid = false;
	}
}

/* All callers run in the input/arbiter thread, never in deferred_click_work.
 * Waiting here makes the generation invalidation a true ownership hand-off:
 * no old timer can observe rewritten tap or terminal-grant state. */
static void cancel_deferred_click_work(struct tap_drag_data *data)
{
	(void)k_work_cancel_delayable_sync(&data->deferred_click_work,
					  &data->deferred_click_sync);
	data->tap_generation++;
	data->deferred_click_generation = 0;
}

static void cancel_tap_drag_after_reset(struct tap_drag_data *data)
{
	/* Do not overwrite recognizer state until a pending timer has quiesced. */
	cancel_deferred_click_work(data);
	data->first_valid = false;
	data->first_tap_grant.valid = false;
	data->second_tap = false;
	data->button_down = false;
	data->drag_motion_active = false;
	data->drag_claim_authorized = false;
	data->drag_start_pending = false;
	data->terminal_pending = false;
	data->session_suppressed = false;
	data->suppressed_owner = TRAVELERS_PAD_OWNER_NONE;
	data->fingers = 0;
	data->previous_fingers = 0;
	data->max_fingers = 0;
	data->drag_fingers = 0;
	data->drag_position_valid = false;
	data->sensor_reset = false;
	memset(data->slot_valid, 0, sizeof(data->slot_valid));
	memset(data->reference_slot_valid, 0, sizeof(data->reference_slot_valid));
}

static void request_drag_claim(struct tap_drag_data *data)
{
	/* Claim the candidate now, but keep HID silent until motion crosses the
	 * drag threshold. / 二回目接触は今claimするが、移動がdrag閾値を越えるまで
	 * HID出力は保留する。 */
	if (!travelers_pad_arbiter_submit(&(struct travelers_pad_arbiter_proposal){
		.frame_seq = data->arbiter_frame_seq,
		.recognizer = TRAVELERS_PAD_RECOGNIZER_TAP_DRAG,
		.intent = TRAVELERS_PAD_PROPOSAL_CLAIM,
		.requested_owner = TRAVELERS_PAD_OWNER_DRAG,
		.capabilities = TRAVELERS_PAD_CAP_POINTER_DELTA | TRAVELERS_PAD_CAP_BUTTON,
		.contact_count = data->fingers,
		.evidence = data->drag_fingers,
	})) {
		LOG_WRN("%uF double-tap drag claim proposal rejected", data->drag_fingers);
		return;
	}
	data->drag_ownership_frame_seq = data->arbiter_frame_seq;
	data->drag_start_pending = true;
	LOG_INF("%uF double-tap drag claim awaiting arbiter dispatch", data->drag_fingers);
}

static void start_drag_motion(struct tap_drag_data *data)
{
	/* The movement that merely crosses the drag activation threshold must not
	 * displace the grabbed object. The router has already suppressed ordinary
	 * pointer output while this double-tap candidate is tracked, so rebase at
	 * the activation frame and send only subsequent motion with the button held.
	 * ドラッグ開始閾値を越えるためだけの移動で、掴んだ対象をずらしてはならない。
	 * double-tap候補の追跡中はrouterが通常ポインタ出力を抑止済みなので、開始frameで
	 * 基準を現在座標へ再設定し、以後の移動だけをボタン押下中に送信する。 */
	if (!report_button(data, true)) {
		return;
	}
	/* DRAG_RESERVED becomes DRAG_ACTIVE only after the physical button-down
	 * succeeded. Edge-hold therefore cannot move a stationary second tap at an
	 * edge while it is still eligible to become a double click.
	 * 物理button-down成功後にだけDRAG_RESERVEDをDRAG_ACTIVEへ進める。これにより
	 * 辺上で静止した2回目tapがdouble click候補の間、edge-holdは移動できない。 */
	if (!travelers_pad_arbiter_activate_owner(
			data->drag_ownership_frame_seq, TRAVELERS_PAD_RECOGNIZER_TAP_DRAG,
			TRAVELERS_PAD_OWNER_DRAG, "tap-drag-button")) {
		(void)report_button(data, false);
		return;
	}
	data->button_down = true;
	data->drag_previous_x = data->x;
	data->drag_previous_y = data->y;
	data->drag_position_valid = true;
	data->drag_motion_active = true;
	LOG_INF("%uF double-tap drag motion started", data->drag_fingers);
}

static bool matches_first_tap(const struct tap_drag_data *data, const struct tap_drag_config *cfg,
			      int64_t now)
{
	return data->first_valid && data->first_fingers == data->fingers &&
	       now - data->first_released_ms <= cfg->double_tap_max_gap_ms &&
	       absolute_int((int)data->x - data->first_x) <= cfg->double_tap_max_distance &&
	       absolute_int((int)data->y - data->first_y) <= cfg->double_tap_max_distance;
}

static void start_double_tap_candidate(struct tap_drag_data *data)
{
	data->second_tap = true;
	cancel_deferred_click_work(data);
	data->drag_claim_authorized = false;
	data->first_valid = false;
	data->drag_fingers = data->fingers;
	request_drag_claim(data);
	LOG_INF("%uF double-tap candidate; drag claim requested", data->drag_fingers);
}

static void begin_contact(struct tap_drag_data *data, const struct tap_drag_config *cfg, int64_t now)
{
	data->started_ms = now;
	data->start_x = data->x;
	data->start_y = data->y;
	data->max_fingers = data->fingers;
	data->moved = false;
	/* The first sensor frame can already contain two or three contacts, so the
	 * settling window cannot be armed only by a later finger-count increase.
	 * 最初のセンサーフレームがすでに2本または3本指の場合があるため、安定化時間を
	 * 指数増加時だけでなく接触開始時にも設定する。 */
	data->multifinger_settle_until_ms =
		data->fingers >= 2 ? now + cfg->multifinger_settle_ms : 0;
	data->second_tap = false;
	snapshot_contact_slots(data);
	if (data->first_valid) {
		LOG_INF("%uF tap start centroid=(%u,%u), previous=%uF centroid=(%u,%u) gap=%lld ms",
			data->fingers, data->x, data->y, data->first_fingers, data->first_x,
			data->first_y, now - data->first_released_ms);
	}
	if (matches_first_tap(data, cfg, now)) {
		start_double_tap_candidate(data);
	}
}

static void finish_contact(const struct device *dev, const struct tap_drag_config *cfg,
			   int64_t now, uint32_t terminal_frame_seq,
			   const struct travelers_pad_owner_token *terminal_token)
{
	struct tap_drag_data *data = dev->data;
	if (data->session_suppressed) {
		/* An exclusive owner (scroll, edge dial, gesture, or Tip-Tap) must
		 * preserve a deferred click from an earlier session, but must never turn
		 * its own release into a 2F tap. */
		if (data->button_down) {
			report_button(data, false);
			data->button_down = false;
		}
		LOG_INF("%uF tap suppressed: arbiter owner=%u", data->max_fingers,
			data->suppressed_owner);
		data->session_suppressed = false;
		data->suppressed_owner = TRAVELERS_PAD_OWNER_NONE;
		data->second_tap = false;
		data->drag_start_pending = false;
		data->drag_claim_authorized = false;
		return;
	}
	/* A higher-priority recognizer can establish ownership on this terminal
	 * frame itself (Tip-Tap is the normal case). The raw tap/drag callback ran
	 * before that decision existed, so session_suppressed cannot see it yet.
	 * Consume the resolved token here instead of requesting a terminal grant
	 * that is guaranteed to be denied. / 上位recognizerはこのterminal frame自身で
	 * 所有権を確立し得る（通常はTip-Tap）。生tap/drag callbackはその決定前に走るため、
	 * session_suppressedではまだ検出できない。必ず拒否されるterminal grantを要求せず、
	 * ここで裁定済みtokenを消費する。 */
	if (terminal_token != NULL && terminal_token->valid &&
	    terminal_token->owner != TRAVELERS_PAD_OWNER_NONE &&
	    terminal_token->owner != TRAVELERS_PAD_OWNER_DRAG) {
		cancel_deferred_click_work(data);
		LOG_INF("%uF tap suppressed: resolved terminal owner=%u", data->max_fingers,
			terminal_token->owner);
		data->first_valid = false;
		data->first_tap_grant.valid = false;
		data->second_tap = false;
		return;
	}
	bool valid_tap = !data->moved && data->max_fingers >= 1 && data->max_fingers <= 3 &&
			 now - data->started_ms <= cfg->tap_max_ms;

	if (data->button_down) {
		report_button(data, false);
		LOG_INF("%uF drag released", data->drag_fingers);
		data->button_down = false;
		data->drag_motion_active = false;
		data->drag_position_valid = false;
	} else if (data->second_tap) {
		if (!data->moved && data->drag_claim_authorized) {
			/* No meaningful movement occurred during the second contact, so it is
			 * a double click. Neither click was sent before this release.
			 * 二回目接触中に有意な移動がなかったためdouble clickとする。ここまで
			 * いずれのclickも送っていない。 */
			struct travelers_pad_terminal_action_grant grant = {};

			if (travelers_pad_arbiter_issue_terminal_action(
				terminal_frame_seq, TRAVELERS_PAD_RECOGNIZER_TAP_DRAG,
				TRAVELERS_PAD_OWNER_DRAG, TRAVELERS_PAD_CAP_BUTTON, &grant) &&
			    report_click(data, &grant, TRAVELERS_PAD_OWNER_DRAG,
					 data->drag_fingers, 2)) {
				LOG_INF("%uF double-tap completed on release", data->drag_fingers);
			}
			data->first_tap_grant.valid = false;
		} else {
			/* Preserve the first tap only if a competing owner prevented this
			 * candidate from becoming a drag or double click.
			 * 競合ownerによりこの候補をdrag/double clickにできなかった場合は、
			 * 一回目tapだけを保持する。 */
			if (report_click(data, &data->first_tap_grant, TRAVELERS_PAD_OWNER_NONE,
					 data->drag_fingers, 1)) {
				LOG_INF("%uF double-tap not armed; delivering first tap only",
					data->drag_fingers);
			}
		}
	} else if (valid_tap) {
		/* Quiesce an expired/racing prior timer before replacing the terminal
		 * grant it could otherwise consume. */
		cancel_deferred_click_work(data);
		LOG_INF("%uF tap released centroid=(%u,%u), duration=%lld ms", data->max_fingers,
			data->start_x, data->start_y, now - data->started_ms);
		data->first_tap_grant = (struct travelers_pad_terminal_action_grant){};
		if (!travelers_pad_arbiter_issue_terminal_action(
				terminal_frame_seq, TRAVELERS_PAD_RECOGNIZER_TAP_DRAG,
				TRAVELERS_PAD_OWNER_NONE, TRAVELERS_PAD_CAP_BUTTON,
				&data->first_tap_grant)) {
			LOG_ERR("%uF tap terminal-action grant rejected", data->max_fingers);
			data->first_valid = false;
			return;
		}
		data->first_valid = true;
		data->first_released_ms = now;
		data->first_x = data->start_x;
		data->first_y = data->start_y;
		data->first_fingers = data->max_fingers;
		data->tap_generation++;
		data->deferred_click_generation = data->tap_generation;
		k_work_schedule(&data->deferred_click_work, K_MSEC(cfg->double_tap_max_gap_ms));
	} else {
		cancel_deferred_click_work(data);
		LOG_INF("%uF tap rejected: duration=%lld ms moved=%d limit-ms=%u", data->max_fingers,
			now - data->started_ms, data->moved, cfg->tap_max_ms);
		data->first_valid = false;
		data->first_tap_grant.valid = false;
	}
	data->second_tap = false;
}

static void process_frame(const struct device *dev)
{
	const struct tap_drag_config *cfg = dev->config;
	struct tap_drag_data *data = dev->data;
	int64_t now = k_uptime_get();

	if (data->fingers != 0) {
		enum travelers_pad_owner arbiter_owner =
			travelers_pad_arbiter_owner_for_frame(data->arbiter_frame_seq);

		/* The arbiter is the only contact-owner authority. Consult the owner on
		 * this forwarded frame instead of importing recognizer-private state.
		 * arbiterだけがcontact owner authorityである。recognizer固有状態を直接
		 * 参照せず、この転送frame自身のownerを確認する。 */
		if (owner_excludes_tap_drag(arbiter_owner)) {
			if (data->button_down) {
				report_button(data, false);
				data->button_down = false;
			}
			data->drag_motion_active = false;
			data->drag_start_pending = false;
			data->drag_claim_authorized = false;
			data->session_suppressed = true;
			data->suppressed_owner = arbiter_owner;
			data->second_tap = false;
		}
		/* F-slot assignment can change between taps; centroid is invariant. */
		update_contact_centroid(data);
	}

	if (data->sensor_reset) {
		if (data->button_down) {
			report_button(data, false);
		}
		cancel_tap_drag_after_reset(data);
		LOG_WRN("tap/drag session cancelled after sensor reset");
		return;
	}
	if (data->fingers != 0 && data->previous_fingers == 0) {
		begin_contact(data, cfg, now);
	}
	if (data->fingers != 0) {
		bool fingers_increased = data->fingers > data->max_fingers;
		bool established_contact_moved = existing_contact_moved(data, cfg->tap_max_move);

		data->max_fingers = MAX(data->max_fingers, data->fingers);
		/* A nominally simultaneous multi-finger tap can arrive as 1F then 2F
		 * (or 3F) on adjacent reports. While it is still stationary, rebase the
		 * tap origin and duration to the completed contact centroid. Otherwise a
		 * deliberately staggered second/third finger is charged against the
		 * final multi-finger tap duration (and the centroid shift can look like
		 * a large, false movement).
		 * 同時に置いたつもりの複数指は、IQS9150では1F→2F（または3F）として
		 * 隣接レポートで届くことがある。静止中なら、完成した接触の重心だけでなく
		 * 開始時刻も再基準化する。これにより、少し遅れて置いた2本目/3本目の時間を
		 * 最終的な複数指タップ時間へ不当に加算しない。 */
		if (fingers_increased && !data->button_down) {
			if (data->second_tap && data->drag_fingers != data->fingers) {
				/* A pending 1F double-tap must not become a 1F drag after a
				 * second finger appears. Deliver the deferred first click, then
				 * let the completed multi-finger contact start a new classification.
				 * 保留中の1本指ダブルタップを、2本目の出現後に1本指ドラッグへ
				 * 変換してはならない。先行する1本指クリックを確定し、完成した
				 * 複数指接触を新しい操作として分類する。 */
				LOG_INF("%uF double-tap candidate cancelled by %uF contact", data->drag_fingers,
					data->fingers);
				(void)report_click(data, &data->first_tap_grant,
						   TRAVELERS_PAD_OWNER_NONE, data->drag_fingers, 1);
				data->second_tap = false;
				data->drag_start_pending = false;
				data->drag_claim_authorized = false;
			}
			if (established_contact_moved) {
				/* Do not erase a real swipe merely because its final finger became
				 * visible in the same report. */
				data->moved = true;
			} else {
				data->start_x = data->x;
				data->start_y = data->y;
				data->started_ms = now;
				data->multifinger_settle_until_ms = now + cfg->multifinger_settle_ms;
			}
			/* The completed multi-finger contact owns tap classification from
			 * here. Movement while only the first finger was present must not
			 * reject a short, stationary 2F/3F tap.
			 * 完成した複数指接触をここからタップ判定の主体とする。最初の1本指だけの
			 * 間の移動によって、その後の短く静止した2本/3本指タップを棄却しない。 */
			if (!established_contact_moved) {
				data->moved = false;
			}
		}
		/* IQS9150 may reveal contacts across adjacent reports. Recheck while the
		 * second touch is still stationary, rather than relying only on its first
		 * report. This is essential for a simultaneous 2F/3F double tap. */
		if (!data->second_tap && !data->button_down && data->max_fingers == data->fingers &&
		    matches_first_tap(data, cfg, now)) {
			start_double_tap_candidate(data);
		}
		if (data->fingers < data->max_fingers && !data->button_down) {
			/* A multi-finger release is not electrically simultaneous: the EV02
			 * can report the remaining finger for one or more frames before it
			 * reports zero contacts. Its centroid is necessarily displaced from
			 * the full-contact centroid, so it must not turn an otherwise still
			 * 2F/3F tap into a moved gesture. Movement is evaluated only while
			 * all fingers that formed the tap are still present.
			 *
			 * 複数指の離脱は電気的には同時でなく、EV02が全指離脱の前に残った
			 * 1本を数フレーム報告することがある。その重心は全指時と必ず異なるため、
			 * 静止した2本/3本指タップを移動扱いにしてはならない。タップ中の移動は、
			 * タップを構成した全指が揃っているフレームだけで判定する。 */
		} else if (data->fingers >= 2 && now < data->multifinger_settle_until_ms &&
			   !established_contact_moved) {
			/* Contact coordinates settle as the second/third finger becomes
			 * valid. Keep rebasing during this short interval rather than
			 * interpreting sensor settling as a user scroll.
			 * 2本目/3本目が有効化される間は座標が安定する。短時間は重心を
			 * 再基準化し、センサーの安定化をユーザーのスクロールと誤認しない。 */
			data->start_x = data->x;
			data->start_y = data->y;
		} else if (established_contact_moved ||
			   absolute_int((int)data->x - data->start_x) > cfg->tap_max_move ||
			   absolute_int((int)data->y - data->start_y) > cfg->tap_max_move) {
			data->moved = true;
		}
		if (data->second_tap && !data->button_down && data->drag_claim_authorized &&
		    !data->drag_motion_active &&
		    (absolute_int((int)data->x - data->start_x) >= cfg->drag_start_move ||
		     absolute_int((int)data->y - data->start_y) >= cfg->drag_start_move)) {
			start_drag_motion(data);
		}
		if (data->button_down && data->drag_motion_active &&
		    data->fingers == data->drag_fingers) {
			if (data->drag_position_valid) {
				report_drag_motion(data, (int16_t)((int)data->x - data->drag_previous_x),
						   (int16_t)((int)data->y - data->drag_previous_y));
			}
			data->drag_previous_x = data->x;
			data->drag_previous_y = data->y;
			data->drag_position_valid = true;
		} else if (data->button_down && data->drag_motion_active) {
			/* Do not convert a partial 2F/3F release into a centroid jump. The
			 * next complete frame is used only to rebase drag motion. */
			data->drag_position_valid = false;
		}
	} else if (data->previous_fingers != 0) {
		/* Terminal clicks are not emitted from the raw callback. Wait for the
		 * arbiter's complete-frame callback, which can mint an exact-once grant
		 * only after all competing primary proposals have resolved.
		 * terminal clickは生callbackから出さない。全競合primary proposalの裁定後に
		 * だけ一回限りgrantを発行できる、arbiterのcomplete-frame callbackを待つ。 */
		data->terminal_pending = true;
		data->terminal_frame_seq = data->arbiter_frame_seq;
		data->terminal_timestamp_ms = now;
	}
	if (data->fingers != 0) {
		snapshot_contact_slots(data);
	}
	data->previous_fingers = data->fingers;
}

static void tap_drag_deliver_resolved_frame(
	void *context, const struct travelers_pad_session_snapshot *snapshot,
	const struct travelers_pad_owner_token *token)
{
	struct tap_drag_data *data = context;
	const struct tap_drag_config *cfg = data->dev->config;

	if (!data->terminal_pending || snapshot->frame_seq != data->terminal_frame_seq) {
		return;
	}
	data->terminal_pending = false;
	if (snapshot->cancelled || snapshot->contact_count != 0) {
		return;
	}
	finish_contact(data->dev, cfg, data->terminal_timestamp_ms, snapshot->frame_seq, token);
}

static void tap_drag_handle_input(const struct device *dev, struct input_event *evt)
{
	struct tap_drag_data *data = dev->data;

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
		} else if (evt->code == INPUT_ABS_X) {
			data->x = evt->value;
		} else if (evt->code == INPUT_ABS_Y) {
			data->y = evt->value;
		} else if (evt->code == INPUT_ABS_Z) {
			data->fingers = evt->value;
		} else if (evt->code == INPUT_ABS_RZ && evt->value) {
			data->sensor_reset = true;
		}
	}
	if (evt->sync) {
		data->arbiter_frame_seq =
			travelers_pad_arbiter_next_frame(TRAVELERS_PAD_RECOGNIZER_TAP_DRAG);
		process_frame(dev);
		travelers_pad_arbiter_observed(TRAVELERS_PAD_RECOGNIZER_TAP_DRAG,
					       data->arbiter_frame_seq);
	}
}

static int tap_drag_init(const struct device *dev)
{
	struct tap_drag_data *data = dev->data;
	const struct tap_drag_config *cfg = dev->config;

	k_work_init_delayable(&data->deferred_click_work, deferred_click_work);
	(void)travelers_pad_arbiter_register(TRAVELERS_PAD_RECOGNIZER_TAP_DRAG);
	(void)travelers_pad_arbiter_bind_resolve_handler(
		TRAVELERS_PAD_RECOGNIZER_TAP_DRAG, tap_drag_resolve_action, data);
	(void)travelers_pad_arbiter_bind_frame_handler(
		TRAVELERS_PAD_RECOGNIZER_TAP_DRAG, tap_drag_deliver_resolved_frame, data);
	LOG_INF("tap/drag active: tap=%ums move=%u settle=%ums double-gap=%ums",
		cfg->tap_max_ms, cfg->tap_max_move, cfg->multifinger_settle_ms,
		cfg->double_tap_max_gap_ms);
	return 0;
}

#define TAP_DRAG_INST(n)                                                                      \
	static struct tap_drag_data tap_drag_data_##n = {.dev = DEVICE_DT_INST_GET(n)};            \
	static const struct tap_drag_config tap_drag_config_##n = {                                \
		.tap_max_ms = DT_INST_PROP_OR(n, tap_max_ms, 180),                                    \
		.tap_max_move = DT_INST_PROP_OR(n, tap_max_move, 75),                                \
		.multifinger_settle_ms = DT_INST_PROP_OR(n, multifinger_settle_ms, 45),              \
		.double_tap_max_gap_ms = DT_INST_PROP_OR(n, double_tap_max_gap_ms, 250),             \
		.double_tap_max_distance = DT_INST_PROP_OR(n, double_tap_max_distance, 120),          \
		.drag_start_move = DT_INST_PROP_OR(n, drag_start_move, 8),                           \
	};                                                                                           \
	static void tap_drag_input_handler_##n(struct input_event *evt)                            \
	{                                                                                            \
		tap_drag_handle_input(DEVICE_DT_INST_GET(n), evt);                                   \
	}                                                                                            \
	INPUT_CALLBACK_DEFINE(DEVICE_DT_GET(DT_INST_PHANDLE(n, input_device)),                     \
			      tap_drag_input_handler_##n);                                                 \
	DEVICE_DT_INST_DEFINE(n, tap_drag_init, NULL, &tap_drag_data_##n, &tap_drag_config_##n, POST_KERNEL, \
			      CONFIG_TRAVELERS_PAD_IQS9150_FUNCTION_INIT_PRIORITY, NULL);

DT_INST_FOREACH_STATUS_OKAY(TAP_DRAG_INST)
