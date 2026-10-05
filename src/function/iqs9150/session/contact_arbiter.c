/*
 * Contact arbitration core / 接触仲裁core.
 *
 * Accept proposals emitted for one logical input frame and resolve them
 * independently of callback order. Zephyr queues input callbacks, so
 * resolution waits until every registered primary recognizer acknowledges that frame.
 * A frame-routing callback runs only after that complete proposal set has been
 * resolved. Primary semantic output is token-gated; the session manager owns
 * only the immutable lifecycle boundary, never contact ownership.
 *
 * 一つの論理input frameに対して出たproposalを収集し、callback順に依存せず解決する。
 * Zephyr input callbackはqueue方式のため、全registered primary recognizerのframe通知を待つ。
 * その完全なproposal集合を解決後にだけframe-routing callbackを呼ぶ。primary semantic
 * outputはtoken gateし、session managerは不変lifecycle境界だけを保持して接触所有権を持たない。
 */

#include <zephyr/logging/log.h>
#include <zephyr/spinlock.h>
#include <zephyr/sys/util.h>

#include <travelers_pad/contact_arbiter.h>

LOG_MODULE_REGISTER(travelers_pad_arbiter, LOG_LEVEL_INF);

#define TRAVELERS_PAD_ARBITER_MAX_PROPOSALS 8
#define TRAVELERS_PAD_ARBITER_MAX_IN_FLIGHT_FRAMES 16
#define TRAVELERS_PAD_ARBITER_MAX_OUTPUT_RECEIPTS 4
#define TRAVELERS_PAD_ARBITER_MAX_DECISIONS 16
#define TRAVELERS_PAD_ARBITER_MAX_TERMINAL_ACTIONS 8
#define TRAVELERS_PAD_ARBITER_MAX_ROUTED_OUTPUT_PERMITS 16

struct arbiter_output_receipt {
	enum travelers_pad_recognizer_id recognizer;
	enum travelers_pad_owner requested_owner;
	uint32_t capability;
	const char *producer;
};

struct arbiter_decision {
	struct travelers_pad_owner_token token;
	uint32_t frame_seq;
	enum travelers_pad_recognizer_id recognizer;
	bool valid;
	bool output_verified;
	bool authoritative_output_verified;
};

struct terminal_action_record {
	struct travelers_pad_terminal_action_grant grant;
	bool consumed;
};

struct routed_output_permit {
	uint32_t frame_seq;
	uint32_t session_id;
	uint32_t owner_generation;
	uint32_t lifecycle_epoch;
	enum travelers_pad_owner owner;
	uint32_t capabilities;
	bool valid;
};

struct arbiter_frame {
	struct travelers_pad_session_snapshot snapshot;
	uint32_t lifecycle_epoch;
	struct travelers_pad_arbiter_proposal proposals[TRAVELERS_PAD_ARBITER_MAX_PROPOSALS];
	struct arbiter_output_receipt output_receipts[TRAVELERS_PAD_ARBITER_MAX_OUTPUT_RECEIPTS];
	uint32_t observed_mask;
	uint8_t proposal_count;
	uint8_t output_receipt_count;
	bool active;
	/* Primary recognizers may decide the frame before asynchronous observers
	 * consume its immutable snapshot. Keep storage alive until every registered
	 * input listener has acknowledged it. / primary recognizerはobserverより先に
	 * frameを裁定できる。全登録input listenerがacknowledgeするまで不変snapshotを保持する。 */
	bool resolved;
	bool terminal_action_open;
	/* A handler must receive a resolved frame only once. Primary handlers run
	 * at primary resolution; observer handlers wait until their own raw input
	 * callback has acknowledged that frame. / handlerは裁定済みframeを一度だけ受ける。
	 * primary handlerはprimary裁定時、observer handlerは自身の生input callbackが同frameを
	 * acknowledgeしてから呼ばれる。 */
	uint32_t frame_handler_mask;
};

struct arbiter_state {
	struct arbiter_frame frames[TRAVELERS_PAD_ARBITER_MAX_IN_FLIGHT_FRAMES];
	struct arbiter_decision decisions[TRAVELERS_PAD_ARBITER_MAX_DECISIONS];
	struct terminal_action_record terminal_actions[TRAVELERS_PAD_ARBITER_MAX_TERMINAL_ACTIONS];
	struct routed_output_permit routed_output_permits[
		TRAVELERS_PAD_ARBITER_MAX_ROUTED_OUTPUT_PERMITS];
	struct travelers_pad_owner_token token;
	struct travelers_pad_owner_token post_session;
	struct {
		uint32_t session_id;
		enum travelers_pad_owner owner;
	} ending;
	uint32_t cancelled_session_id;
	uint32_t next_owner_generation;
	uint32_t lifecycle_epoch;
	uint32_t registered_mask;
	uint32_t primary_registered_mask;
	struct {
		travelers_pad_arbiter_resolve_handler_t handler;
		void *context;
	} handlers[TRAVELERS_PAD_RECOGNIZER_COUNT];
	struct {
		travelers_pad_arbiter_frame_handler_t handler;
		void *context;
	} frame_handlers[TRAVELERS_PAD_RECOGNIZER_COUNT];
	uint8_t next_decision;
	uint8_t next_terminal_action;
	uint8_t next_routed_output_permit;
	uint32_t next_terminal_action_generation;
};

static struct arbiter_state state;
/* Input callbacks resolve frames in FIFO order, while delayable work can ask
 * for token/grant authority concurrently. Guard every authority transition and
 * authority check so a cancel cannot interleave with a successful check. */
static struct k_spinlock state_lock;

static bool is_primary_recognizer(enum travelers_pad_recognizer_id recognizer)
{
	switch (recognizer) {
	case TRAVELERS_PAD_RECOGNIZER_ROUTER:
	case TRAVELERS_PAD_RECOGNIZER_TAP_DRAG:
	case TRAVELERS_PAD_RECOGNIZER_MULTIFINGER_GESTURE:
	case TRAVELERS_PAD_RECOGNIZER_EDGE_DIAL:
	case TRAVELERS_PAD_RECOGNIZER_TIP_TAP:
	case TRAVELERS_PAD_RECOGNIZER_TRANSFORM_GESTURE:
		return true;
	case TRAVELERS_PAD_RECOGNIZER_SCROLL_INERTIA:
	case TRAVELERS_PAD_RECOGNIZER_EDGE_HOLD:
	case TRAVELERS_PAD_RECOGNIZER_COUNT:
	default:
		return false;
	}
}

static const char *owner_name(enum travelers_pad_owner owner)
{
	switch (owner) {
	case TRAVELERS_PAD_OWNER_DRAG:
		return "drag";
	case TRAVELERS_PAD_OWNER_SCROLL:
		return "scroll";
	case TRAVELERS_PAD_OWNER_TRANSFORM_GESTURE:
		return "transform";
	case TRAVELERS_PAD_OWNER_MULTI_GESTURE:
		return "gesture";
	case TRAVELERS_PAD_OWNER_EDGE_DIAL:
		return "edge-dial";
	case TRAVELERS_PAD_OWNER_TIP_TAP:
		return "tip-tap";
	case TRAVELERS_PAD_OWNER_NONE:
	default:
		return "none";
	}
}

/* This table declares policy, rather than accidentally inheriting the order in
 * which Zephyr input callbacks were linked. Higher values win a same-frame
 * conflict. It is authoritative for all contact ownership and primary output.
 * この表はZephyr input callbackのリンク順ではなく方針を明示する。同一frameの
 * 競合では大きい値が勝つ。全接触所有権およびprimary出力に対して正である。 */
static uint8_t owner_priority(enum travelers_pad_owner owner)
{
	switch (owner) {
	case TRAVELERS_PAD_OWNER_EDGE_DIAL:
		return 100;
	case TRAVELERS_PAD_OWNER_DRAG:
		return 90;
	case TRAVELERS_PAD_OWNER_TRANSFORM_GESTURE:
		return 85;
	case TRAVELERS_PAD_OWNER_MULTI_GESTURE:
		return 80;
	case TRAVELERS_PAD_OWNER_SCROLL:
		return 70;
	case TRAVELERS_PAD_OWNER_TIP_TAP:
		return 60;
	case TRAVELERS_PAD_OWNER_NONE:
	default:
		return 0;
	}
}

static bool token_has_capabilities(const struct travelers_pad_owner_token *token,
				   enum travelers_pad_owner owner, uint32_t capability)
{
	return token != NULL && token->valid && token->owner == owner &&
	       (token->capabilities & capability) == capability;
}

static void invalidate_decisions_for_session(uint32_t session_id)
{
	for (uint8_t i = 0; i < ARRAY_SIZE(state.decisions); i++) {
		if (state.decisions[i].valid && state.decisions[i].token.session_id == session_id) {
			state.decisions[i].token.valid = false;
			state.decisions[i].valid = false;
		}
	}
}

static void invalidate_routed_output_permits_for_session(uint32_t session_id)
{
	for (uint8_t i = 0; i < ARRAY_SIZE(state.routed_output_permits); i++) {
		if (state.routed_output_permits[i].valid &&
		    state.routed_output_permits[i].session_id == session_id) {
			state.routed_output_permits[i].valid = false;
		}
	}
}

static void revoke_session(uint32_t session_id, const char *reason)
{
	bool revoked = false;

	if (state.token.valid && state.token.session_id == session_id) {
		state.token.valid = false;
		revoked = true;
	}
	if (state.post_session.valid && state.post_session.session_id == session_id) {
		state.post_session.valid = false;
		revoked = true;
	}
	invalidate_decisions_for_session(session_id);
	invalidate_routed_output_permits_for_session(session_id);
	if (revoked) {
		LOG_INF("arbiter session=%u leases revoked reason=%s", session_id, reason);
	}
}

void travelers_pad_arbiter_cancel_all(const char *reason)
{
	k_spinlock_key_t key = k_spin_lock(&state_lock);
	uint32_t epoch;
	/* This is intentionally independent of session_id. A runtime sensor reset
	 * can arrive after the terminal snapshot has cleared, while inertia, a
	 * terminal click, or an edge-hold work item is still pending. */
	state.lifecycle_epoch++;
	if (state.lifecycle_epoch == 0U) {
		state.lifecycle_epoch = 1U;
	}
	state.token.valid = false;
	state.post_session.valid = false;
	state.ending.session_id = 0;
	state.ending.owner = TRAVELERS_PAD_OWNER_NONE;
	for (uint8_t index = 0; index < ARRAY_SIZE(state.decisions); index++) {
		state.decisions[index].valid = false;
		state.decisions[index].token.valid = false;
	}
	for (uint8_t index = 0; index < ARRAY_SIZE(state.terminal_actions); index++) {
		state.terminal_actions[index].grant.valid = false;
		state.terminal_actions[index].consumed = true;
	}
	for (uint8_t index = 0; index < ARRAY_SIZE(state.routed_output_permits); index++) {
		state.routed_output_permits[index].valid = false;
	}
	epoch = state.lifecycle_epoch;
	k_spin_unlock(&state_lock, key);
	LOG_WRN("arbiter global cancel epoch=%u reason=%s", epoch,
		reason == NULL ? "unspecified" : reason);
}

bool travelers_pad_arbiter_register(enum travelers_pad_recognizer_id recognizer)
{
	uint32_t bit;

	if (recognizer >= TRAVELERS_PAD_RECOGNIZER_COUNT) {
		return false;
	}
	bit = 1U << recognizer;
	if (state.registered_mask & bit) {
		LOG_WRN("arbiter recognizer=%u registered twice", recognizer);
		return false;
	}
	state.registered_mask |= bit;
	if (is_primary_recognizer(recognizer)) {
		state.primary_registered_mask |= bit;
		LOG_INF("arbiter primary recognizer=%u registered", recognizer);
	} else {
		LOG_INF("arbiter observer=%u registered", recognizer);
	}
	return true;
}

bool travelers_pad_arbiter_bind_resolve_handler(
	enum travelers_pad_recognizer_id recognizer,
	travelers_pad_arbiter_resolve_handler_t handler, void *context)
{
	if (recognizer >= TRAVELERS_PAD_RECOGNIZER_COUNT || handler == NULL ||
	    (state.registered_mask & (1U << recognizer)) == 0U ||
	    state.handlers[recognizer].handler != NULL) {
		LOG_WRN("resolver handler rejected recognizer=%u", recognizer);
		return false;
	}
	state.handlers[recognizer].handler = handler;
	state.handlers[recognizer].context = context;
	LOG_INF("resolver handler bound recognizer=%u", recognizer);
	return true;
}

bool travelers_pad_arbiter_bind_frame_handler(
	enum travelers_pad_recognizer_id recognizer,
	travelers_pad_arbiter_frame_handler_t handler, void *context)
{
	if (recognizer >= TRAVELERS_PAD_RECOGNIZER_COUNT || handler == NULL ||
	    (state.registered_mask & (1U << recognizer)) == 0U ||
	    state.frame_handlers[recognizer].handler != NULL) {
		LOG_WRN("frame handler rejected recognizer=%u", recognizer);
		return false;
	}
	state.frame_handlers[recognizer].handler = handler;
	state.frame_handlers[recognizer].context = context;
	LOG_INF("frame handler bound recognizer=%u", recognizer);
	return true;
}

static struct arbiter_frame *frame_for(uint32_t frame_seq)
{
	for (uint8_t i = 0; i < ARRAY_SIZE(state.frames); i++) {
		if (state.frames[i].active && state.frames[i].snapshot.frame_seq == frame_seq) {
			return &state.frames[i];
		}
	}
	return NULL;
}

static struct arbiter_decision *decision_for(uint32_t frame_seq)
{
	for (uint8_t i = 0; i < ARRAY_SIZE(state.decisions); i++) {
		if (state.decisions[i].valid && state.decisions[i].frame_seq == frame_seq) {
			return &state.decisions[i];
		}
	}
	return NULL;
}

static bool proposal_wins_over(enum travelers_pad_owner candidate_owner,
			       enum travelers_pad_recognizer_id candidate_recognizer,
			       enum travelers_pad_owner current_owner,
			       enum travelers_pad_recognizer_id current_recognizer)
{
	return owner_priority(candidate_owner) > owner_priority(current_owner) ||
	       (owner_priority(candidate_owner) == owner_priority(current_owner) &&
		candidate_recognizer < current_recognizer);
}

void travelers_pad_arbiter_begin_frame(const struct travelers_pad_session_snapshot *snapshot)
{
	struct arbiter_frame *frame = NULL;
	k_spinlock_key_t key;

	if (snapshot == NULL) {
		return;
	}
	/* A cancellation is authoritative even when the session manager has already
	 * cleared the active snapshot and consequently reports session_id == 0. */
	if (snapshot->cancelled) {
		travelers_pad_arbiter_cancel_all("sensor-reset-frame");
	}
	if (snapshot->session_id == 0) {
		return;
	}
	key = k_spin_lock(&state_lock);
	/* Cancellation is a lifecycle boundary, not an ordinary zero-contact
	 * release. Revoke every authority for the affected session before queued
	 * recognizers observe the cancellation frame, so delayed work can never
	 * turn a reset into post-session output.
	 * cancellationは通常のzero-contact releaseではなくlifecycle境界である。
	 * queued recognizerがcancel frameを観測する前に当該sessionの全権限を失効し、
	 * 遅延workがresetをpost-session出力へ変換できないようにする。 */
	if (snapshot->cancelled) {
		state.cancelled_session_id = snapshot->session_id;
		revoke_session(snapshot->session_id, "cancel");
		state.ending.session_id = 0;
		state.ending.owner = TRAVELERS_PAD_OWNER_NONE;
	} else if (snapshot->contact_count != 0 && state.post_session.valid &&
		   state.post_session.session_id != snapshot->session_id) {
		/* Any new physical contact revokes the preceding inertial lease. */
		revoke_session(state.post_session.session_id, "new-contact");
	}
	for (uint8_t i = 0; i < ARRAY_SIZE(state.frames); i++) {
		if (!state.frames[i].active) {
			frame = &state.frames[i];
			break;
		}
	}
	if (frame == NULL) {
		k_spin_unlock(&state_lock, key);
		LOG_ERR("arbiter frame queue overflow seq=%u", snapshot->frame_seq);
		return;
	}
	*frame = (struct arbiter_frame){
		.snapshot = *snapshot,
		.lifecycle_epoch = state.lifecycle_epoch,
		.active = true,
	};
	k_spin_unlock(&state_lock, key);
}

static bool frame_accepts_recognizer(const struct arbiter_frame *frame,
					  enum travelers_pad_recognizer_id recognizer)
{
	return frame != NULL && (state.registered_mask & (1U << recognizer)) != 0U;
}

uint32_t travelers_pad_arbiter_next_frame(enum travelers_pad_recognizer_id recognizer)
{
	const struct arbiter_frame *next = NULL;
	uint32_t bit;
	k_spinlock_key_t key;

	if (recognizer >= TRAVELERS_PAD_RECOGNIZER_COUNT) {
		return 0;
	}
	bit = 1U << recognizer;
	key = k_spin_lock(&state_lock);
	if ((state.registered_mask & bit) == 0U) {
		k_spin_unlock(&state_lock, key);
		return 0;
	}
	for (uint8_t i = 0; i < ARRAY_SIZE(state.frames); i++) {
		const struct arbiter_frame *frame = &state.frames[i];

		if (!frame->active || (frame->observed_mask & bit) != 0U) {
			continue;
		}
		if (next == NULL || frame->snapshot.frame_seq < next->snapshot.frame_seq) {
			next = frame;
		}
	}
	uint32_t frame_seq = next == NULL ? 0 : next->snapshot.frame_seq;
	k_spin_unlock(&state_lock, key);
	return frame_seq;
}

bool travelers_pad_arbiter_submit(const struct travelers_pad_arbiter_proposal *proposal)
{
	struct arbiter_frame *frame;
	k_spinlock_key_t key;

	if (proposal == NULL ||
	    proposal->recognizer >= TRAVELERS_PAD_RECOGNIZER_COUNT ||
	    proposal->requested_owner == TRAVELERS_PAD_OWNER_NONE) {
		return false;
	}
	key = k_spin_lock(&state_lock);
	frame = frame_for(proposal->frame_seq);
	if (!frame_accepts_recognizer(frame, proposal->recognizer)) {
		k_spin_unlock(&state_lock, key);
		LOG_WRN("arbiter proposal from unregistered recognizer=%u", proposal->recognizer);
		return false;
	}
	if (frame->proposal_count >= ARRAY_SIZE(frame->proposals)) {
		k_spin_unlock(&state_lock, key);
		LOG_ERR("arbiter proposal queue overflow session=%u", frame->snapshot.session_id);
		return false;
	}
	frame->proposals[frame->proposal_count++] = *proposal;
	k_spin_unlock(&state_lock, key);
	return true;
}

static const struct travelers_pad_arbiter_proposal *select_winner(const struct arbiter_frame *frame)
{
	const struct travelers_pad_arbiter_proposal *winner = NULL;

	for (uint8_t i = 0; i < frame->proposal_count; i++) {
		const struct travelers_pad_arbiter_proposal *proposal = &frame->proposals[i];

		if (proposal->intent != TRAVELERS_PAD_PROPOSAL_CLAIM) {
			continue;
		}
		if (winner == NULL || proposal_wins_over(proposal->requested_owner,
							 proposal->recognizer,
							 winner->requested_owner,
							 winner->recognizer)) {
			/* The final recognizer-ID comparison is deterministic only. Primary
			 * claims are intentionally assigned distinct policy priorities. */
			winner = proposal;
		}
	}
	return winner;
}

static bool decision_allows(const struct travelers_pad_owner_token *token,
				    enum travelers_pad_recognizer_id winner,
				    const struct arbiter_output_receipt *receipt)
{
	return token != NULL && token->valid && winner == receipt->recognizer &&
	       token->owner == receipt->requested_owner &&
	       (token->capabilities & receipt->capability) == receipt->capability;
}

static void validate_receipts(const struct arbiter_frame *frame,
			      enum travelers_pad_recognizer_id winner)
{
	bool accepted = true;

	for (uint8_t i = 0; i < frame->output_receipt_count; i++) {
		const struct arbiter_output_receipt *receipt = &frame->output_receipts[i];

		if (!decision_allows(&state.token, winner, receipt)) {
			accepted = false;
			LOG_ERR("terminal output record would reject producer=%s session=%u frame=%u owner=%s caps=0x%x",
				receipt->producer, frame->snapshot.session_id, frame->snapshot.frame_seq,
				owner_name(receipt->requested_owner), receipt->capability);
		}
	}
	if (frame->output_receipt_count != 0) {
		LOG_INF("terminal output record preflight session=%u frame=%u winner=%s outputs=%u %s",
			frame->snapshot.session_id, frame->snapshot.frame_seq,
			owner_name(state.token.owner), frame->output_receipt_count,
			accepted ? "accepted" : "rejected");
	}
}

static void dispatch_frame_handler(struct arbiter_frame *frame,
				   enum travelers_pad_recognizer_id recognizer)
{
	struct travelers_pad_owner_token token_copy;
	const struct travelers_pad_owner_token *token = NULL;
	travelers_pad_arbiter_frame_handler_t handler;
	void *context;
	struct travelers_pad_session_snapshot snapshot;
	uint32_t bit;

	if (recognizer >= TRAVELERS_PAD_RECOGNIZER_COUNT) {
		return;
	}
	k_spinlock_key_t key = k_spin_lock(&state_lock);
	bit = 1U << recognizer;
	if ((frame->frame_handler_mask & bit) != 0U ||
	    state.frame_handlers[recognizer].handler == NULL) {
		k_spin_unlock(&state_lock, key);
		return;
	}
	if (state.token.valid && state.token.session_id == frame->snapshot.session_id) {
		token_copy = state.token;
		token = &token_copy;
	}
	frame->frame_handler_mask |= bit;
	handler = state.frame_handlers[recognizer].handler;
	context = state.frame_handlers[recognizer].context;
	snapshot = frame->snapshot;
	k_spin_unlock(&state_lock, key);
	handler(context, &snapshot, token);
}

static void dispatch_frame_handlers(struct arbiter_frame *frame)
{
	/* Primary handlers are intentionally invoked immediately after the final
	 * same-frame decision. This makes router forwarding independent from
	 * callback order. Observers which have already consumed the raw frame can
	 * run now too; later observers are dispatched by arbiter_observed().
	 * primary handlerは同一frameの最終裁定直後に呼ぶため、router転送はcallback順に
	 * 依存しない。すでに生frameを消費済みのobserverもここで実行し、後続observerは
	 * arbiter_observed()からdispatchする。 */
	for (uint8_t recognizer = 0; recognizer < TRAVELERS_PAD_RECOGNIZER_COUNT;
	     recognizer++) {
		uint32_t bit = 1U << recognizer;
		bool dispatch;
		k_spinlock_key_t key = k_spin_lock(&state_lock);

		dispatch = is_primary_recognizer(recognizer) || (frame->observed_mask & bit) != 0U;
		k_spin_unlock(&state_lock, key);
		if (dispatch) {
			dispatch_frame_handler(frame, recognizer);
		}
	}
}

static void resolve_frame(struct arbiter_frame *frame)
{
	const struct travelers_pad_arbiter_proposal *winner;
	bool new_token = false;
	bool terminal_scroll = false;
	uint32_t terminal_session_id;
	travelers_pad_arbiter_resolve_handler_t resolve_handler = NULL;
	void *resolve_context = NULL;
	struct travelers_pad_owner_token resolved_token = {};
	k_spinlock_key_t key = k_spin_lock(&state_lock);

	if (frame->lifecycle_epoch != state.lifecycle_epoch || frame->snapshot.cancelled ||
	    state.cancelled_session_id == frame->snapshot.session_id) {
		/* Claims arriving from an older queued frame may never resurrect an
		 * owner after this session has been cancelled. */
		revoke_session(frame->snapshot.session_id, "cancelled-frame");
		k_spin_unlock(&state_lock, key);
		dispatch_frame_handlers(frame);
		return;
	}

	/* A new resolved session invalidates any completed session token. The
	 * asynchronous queue can hold several frames, so this must happen at
	 * resolution time rather than when the producer enqueues a frame. */
	if (state.token.valid && state.token.session_id != frame->snapshot.session_id) {
		state.token.valid = false;
	}
	winner = select_winner(frame);
	if (winner != NULL && !state.token.valid) {
		state.token = (struct travelers_pad_owner_token){
				.session_id = frame->snapshot.session_id,
				.owner_generation = ++state.next_owner_generation,
				.lifecycle_epoch = state.lifecycle_epoch,
			.owner = winner->requested_owner,
			.capabilities = winner->capabilities,
			.kind = TRAVELERS_PAD_TOKEN_COMMITTED,
			.phase = winner->requested_owner == TRAVELERS_PAD_OWNER_DRAG ?
				TRAVELERS_PAD_OWNER_PHASE_RESERVED : TRAVELERS_PAD_OWNER_PHASE_ACTIVE,
			.valid = true,
		};
		state.decisions[state.next_decision++ % ARRAY_SIZE(state.decisions)] =
			(struct arbiter_decision){
				.token = state.token,
				.frame_seq = frame->snapshot.frame_seq,
				.recognizer = winner->recognizer,
				.valid = true,
			};
		new_token = true;
	}

	if (new_token) {
		resolved_token = state.token;
		resolve_handler = state.handlers[winner->recognizer].handler;
		resolve_context = state.handlers[winner->recognizer].context;
		LOG_INF("arbiter session=%u frame=%u winner=%s recognizer=%u caps=0x%x",
			frame->snapshot.session_id, frame->snapshot.frame_seq,
			owner_name(state.token.owner), winner->recognizer, state.token.capabilities);
		/* A first semantic action was staged by the winning recognizer while this
		 * frame was unresolved. Dispatch it only after the immutable decision is
		 * in the ledger. / 未解決frame中に勝者recognizerが保留した初回semantic
		 * actionを、不変decisionをledgerへ記録した後だけdispatchする。 */
	}
	if (frame->output_receipt_count != 0) {
		validate_receipts(frame, winner == NULL ? TRAVELERS_PAD_RECOGNIZER_COUNT :
						       winner->recognizer);
	}
	/* Terminal grants are minted only during complete-frame delivery, while the
	 * final owner token (if any) is still live. */
	frame->terminal_action_open = frame->snapshot.contact_count == 0;
	k_spin_unlock(&state_lock, key);
	if (resolve_handler != NULL) {
		resolve_handler(resolve_context, &resolved_token, frame->snapshot.frame_seq);
	}
	dispatch_frame_handlers(frame);
	key = k_spin_lock(&state_lock);
	frame->terminal_action_open = false;

	/* A terminal frame may still carry a proposal or terminal release record.
	 * Preserve the result until validation is complete, then invalidate it so no later
	 * work can mistake it for a live-session token. */
	if (frame->lifecycle_epoch == state.lifecycle_epoch && frame->snapshot.contact_count == 0) {
		terminal_session_id = frame->snapshot.session_id;
		state.ending.session_id = frame->snapshot.session_id;
		state.ending.owner = state.token.valid ? state.token.owner : TRAVELERS_PAD_OWNER_NONE;
		terminal_scroll = state.token.valid &&
			state.token.owner == TRAVELERS_PAD_OWNER_SCROLL && !frame->snapshot.cancelled;
		if (terminal_scroll) {
			state.post_session = (struct travelers_pad_owner_token){
					.session_id = terminal_session_id,
					.owner_generation = ++state.next_owner_generation,
					.lifecycle_epoch = state.lifecycle_epoch,
				.owner = TRAVELERS_PAD_OWNER_SCROLL,
				.capabilities = TRAVELERS_PAD_CAP_SCROLL_DELTA,
				.kind = TRAVELERS_PAD_TOKEN_POST_SESSION,
				.phase = TRAVELERS_PAD_OWNER_PHASE_ACTIVE,
				.valid = true,
			};
			LOG_INF("arbiter post-session lease session=%u generation=%u owner=scroll",
				terminal_session_id, state.post_session.owner_generation);
		}
		state.token.valid = false;
		invalidate_decisions_for_session(terminal_session_id);
		invalidate_routed_output_permits_for_session(terminal_session_id);
	}
	k_spin_unlock(&state_lock, key);
}

enum travelers_pad_owner travelers_pad_arbiter_owner_for_frame(uint32_t frame_seq)
{
	const struct arbiter_frame *frame = frame_for(frame_seq);
	uint32_t session_id;

	if (frame == NULL) {
		return TRAVELERS_PAD_OWNER_NONE;
	}
	session_id = frame->snapshot.session_id;
	if (state.token.valid && state.token.session_id == session_id) {
		return state.token.owner;
	}
	if (frame->snapshot.contact_count == 0 && state.ending.session_id == session_id) {
		return state.ending.owner;
	}
	return TRAVELERS_PAD_OWNER_NONE;
}

bool travelers_pad_arbiter_get_frame_snapshot(uint32_t frame_seq,
					       struct travelers_pad_session_snapshot *snapshot)
{
	const struct arbiter_frame *frame = frame_for(frame_seq);

	if (frame == NULL || snapshot == NULL) {
		return false;
	}
	*snapshot = frame->snapshot;
	return true;
}

bool travelers_pad_arbiter_issue_terminal_action(
	uint32_t frame_seq, enum travelers_pad_recognizer_id recognizer,
	enum travelers_pad_owner owner, uint32_t capability,
	struct travelers_pad_terminal_action_grant *grant)
{
	struct arbiter_frame *frame;
	struct terminal_action_record *record;
	bool owner_allowed;
	k_spinlock_key_t key;
	uint32_t session_id;
	uint32_t generation;

	if (grant == NULL || recognizer >= TRAVELERS_PAD_RECOGNIZER_COUNT ||
	    capability == TRAVELERS_PAD_CAP_NONE) {
		return false;
	}
	key = k_spin_lock(&state_lock);
	frame = frame_for(frame_seq);
	if (frame == NULL || !frame->resolved || !frame->terminal_action_open ||
	    frame->snapshot.cancelled || frame->snapshot.contact_count != 0) {
		k_spin_unlock(&state_lock, key);
		return false;
	}
	/* A bare tap may be granted only if no primary recognizer owns the session.
	 * A double-tap candidate is the narrow exception: its own reserved DRAG
	 * token permits its terminal double-click but no other recognizer may use it.
	 * 通常tapはprimary ownerがいない時だけgrantする。唯一の例外は予約済みDRAGで、
	 * 自身のterminal double-clickだけを許可し、他recognizerは利用できない。 */
	if (owner == TRAVELERS_PAD_OWNER_NONE) {
		owner_allowed = !state.token.valid;
	} else {
		owner_allowed = state.token.valid && state.token.session_id == frame->snapshot.session_id &&
			state.token.owner == owner && token_has_capabilities(&state.token, owner, capability);
	}
	if (!owner_allowed) {
		k_spin_unlock(&state_lock, key);
		LOG_WRN("terminal action denied session=%u frame=%u recognizer=%u owner=%s",
			frame->snapshot.session_id, frame_seq, recognizer, owner_name(owner));
		return false;
	}

	record = &state.terminal_actions[
		state.next_terminal_action++ % ARRAY_SIZE(state.terminal_actions)];
	*record = (struct terminal_action_record){
		.grant = {
				.session_id = frame->snapshot.session_id,
				.grant_generation = ++state.next_terminal_action_generation,
				.lifecycle_epoch = state.lifecycle_epoch,
			.recognizer = recognizer,
			.owner = owner,
			.capability = capability,
			.valid = true,
		},
	};
	*grant = record->grant;
	session_id = grant->session_id;
	generation = grant->grant_generation;
	k_spin_unlock(&state_lock, key);
	LOG_INF("terminal action granted session=%u frame=%u generation=%u recognizer=%u owner=%s",
		session_id, frame_seq, generation, recognizer, owner_name(owner));
	return true;
}

bool travelers_pad_arbiter_consume_terminal_action(
	struct travelers_pad_terminal_action_grant *grant,
	enum travelers_pad_recognizer_id recognizer, enum travelers_pad_owner owner,
	uint32_t capability, const char *producer)
{
	k_spinlock_key_t key;
	bool consumed = false;
	uint32_t session_id = 0;
	uint32_t generation = 0;

	if (grant == NULL || !grant->valid || recognizer >= TRAVELERS_PAD_RECOGNIZER_COUNT ||
	    grant->recognizer != recognizer || grant->owner != owner ||
	    (grant->capability & capability) != capability) {
		return false;
	}
	key = k_spin_lock(&state_lock);
	for (uint8_t index = 0; index < ARRAY_SIZE(state.terminal_actions); index++) {
		struct terminal_action_record *record = &state.terminal_actions[index];

		if (!record->grant.valid || record->consumed ||
			    record->grant.grant_generation != grant->grant_generation ||
			    record->grant.lifecycle_epoch != grant->lifecycle_epoch ||
		    record->grant.session_id != grant->session_id ||
		    record->grant.recognizer != recognizer || record->grant.owner != owner ||
		    (record->grant.capability & capability) != capability) {
			continue;
		}
		record->consumed = true;
		grant->valid = false;
		session_id = grant->session_id;
		generation = grant->grant_generation;
		consumed = true;
		break;
	}
	k_spin_unlock(&state_lock, key);
	if (consumed) {
		LOG_INF("terminal action consumed session=%u generation=%u producer=%s",
			session_id, generation, producer);
	}
	return consumed;
}

bool travelers_pad_arbiter_authorize_output(uint32_t ownership_frame_seq,
					    enum travelers_pad_recognizer_id recognizer,
					    enum travelers_pad_owner requested_owner,
					    uint32_t capability, const char *producer)
{
	struct arbiter_decision *decision;
	struct arbiter_output_receipt receipt = {
		.recognizer = recognizer,
		.requested_owner = requested_owner,
		.capability = capability,
		.producer = producer,
	};

	k_spinlock_key_t key = k_spin_lock(&state_lock);
	/* decision_for() above only returns stable in-flight storage; re-read it
	 * while holding the lock before relying on its authority fields. */
	decision = decision_for(ownership_frame_seq);
	if (decision == NULL ||
	    !decision_allows(&decision->token, decision->recognizer, &receipt) ||
	    !state.token.valid || state.token.kind != TRAVELERS_PAD_TOKEN_COMMITTED ||
	    state.token.session_id != decision->token.session_id ||
		    state.token.owner_generation != decision->token.owner_generation ||
		    state.token.lifecycle_epoch != decision->token.lifecycle_epoch ||
		    state.token.lifecycle_epoch != state.lifecycle_epoch) {
		k_spin_unlock(&state_lock, key);
		LOG_ERR("arbiter output rejected producer=%s ownership-frame=%u owner=%s caps=0x%x",
			producer, ownership_frame_seq, owner_name(requested_owner), capability);
		return false;
	}
	bool first_authorization = !decision->authoritative_output_verified;
	if (!decision->authoritative_output_verified) {
		decision->authoritative_output_verified = true;
	}
	k_spin_unlock(&state_lock, key);
	if (first_authorization) {
		LOG_INF("arbiter output authorized ownership-frame=%u winner=%s producer=%s",
			ownership_frame_seq, owner_name(requested_owner), producer);
	}
	return true;
}

bool travelers_pad_arbiter_begin_primary_output(
	uint32_t ownership_frame_seq, enum travelers_pad_recognizer_id recognizer,
	enum travelers_pad_owner requested_owner, uint32_t capability, const char *producer,
	struct travelers_pad_arbiter_output_guard *guard)
{
	struct arbiter_decision *decision;
	struct arbiter_output_receipt receipt = {
		.recognizer = recognizer,
		.requested_owner = requested_owner,
		.capability = capability,
		.producer = producer,
	};
	if (guard == NULL) {
		return false;
	}
	*guard = (struct travelers_pad_arbiter_output_guard){};
	guard->key = k_spin_lock(&state_lock);
	decision = decision_for(ownership_frame_seq);
	if (decision == NULL ||
	    !decision_allows(&decision->token, decision->recognizer, &receipt) ||
	    !state.token.valid || state.token.kind != TRAVELERS_PAD_TOKEN_COMMITTED ||
	    state.token.session_id != decision->token.session_id ||
	    state.token.owner_generation != decision->token.owner_generation ||
	    state.token.lifecycle_epoch != decision->token.lifecycle_epoch ||
	    state.token.lifecycle_epoch != state.lifecycle_epoch) {
		k_spin_unlock(&state_lock, guard->key);
		LOG_ERR("arbiter output rejected producer=%s ownership-frame=%u owner=%s caps=0x%x",
			producer, ownership_frame_seq, owner_name(requested_owner), capability);
		return false;
	}
	decision->authoritative_output_verified = true;
	guard->held = true;
	return true;
}

bool travelers_pad_arbiter_prepare_routed_output(
	uint32_t frame_seq, enum travelers_pad_owner owner, uint32_t capability)
{
	struct arbiter_frame *frame;
	struct routed_output_permit *permit;
	bool authorized;
	k_spinlock_key_t key = k_spin_lock(&state_lock);

	frame = frame_for(frame_seq);
	authorized = frame != NULL && frame->resolved && !frame->snapshot.cancelled &&
		frame->snapshot.contact_count != 0 && frame->lifecycle_epoch == state.lifecycle_epoch;
	if (authorized && owner == TRAVELERS_PAD_OWNER_NONE) {
		authorized = !state.token.valid;
	} else if (authorized) {
		authorized = state.token.valid && state.token.kind == TRAVELERS_PAD_TOKEN_COMMITTED &&
			state.token.session_id == frame->snapshot.session_id &&
			state.token.lifecycle_epoch == state.lifecycle_epoch &&
			token_has_capabilities(&state.token, owner, capability);
	}
	if (!authorized) {
		k_spin_unlock(&state_lock, key);
		return false;
	}
	permit = &state.routed_output_permits[
		state.next_routed_output_permit++ % ARRAY_SIZE(state.routed_output_permits)];
	*permit = (struct routed_output_permit){
		.frame_seq = frame_seq,
		.session_id = frame->snapshot.session_id,
		.owner_generation = owner == TRAVELERS_PAD_OWNER_NONE ? 0 : state.token.owner_generation,
		.lifecycle_epoch = state.lifecycle_epoch,
		.owner = owner,
		.capabilities = capability,
		.valid = true,
	};
	k_spin_unlock(&state_lock, key);
	return true;
}

bool travelers_pad_arbiter_begin_routed_output(
	uint32_t frame_seq, enum travelers_pad_owner owner, uint32_t capability,
	const char *producer, struct travelers_pad_arbiter_output_guard *guard)
{
	struct routed_output_permit *permit = NULL;
	bool authorized = false;

	ARG_UNUSED(producer);
	if (guard == NULL) {
		return false;
	}
	*guard = (struct travelers_pad_arbiter_output_guard){};
	guard->key = k_spin_lock(&state_lock);
	for (uint8_t index = 0; index < ARRAY_SIZE(state.routed_output_permits); index++) {
		if (state.routed_output_permits[index].valid &&
		    state.routed_output_permits[index].frame_seq == frame_seq) {
			permit = &state.routed_output_permits[index];
			break;
		}
	}
	if (permit != NULL && permit->owner == owner &&
	    (permit->capabilities & capability) == capability &&
	    permit->lifecycle_epoch == state.lifecycle_epoch) {
		if (owner == TRAVELERS_PAD_OWNER_NONE) {
			authorized = !state.token.valid && state.ending.session_id != permit->session_id;
		} else {
			authorized = state.token.valid && state.token.kind == TRAVELERS_PAD_TOKEN_COMMITTED &&
			state.token.session_id == permit->session_id &&
			state.token.owner_generation == permit->owner_generation &&
			state.token.lifecycle_epoch == permit->lifecycle_epoch &&
			token_has_capabilities(&state.token, owner, capability);
		}
	}
	if (!authorized) {
		k_spin_unlock(&state_lock, guard->key);
		return false;
	}
	guard->held = true;
	return true;
}

bool travelers_pad_arbiter_acquire_auxiliary_lease(
	enum travelers_pad_auxiliary_id auxiliary, enum travelers_pad_owner required_owner,
	uint32_t capability, struct travelers_pad_auxiliary_lease *lease)
{
	k_spinlock_key_t key = k_spin_lock(&state_lock);
	if (lease == NULL || auxiliary >= TRAVELERS_PAD_AUXILIARY_COUNT ||
	    !state.token.valid || state.token.kind != TRAVELERS_PAD_TOKEN_COMMITTED ||
	    state.token.phase != TRAVELERS_PAD_OWNER_PHASE_ACTIVE ||
	    state.token.lifecycle_epoch != state.lifecycle_epoch ||
	    !token_has_capabilities(&state.token, required_owner, capability)) {
		k_spin_unlock(&state_lock, key);
		return false;
	}
	*lease = (struct travelers_pad_auxiliary_lease){
		.session_id = state.token.session_id,
		.owner_generation = state.token.owner_generation,
		.lifecycle_epoch = state.token.lifecycle_epoch,
		.auxiliary = auxiliary,
		.owner = required_owner,
		.capabilities = capability,
		.valid = true,
	};
	k_spin_unlock(&state_lock, key);
	return true;
}

bool travelers_pad_arbiter_authorize_auxiliary_output(
	const struct travelers_pad_auxiliary_lease *lease,
	enum travelers_pad_auxiliary_id auxiliary, enum travelers_pad_owner required_owner,
	uint32_t capability, const char *producer)
{
	ARG_UNUSED(producer);
	k_spinlock_key_t key = k_spin_lock(&state_lock);
	bool authorized;

	/* Auxiliary output has no independent owner. The only authorization is the
	 * live primary token, so a delayed timer cannot continue after release,
	 * cancellation, or a higher-priority ownership hand-off. */
	authorized = lease != NULL && lease->valid && auxiliary < TRAVELERS_PAD_AUXILIARY_COUNT &&
	       lease->auxiliary == auxiliary && lease->owner == required_owner &&
	       lease->lifecycle_epoch == state.lifecycle_epoch &&
	       (lease->capabilities & capability) == capability && state.token.valid &&
	       state.token.kind == TRAVELERS_PAD_TOKEN_COMMITTED &&
	       state.token.phase == TRAVELERS_PAD_OWNER_PHASE_ACTIVE &&
	       state.token.session_id == lease->session_id &&
	       state.token.owner_generation == lease->owner_generation &&
	       state.token.lifecycle_epoch == lease->lifecycle_epoch &&
	       token_has_capabilities(&state.token, required_owner, capability);
	k_spin_unlock(&state_lock, key);
	return authorized;
}

bool travelers_pad_arbiter_begin_auxiliary_output(
	const struct travelers_pad_auxiliary_lease *lease,
	enum travelers_pad_auxiliary_id auxiliary, enum travelers_pad_owner required_owner,
	uint32_t capability, const char *producer, struct travelers_pad_arbiter_output_guard *guard)
{
	ARG_UNUSED(producer);
	bool authorized;

	if (guard == NULL) {
		return false;
	}
	*guard = (struct travelers_pad_arbiter_output_guard){};
	guard->key = k_spin_lock(&state_lock);
	authorized = lease != NULL && lease->valid && auxiliary < TRAVELERS_PAD_AUXILIARY_COUNT &&
		lease->auxiliary == auxiliary && lease->owner == required_owner &&
		lease->lifecycle_epoch == state.lifecycle_epoch &&
		(lease->capabilities & capability) == capability && state.token.valid &&
		state.token.kind == TRAVELERS_PAD_TOKEN_COMMITTED &&
		state.token.phase == TRAVELERS_PAD_OWNER_PHASE_ACTIVE &&
		state.token.session_id == lease->session_id &&
		state.token.owner_generation == lease->owner_generation &&
		state.token.lifecycle_epoch == lease->lifecycle_epoch &&
		token_has_capabilities(&state.token, required_owner, capability);
	if (!authorized) {
		k_spin_unlock(&state_lock, guard->key);
		return false;
	}
	guard->held = true;
	return true;
}

bool travelers_pad_arbiter_activate_owner(uint32_t ownership_frame_seq,
					  enum travelers_pad_recognizer_id recognizer,
					  enum travelers_pad_owner requested_owner,
					  const char *producer)
{
	struct arbiter_decision *decision;
	struct arbiter_output_receipt receipt = {
		.recognizer = recognizer,
		.requested_owner = requested_owner,
		.capability = TRAVELERS_PAD_CAP_BUTTON,
		.producer = producer,
	};

	k_spinlock_key_t key = k_spin_lock(&state_lock);
	decision = decision_for(ownership_frame_seq);
	if (decision == NULL || !decision_allows(&decision->token, decision->recognizer, &receipt) ||
	    !state.token.valid || state.token.kind != TRAVELERS_PAD_TOKEN_COMMITTED ||
	    state.token.session_id != decision->token.session_id ||
	    state.token.owner_generation != decision->token.owner_generation ||
	    state.token.lifecycle_epoch != decision->token.lifecycle_epoch ||
	    state.token.lifecycle_epoch != state.lifecycle_epoch ||
	    state.token.phase != TRAVELERS_PAD_OWNER_PHASE_RESERVED) {
		k_spin_unlock(&state_lock, key);
		LOG_ERR("arbiter activation rejected producer=%s ownership-frame=%u owner=%s",
			producer, ownership_frame_seq, owner_name(requested_owner));
		return false;
	}
	state.token.phase = TRAVELERS_PAD_OWNER_PHASE_ACTIVE;
	uint32_t session_id = state.token.session_id;
	uint32_t generation = state.token.owner_generation;
	k_spin_unlock(&state_lock, key);
	LOG_INF("arbiter owner active session=%u generation=%u owner=%s producer=%s",
		session_id, generation, owner_name(requested_owner), producer);
	return true;
}

bool travelers_pad_arbiter_acquire_post_session_lease(
	uint32_t session_id, enum travelers_pad_owner required_owner, uint32_t capability,
	struct travelers_pad_owner_token *lease)
{
	k_spinlock_key_t key = k_spin_lock(&state_lock);
	if (lease == NULL || !state.post_session.valid ||
	    state.post_session.kind != TRAVELERS_PAD_TOKEN_POST_SESSION ||
	    state.post_session.session_id != session_id ||
		    state.post_session.lifecycle_epoch != state.lifecycle_epoch ||
		    !token_has_capabilities(&state.post_session, required_owner, capability)) {
		k_spin_unlock(&state_lock, key);
		return false;
	}
	*lease = state.post_session;
	k_spin_unlock(&state_lock, key);
	return true;
}

bool travelers_pad_arbiter_authorize_post_session_output(
	const struct travelers_pad_owner_token *lease, enum travelers_pad_owner required_owner,
	uint32_t capability, const char *producer)
{
	ARG_UNUSED(producer);
	k_spinlock_key_t key = k_spin_lock(&state_lock);
	bool authorized;

	authorized = lease != NULL && lease->valid && lease->kind == TRAVELERS_PAD_TOKEN_POST_SESSION &&
	       token_has_capabilities(lease, required_owner, capability) &&
	       state.post_session.valid && state.post_session.kind == TRAVELERS_PAD_TOKEN_POST_SESSION &&
	       state.post_session.session_id == lease->session_id &&
	       state.post_session.owner_generation == lease->owner_generation &&
	       state.post_session.lifecycle_epoch == lease->lifecycle_epoch &&
	       state.post_session.lifecycle_epoch == state.lifecycle_epoch &&
	       token_has_capabilities(&state.post_session, required_owner, capability);
	k_spin_unlock(&state_lock, key);
	return authorized;
}

bool travelers_pad_arbiter_begin_post_session_output(
	const struct travelers_pad_owner_token *lease, enum travelers_pad_owner required_owner,
	uint32_t capability, const char *producer, struct travelers_pad_arbiter_output_guard *guard)
{
	ARG_UNUSED(producer);
	bool authorized;

	if (guard == NULL) {
		return false;
	}
	*guard = (struct travelers_pad_arbiter_output_guard){};
	guard->key = k_spin_lock(&state_lock);
	authorized = lease != NULL && lease->valid && lease->kind == TRAVELERS_PAD_TOKEN_POST_SESSION &&
		token_has_capabilities(lease, required_owner, capability) && state.post_session.valid &&
		state.post_session.kind == TRAVELERS_PAD_TOKEN_POST_SESSION &&
		state.post_session.session_id == lease->session_id &&
		state.post_session.owner_generation == lease->owner_generation &&
		state.post_session.lifecycle_epoch == lease->lifecycle_epoch &&
		state.post_session.lifecycle_epoch == state.lifecycle_epoch &&
		token_has_capabilities(&state.post_session, required_owner, capability);
	if (!authorized) {
		k_spin_unlock(&state_lock, guard->key);
		return false;
	}
	guard->held = true;
	return true;
}

bool travelers_pad_arbiter_begin_terminal_action_output(
	struct travelers_pad_terminal_action_grant *grant,
	enum travelers_pad_recognizer_id recognizer, enum travelers_pad_owner owner,
	uint32_t capability, const char *producer, struct travelers_pad_arbiter_output_guard *guard)
{
	bool consumed = false;
	if (guard == NULL || grant == NULL || !grant->valid ||
		recognizer >= TRAVELERS_PAD_RECOGNIZER_COUNT || grant->recognizer != recognizer ||
		grant->owner != owner || (grant->capability & capability) != capability) {
		return false;
	}
	*guard = (struct travelers_pad_arbiter_output_guard){};
	guard->key = k_spin_lock(&state_lock);
	for (uint8_t index = 0; index < ARRAY_SIZE(state.terminal_actions); index++) {
		struct terminal_action_record *record = &state.terminal_actions[index];

		if (!record->grant.valid || record->consumed ||
		    record->grant.grant_generation != grant->grant_generation ||
		    record->grant.lifecycle_epoch != grant->lifecycle_epoch ||
		    record->grant.lifecycle_epoch != state.lifecycle_epoch ||
		    record->grant.session_id != grant->session_id ||
		    record->grant.recognizer != recognizer || record->grant.owner != owner ||
		    (record->grant.capability & capability) != capability) {
			continue;
		}
		record->consumed = true;
		grant->valid = false;
		consumed = true;
		break;
	}
	if (!consumed) {
		k_spin_unlock(&state_lock, guard->key);
		return false;
	}
	guard->held = true;
	guard->terminal_action_consumed = true;
	guard->terminal_action_session_id = grant->session_id;
	guard->terminal_action_generation = grant->grant_generation;
	guard->terminal_action_producer = producer;
	return true;
}

void travelers_pad_arbiter_end_output(struct travelers_pad_arbiter_output_guard *guard)
{
	bool terminal_action_consumed;
	uint32_t session_id;
	uint32_t generation;
	const char *producer;

	if (guard != NULL && guard->held) {
		terminal_action_consumed = guard->terminal_action_consumed;
		session_id = guard->terminal_action_session_id;
		generation = guard->terminal_action_generation;
		producer = guard->terminal_action_producer;
		guard->held = false;
		k_spin_unlock(&state_lock, guard->key);
		if (terminal_action_consumed) {
			LOG_INF("terminal action consumed session=%u generation=%u producer=%s", session_id,
				generation, producer);
		}
	}
}

void travelers_pad_arbiter_observed(enum travelers_pad_recognizer_id recognizer, uint32_t frame_seq)
{
	struct arbiter_frame *frame = frame_for(frame_seq);
	bool resolve = false;
	bool dispatch_observer = false;
	k_spinlock_key_t key;

	key = k_spin_lock(&state_lock);
	frame = frame_for(frame_seq);
	if (!frame_accepts_recognizer(frame, recognizer)) {
		k_spin_unlock(&state_lock, key);
		return;
	}
	frame->observed_mask |= 1U << recognizer;
	/* Observers consume the resolved lifecycle result; they never participate in
	 * selecting it. Their callback order or disabled state therefore cannot hold
	 * up pointer/scroll routing. / observerは解決済みlifecycle結果を消費するだけで
	 * 選択には参加しない。従ってobserverのcallback順や無効状態でpointer/scrollの
	 * routingを待たせない。 */
	if (!frame->resolved &&
	    (frame->observed_mask & state.primary_registered_mask) == state.primary_registered_mask) {
		frame->resolved = true;
		resolve = true;
	}
	dispatch_observer = frame->resolved && !is_primary_recognizer(recognizer);
	k_spin_unlock(&state_lock, key);
	if (resolve) {
		resolve_frame(frame);
	}
	/* An observer may be called after primary resolution, or it may have
	 * consumed its raw frame before the final primary acknowledgement. In both
	 * cases dispatch exactly once only when both facts are true.
	 * observerはprimary裁定後に呼ばれる場合と、最終primary acknowledgement前に生frameを
	 * 消費済みの場合がある。両方の条件が満たされた時だけ一度dispatchする。 */
	if (dispatch_observer || (resolve && !is_primary_recognizer(recognizer))) {
		dispatch_frame_handler(frame, recognizer);
	}
	/* Keep the immutable frame available to late observer callbacks. */
	key = k_spin_lock(&state_lock);
	if ((frame->observed_mask & state.registered_mask) == state.registered_mask) {
		frame->active = false;
	}
	k_spin_unlock(&state_lock, key);
}

bool travelers_pad_arbiter_note_terminal_output(uint32_t ownership_frame_seq,
					 enum travelers_pad_recognizer_id recognizer,
					 enum travelers_pad_owner requested_owner,
					 uint32_t capability,
					 const char *producer)
{
	struct arbiter_frame *frame = frame_for(ownership_frame_seq);
	struct arbiter_output_receipt receipt = {
		.recognizer = recognizer,
		.requested_owner = requested_owner,
		.capability = capability,
		.producer = producer,
	};
	struct arbiter_decision *decision;

	if (frame != NULL) {
		if (frame->output_receipt_count >= ARRAY_SIZE(frame->output_receipts)) {
			LOG_ERR("terminal output record receipt overflow session=%u frame=%u",
				frame->snapshot.session_id, ownership_frame_seq);
			return true;
		}
		frame->output_receipts[frame->output_receipt_count++] = receipt;
		return true;
	}

	decision = decision_for(ownership_frame_seq);
	if (decision == NULL ||
	    !decision_allows(&decision->token, decision->recognizer, &receipt)) {
		LOG_ERR("terminal output record would reject producer=%s ownership-frame=%u owner=%s caps=0x%x",
			producer, ownership_frame_seq, owner_name(requested_owner), capability);
	} else if (!decision->output_verified) {
		decision->output_verified = true;
		LOG_INF("terminal output record preflight ownership-frame=%u winner=%s producer=%s accepted",
			ownership_frame_seq, owner_name(decision->token.owner), producer);
	}
	/* This is diagnostic only. Releases remain unconditional so a stale token
	 * cannot leave a host button held. / これは診断専用である。stale tokenで
	 * ホスト側のボタンが押下状態に残らないよう、releaseは常に送信する。 */
	return true;
}
