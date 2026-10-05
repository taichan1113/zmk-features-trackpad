/*
 * Travelers Pad contact-arbiter vocabulary.
 *
 * Recognizers submit immutable proposals and the resolver selects the only
 * contact owner. Primary semantic output requires the issued token, while
 * auxiliary continuation output is delegated explicitly by that live token.
 *
 * Travelers Pad 接触arbiter用の共通語彙。
 * recognizerが不変proposalを提出し、resolverが唯一のcontact ownerを選ぶ。
 * primary semantic出力には発行tokenを必須とし、補助継続出力もlive tokenから
 * 明示的に委譲する。
 */
#ifndef TRAVELERS_PAD_CONTACT_ARBITER_H_
#define TRAVELERS_PAD_CONTACT_ARBITER_H_

#include <stdbool.h>
#include <stdint.h>

#include <zephyr/spinlock.h>

#include <travelers_pad/contact_session.h>

enum travelers_pad_owner {
	TRAVELERS_PAD_OWNER_NONE,
	TRAVELERS_PAD_OWNER_DRAG,
	TRAVELERS_PAD_OWNER_SCROLL,
	TRAVELERS_PAD_OWNER_TRANSFORM_GESTURE,
	TRAVELERS_PAD_OWNER_MULTI_GESTURE,
	TRAVELERS_PAD_OWNER_EDGE_DIAL,
	TRAVELERS_PAD_OWNER_TIP_TAP,
};

enum travelers_pad_recognizer_id {
	TRAVELERS_PAD_RECOGNIZER_ROUTER,
	TRAVELERS_PAD_RECOGNIZER_TAP_DRAG,
	TRAVELERS_PAD_RECOGNIZER_MULTIFINGER_GESTURE,
	TRAVELERS_PAD_RECOGNIZER_EDGE_DIAL,
	TRAVELERS_PAD_RECOGNIZER_TIP_TAP,
	/* Two-finger pinch/rotation geometry is a primary recognizer. Once it
	 * claims a transform, the same session cannot fall back to scroll or tap. */
	TRAVELERS_PAD_RECOGNIZER_TRANSFORM_GESTURE,
	/* Auxiliary observer: proposes no owner and receives frames only so
	 * inertia can associate a physical release with the arbiter token. The
	 * preceding six IDs remain the primary recognizers. */
	TRAVELERS_PAD_RECOGNIZER_SCROLL_INERTIA,
	/* Auxiliary observer: detects edge-hold continuation candidates from raw
	 * contacts. It has no claim of its own; delayed output is delegated by the
	 * current drag or scroll owner token. */
	TRAVELERS_PAD_RECOGNIZER_EDGE_HOLD,
	TRAVELERS_PAD_RECOGNIZER_COUNT,
};

enum travelers_pad_auxiliary_id {
	TRAVELERS_PAD_AUXILIARY_EDGE_HOLD,
	TRAVELERS_PAD_AUXILIARY_COUNT,
};

enum travelers_pad_proposal_intent {
	/* A recognizer is tracking a possible action but requests no ownership. */
	TRAVELERS_PAD_PROPOSAL_CANDIDATE,
	/* The recognizer requests exclusive session ownership. */
	TRAVELERS_PAD_PROPOSAL_CLAIM,
};

enum travelers_pad_output_capability {
	TRAVELERS_PAD_CAP_NONE = 0,
	TRAVELERS_PAD_CAP_POINTER_DELTA = (1U << 0),
	TRAVELERS_PAD_CAP_SCROLL_DELTA = (1U << 1),
	TRAVELERS_PAD_CAP_BUTTON = (1U << 2),
	TRAVELERS_PAD_CAP_BEHAVIOR_ACTION = (1U << 3),
};

enum travelers_pad_token_kind {
	TRAVELERS_PAD_TOKEN_NONE,
	TRAVELERS_PAD_TOKEN_COMMITTED,
	TRAVELERS_PAD_TOKEN_POST_SESSION,
};

/* A committed owner may reserve an interpretation before it can safely emit
 * its semantic output. A double-tap drag is the current example: it reserves
 * the contact at second touch, and becomes active only after button-down.
 * Committed ownerはsemantic出力を安全に行える前に解釈を予約し得る。現在は
 * double-tap dragが例で、2回目接触で予約し、button-down後にだけactiveになる。 */
enum travelers_pad_owner_phase {
	TRAVELERS_PAD_OWNER_PHASE_NONE,
	TRAVELERS_PAD_OWNER_PHASE_RESERVED,
	TRAVELERS_PAD_OWNER_PHASE_ACTIVE,
};

/* Evidence is recognizer-local trace data only. It must never affect callback
 * ordering or the resolver policy. */
struct travelers_pad_arbiter_proposal {
	uint32_t frame_seq;
	enum travelers_pad_recognizer_id recognizer;
	enum travelers_pad_proposal_intent intent;
	enum travelers_pad_owner requested_owner;
	uint32_t capabilities;
	uint8_t contact_count;
	uint32_t evidence;
};

struct travelers_pad_owner_token {
	uint32_t session_id;
	uint32_t owner_generation;
	/* Global cancellation generation. A reset outside an active contact session
	 * changes this value, invalidating copied leases and grants as well. */
	uint32_t lifecycle_epoch;
	enum travelers_pad_owner owner;
	uint32_t capabilities;
	enum travelers_pad_token_kind kind;
	enum travelers_pad_owner_phase phase;
	bool valid;
};

/* A terminal action is intentionally separate from a live owner token. It is
 * minted only after a normal terminal frame has been fully resolved and may be
 * consumed exactly once by its originating recognizer. This suits a delayed
 * tap/double-click without allowing a stale timer to acquire session ownership.
 * terminal actionはlive owner tokenとは意図的に分離する。通常terminal frameの裁定後にだけ
 * 発行され、発行元recognizerが一度だけ消費できる。遅延tap/double-clickに適し、古いtimerが
 * session所有権を取得することはできない。 */
struct travelers_pad_terminal_action_grant {
	uint32_t session_id;
	uint32_t grant_generation;
	uint32_t lifecycle_epoch;
	enum travelers_pad_recognizer_id recognizer;
	enum travelers_pad_owner owner;
	uint32_t capability;
	bool valid;
};

/* An auxiliary lease is deliberately a copy of the exact primary token that
 * armed delayed output. It prevents an old edge-hold work item from attaching
 * itself to a later DRAG or SCROLL owner of the same kind. */
struct travelers_pad_auxiliary_lease {
	uint32_t session_id;
	uint32_t owner_generation;
	uint32_t lifecycle_epoch;
	enum travelers_pad_auxiliary_id auxiliary;
	enum travelers_pad_owner owner;
	uint32_t capabilities;
	bool valid;
};

/* A broker-held guard keeps the authority check and a non-blocking HID report
 * in one cancellation-atomic interval. It is an internal transport contract:
 * while held, callers must only perform an IRQ-safe, non-blocking report and
 * must release it immediately. It must not cover arbitrary behavior callbacks.
 * broker保持guardは、authority照合とnon-blocking HID reportを一つのcancel原子的な
 * 区間にする内部transport契約である。保持中はIRQ-safeかつnon-blockingなreportだけを
 * 行い、直ちに解放する。任意のbehavior callbackを囲んではならない。 */
struct travelers_pad_arbiter_output_guard {
	k_spinlock_key_t key;
	bool held;
	bool terminal_action_consumed;
	uint32_t terminal_action_session_id;
	uint32_t terminal_action_generation;
	const char *terminal_action_producer;
};

/* A recognizer can defer the semantic action which first establishes its
 * ownership until the resolver has issued a token.  This is needed because
 * the primary input listeners run asynchronously: at proposal time the winning
 * token does not exist yet.  The callback runs exactly once for a newly
 * committed token, before a terminal frame invalidates it.
 * recognizerは、所有権を最初に確立するsemantic actionを、resolverがtokenを
 * 発行するまで保留できる。primary input listenerは非同期なのでproposal時点には
 * 勝者tokenがまだ存在しないためである。callbackは新規commit tokenごとに一度、
 * terminal frameがtokenを無効化する前に実行される。 */
typedef void (*travelers_pad_arbiter_resolve_handler_t)(
	void *context, const struct travelers_pad_owner_token *token, uint32_t ownership_frame_seq);

/* A primary recognizer's callback runs once after the complete primary proposal
 * set is resolved. An observer's callback runs once only after that same
 * resolution *and* after the observer has consumed its own raw frame. It is
 * for consumers that need an immutable resolved frame; it must not submit a
 * proposal for that frame.
 * primary recognizerのcallbackは完全なprimary proposal集合の裁定後に一度だけ呼ばれる。
 * observerのcallbackは同じ裁定後、かつobserver自身が生frameを消費した後に一度だけ呼ばれる。
 * 不変の裁定済みframeを必要とするconsumer用であり、同じframeへのproposalを追加してはならない。 */
typedef void (*travelers_pad_arbiter_frame_handler_t)(
	void *context, const struct travelers_pad_session_snapshot *snapshot,
	const struct travelers_pad_owner_token *token);

/* Recognizers register from their DT-backed init function. The six primary
 * recognizers form the decision barrier; edge-hold and inertia are observers
 * and are deliberately excluded from it. All registered functions still
 * acknowledge before the frame storage is reclaimed.
 * recognizerはDT-backed initから登録する。6個のprimary recognizerだけが裁定barrierを
 * 構成し、edge-holdとinertiaはobserverとして意図的にbarrierから除外する。ただし、frame
 * storageの回収前には登録済み全機能がacknowledgeする。 */
bool travelers_pad_arbiter_register(enum travelers_pad_recognizer_id recognizer);
bool travelers_pad_arbiter_bind_resolve_handler(
	enum travelers_pad_recognizer_id recognizer,
	travelers_pad_arbiter_resolve_handler_t handler, void *context);
bool travelers_pad_arbiter_bind_frame_handler(
	enum travelers_pad_recognizer_id recognizer,
	travelers_pad_arbiter_frame_handler_t handler, void *context);

/* Called by the session manager around one synchronously forwarded frame. */
void travelers_pad_arbiter_begin_frame(const struct travelers_pad_session_snapshot *snapshot);
/* Invalidate all authority, including authority that survived a normal
 * release. The session manager calls this as soon as the sensor reports a
 * reset, before any queued recognizer can observe that reset frame. */
void travelers_pad_arbiter_cancel_all(const char *reason);
/* Each recognizer receives session-manager frames in FIFO order. Obtain its
 * earliest unacknowledged frame immediately before processing an input sync.
 * 各recognizerはsession manager由来frameをFIFO順で受ける。input syncを処理する
 * 直前に、最も古い未通知frameを得る。 */
uint32_t travelers_pad_arbiter_next_frame(enum travelers_pad_recognizer_id recognizer);
bool travelers_pad_arbiter_submit(const struct travelers_pad_arbiter_proposal *proposal);
void travelers_pad_arbiter_observed(enum travelers_pad_recognizer_id recognizer, uint32_t frame_seq);
/* Return the committed arbiter owner applicable while processing one forwarded
 * frame. A same-frame claim is deliberately invisible until all recognizers
 * have observed the frame; routing that requires the new decision must use a
 * frame handler. The committed owner remains visible for its terminal release.
 * 転送済みframeに適用されるcommit済みarbiter ownerを返す。同一frameのclaimは、全
 * recognizerがframeを観測するまで意図的に不可視である。新しい決定を必要とするroutingは
 * frame handlerを使う。commit済みownerはterminal releaseでも見える。 */
enum travelers_pad_owner travelers_pad_arbiter_owner_for_frame(uint32_t frame_seq);
/* Return immutable lifecycle metadata for a queued frame. Observers use this
 * to distinguish a normal terminal release from a sensor-reset cancellation.
 * queued frameの不変lifecycle情報を返す。observerは通常releaseとsensor reset
 * cancellationを区別するために使う。 */
bool travelers_pad_arbiter_get_frame_snapshot(uint32_t frame_seq,
					       struct travelers_pad_session_snapshot *snapshot);

/* Issue a one-shot terminal-action grant from a fully resolved normal release.
 * owner=NONE is allowed only when the frame has no committed owner; a non-NONE
 * owner must match the active committed token and its capability. The grant is
 * later consumed atomically by the output broker.
 * fully resolved済みの通常releaseから、一回限りterminal-action grantを発行する。
 * owner=NONEはcommit済みownerが無いframeだけで許可する。NONE以外はactiveなcommit
 * tokenとcapabilityの一致が必要である。grantは後でoutput brokerが原子的に消費する。 */
bool travelers_pad_arbiter_issue_terminal_action(
	uint32_t frame_seq, enum travelers_pad_recognizer_id recognizer,
	enum travelers_pad_owner owner, uint32_t capability,
	struct travelers_pad_terminal_action_grant *grant);
bool travelers_pad_arbiter_consume_terminal_action(
	struct travelers_pad_terminal_action_grant *grant,
	enum travelers_pad_recognizer_id recognizer, enum travelers_pad_owner owner,
	uint32_t capability, const char *producer);

/* Record a terminal release against the frame that established its ownership.
 * Releases are deliberately never blocked: a stale token must not leave a
 * host button held. The record is diagnostic only and can expose an invalid
 * release path without changing that fail-safe rule.
 * terminal releaseを、その所有権を確立したframeへ記録する。stale tokenによって
 * ホストのボタンが押下状態に残らないよう、releaseは意図的に止めない。この記録は
 * 診断専用であり、安全側の規則を変えずに不正なrelease経路を検出する。 */
bool travelers_pad_arbiter_note_terminal_output(uint32_t ownership_frame_seq,
					 enum travelers_pad_recognizer_id recognizer,
					 enum travelers_pad_owner requested_owner,
					 uint32_t capability,
					 const char *producer);

/* Check a semantic action against an already-resolved token. This is
 * authoritative and returns false on mismatch.
 * It is used by resolver-dispatched first actions and primary recognizers'
 * continuation actions. Releases remain fail-safe so a stale token cannot
 * leave a host button held.
 * 解決済みtokenに対してsemantic actionを照合する。これはauthoritativeで、不一致なら
 * falseを返す。resolver-dispatchされた初回
 * actionおよびprimary recognizerの継続actionで使用する。releaseはstale tokenで
 * host側のボタンが押下状態に残らないよう、常に安全側で送る。 */
bool travelers_pad_arbiter_authorize_output(uint32_t ownership_frame_seq,
					    enum travelers_pad_recognizer_id recognizer,
					    enum travelers_pad_owner requested_owner,
					    uint32_t capability, const char *producer);

/* Begin an atomic authorization-to-HID-report interval. Use only from the
 * output broker, then call travelers_pad_arbiter_end_output() immediately
 * after the K_NO_WAIT input report. */
bool travelers_pad_arbiter_begin_primary_output(
	uint32_t ownership_frame_seq, enum travelers_pad_recognizer_id recognizer,
	enum travelers_pad_owner requested_owner, uint32_t capability, const char *producer,
	struct travelers_pad_arbiter_output_guard *guard);

/* The router labels an internally forwarded contact frame before it enters the
 * asynchronous pointer adapter. The adapter must present the exact frame
 * sequence to begin_routed_output() before emitting a resulting REL report.
 * This prevents a queued old contact frame from borrowing later authority. */
bool travelers_pad_arbiter_prepare_routed_output(
	uint32_t frame_seq, enum travelers_pad_owner owner, uint32_t capability);
bool travelers_pad_arbiter_begin_routed_output(
	uint32_t frame_seq, enum travelers_pad_owner owner, uint32_t capability,
	const char *producer, struct travelers_pad_arbiter_output_guard *guard);

/* An auxiliary function may extend an already-committed primary action, but
 * may not establish ownership itself. This validates its delayed output
 * against the live token. It is intentionally quiet on a mismatch because
 * timers may wake after a normal release or ownership hand-off.
 * 補助機能は確定済みprimary操作を継続できるが、自身で所有権は確立できない。
 * 遅延出力をlive tokenに対して検証する。通常の解放・所有権移行後にもtimerが
 * 起き得るため、不一致は意図的に静かに扱う。 */
bool travelers_pad_arbiter_authorize_auxiliary_output(
	const struct travelers_pad_auxiliary_lease *lease,
	enum travelers_pad_auxiliary_id auxiliary, enum travelers_pad_owner required_owner,
	uint32_t capability, const char *producer);
bool travelers_pad_arbiter_begin_auxiliary_output(
	const struct travelers_pad_auxiliary_lease *lease,
	enum travelers_pad_auxiliary_id auxiliary, enum travelers_pad_owner required_owner,
	uint32_t capability, const char *producer, struct travelers_pad_arbiter_output_guard *guard);

bool travelers_pad_arbiter_acquire_auxiliary_lease(
	enum travelers_pad_auxiliary_id auxiliary, enum travelers_pad_owner required_owner,
	uint32_t capability, struct travelers_pad_auxiliary_lease *lease);

/* Promote a reserved committed owner only after its primary output succeeded.
 * Auxiliary output is restricted to ACTIVE owners, so a stationary double-tap
 * candidate cannot start edge continuation before it has actually pressed a
 * button. / 予約済みcommitted ownerをprimary出力成功後にだけ昇格する。補助出力は
 * ACTIVE ownerに制限するため、静止したdouble-tap候補はbuttonを押す前にedge継続を
 * 開始できない。 */
bool travelers_pad_arbiter_activate_owner(uint32_t ownership_frame_seq,
					  enum travelers_pad_recognizer_id recognizer,
					  enum travelers_pad_owner requested_owner,
					  const char *producer);

/* A normal completed scroll may create one revocable post-session lease for
 * inertia. The copied lease is valid only while the arbiter retains exactly
 * the same session and generation; new contact or cancellation revokes it.
 * 正常終了したscrollはinertia用の取消可能なpost-session leaseを一つ作れる。copyした
 * leaseはarbiterが同一session/generationを保持する間だけ有効で、新規接触または
 * cancellationで失効する。 */
bool travelers_pad_arbiter_acquire_post_session_lease(
	uint32_t session_id, enum travelers_pad_owner required_owner, uint32_t capability,
	struct travelers_pad_owner_token *lease);
bool travelers_pad_arbiter_authorize_post_session_output(
	const struct travelers_pad_owner_token *lease, enum travelers_pad_owner required_owner,
	uint32_t capability, const char *producer);
bool travelers_pad_arbiter_begin_post_session_output(
	const struct travelers_pad_owner_token *lease, enum travelers_pad_owner required_owner,
	uint32_t capability, const char *producer, struct travelers_pad_arbiter_output_guard *guard);

/* Atomically consume a terminal grant and retain the output guard for the
 * complete non-blocking click pulse. */
bool travelers_pad_arbiter_begin_terminal_action_output(
	struct travelers_pad_terminal_action_grant *grant,
	enum travelers_pad_recognizer_id recognizer, enum travelers_pad_owner owner,
	uint32_t capability, const char *producer, struct travelers_pad_arbiter_output_guard *guard);
void travelers_pad_arbiter_end_output(struct travelers_pad_arbiter_output_guard *guard);

#endif
