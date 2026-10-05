/*
 * Travelers Pad contact output broker.
 *
 * All host-facing relative motion, wheel, button reports, and primary virtual
 * KSCAN actions produced by a contact-session feature pass through this
 * interface. The broker is the single place that checks the arbiter token or
 * post-session lease immediately before emitting an action. Internal normalized
 * contact forwarding is intentionally outside this interface.
 *
 * Travelers Pad 接触出力broker。
 * 接触session機能が生成するホスト向けの相対移動、ホイール、ボタンreport、およびprimaryの
 * 仮想KSCAN actionは、すべてこのinterfaceを通す。brokerはactionを出す直前にarbiter
 * tokenまたはpost-session leaseを検査する唯一の場所である。正規化接触frameの内部配送は
 * 意図的に対象外とする。
 */
#ifndef TRAVELERS_PAD_CONTACT_OUTPUT_BROKER_H_
#define TRAVELERS_PAD_CONTACT_OUTPUT_BROKER_H_

#include <stdbool.h>
#include <stdint.h>

#include <zephyr/device.h>
#include <zephyr/drivers/kscan.h>

#include <travelers_pad/contact_arbiter.h>

/* Init-only registration of a private canonical wheel observation stream.
 * The broker copies successful routed/auxiliary wheel values before processors;
 * post-session output is never copied, preventing inertia feedback. The sink
 * must not have a host input-listener or input processors attached.
 * 初期化時だけ専用の標準wheel観測streamを登録する。processor前の値を複製し、
 * post-session出力は複製しない。観測sinkへHID listenerやprocessorを接続しない。 */
struct travelers_pad_wheel_observation {
	const struct device *source;
	const struct device *sink;
	struct travelers_pad_wheel_observation *next;
};
int travelers_pad_output_register_wheel_observation(struct travelers_pad_wheel_observation *route);

/* A primary owner output is emitted only while its exact arbiter token is
 * current. / primary ownerの出力は、完全に一致するarbiter tokenが現行の間だけ送る。 */
int travelers_pad_output_primary(const struct device *dev, uint32_t ownership_frame_seq,
				enum travelers_pad_recognizer_id recognizer,
				enum travelers_pad_owner owner, uint32_t capability,
				uint8_t type, uint16_t code, int32_t value, bool sync,
				const char *producer);

/* An auxiliary output is emitted only while the delegated active owner is
 * current. / auxiliary出力は委譲されたactive ownerが現行の間だけ送る。 */
int travelers_pad_output_auxiliary(const struct device *dev,
				  const struct travelers_pad_auxiliary_lease *lease,
				  enum travelers_pad_auxiliary_id auxiliary,
				  enum travelers_pad_owner owner, uint32_t capability,
				  uint8_t type, uint16_t code, int32_t value, bool sync,
				  const char *producer);

/* A post-session output is emitted only while its copied, revocable lease is
 * current. / post-session出力は、コピーされた取消可能leaseが現行の間だけ送る。 */
int travelers_pad_output_post_session(const struct device *dev,
				    const struct travelers_pad_owner_token *lease,
				    enum travelers_pad_owner owner, uint32_t capability,
				    uint8_t type, uint16_t code, int32_t value, bool sync,
				    const char *producer);

/* Releases must remain fail-safe. Record the terminal path for diagnostics,
 * then always emit it even if the previous token was revoked.
 * releaseはfail-safeを維持する。診断用にterminal経路を記録した後、前のtokenが
 * 失効済みでも必ず送る。 */
int travelers_pad_output_terminal(const struct device *dev, uint32_t ownership_frame_seq,
				 enum travelers_pad_recognizer_id recognizer,
				 enum travelers_pad_owner owner, uint32_t capability,
				 uint8_t type, uint16_t code, int32_t value, bool sync,
				 const char *producer);

/* Consume one terminal-action grant and emit one or more complete mouse-click
 * pulses as one semantic action. A delayed tap cannot replay after its grant
 * was consumed or overwritten.
 * terminal-action grantを一度消費し、1回以上の完全なmouse-click pulseを一つのsemantic
 * actionとして出力する。遅延tapはgrantが消費済みまたは上書き済みなら再生できない。 */
int travelers_pad_output_terminal_click(
	const struct device *dev, struct travelers_pad_terminal_action_grant *grant,
	enum travelers_pad_recognizer_id recognizer, enum travelers_pad_owner owner,
	uint16_t button, uint8_t click_count, const char *producer);

/* A primary virtual-KSCAN action is authorized and emitted as an inseparable
 * press/release pair. Primary recognizers must not authorize a token in one
 * place and invoke the KSCAN callback through a separate path.
 * primaryの仮想KSCAN actionは、認可と不可分なpress/release対として送信する。primary
 * recognizerは、ある場所でtokenを認可し、別経路でKSCAN callbackを呼んではならない。 */
int travelers_pad_output_behavior_primary(
	const struct device *dev, kscan_callback_t callback, uint32_t ownership_frame_seq,
	enum travelers_pad_recognizer_id recognizer, enum travelers_pad_owner owner,
	uint16_t row, uint16_t column, const char *producer);

/* Canonical pointer output is already selected by the router before it reaches
 * the pointer adapter. It has no independent recognizer claim, but uses the
 * same transport point so every host-facing REL event is centralized.
 * canonical pointer出力はpointer adapter到達前にrouterで選別済みである。独立した
 * recognizer claimは持たないが、全ホスト向けREL eventを集約するため同じtransportを使う。 */
int travelers_pad_output_routed_pointer(const struct device *dev, uint32_t routed_frame_seq,
				 enum travelers_pad_owner owner, uint32_t capability,
				 uint8_t type, uint16_t code, int32_t value, bool sync,
				 const char *producer);

/* A cancellation makes every broker-owned held button eligible for release.
 * Failed releases remain pending and are retried by the next broker request. */
void travelers_pad_output_cancel_all(const char *producer);

#endif
