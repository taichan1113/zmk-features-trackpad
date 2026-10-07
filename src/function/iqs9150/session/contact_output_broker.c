/*
 * Travelers Pad contact output broker implementation.
 *
 * Authorization and transport deliberately live together: a future producer
 * cannot accidentally check one token and emit from an unrelated delayed path.
 *
 * Travelers Pad 接触出力brokerの実装。
 * authorizationとtransportを意図的に同居させる。将来のproducerがあるtokenを検査して
 * 別の遅延経路から出力してしまうことを防ぐためである。
 */

#include <errno.h>

#include <zephyr/input/input.h>
#include <zephyr/logging/log.h>
#include <zephyr/spinlock.h>
#include <zephyr/sys/util.h>

#include <travelers_pad/contact_output_broker.h>

LOG_MODULE_REGISTER(travelers_pad_output_broker, LOG_LEVEL_INF);

#define TRAVELERS_PAD_OUTPUT_MAX_HELD_BUTTONS 4

/* The broker owns this small ledger rather than relying on recognizer-local
 * state alone. A terminal release is therefore permitted only for the exact
 * trackpad press that passed through this transport. This prevents a future
 * stale path from releasing the same host button held by another producer.
 * recognizer局所stateだけに頼らず、brokerがこの小さなledgerを所有する。terminal
 * releaseはこのtransportを通った正確なtrackpad pressにだけ許可される。これにより将来の
 * stale経路が、別producerが保持する同じhost buttonをreleaseすることを防ぐ。 */
struct held_button_record {
	const struct device *dev;
	uint32_t ownership_frame_seq;
	enum travelers_pad_recognizer_id recognizer;
	enum travelers_pad_owner owner;
	uint16_t button;
	bool held;
	bool release_pending;
	bool release_in_flight;
};

static struct held_button_record held_buttons[TRAVELERS_PAD_OUTPUT_MAX_HELD_BUTTONS];
static struct k_spinlock held_buttons_lock;

static void cancel_clicks(uint16_t button, bool all);
static bool button_busy(uint16_t button);

static bool held_button_matches(const struct held_button_record *record, const struct device *dev,
				uint32_t ownership_frame_seq,
				enum travelers_pad_recognizer_id recognizer,
				enum travelers_pad_owner owner, uint16_t button)
{
	return record->held && record->dev == dev &&
	       record->ownership_frame_seq == ownership_frame_seq &&
	       record->recognizer == recognizer && record->owner == owner &&
	       record->button == button;
}

static bool held_button_exists(const struct device *dev, uint32_t ownership_frame_seq,
			       enum travelers_pad_recognizer_id recognizer,
			       enum travelers_pad_owner owner, uint16_t button)
{
	k_spinlock_key_t key = k_spin_lock(&held_buttons_lock);
	bool found = false;

	for (size_t index = 0; index < ARRAY_SIZE(held_buttons); index++) {
		if (held_button_matches(&held_buttons[index], dev, ownership_frame_seq, recognizer,
					owner, button)) {
			found = true;
			break;
		}
	}
	k_spin_unlock(&held_buttons_lock, key);
	return found;
}

static bool remember_held_button(const struct device *dev, uint32_t ownership_frame_seq,
				enum travelers_pad_recognizer_id recognizer,
				enum travelers_pad_owner owner, uint16_t button)
{
	k_spinlock_key_t key = k_spin_lock(&held_buttons_lock);
	struct held_button_record *empty = NULL;

	for (size_t index = 0; index < ARRAY_SIZE(held_buttons); index++) {
		if (held_button_matches(&held_buttons[index], dev, ownership_frame_seq, recognizer,
					owner, button)) {
			k_spin_unlock(&held_buttons_lock, key);
			return false;
		}
		if (!held_buttons[index].held && empty == NULL) {
			empty = &held_buttons[index];
		}
	}
	if (empty != NULL) {
		*empty = (struct held_button_record){
			.dev = dev,
			.ownership_frame_seq = ownership_frame_seq,
			.recognizer = recognizer,
			.owner = owner,
			.button = button,
			.held = true,
		};
	}
	k_spin_unlock(&held_buttons_lock, key);
	return empty != NULL;
}

static bool take_held_button(const struct device *dev, uint32_t ownership_frame_seq,
			     enum travelers_pad_recognizer_id recognizer,
			     enum travelers_pad_owner owner, uint16_t button)
{
	k_spinlock_key_t key = k_spin_lock(&held_buttons_lock);
	bool found = false;

	for (size_t index = 0; index < ARRAY_SIZE(held_buttons); index++) {
		if (held_button_matches(&held_buttons[index], dev, ownership_frame_seq, recognizer,
					owner, button)) {
			held_buttons[index].held = false;
			found = true;
			break;
		}
	}
	k_spin_unlock(&held_buttons_lock, key);
	return found;
}

static bool begin_button_release(const struct device *dev, uint32_t ownership_frame_seq,
				 enum travelers_pad_recognizer_id recognizer,
				 enum travelers_pad_owner owner, uint16_t button)
{
	k_spinlock_key_t key = k_spin_lock(&held_buttons_lock);
	bool found = false;

	for (size_t index = 0; index < ARRAY_SIZE(held_buttons); index++) {
		struct held_button_record *record = &held_buttons[index];

		if (held_button_matches(record, dev, ownership_frame_seq, recognizer, owner, button) &&
		    !record->release_in_flight) {
			record->release_pending = true;
			record->release_in_flight = true;
			found = true;
			break;
		}
	}
	k_spin_unlock(&held_buttons_lock, key);
	return found;
}

static void finish_button_release(const struct device *dev, uint32_t ownership_frame_seq,
				  enum travelers_pad_recognizer_id recognizer,
				  enum travelers_pad_owner owner, uint16_t button, bool delivered)
{
	k_spinlock_key_t key = k_spin_lock(&held_buttons_lock);

	for (size_t index = 0; index < ARRAY_SIZE(held_buttons); index++) {
		struct held_button_record *record = &held_buttons[index];

		if (!held_button_matches(record, dev, ownership_frame_seq, recognizer, owner, button)) {
			continue;
		}
		if (delivered) {
			record->held = false;
			record->release_pending = false;
		} else {
			record->release_pending = true;
		}
		record->release_in_flight = false;
		break;
	}
	k_spin_unlock(&held_buttons_lock, key);
}

static int emit(const struct device *dev, uint8_t type, uint16_t code, int32_t value, bool sync,
		const char *producer)
{
	int rc = input_report(dev, type, code, value, sync, K_NO_WAIT);

	if (rc != 0) {
		LOG_ERR("%s output report failed: %d", producer, rc);
	}
	return rc;
}

/* `input_report(..., K_NO_WAIT)` only enqueues the report in this firmware's
 * threaded input configuration. Keep the arbiter output guard as short as
 * possible and log only after it has been released. */
static struct travelers_pad_wheel_observation *wheel_observations;

int travelers_pad_output_register_wheel_observation(struct travelers_pad_wheel_observation *route)
{
	if (route == NULL || route->source == NULL || route->sink == NULL ||
	    route->source == route->sink) {
		return -EINVAL;
	}
	for (struct travelers_pad_wheel_observation *entry = wheel_observations;
	     entry != NULL; entry = entry->next) {
		if (entry == route || entry->sink == route->sink) {
			return -EALREADY;
		}
	}
	route->next = wheel_observations;
	wheel_observations = route;
	return 0;
}

/* A separate queued event owns its value: mutation of the host event by a
 * scaler/transform cannot affect the observer, regardless of callback order.
 * 別queue eventに値を複製し、HID側の値の書換えをcallback順に関係なく隔離する。 */
static int copy_canonical_wheel(const struct device *dev, uint8_t type, uint16_t code,
			      int32_t value, bool sync)
{
	int result = 0;
	if (type != INPUT_EV_REL || (code != INPUT_REL_WHEEL && code != INPUT_REL_HWHEEL)) {
		return 0;
	}
	for (const struct travelers_pad_wheel_observation *entry = wheel_observations;
	     entry != NULL; entry = entry->next) {
		if (entry->source == dev) {
			result |= input_report(entry->sink, type, code, value, sync, K_NO_WAIT);
		}
	}
	return result;
}

/* Passive input-source device; created with the always-present session broker.
 * session brokerとともに専用の受動input sourceを生成する。 */
#define WHEEL_OBSERVATION_INST(node_id) \
	DEVICE_DT_DEFINE(node_id, NULL, NULL, NULL, NULL, POST_KERNEL, \
			 CONFIG_KERNEL_INIT_PRIORITY_DEVICE, NULL);
DT_FOREACH_STATUS_OKAY(zmk_travelers_pad_canonical_wheel_input, WHEEL_OBSERVATION_INST)

static int emit_guarded(const struct device *dev, uint8_t type, uint16_t code, int32_t value,
			bool sync, const char *producer,
			struct travelers_pad_arbiter_output_guard *guard, bool observe_wheel)
{
	int rc = input_report(dev, type, code, value, sync, K_NO_WAIT);
	int observation_rc = rc == 0 && observe_wheel ?
		copy_canonical_wheel(dev, type, code, value, sync) : 0;

	travelers_pad_arbiter_end_output(guard);
	if (rc != 0) {
		LOG_ERR("%s output report failed: %d", producer, rc);
	}
	if (observation_rc != 0) {
		LOG_WRN("%s canonical wheel observation dropped: %d", producer, observation_rc);
	}
	return rc;
}

static void retry_pending_button_ups(const char *producer)
{
	for (size_t index = 0; index < ARRAY_SIZE(held_buttons); index++) {
		struct held_button_record record = {};
		k_spinlock_key_t key = k_spin_lock(&held_buttons_lock);

		if (held_buttons[index].held && held_buttons[index].release_pending &&
		    !held_buttons[index].release_in_flight) {
			held_buttons[index].release_in_flight = true;
			record = held_buttons[index];
		}
		k_spin_unlock(&held_buttons_lock, key);
		if (!record.held) {
			continue;
		}
		if (emit(record.dev, INPUT_EV_KEY, record.button, 0, true, producer) == 0) {
			finish_button_release(record.dev, record.ownership_frame_seq,
					      record.recognizer, record.owner, record.button, true);
			LOG_INF("%s retried button-up frame=%u button=%u", producer,
				record.ownership_frame_seq, record.button);
		} else {
			finish_button_release(record.dev, record.ownership_frame_seq,
					      record.recognizer, record.owner, record.button, false);
		}
	}
}

void travelers_pad_output_cancel_all(const char *producer)
{
	cancel_clicks(0, true);
	for (size_t index = 0; index < ARRAY_SIZE(held_buttons); index++) {
		k_spinlock_key_t key = k_spin_lock(&held_buttons_lock);
		if (held_buttons[index].held) {
			held_buttons[index].release_pending = true;
		}
		k_spin_unlock(&held_buttons_lock, key);
	}
	retry_pending_button_ups(producer == NULL ? "broker-cancel" : producer);
}

int travelers_pad_output_primary(const struct device *dev, uint32_t ownership_frame_seq,
				enum travelers_pad_recognizer_id recognizer,
				enum travelers_pad_owner owner, uint32_t capability,
				uint8_t type, uint16_t code, int32_t value, bool sync,
				const char *producer)
{
	int rc;
	struct travelers_pad_arbiter_output_guard guard;
	retry_pending_button_ups("broker-primary-retry");

	if (!travelers_pad_arbiter_begin_primary_output(ownership_frame_seq, recognizer, owner,
						capability, producer, &guard)) {
		return -EACCES;
	}
	/* A live drag takes precedence over queued tap pulses for that button. */
	if (capability == TRAVELERS_PAD_CAP_BUTTON && type == INPUT_EV_KEY && value != 0) {
		cancel_clicks(code, false);
		if (button_busy(code)) {
			travelers_pad_arbiter_end_output(&guard);
			return -EAGAIN;
		}
	}
	if (capability == TRAVELERS_PAD_CAP_BUTTON && type == INPUT_EV_KEY && value != 0 &&
	    held_button_exists(dev, ownership_frame_seq, recognizer, owner, code)) {
		travelers_pad_arbiter_end_output(&guard);
		LOG_WRN("%s duplicate held button denied frame=%u button=%u", producer,
			ownership_frame_seq, code);
		return -EALREADY;
	}
	if (capability == TRAVELERS_PAD_CAP_BUTTON && type == INPUT_EV_KEY && value != 0 &&
	    !remember_held_button(dev, ownership_frame_seq, recognizer, owner, code)) {
		travelers_pad_arbiter_end_output(&guard);
		LOG_ERR("%s held-button ledger full frame=%u button=%u", producer,
			ownership_frame_seq, code);
		return -ENOMEM;
	}
	rc = emit_guarded(dev, type, code, value, sync, producer, &guard, false);
	if (rc != 0 && capability == TRAVELERS_PAD_CAP_BUTTON && type == INPUT_EV_KEY && value != 0) {
		(void)take_held_button(dev, ownership_frame_seq, recognizer, owner, code);
	}
	return rc;
}

int travelers_pad_output_auxiliary(const struct device *dev,
				  const struct travelers_pad_auxiliary_lease *lease,
				  enum travelers_pad_auxiliary_id auxiliary,
				  enum travelers_pad_owner owner, uint32_t capability,
				  uint8_t type, uint16_t code, int32_t value, bool sync,
				  const char *producer)
{
	struct travelers_pad_arbiter_output_guard guard;
	retry_pending_button_ups("broker-auxiliary-retry");
	if (!travelers_pad_arbiter_begin_auxiliary_output(lease, auxiliary, owner, capability,
							  producer, &guard)) {
		return -EACCES;
	}
	return emit_guarded(dev, type, code, value, sync, producer, &guard,
			    owner == TRAVELERS_PAD_OWNER_SCROLL);
}

int travelers_pad_output_post_session(const struct device *dev,
				    const struct travelers_pad_owner_token *lease,
				    enum travelers_pad_owner owner, uint32_t capability,
				    uint8_t type, uint16_t code, int32_t value, bool sync,
				    const char *producer)
{
	struct travelers_pad_arbiter_output_guard guard;
	retry_pending_button_ups("broker-post-session-retry");
	if (!travelers_pad_arbiter_begin_post_session_output(lease, owner, capability, producer,
							    &guard)) {
		return -EACCES;
	}
	return emit_guarded(dev, type, code, value, sync, producer, &guard, false);
}

int travelers_pad_output_terminal(const struct device *dev, uint32_t ownership_frame_seq,
				 enum travelers_pad_recognizer_id recognizer,
				 enum travelers_pad_owner owner, uint32_t capability,
				 uint8_t type, uint16_t code, int32_t value, bool sync,
				 const char *producer)
{
	int rc;

	retry_pending_button_ups("broker-terminal-retry");
	if (capability == TRAVELERS_PAD_CAP_BUTTON && type == INPUT_EV_KEY && value == 0 &&
	    !begin_button_release(dev, ownership_frame_seq, recognizer, owner, code)) {
		LOG_WRN("%s terminal button release denied: no matching held press frame=%u button=%u",
			producer, ownership_frame_seq, code);
		return -EACCES;
	}
	(void)travelers_pad_arbiter_note_terminal_output(ownership_frame_seq, recognizer, owner,
						 capability, producer);
	rc = emit(dev, type, code, value, sync, producer);
	if (capability == TRAVELERS_PAD_CAP_BUTTON && type == INPUT_EV_KEY && value == 0) {
		finish_button_release(dev, ownership_frame_seq, recognizer, owner, code, rc == 0);
	}
	return rc;
}

/* Accepted terminal actions become bounded broker-owned transactions. The
 * original grant is consumed once at enqueue; delayed pulses never replay it.
 * No sleeps or arbiter locks are held across the pacing interval.
 */
#define CLICK_QUEUE_SIZE 8
struct paced_click {
	const struct device *dev;
	uint32_t id;
	enum travelers_pad_recognizer_id recognizer;
	enum travelers_pad_owner owner;
	uint16_t button;
	uint8_t remaining;
	bool down;
};
static struct paced_click click_queue[CLICK_QUEUE_SIZE];
static uint8_t click_head, queued_clicks;
static int64_t click_deadline;
static struct k_spinlock click_lock;
static void click_work_handler(struct k_work *work);
static K_WORK_DELAYABLE_DEFINE(click_work, click_work_handler);

static bool button_busy(uint16_t button)
{
	k_spinlock_key_t key = k_spin_lock(&held_buttons_lock);
	bool busy = false;
	for (size_t i = 0; i < ARRAY_SIZE(held_buttons); i++) {
		busy |= held_buttons[i].held && held_buttons[i].button == button;
	}
	k_spin_unlock(&held_buttons_lock, key);
	return busy;
}

static int release_click(struct paced_click *click)
{
	if (!held_button_exists(click->dev, click->id, click->recognizer,
				click->owner, click->button)) {
		return 0;
	}
	if (!begin_button_release(click->dev, click->id, click->recognizer,
				  click->owner, click->button)) {
		return -EAGAIN;
	}
	int rc = input_report(click->dev, INPUT_EV_KEY, click->button, 0, true, K_NO_WAIT);
	finish_button_release(click->dev, click->id, click->recognizer,
			      click->owner, click->button, rc == 0);
	return rc;
}

static void cancel_clicks(uint16_t button, bool all)
{
	k_spinlock_key_t key = k_spin_lock(&click_lock);
	for (uint8_t i = 0; i < queued_clicks; i++) {
		struct paced_click *click = &click_queue[(click_head + i) % CLICK_QUEUE_SIZE];
		if (all || click->button == button) {
			click->remaining = 0;
			if (click->down && release_click(click) == 0) {
				click->down = false;
			}
		}
	}
	/* Failed button-ups remain in the exact-owner ledger for retry. */
	k_spin_unlock(&click_lock, key);
}

static void click_work_handler(struct k_work *work)
{
	ARG_UNUSED(work);
	int emitted_value = -1;
	uint16_t emitted_button = 0;
	k_spinlock_key_t key = k_spin_lock(&click_lock);
	int64_t now = k_uptime_get();
	if (now < click_deadline) {
		k_work_reschedule(&click_work, K_MSEC(click_deadline - now));
		k_spin_unlock(&click_lock, key);
		return;
	}
	while (queued_clicks != 0) {
		struct paced_click *click = &click_queue[click_head];
		if (click->down) {
			if (release_click(click) != 0) {
				click_deadline = now + CONFIG_TRAVELERS_PAD_CLICK_PRESS_MS;
				break;
			}
			click->down = false;
			emitted_value = 0;
			emitted_button = click->button;
			click_deadline = k_uptime_get() + CONFIG_TRAVELERS_PAD_CLICK_GAP_MS;
			break;
		}
		if (click->remaining == 0) {
			click_head = (click_head + 1) % CLICK_QUEUE_SIZE;
			queued_clicks--;
			continue;
		}
		/* Never let a delayed tap press/release disturb a live drag. */
		if (button_busy(click->button)) {
			click->remaining = 0;
			continue;
		}
		if (!remember_held_button(click->dev, click->id, click->recognizer,
					  click->owner, click->button)) {
			click_deadline = now + CONFIG_TRAVELERS_PAD_CLICK_PRESS_MS;
			break;
		}
		int rc = input_report(click->dev, INPUT_EV_KEY, click->button, 1, true, K_NO_WAIT);
		if (rc != 0) {
			(void)take_held_button(click->dev, click->id, click->recognizer,
					      click->owner, click->button);
			click_deadline = now + CONFIG_TRAVELERS_PAD_CLICK_PRESS_MS;
			break;
		}
		click->down = true;
		emitted_value = 1;
		emitted_button = click->button;
		click->remaining--;
		click_deadline = k_uptime_get() + CONFIG_TRAVELERS_PAD_CLICK_PRESS_MS;
		break;
	}
	if (queued_clicks != 0) {
		k_work_reschedule(&click_work, K_MSEC(MAX(1, click_deadline - now)));
	}
	k_spin_unlock(&click_lock, key);
	if (emitted_value >= 0) {
		LOG_INF("paced click button=%u value=%d t=%lld", emitted_button,
			emitted_value, (long long)now);
	}
}

int travelers_pad_output_terminal_click(
	const struct device *dev, struct travelers_pad_terminal_action_grant *grant,
	enum travelers_pad_recognizer_id recognizer, enum travelers_pad_owner owner,
	uint16_t button, uint8_t click_count, const char *producer)
{
	struct travelers_pad_arbiter_output_guard guard;
	retry_pending_button_ups("broker-terminal-click-retry");

	if (click_count == 0 || click_count > 2) {
		return -EINVAL;
	}
	if (!travelers_pad_arbiter_begin_terminal_action_output(
		grant, recognizer, owner, TRAVELERS_PAD_CAP_BUTTON, producer, &guard)) {
		return -EACCES;
	}
	k_spinlock_key_t key = k_spin_lock(&click_lock);
	if (queued_clicks == CLICK_QUEUE_SIZE) {
		k_spin_unlock(&click_lock, key);
		travelers_pad_arbiter_end_output(&guard);
		return -ENOSPC;
	}
	click_queue[(click_head + queued_clicks) % CLICK_QUEUE_SIZE] = (struct paced_click){
		.dev = dev, .id = grant->grant_generation, .recognizer = recognizer,
		.owner = owner, .button = button, .remaining = click_count,
	};
	queued_clicks++;
	k_work_reschedule(&click_work, K_MSEC(MAX(1, click_deadline - k_uptime_get())));
	k_spin_unlock(&click_lock, key);
	travelers_pad_arbiter_end_output(&guard);
	LOG_INF("terminal click queued button=%u count=%u press=%dms gap=%dms",
		button, click_count, CONFIG_TRAVELERS_PAD_CLICK_PRESS_MS,
		CONFIG_TRAVELERS_PAD_CLICK_GAP_MS);
	return 0;
}

int travelers_pad_output_behavior_primary(
	const struct device *dev, kscan_callback_t callback, uint32_t ownership_frame_seq,
	enum travelers_pad_recognizer_id recognizer, enum travelers_pad_owner owner,
	uint16_t row, uint16_t column, const char *producer)
{
	retry_pending_button_ups("broker-behavior-retry");
	if (callback == NULL) {
		LOG_WRN("%s ignored: KSCAN callback is not configured", producer);
		return -ENODEV;
	}
	if (!travelers_pad_arbiter_authorize_output(
		ownership_frame_seq, recognizer, owner, TRAVELERS_PAD_CAP_BEHAVIOR_ACTION,
		producer)) {
		return -EACCES;
	}

	callback(dev, row, column, true);
	callback(dev, row, column, false);
	return 0;
}

int travelers_pad_output_routed_pointer(const struct device *dev, uint32_t routed_frame_seq,
				 enum travelers_pad_owner owner, uint32_t capability,
				 uint8_t type, uint16_t code, int32_t value, bool sync,
				 const char *producer)
{
	struct travelers_pad_arbiter_output_guard guard;

	retry_pending_button_ups("broker-pointer-retry");
	if (!travelers_pad_arbiter_begin_routed_output(routed_frame_seq, owner, capability,
							   producer, &guard)) {
		LOG_DBG("%s routed output denied frame=%u owner=%u", producer, routed_frame_seq,
			owner);
		return -EACCES;
	}
	return emit_guarded(dev, type, code, value, sync, producer, &guard,
			    owner == TRAVELERS_PAD_OWNER_SCROLL);
}
