/*
 * Contact-session manager / 接触セッション管理器.
 *
 * The session manager observes the normalized raw frame stream and establishes
 * the canonical session boundary. Contact ownership is intentionally absent:
 * the arbiter is the only ownership authority.
 *
 * session managerは正規化済み生frameを観測し、正規のsession境界を確立する。
 * 接触所有権は意図的に保持せず、arbiterだけを唯一の所有権authorityとする。
 */
#define DT_DRV_COMPAT zmk_travelers_pad_contact_session_manager

#include <zephyr/device.h>
#include <zephyr/dt-bindings/input/input-event-codes.h>
#include <zephyr/input/input.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <travelers_pad/contact_arbiter.h>
#include <travelers_pad/contact_output_broker.h>
#include <travelers_pad/contact_session.h>
#include <travelers_pad/contact_stability.h>
#include <travelers_pad/multitouch.h>

LOG_MODULE_REGISTER(travelers_pad_session, LOG_LEVEL_INF);

/* Private unsynchronized wake marker, never a contact frame or owner claim.
 * 非syncの内部wake marker。接触frame・所有権claimとしては扱わない。 */
#define SESSION_CONFIRM_WAKE 0x7f01

struct session_data {
	const struct device *dev;
	struct k_work_delayable confirm_work;
	struct k_mutex lock;
	bool receiving_frame;
	uint32_t next_session_id;
	uint32_t next_frame_seq;
	struct travelers_pad_session_snapshot snapshot;
	/* The active snapshot is cleared on a terminal frame. Keep the immutable
	 * terminal copy for the arbiter while downstream recognizers process that
	 * same frame. / terminal frameでactive snapshotを消しても、downstream
	 * recognizerが同frameを処理する間は不変copyをarbiterへ渡す。 */
	struct travelers_pad_session_snapshot dispatch_snapshot;
	bool reset_pending;
	uint8_t slot, raw_mask, emitted_mask, representative;
	uint16_t x[TRAVELERS_PAD_MAX_CONTACTS], y[TRAVELERS_PAD_MAX_CONTACTS];
	int32_t id[TRAVELERS_PAD_MAX_CONTACTS], emitted_id[TRAVELERS_PAD_MAX_CONTACTS];
	bool have_representative;
	struct travelers_pad_contact_stability stability;
};

struct session_config { uint16_t extra_contact_confirm_ms; };

static void session_confirm_work(struct k_work *work)
{
	struct session_data *data = CONTAINER_OF(k_work_delayable_from_work(work),
		struct session_data, confirm_work);
	/* Only enqueue a wake; raw/session fields are accessed in the serialized
	 * callbacks below. Queue pressure must not silently lose confirmation.
	 * wakeだけを投入し、raw/session状態は直列化callback内で扱う。 */
	if (input_report(data->dev, INPUT_EV_MSC, SESSION_CONFIRM_WAKE, 0, false,
			 K_NO_WAIT) != 0)
		k_work_reschedule(&data->confirm_work, K_MSEC(1));
}

static void session_schedule_confirmation(struct session_data *data, uint16_t confirm_ms)
{
	if (data->stability.pending) {
		int64_t remaining = data->stability.pending_since + confirm_ms - k_uptime_get();
		k_work_reschedule(&data->confirm_work, K_MSEC(MAX(remaining, 1)));
	} else {
		k_work_cancel_delayable(&data->confirm_work);
	}
}

static void session_process(const struct device *dev)
{
	struct session_data *data = dev->data;
	struct travelers_pad_session_snapshot *s = &data->snapshot;

	if (data->reset_pending) {
		if (s->contact_count || s->session_id) {
			LOG_WRN("session=%u cancelled", s->session_id);
		}
		data->dispatch_snapshot = *s;
		data->dispatch_snapshot.contact_count = 0;
		data->dispatch_snapshot.cancelled = true;
		/* `dispatch_snapshot` carries the one cancellation event downstream.
		 * The next physical contact must start a fresh non-cancelled session;
		 * retaining this flag would poison every later session after one reset.
		 * `dispatch_snapshot`だけが一回分のcancel eventを下流へ渡す。次の物理接触は
		 * 新しい非cancel sessionとして開始しなければならない。flagを残すと一度の
		 * reset後の全sessionがcancel扱いになる。 */
		*s = (struct travelers_pad_session_snapshot){};
		data->reset_pending = false;
		return;
	}
	if (s->contact_count && s->session_id == 0) {
		s->session_id = ++data->next_session_id;
		s->peak_contact_count = s->contact_count;
		LOG_INF("session=%u start contacts=%u", s->session_id, s->contact_count);
	}
	if (s->contact_count > s->peak_contact_count) {
		s->peak_contact_count = s->contact_count;
	}
	if (s->contact_count == 0 && s->session_id) {
		LOG_INF("session=%u end peak=%u", s->session_id, s->peak_contact_count);
		data->dispatch_snapshot = *s;
		*s = (struct travelers_pad_session_snapshot){};
		return;
	}
	data->dispatch_snapshot = *s;
}

static void session_emit_frame(const struct device *dev)
{
	struct session_data *data = dev->data;
	const struct session_config *cfg = dev->config;
	bool reset = data->reset_pending;
	if (reset) {
		data->raw_mask = 0;
		data->stability = (struct travelers_pad_contact_stability){};
	}
	uint8_t mask = travelers_pad_stabilize_contacts(&data->stability, data->raw_mask,
		k_uptime_get(), cfg->extra_contact_confirm_ms);
	session_schedule_confirmation(data, cfg->extra_contact_confirm_ms);
	uint8_t first = data->have_representative && (mask & BIT(data->representative)) ?
		data->representative : travelers_pad_first_contact(mask);
	uint8_t second = first < TRAVELERS_PAD_MAX_CONTACTS ?
		travelers_pad_first_contact(mask & ~BIT(first)) : TRAVELERS_PAD_MAX_CONTACTS;
	bool changed = mask && data->have_representative && first != data->representative;
	data->snapshot.contact_count = travelers_pad_contact_count(mask);
	data->snapshot.frame_seq = ++data->next_frame_seq;
	data->snapshot.timestamp_ms = k_uptime_get();
	session_process(dev);
	/* Every primary receives the same filtered frame. No proposal/owner change.
	 * 全primaryへ同一の安定化frameを配信。proposal/ownerは操作しない。 */
	travelers_pad_arbiter_begin_frame(&data->dispatch_snapshot);
	int rc = 0;
	for (uint8_t slot = 0; slot < TRAVELERS_PAD_MAX_CONTACTS; slot++) {
		bool active = mask & BIT(slot);
		bool was_active = data->emitted_mask & BIT(slot);
		if (!active && !was_active) continue;
		rc |= input_report_abs(dev, INPUT_ABS_MT_SLOT, slot, false, K_NO_WAIT);
		if (active != was_active || (active && data->id[slot] != data->emitted_id[slot])) {
			rc |= input_report_abs(dev, TRAVELERS_PAD_INPUT_ABS_MT_TRACKING_ID,
				active ? data->id[slot] : -1, false, K_NO_WAIT);
		}
		if (active) {
			rc |= input_report_abs(dev, TRAVELERS_PAD_INPUT_ABS_MT_POSITION_X, data->x[slot], false, K_NO_WAIT);
			rc |= input_report_abs(dev, TRAVELERS_PAD_INPUT_ABS_MT_POSITION_Y, data->y[slot], false, K_NO_WAIT);
		}
		data->emitted_id[slot] = data->id[slot];
	}
	if (reset) rc |= input_report_abs(dev, INPUT_ABS_RZ, 1, false, K_NO_WAIT);
	rc |= input_report(dev, INPUT_EV_MSC, TRAVELERS_PAD_INPUT_MSC_CONTACT_REFERENCE_CHANGED,
		changed, false, K_NO_WAIT);
	rc |= input_report_abs(dev, INPUT_ABS_X, mask ? data->x[first] : UINT16_MAX, false, K_NO_WAIT);
	rc |= input_report_abs(dev, INPUT_ABS_Y, mask ? data->y[first] : UINT16_MAX, false, K_NO_WAIT);
	rc |= input_report_abs(dev, INPUT_ABS_RX, second < TRAVELERS_PAD_MAX_CONTACTS ? data->x[second] : UINT16_MAX, false, K_NO_WAIT);
	rc |= input_report_abs(dev, INPUT_ABS_RY, second < TRAVELERS_PAD_MAX_CONTACTS ? data->y[second] : UINT16_MAX, false, K_NO_WAIT);
	rc |= input_report_abs(dev, INPUT_ABS_RUDDER, second < TRAVELERS_PAD_MAX_CONTACTS, false, K_NO_WAIT);
	rc |= input_report_abs(dev, INPUT_ABS_Z, data->snapshot.contact_count, false, K_NO_WAIT);
	/* snapshot was cleared at normal release; both values are zero then. */
	rc |= input_report_key(dev, INPUT_BTN_TOUCH, mask != 0, true, K_NO_WAIT);
	data->emitted_mask = mask;
	data->representative = first;
	data->have_representative = mask != 0;
	if (rc != 0) LOG_ERR("session frame forward failed: %d", rc);
}

static void session_input(const struct device *dev, struct input_event *evt)
{
	struct session_data *data = dev->data;
	k_mutex_lock(&data->lock, K_FOREVER);
	data->receiving_frame = true;
	if (evt->type == INPUT_EV_ABS && evt->code == INPUT_ABS_MT_SLOT) {
		data->slot = evt->value;
	} else if (evt->type == INPUT_EV_ABS && data->slot < TRAVELERS_PAD_MAX_CONTACTS &&
		   evt->code == TRAVELERS_PAD_INPUT_ABS_MT_TRACKING_ID) {
		data->id[data->slot] = evt->value;
		if (evt->value >= 0) data->raw_mask |= BIT(data->slot);
		else data->raw_mask &= ~BIT(data->slot);
	} else if (evt->type == INPUT_EV_ABS && data->slot < TRAVELERS_PAD_MAX_CONTACTS &&
		   evt->code == TRAVELERS_PAD_INPUT_ABS_MT_POSITION_X) {
		data->x[data->slot] = evt->value;
	} else if (evt->type == INPUT_EV_ABS && data->slot < TRAVELERS_PAD_MAX_CONTACTS &&
		   evt->code == TRAVELERS_PAD_INPUT_ABS_MT_POSITION_Y) {
		data->y[data->slot] = evt->value;
	} else if (evt->type == INPUT_EV_ABS && evt->code == INPUT_ABS_Z && evt->value == 0) {
		data->raw_mask = 0;
	} else if (evt->type == INPUT_EV_ABS && evt->code == INPUT_ABS_RZ && evt->value) {
		data->reset_pending = true;
		/* Do not wait for a session-bound cancellation frame: the active snapshot
		 * may already have been cleared after a normal release while post-session
		 * output is still pending. */
		travelers_pad_arbiter_cancel_all("sensor-reset-event");
		travelers_pad_output_cancel_all("sensor-reset-event");
	}
	if (evt->sync) {
		session_emit_frame(dev);
		data->receiving_frame = false;
	}
	k_mutex_unlock(&data->lock);
}

static void session_confirm_input(const struct device *dev, struct input_event *evt)
{
	if (evt->type != INPUT_EV_MSC || evt->code != SESSION_CONFIRM_WAKE) return;
	struct session_data *data = dev->data;
	const struct session_config *cfg = dev->config;
	k_mutex_lock(&data->lock, K_FOREVER);
	/* A stale wake after release/reset is harmless. Never confirm a partially
	 * received physical frame, nor let an old wake mature a new candidate early.
	 * 解放/reset後の古いwakeは無効。受信途中frameや新候補の早期確定も防ぐ。 */
	if (data->stability.pending && !data->reset_pending) {
		if (data->receiving_frame) {
			k_work_reschedule(&data->confirm_work, K_MSEC(1));
		} else if (k_uptime_get() - data->stability.pending_since < cfg->extra_contact_confirm_ms) {
			session_schedule_confirmation(data, cfg->extra_contact_confirm_ms);
		} else {
			LOG_INF("contact confirmation deadline reached; emitting complete frame");
			session_emit_frame(dev);
		}
	}
	k_mutex_unlock(&data->lock);
}

static int session_init(const struct device *dev)
{
	struct session_data *data = dev->data;
	data->dev = dev;
	k_mutex_init(&data->lock);
	k_work_init_delayable(&data->confirm_work, session_confirm_work);
	return 0;
}

#define SESSION_INST(n) \
	static struct session_data session_data_##n; \
	static const struct session_config session_config_##n = { \
		.extra_contact_confirm_ms = DT_INST_PROP_OR(n, extra_contact_confirm_ms, 35) }; \
	static void session_input_##n(struct input_event *evt) { session_input(DEVICE_DT_INST_GET(n), evt); } \
	static void session_confirm_##n(struct input_event *evt) { session_confirm_input(DEVICE_DT_INST_GET(n), evt); } \
	INPUT_CALLBACK_DEFINE(DEVICE_DT_GET(DT_INST_PHANDLE(n, input_device)), session_input_##n); \
	INPUT_CALLBACK_DEFINE(DEVICE_DT_INST_GET(n), session_confirm_##n); \
	DEVICE_DT_INST_DEFINE(n, session_init, NULL, &session_data_##n, &session_config_##n, POST_KERNEL, \
		CONFIG_TRAVELERS_PAD_IQS9150_FUNCTION_INIT_PRIORITY, NULL);
DT_INST_FOREACH_STATUS_OKAY(SESSION_INST)
