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

LOG_MODULE_REGISTER(travelers_pad_session, LOG_LEVEL_INF);

struct session_data {
	uint32_t next_session_id;
	uint32_t next_frame_seq;
	struct travelers_pad_session_snapshot snapshot;
	/* The active snapshot is cleared on a terminal frame. Keep the immutable
	 * terminal copy for the arbiter while downstream recognizers process that
	 * same frame. / terminal frameでactive snapshotを消しても、downstream
	 * recognizerが同frameを処理する間は不変copyをarbiterへ渡す。 */
	struct travelers_pad_session_snapshot dispatch_snapshot;
	bool reset_pending;
};

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

static void session_input(const struct device *dev, struct input_event *evt)
{
	struct session_data *data = dev->data;
	if (evt->type == INPUT_EV_ABS && evt->code == INPUT_ABS_Z) {
		data->snapshot.contact_count = evt->value;
	} else if (evt->type == INPUT_EV_ABS && evt->code == INPUT_ABS_RZ && evt->value) {
		data->reset_pending = true;
		/* Do not wait for a session-bound cancellation frame: the active snapshot
		 * may already have been cleared after a normal release while post-session
		 * output is still pending. */
		travelers_pad_arbiter_cancel_all("sensor-reset-event");
		travelers_pad_output_cancel_all("sensor-reset-event");
	}
	if (evt->sync) {
		/* frame_seq is global rather than per-session so it can identify a
		 * forwarded frame after it crosses Zephyr's asynchronous input queue. */
		data->snapshot.frame_seq = ++data->next_frame_seq;
		data->snapshot.timestamp_ms = k_uptime_get();
		session_process(dev);
		/* The configured input mode is queued, not synchronous. Store the logical
		 * frame now; each arbiter recognizer later obtains its next unacknowledged
		 * frame in FIFO order while handling the forwarded sync event. No extra
		 * input event is injected, so a full queue cannot drop contact data.
		 * 設定されたinput modeは同期でなくqueue方式である。論理frameをここで保存し、
		 * 各arbiter recognizerはforwardされたsync event処理時にFIFO順で未通知frameを
		 * 得る。余分なinput eventを挿入しないため、満杯queueでも接触dataを落とさない。 */
		travelers_pad_arbiter_begin_frame(&data->dispatch_snapshot);
	}
	/* This is the sole subscription to the driver raw stream. Re-publish the
	 * immutable normalized frame after session bookkeeping; recognizers migrate
	 * to this device before their proposal APIs are introduced. */
	if (input_report(dev, evt->type, evt->code, evt->value, evt->sync, K_NO_WAIT) != 0) {
		LOG_ERR("session frame forward failed");
	}
}

#define SESSION_INST(n) \
	static struct session_data session_data_##n; \
	static void session_input_##n(struct input_event *evt) { session_input(DEVICE_DT_INST_GET(n), evt); } \
	INPUT_CALLBACK_DEFINE(DEVICE_DT_GET(DT_INST_PHANDLE(n, input_device)), session_input_##n); \
	DEVICE_DT_INST_DEFINE(n, NULL, NULL, &session_data_##n, NULL, POST_KERNEL, \
		CONFIG_TRAVELERS_PAD_IQS9150_FUNCTION_INIT_PRIORITY, NULL);
DT_INST_FOREACH_STATUS_OKAY(SESSION_INST)
