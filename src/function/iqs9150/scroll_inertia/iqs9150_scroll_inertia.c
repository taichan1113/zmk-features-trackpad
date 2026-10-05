/*
 * Travelers Pad two-finger scroll inertia.
 *
 * Canonical pointer wheel values exist only after the router has committed a
 * two-finger session. This function observes those values, and receives the
 * raw contact stream as an arbiter observer so a physical release can be
 * associated with the committed scroll token. Router-internal cancellation
 * frames (such as pinch/rotation ownership) therefore cannot start inertia.
 *
 * 標準ポインタのホイール値はrouterが2本指セッションを確定した後だけに存在する。
 * 本機能はその値を観測し、arbiter observerとして生接触ストリームを受け取る。
 * 物理的な解放を確定scroll tokenと対応付けることで、ピンチ／回転取得などrouter内部の
 * キャンセルフレームで慣性を開始しない。
 */

#define DT_DRV_COMPAT zmk_travelers_pad_scroll_inertia

#include <errno.h>

#include <zephyr/device.h>
#include <zephyr/dt-bindings/input/input-event-codes.h>
#include <zephyr/input/input.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <travelers_pad/contact_arbiter.h>
#include <travelers_pad/contact_output_broker.h>
#include <travelers_pad/inertia_math.h>

LOG_MODULE_REGISTER(travelers_pad_scroll_inertia, LOG_LEVEL_INF);

struct scroll_inertia_config {
	const struct device *wheel_input_device;
	const struct device *wheel_output_device;
	uint16_t start_delay_ms;
	uint16_t repeat_ms;
	uint8_t decay_numerator;
	uint8_t decay_denominator;
	uint16_t stop_delta;
	uint16_t max_delta;
	uint16_t half_life_ms;
	uint16_t velocity_window_ms;
	uint16_t release_timeout_ms;
	uint16_t max_duration_ms;
	uint16_t stop_speed;
};

struct scroll_inertia_data {
	const struct device *dev;
	uint8_t contact_fingers;
	int16_t horizontal_velocity;
	int16_t vertical_velocity;
	int32_t horizontal_velocity_fp;
	int32_t vertical_velocity_fp;
	int64_t horizontal_remainder;
	int64_t vertical_remainder;
	int64_t last_wheel_ms;
	int64_t last_contact_ms;
	int64_t release_ms;
	int64_t last_tick_ms;
	uint32_t contact_interval_ms;
	bool have_wheel;
	bool emitting;
	bool release_pending;
	bool post_session_pending;
	uint32_t arbiter_frame_seq;
	uint32_t post_session_id;
	struct travelers_pad_owner_token post_session_lease;
	struct k_work_delayable work;
	struct k_work_sync work_sync;
	struct travelers_pad_wheel_observation wheel_observation;
	uint32_t work_generation;
};

static int16_t absolute_int16(int16_t value)
{
	return value < 0 ? -value : value;
}

static int16_t clamp_delta(int32_t value, uint16_t maximum)
{
	if (value > (int16_t)maximum) {
		return maximum;
	}
	if (value < -(int16_t)maximum) {
		return -maximum;
	}
	return value;
}

static int16_t decay_velocity(int16_t value, const struct scroll_inertia_config *cfg)
{
	return (int16_t)(((int32_t)value * cfg->decay_numerator) / cfg->decay_denominator);
}

static void clear_inertia_state(struct scroll_inertia_data *data)
{
	data->horizontal_velocity = 0;
	data->vertical_velocity = 0;
	data->horizontal_velocity_fp = 0;
	data->vertical_velocity_fp = 0;
	data->horizontal_remainder = 0;
	data->vertical_remainder = 0;
	data->last_wheel_ms = 0;
	data->last_contact_ms = 0;
	data->contact_interval_ms = 0;
	data->have_wheel = false;
	data->release_pending = false;
	data->post_session_pending = false;
	data->post_session_id = 0;
	data->post_session_lease = (struct travelers_pad_owner_token){};
}

/* Contact input can replace the lease and velocity while the system workqueue
 * is delivering a glide tick. Quiesce that handler first so an old invocation
 * cannot observe state belonging to a following contact. This is intentionally
 * never called by inertia_work_handler(): waiting for the current work item
 * from its own handler is undefined. */
static void stop_inertia(const struct device *dev)
{
	struct scroll_inertia_data *data = dev->data;

	(void)k_work_cancel_delayable_sync(&data->work, &data->work_sync);
	data->work_generation++;
	clear_inertia_state(data);
}

static void stop_inertia_from_work(struct scroll_inertia_data *data)
{
	data->work_generation++;
	clear_inertia_state(data);
}

static void inertia_work_handler(struct k_work *work)
{
	struct scroll_inertia_data *data = CONTAINER_OF(work, struct scroll_inertia_data, work.work);
	const struct scroll_inertia_config *cfg = data->dev->config;
	int rc;
	uint32_t generation = data->work_generation;

	if (generation != data->work_generation || data->contact_fingers != 0) {
		stop_inertia_from_work(data);
		return;
	}
	if (!data->post_session_lease.valid) {
		if (!data->post_session_pending ||
		    !travelers_pad_arbiter_acquire_post_session_lease(
			    data->post_session_id, TRAVELERS_PAD_OWNER_SCROLL,
			    TRAVELERS_PAD_CAP_SCROLL_DELTA, &data->post_session_lease)) {
			LOG_DBG("scroll inertia cancelled: no normal-end lease");
			stop_inertia_from_work(data);
			return;
		}
		data->post_session_pending = false;
		LOG_INF("scroll inertia lease acquired session=%u generation=%u",
			data->post_session_lease.session_id,
			data->post_session_lease.owner_generation);
	}
	bool stopped;
	if (cfg->half_life_ms != 0) {
		int64_t now = k_uptime_get();
		if (now - data->release_ms >= cfg->max_duration_ms) {
			LOG_INF("scroll inertia stopped: duration limit");
			stop_inertia_from_work(data);
			return;
		}
		uint32_t elapsed = (uint32_t)(now - data->last_tick_ms);
		int32_t old_h = data->horizontal_velocity_fp;
		int32_t old_v = data->vertical_velocity_fp;
		data->last_tick_ms = now;
		data->horizontal_velocity_fp = travelers_pad_inertia_decay(old_h, elapsed, cfg->half_life_ms);
		data->vertical_velocity_fp = travelers_pad_inertia_decay(old_v, elapsed, cfg->half_life_ms);
		/* Integrate mean velocity over actual elapsed time, including the first
		 * delay. Keep residual travel and cap reports, not the time estimate.
		 * 初回遅延も含め実経過時間で平均速度を積分。移動端数を保持し出力を上限制限。 */
		int32_t h = travelers_pad_inertia_integrate((int32_t)(((int64_t)old_h + data->horizontal_velocity_fp) / 2),
			elapsed, &data->horizontal_remainder);
		int32_t v = travelers_pad_inertia_integrate((int32_t)(((int64_t)old_v + data->vertical_velocity_fp) / 2),
			elapsed, &data->vertical_remainder);
		data->horizontal_velocity = (int16_t)CLAMP(h, -(int32_t)cfg->max_delta, (int32_t)cfg->max_delta);
		data->vertical_velocity = (int16_t)CLAMP(v, -(int32_t)cfg->max_delta, (int32_t)cfg->max_delta);
		int64_t stop = (int64_t)cfg->stop_speed * TRAVELERS_PAD_INERTIA_ONE;
		int64_t h_speed = (int64_t)data->horizontal_velocity_fp * 1000;
		int64_t v_speed = (int64_t)data->vertical_velocity_fp * 1000;
		stopped = (h_speed < 0 ? -h_speed : h_speed) < stop &&
			  (v_speed < 0 ? -v_speed : v_speed) < stop;
	} else {
		data->horizontal_velocity = decay_velocity(data->horizontal_velocity, cfg);
		data->vertical_velocity = decay_velocity(data->vertical_velocity, cfg);
		stopped = absolute_int16(data->horizontal_velocity) < cfg->stop_delta &&
			  absolute_int16(data->vertical_velocity) < cfg->stop_delta;
	}
	if (stopped) {
		LOG_INF("scroll inertia stopped");
		stop_inertia_from_work(data);
		return;
	}

	data->emitting = true;
	if (data->horizontal_velocity != 0) {
		rc = travelers_pad_output_post_session(
			cfg->wheel_output_device, &data->post_session_lease, TRAVELERS_PAD_OWNER_SCROLL,
			TRAVELERS_PAD_CAP_SCROLL_DELTA, INPUT_EV_REL, INPUT_REL_HWHEEL,
			data->horizontal_velocity, data->vertical_velocity == 0, "scroll-inertia-hwheel");
		if (rc != 0) {
			if (rc == -EACCES) {
				LOG_DBG("scroll inertia cancelled: lease revoked");
				stop_inertia_from_work(data);
				return;
			}
			LOG_ERR("inertia horizontal report failed: %d", rc);
		}
	}
	if (data->vertical_velocity != 0) {
		rc = travelers_pad_output_post_session(
			cfg->wheel_output_device, &data->post_session_lease, TRAVELERS_PAD_OWNER_SCROLL,
			TRAVELERS_PAD_CAP_SCROLL_DELTA, INPUT_EV_REL, INPUT_REL_WHEEL,
			data->vertical_velocity, true, "scroll-inertia-wheel");
		if (rc != 0) {
			if (rc == -EACCES) {
				LOG_DBG("scroll inertia cancelled: lease revoked");
				stop_inertia_from_work(data);
				return;
			}
			LOG_ERR("inertia vertical report failed: %d", rc);
		}
	}
	data->emitting = false;
	if (generation == data->work_generation) {
		k_work_schedule(&data->work, K_MSEC(cfg->repeat_ms));
	}
}

static void inertia_handle_contact(const struct device *dev, struct input_event *evt)
{
	struct scroll_inertia_data *data = dev->data;

	if (evt->type == INPUT_EV_ABS && evt->code == INPUT_ABS_Z) {
		uint8_t previous_fingers = data->contact_fingers;

		data->contact_fingers = evt->value;
		if (data->contact_fingers != 0 && previous_fingers == 0) {
			/* Any new physical contact cancels an older glide. */
			stop_inertia(dev);
			data->release_pending = false;
		} else if (previous_fingers == 2) {
			/* Two contacts alone are not a scroll. Tip-Tap and a stationary
			 * two-finger click must not reuse an earlier wheel velocity. The
			 * arbiter owner is checked at the eventual terminal frame. */
			data->release_pending = true;
		}
	}
	/* The resolved observer handler performs cancellation and terminal-release
	 * decisions. At this point only record raw lifecycle evidence; it must not
	 * inspect an unresolved owner whose primary decision may not exist yet.
	 * 裁定済みobserver handlerがcancelとterminal releaseを判断する。ここでは生lifecycle
	 * evidenceだけを記録し、まだ存在しないかもしれないprimary決定を参照しない。 */
}

static void inertia_deliver_resolved_frame(
	void *context, const struct travelers_pad_session_snapshot *snapshot,
	const struct travelers_pad_owner_token *token)
{
	struct scroll_inertia_data *data = context;
	const struct scroll_inertia_config *cfg = data->dev->config;
	int64_t now = k_uptime_get();
	if (snapshot->contact_count != 0) {
		if (data->last_contact_ms != 0 && now > data->last_contact_ms) {
			data->contact_interval_ms = (uint32_t)(now - data->last_contact_ms);
		}
		data->last_contact_ms = now;
	}

	if (snapshot->cancelled) {
		/* A sensor reset is not a finger release. It must stop a running glide
		 * and may never create a new post-session lease. */
		LOG_INF("scroll inertia cancelled by sensor reset session=%u", snapshot->session_id);
		stop_inertia(data->dev);
		return;
	}

	if (data->release_pending && snapshot->contact_count == 0) {
		if (token != NULL && token->valid && token->owner == TRAVELERS_PAD_OWNER_SCROLL &&
		    (data->horizontal_velocity != 0 || data->vertical_velocity != 0) &&
		    (cfg->half_life_ms == 0 ||
		     (data->have_wheel && now - data->last_wheel_ms <= cfg->release_timeout_ms))) {
			/* The arbiter creates the post-session lease when this terminal frame
			 * resolves. Delay work until then and acquire that exact session lease
			 * before sending any wheel value. */
			data->post_session_pending = true;
			data->post_session_id = snapshot->session_id;
			data->post_session_lease = (struct travelers_pad_owner_token){};
			data->release_ms = now;
			data->last_tick_ms = now;
			if (cfg->half_life_ms != 0) {
				uint32_t age = (uint32_t)(now - data->last_wheel_ms);
				data->horizontal_velocity_fp = travelers_pad_inertia_decay(
					data->horizontal_velocity_fp, age, cfg->half_life_ms);
				data->vertical_velocity_fp = travelers_pad_inertia_decay(
					data->vertical_velocity_fp, age, cfg->half_life_ms);
				LOG_INF("inertia seed speed h=%d v=%d counts/s age=%ums",
					(int32_t)((int64_t)data->horizontal_velocity_fp * 1000 / TRAVELERS_PAD_INERTIA_ONE),
					(int32_t)((int64_t)data->vertical_velocity_fp * 1000 / TRAVELERS_PAD_INERTIA_ONE), age);
			}
			LOG_INF("scroll inertia pending normal-end lease session=%u h=%d v=%d",
				snapshot->session_id, data->horizontal_velocity, data->vertical_velocity);
			k_work_schedule(&data->work, K_MSEC(cfg->start_delay_ms));
		} else if (token == NULL || !token->valid ||
			   token->owner != TRAVELERS_PAD_OWNER_SCROLL) {
			/* The raw contact stream may look like a two-finger release for a
			 * click, Tip-Tap, geometry, or higher-priority action. */
			data->horizontal_velocity = 0;
			data->vertical_velocity = 0;
		} else if (cfg->half_life_ms != 0) {
			LOG_INF("scroll inertia skipped: stale/no motion");
			clear_inertia_state(data);
		}
		data->release_pending = false;
	}
}

static void inertia_handle_wheel(const struct device *dev, struct input_event *evt)
{
	struct scroll_inertia_data *data = dev->data;
	const struct scroll_inertia_config *cfg = dev->config;
	int16_t value;

	if (data->emitting || data->contact_fingers != 2 || evt->type != INPUT_EV_REL) {
		return;
	}
	value = clamp_delta(evt->value, cfg->max_delta);
	if ((evt->code != INPUT_REL_HWHEEL && evt->code != INPUT_REL_WHEEL) || value == 0) {
		return;
	}
	if (cfg->half_life_ms != 0) {
		int64_t now = k_uptime_get();
		bool fresh = !data->have_wheel || now - data->last_wheel_ms > cfg->release_timeout_ms;
		uint32_t fallback = MAX(data->contact_interval_ms, cfg->repeat_ms);
		uint32_t elapsed = fresh || now <= data->last_wheel_ms ? fallback :
			(uint32_t)(now - data->last_wheel_ms);
		if (evt->code == INPUT_REL_HWHEEL) {
			fresh |= data->vertical_velocity_fp != 0;
			data->horizontal_velocity_fp = travelers_pad_inertia_velocity(
				data->horizontal_velocity_fp, value, elapsed, cfg->velocity_window_ms, fresh);
			data->vertical_velocity_fp = 0;
		} else {
			fresh |= data->horizontal_velocity_fp != 0;
			data->vertical_velocity_fp = travelers_pad_inertia_velocity(
				data->vertical_velocity_fp, value, elapsed, cfg->velocity_window_ms, fresh);
			data->horizontal_velocity_fp = 0;
		}
		data->last_wheel_ms = now;
		data->have_wheel = true;
	}
	if (evt->code == INPUT_REL_HWHEEL && value != 0) {
		data->horizontal_velocity = value;
		data->vertical_velocity = 0;
	} else if (evt->code == INPUT_REL_WHEEL && value != 0) {
		data->horizontal_velocity = 0;
		data->vertical_velocity = value;
	}
}

static int inertia_init(const struct device *dev)
{
	const struct scroll_inertia_config *cfg = dev->config;
	struct scroll_inertia_data *data = dev->data;

	if (cfg->repeat_ms == 0 || cfg->max_delta == 0 || cfg->max_delta > INT16_MAX ||
	    (cfg->half_life_ms == 0 && (cfg->stop_delta == 0 || cfg->decay_denominator == 0 ||
	                              cfg->decay_numerator >= cfg->decay_denominator)) ||
	    (cfg->half_life_ms != 0 && (cfg->release_timeout_ms == 0 || cfg->max_duration_ms == 0 ||
	                               cfg->stop_speed == 0))) {
		return -EINVAL;
	}
	k_work_init_delayable(&data->work, inertia_work_handler);
	data->wheel_observation = (struct travelers_pad_wheel_observation){
		.source = cfg->wheel_output_device,
		.sink = cfg->wheel_input_device,
	};
	/* Observe an isolated copy, never the mutable host listener's event.
	 * HID listenerが書き換えるeventでなく、専用deviceの複製を観測する。 */
	int rc = travelers_pad_output_register_wheel_observation(&data->wheel_observation);
	if (rc != 0) {
		LOG_ERR("inertia requires a private canonical wheel input: %d", rc);
		return rc;
	}
	(void)travelers_pad_arbiter_register(TRAVELERS_PAD_RECOGNIZER_SCROLL_INERTIA);
	(void)travelers_pad_arbiter_bind_frame_handler(
		TRAVELERS_PAD_RECOGNIZER_SCROLL_INERTIA, inertia_deliver_resolved_frame, data);
	LOG_INF("scroll inertia active: delay=%ums period=%ums decay=%u/%u stop=%u",
		cfg->start_delay_ms, cfg->repeat_ms, cfg->decay_numerator, cfg->decay_denominator,
		cfg->stop_delta);
	LOG_INF("inertia mode=%s half-life=%ums window=%ums release-timeout=%ums max-duration=%ums",
		cfg->half_life_ms ? "time-based" : "legacy", cfg->half_life_ms,
		cfg->velocity_window_ms, cfg->release_timeout_ms, cfg->max_duration_ms);
	return 0;
}

#define SCROLL_INERTIA_INST(n)                                                                 \
	BUILD_ASSERT(DT_NODE_HAS_COMPAT(DT_INST_PHANDLE(n, wheel_input_device),                  \
		zmk_travelers_pad_canonical_wheel_input), "inertia needs a private canonical source"); \
	static struct scroll_inertia_data scroll_inertia_data_##n = {.dev = DEVICE_DT_INST_GET(n)};  \
	static const struct scroll_inertia_config scroll_inertia_config_##n = {                      \
		.wheel_input_device = DEVICE_DT_GET(DT_INST_PHANDLE(n, wheel_input_device)),            \
		.wheel_output_device = DEVICE_DT_GET(DT_INST_PHANDLE(n, wheel_output_device)),          \
		.start_delay_ms = DT_INST_PROP_OR(n, start_delay_ms, 20),                                \
		.repeat_ms = DT_INST_PROP_OR(n, repeat_ms, 24),                                           \
		.decay_numerator = DT_INST_PROP_OR(n, decay_numerator, 3),                               \
		.decay_denominator = DT_INST_PROP_OR(n, decay_denominator, 4),                           \
		.stop_delta = DT_INST_PROP_OR(n, stop_delta, 2),                                         \
		.max_delta = DT_INST_PROP_OR(n, max_delta, 64),                                          \
		.half_life_ms = DT_INST_PROP(n, half_life_ms),                                         \
		.velocity_window_ms = DT_INST_PROP(n, velocity_window_ms),                             \
		.release_timeout_ms = DT_INST_PROP(n, release_timeout_ms),                             \
		.max_duration_ms = DT_INST_PROP(n, max_duration_ms),                                   \
		.stop_speed = DT_INST_PROP(n, stop_speed),                                             \
	};                                                                                           \
	static void scroll_inertia_contact_handler_##n(struct input_event *evt)                      \
	{                                                                                            \
		struct scroll_inertia_data *data = &scroll_inertia_data_##n;                             \
		if (evt->sync) {                                                                          \
			data->arbiter_frame_seq = travelers_pad_arbiter_next_frame(                              \
				TRAVELERS_PAD_RECOGNIZER_SCROLL_INERTIA);                                            \
		}                                                                                         \
		inertia_handle_contact(DEVICE_DT_INST_GET(n), evt);                                      \
		if (evt->sync) {                                                                          \
			travelers_pad_arbiter_observed(TRAVELERS_PAD_RECOGNIZER_SCROLL_INERTIA,                \
				data->arbiter_frame_seq);                                                            \
		}                                                                                         \
	}                                                                                            \
	static void scroll_inertia_wheel_handler_##n(struct input_event *evt)                        \
	{                                                                                            \
		inertia_handle_wheel(DEVICE_DT_INST_GET(n), evt);                                        \
	}                                                                                            \
	INPUT_CALLBACK_DEFINE(DEVICE_DT_GET(DT_INST_PHANDLE(n, contact_input_device)),               \
			      scroll_inertia_contact_handler_##n);                                          \
	INPUT_CALLBACK_DEFINE(DEVICE_DT_GET(DT_INST_PHANDLE(n, wheel_input_device)),                 \
			      scroll_inertia_wheel_handler_##n);                                            \
	DEVICE_DT_INST_DEFINE(n, inertia_init, NULL, &scroll_inertia_data_##n,                       \
			      &scroll_inertia_config_##n, POST_KERNEL,                                     \
			      CONFIG_TRAVELERS_PAD_IQS9150_FUNCTION_INIT_PRIORITY, NULL);

DT_INST_FOREACH_STATUS_OKAY(SCROLL_INERTIA_INST)
