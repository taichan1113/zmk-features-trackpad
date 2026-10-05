/*
 * Travelers Pad IQS9150 active-touch input isolation function.
 *
 * zmk-input-matrix is an input processor. Attaching it directly to the
 * normalized session stream would invoke a ZMK input listener for every idle
 * report frame.
 * Moreover, an input processor that suppresses an event mutates Zephyr's
 * shared event object, which must not affect other session consumers.
 *
 * This device mirrors only active one-finger contact frames (and the one
 * release frame) onto a private input device. The matrix listener may safely
 * suppress those private events, while idle and multi-finger IQS9150 reports
 * produce no grid-listener work.
 *
 * zmk-input-matrixはinput processorである。正規化sessionストリームへ直接接続すると、
 * 待機中のレポートframeごとにZMK input listenerが動く。またevent抑止はZephyrの共有eventを
 * 変更するため、他のsession consumerへ影響してはならない。
 *
 * このデバイスは1本指接触中のフレームと、最後の解放フレームだけを専用input deviceへ複製する。
 * そのため格子listenerは専用eventだけを安全に抑止でき、未接触・多指IQS9150レポートは処理しない。
 */

#define DT_DRV_COMPAT zmk_travelers_pad_iqs9150_matrix_input

#include <zephyr/device.h>
#include <zephyr/dt-bindings/input/input-event-codes.h>
#include <zephyr/input/input.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(travelers_pad_matrix_input, LOG_LEVEL_INF);

struct matrix_input_config {
    const struct device *input;
    uint16_t one_finger_settle_ms;
};

struct matrix_input_data {
    const struct device *dev;
    int32_t x;
    int32_t y;
    uint8_t fingers;
    bool touch;
    bool forwarded;
    bool confirming;
    struct k_work_delayable confirm_work;
};

static int matrix_input_forward(const struct device *dev, bool touch)
{
    struct matrix_input_data *data = dev->data;
    int rc = input_report_abs(dev, INPUT_ABS_X, data->x, false, K_NO_WAIT);

    rc |= input_report_abs(dev, INPUT_ABS_Y, data->y, false, K_NO_WAIT);
    rc |= input_report_key(dev, INPUT_BTN_TOUCH, touch, true, K_NO_WAIT);
    data->forwarded = touch;
    return rc;
}

static void matrix_input_confirm_work(struct k_work *work)
{
    struct matrix_input_data *data =
        CONTAINER_OF(work, struct matrix_input_data, confirm_work.work);

    data->confirming = false;

    if (!data->touch || data->fingers != 1U || data->forwarded) {
        return;
    }

    int rc = matrix_input_forward(data->dev, true);
    if (rc != 0) {
        LOG_ERR("matrix input confirmed frame report failed: %d", rc);
    }
}

static void matrix_input_handle_event(const struct device *dev, struct input_event *evt)
{
    struct matrix_input_data *data = dev->data;
    const struct matrix_input_config *config = dev->config;

    if (evt->type == INPUT_EV_ABS) {
        if (evt->code == INPUT_ABS_X) {
            data->x = evt->value;
        } else if (evt->code == INPUT_ABS_Y) {
            data->y = evt->value;
        } else if (evt->code == INPUT_ABS_Z) {
            data->fingers = evt->value;
        }
        return;
    }

    if (evt->type != INPUT_EV_KEY || evt->code != INPUT_BTN_TOUCH) {
        return;
    }

    bool grid_touch;

    data->touch = evt->value != 0;
    grid_touch = data->touch && data->fingers == 1U;

    /* IQS9150 can report one contact for one frame while a multi-finger
     * gesture is being formed. Confirm one-finger contact before forwarding.
     * IQS9150は多指接触の形成中に一フレームだけ1本指と報告することがあるため、
     * 1本指接触を確定してから転送する。 */
    if (grid_touch && !data->forwarded && !data->confirming) {
        data->confirming = true;
        (void)k_work_reschedule(&data->confirm_work,
                                K_MSEC(config->one_finger_settle_ms));
        return;
    }

    if (!grid_touch) {
        (void)k_work_cancel_delayable(&data->confirm_work);
        data->confirming = false;
    }

    /* If a forwarded one-finger contact grows into multi-finger, forward
     * exactly one release to avoid a held virtual key.
     * 転送済みの1本指が多指へ移行したときだけ、仮想キーを残さない解放を1回だけ転送する。 */
    if (!data->forwarded) {
        return;
    }

    int rc = matrix_input_forward(dev, grid_touch);
    if (rc != 0) {
        LOG_ERR("matrix input frame report failed: %d", rc);
    }
}

static int matrix_input_init(const struct device *dev)
{
    struct matrix_input_data *data = dev->data;

    data->dev = dev;
    k_work_init_delayable(&data->confirm_work, matrix_input_confirm_work);
    return 0;
}

#define MATRIX_INPUT_INIT(n)                                                                    \
    static struct matrix_input_data matrix_input_data_##n;                                      \
    static const struct matrix_input_config matrix_input_config_##n = {                         \
        .input = DEVICE_DT_GET(DT_INST_PHANDLE(n, input_device)),                               \
        .one_finger_settle_ms = DT_INST_PROP_OR(n, one_finger_settle_ms, 45),                  \
    };                                                                                            \
    static void matrix_input_handler_##n(struct input_event *evt)                               \
    {                                                                                             \
        matrix_input_handle_event(DEVICE_DT_INST_GET(n), evt);                                  \
    }                                                                                             \
    INPUT_CALLBACK_DEFINE(DEVICE_DT_GET(DT_INST_PHANDLE(n, input_device)),                     \
                          matrix_input_handler_##n);                                            \
    DEVICE_DT_INST_DEFINE(n, matrix_input_init, NULL, &matrix_input_data_##n,                  \
                          &matrix_input_config_##n,                                             \
                          POST_KERNEL, CONFIG_TRAVELERS_PAD_IQS9150_FUNCTION_INIT_PRIORITY, NULL);

DT_INST_FOREACH_STATUS_OKAY(MATRIX_INPUT_INIT)
