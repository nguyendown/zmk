/*
 * Copyright (c) 2022 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#include "debounce.h"

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/kscan.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/__assert.h>
#include <zephyr/sys/util.h>

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

#define DT_DRV_COMPAT zmk_kscan_gpio_charlieplex

#define INST_GPIOS_LEN(n) DT_INST_PROP_LEN(n, gpios)
#define INST_STATE_LEN(n) (INST_GPIOS_LEN(n) * INST_GPIOS_LEN(n))

#if CONFIG_ZMK_KSCAN_DEBOUNCE_PRESS_MS >= 0
#define INST_DEBOUNCE_PRESS_MS(n) CONFIG_ZMK_KSCAN_DEBOUNCE_PRESS_MS
#else
#define INST_DEBOUNCE_PRESS_MS(n) DT_INST_PROP(n, debounce_press_ms)
#endif

#if CONFIG_ZMK_KSCAN_DEBOUNCE_RELEASE_MS >= 0
#define INST_DEBOUNCE_RELEASE_MS(n) CONFIG_ZMK_KSCAN_DEBOUNCE_RELEASE_MS
#else
#define INST_DEBOUNCE_RELEASE_MS(n) DT_INST_PROP(n, debounce_release_ms)
#endif

#define KSCAN_GPIO_CFG_INIT(idx, inst_idx)                                                         \
    GPIO_DT_SPEC_GET_BY_IDX(DT_DRV_INST(inst_idx), gpios, idx)

struct kscan_charlieplex_data {
    const struct device *dev;
    kscan_callback_t callback;
    struct k_work_delayable work;
    int64_t scan_time;
    struct debounce_state *debounce_state;
};

struct kscan_charlieplex_config {
    const struct gpio_dt_spec *gpios;
    size_t gpios_len;
    struct debounce_config debounce_config;
    int32_t debounce_scan_period_ms;
    int32_t poll_period_ms;
};

static int state_index_io(const struct kscan_charlieplex_config *config, const int input_idx,
                          const int output_idx) {
    __ASSERT(input_idx < config->gpios_len, "Invalid input %i", input_idx);
    __ASSERT(output_idx < config->gpios_len, "Invalid output %i", output_idx);

    return ((input_idx * config->gpios_len) + output_idx);
}

static int kscan_charlieplex_read(const struct device *dev) {
    struct kscan_charlieplex_data *data = dev->data;
    const struct kscan_charlieplex_config *config = dev->config;

    // Process the new state.
    bool continue_scan = false;

    for (int o = 0; o < config->gpios_len; o++) {
        const struct gpio_dt_spec *out_gpio = &config->gpios[o];

        // Init output
        if (!device_is_ready(out_gpio->port)) {
            LOG_ERR("GPIO is not ready: %s", out_gpio->port->name);
            return -ENODEV;
        }

        int err = gpio_pin_configure_dt(out_gpio, GPIO_OUTPUT);
        if (err) {
            LOG_ERR("Unable to configure pin %u on %s for output", out_gpio->pin,
                    out_gpio->port->name);
            return err;
        }

        // Init input
        for (int i = (o + 1) % config->gpios_len; i != o; i = (i + 1) % config->gpios_len) {
            const struct gpio_dt_spec *in_gpio = &config->gpios[i];

            if (!device_is_ready(in_gpio->port)) {
                LOG_ERR("GPIO is not ready: %s", in_gpio->port->name);
                return -ENODEV;
            }

            err = gpio_pin_configure_dt(in_gpio, GPIO_INPUT);
            if (err) {
                LOG_ERR("Unable to configure pin %u on %s for input", in_gpio->pin,
                        in_gpio->port->name);
                return err;
            }
        }

        err = gpio_pin_set_dt(out_gpio, 1);
        if (err) {
            LOG_ERR("Failed to set output %i active: %i", o, err);
            return err;
        }

#if CONFIG_ZMK_KSCAN_CHARLIEPLEX_WAIT_BEFORE_INPUTS > 0
        k_busy_wait(CONFIG_ZMK_KSCAN_CHARLIEPLEX_WAIT_BEFORE_INPUTS);
#endif

        for (int i = (o + 1) % config->gpios_len; i != o; i = (i + 1) % config->gpios_len) {
            const struct gpio_dt_spec *in_gpio = &config->gpios[i];

            const int index = state_index_io(config, i, o);
            const bool is_active = gpio_pin_get_dt(in_gpio);
            struct debounce_state *state = &data->debounce_state[index];

            debounce_update(state, is_active, config->debounce_scan_period_ms,
                            &config->debounce_config);

            if (debounce_get_changed(state)) {
                const bool is_pressed = debounce_is_pressed(state);

                LOG_DBG("Sending event at %i,%i state %s", o, i, is_pressed ? "on" : "off");
                data->callback(dev, o, i, is_pressed);
            }

            continue_scan = continue_scan || debounce_is_active(state);
        }

        err = gpio_pin_set_dt(out_gpio, 0);
        if (err) {
            LOG_ERR("Failed to set output %i inactive: %i", o, err);
            return err;
        }

#if CONFIG_ZMK_KSCAN_CHARLIEPLEX_WAIT_BETWEEN_OUTPUTS > 0
        k_busy_wait(CONFIG_ZMK_KSCAN_CHARLIEPLEX_WAIT_BETWEEN_OUTPUTS);
#endif
    }

    if (continue_scan) {
        data->scan_time += config->debounce_scan_period_ms;
    } else {
        data->scan_time += config->poll_period_ms;
    }

    k_work_reschedule(&data->work, K_TIMEOUT_ABS_MS(data->scan_time));

    return 0;
}

static void kscan_charlieplex_work_handler(struct k_work *work) {
    struct k_work_delayable *dwork = CONTAINER_OF(work, struct k_work_delayable, work);
    struct kscan_charlieplex_data *data = CONTAINER_OF(dwork, struct kscan_charlieplex_data, work);
    kscan_charlieplex_read(data->dev);
}

static int kscan_charlieplex_configure(const struct device *dev, const kscan_callback_t callback) {
    struct kscan_charlieplex_data *data = dev->data;

    if (!callback) {
        return -EINVAL;
    }

    data->callback = callback;
    return 0;
}

static int kscan_charlieplex_enable(const struct device *dev) {
    struct kscan_charlieplex_data *data = dev->data;

    data->scan_time = k_uptime_get();

    // Read will automatically start interrupts/polling once done.
    return kscan_charlieplex_read(dev);
}

static int kscan_charlieplex_disable(const struct device *dev) {
    struct kscan_charlieplex_data *data = dev->data;

    k_work_cancel_delayable(&data->work);

    return 0;
}

static int kscan_charlieplex_init(const struct device *dev) {
    struct kscan_charlieplex_data *data = dev->data;

    data->dev = dev;

    k_work_init_delayable(&data->work, kscan_charlieplex_work_handler);

    return 0;
}

static const struct kscan_driver_api kscan_charlieplex_api = {
    .config = kscan_charlieplex_configure,
    .enable_callback = kscan_charlieplex_enable,
    .disable_callback = kscan_charlieplex_disable,
};

#define KSCAN_CHARLIEPLEX_INIT(n)                                                                  \
    BUILD_ASSERT(INST_DEBOUNCE_PRESS_MS(n) <= DEBOUNCE_COUNTER_MAX,                                \
                 "ZMK_KSCAN_DEBOUNCE_PRESS_MS or debounce-press-ms is too large");                 \
    BUILD_ASSERT(INST_DEBOUNCE_RELEASE_MS(n) <= DEBOUNCE_COUNTER_MAX,                              \
                 "ZMK_KSCAN_DEBOUNCE_RELEASE_MS or debounce-release-ms is too large");             \
                                                                                                   \
    static const struct gpio_dt_spec kscan_charlieplex_gpios_##n[] = {                             \
        LISTIFY(INST_GPIOS_LEN(n), KSCAN_GPIO_CFG_INIT, (, ), n)};                                 \
                                                                                                   \
    static struct debounce_state kscan_charlieplex_state_##n[INST_STATE_LEN(n)];                   \
                                                                                                   \
    static struct kscan_charlieplex_data kscan_charlieplex_data_##n = {                            \
        .debounce_state = kscan_charlieplex_state_##n,                                             \
    };                                                                                             \
                                                                                                   \
    static struct kscan_charlieplex_config kscan_charlieplex_config_##n = {                        \
        .gpios = kscan_charlieplex_gpios_##n,                                                      \
        .gpios_len = INST_GPIOS_LEN(n),                                                            \
        .debounce_config =                                                                         \
            {                                                                                      \
                .debounce_press_ms = INST_DEBOUNCE_PRESS_MS(n),                                    \
                .debounce_release_ms = INST_DEBOUNCE_RELEASE_MS(n),                                \
            },                                                                                     \
        .debounce_scan_period_ms = DT_INST_PROP(n, debounce_scan_period_ms),                       \
        .poll_period_ms = DT_INST_PROP(n, poll_period_ms),                                         \
    };                                                                                             \
    DEVICE_DT_INST_DEFINE(n, &kscan_charlieplex_init, NULL, &kscan_charlieplex_data_##n,           \
                          &kscan_charlieplex_config_##n, APPLICATION,                              \
                          CONFIG_APPLICATION_INIT_PRIORITY, &kscan_charlieplex_api);

DT_INST_FOREACH_STATUS_OKAY(KSCAN_CHARLIEPLEX_INIT);
