/*
 * Copyright (c) 2025 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 *
 * Port of the old tyamanak/zmk-pmw3610-driver fork's "adjustable mouse speed"
 * magnitude-based acceleration (see magnitude-original.diff) to a ZMK input
 * processor, so it works with the badjeff/zmk-pmw3610-driver fork which does
 * not implement it in-driver.
 *
 * Original algorithm (float, driver-side):
 *   magnitude   = sqrt(x^2 + y^2)
 *   multiplier  = 1.0 + magnitude / 10.0
 *   multiplier  = clamp(multiplier, min, max)
 *   x *= multiplier; y *= multiplier   (float -> int cast, truncation toward zero)
 *
 * This port uses fixed-point (thousandths) integer arithmetic and an
 * integer square root, so no libm/float dependency is required:
 *   mult_fp = 1000 + isqrt(x*x + y*y) * 1000 / divisor
 *   mult_fp = clamp(mult_fp, min_multiplier, max_multiplier)
 *   value   = value * mult_fp / 1000   (C integer division truncates toward
 *                                        zero, matching the original float->int cast)
 */

#define DT_DRV_COMPAT zmk_input_processor_magnitude_scaler

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/sys/util.h>
#include <drivers/input_processor.h>

#include <zephyr/logging/log.h>

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

struct magnitude_scaler_config {
    int32_t min_multiplier;
    int32_t max_multiplier;
    int32_t divisor;
};

/* Per-instance state: the most recently seen X and Y values within the
 * current input report. badjeff/zmk-pmw3610-driver only emits REL_X /
 * REL_Y for axes that actually moved (see pmw3610.c ~497-508: separate
 * `if (have_x)` / `if (have_y)` sends), and marks the *last* event of a
 * report as sync=true (this may be REL_X alone, REL_Y alone, or REL_Y
 * after REL_X). We therefore accumulate whichever axis/axes arrive, scale
 * using the values seen so far in this report, and reset both back to 0
 * once the sync event has been processed so no value survives into the
 * next report. This means an X-only report is scaled using |x| alone
 * (correct, since y is genuinely 0 for that report), while in an X+Y
 * report the REL_X event (not yet sync) is scaled using |x| alone before
 * REL_Y arrives, and REL_Y (sync) is scaled using the accurate
 * sqrt(x^2+y^2). At 125Hz polling this per-report approximation on the
 * non-final axis of a two-axis report is not perceptible. */
struct magnitude_scaler_data {
    int32_t last_x;
    int32_t last_y;
};

/* Bit-by-bit integer square root (no float/libm dependency). */
static uint32_t isqrt32(uint32_t n) {
    uint32_t res = 0;
    uint32_t bit = 1u << 30; /* second-to-top bit set, for 32-bit ints */

    while (bit > n) {
        bit >>= 2;
    }

    while (bit != 0) {
        if (n >= res + bit) {
            n -= res + bit;
            res = (res >> 1) + bit;
        } else {
            res >>= 1;
        }
        bit >>= 2;
    }

    return res;
}

static int magnitude_scaler_handle_event(const struct device *dev, struct input_event *event,
                                         uint32_t param1, uint32_t param2,
                                         struct zmk_input_processor_state *state) {
    const struct magnitude_scaler_config *cfg = dev->config;
    struct magnitude_scaler_data *data = dev->data;

    if (event->type != INPUT_EV_REL) {
        return ZMK_INPUT_PROC_CONTINUE;
    }

    /* Only REL_X / REL_Y (trackball movement) are accelerated. Scroll wheel
     * events (REL_WHEEL / REL_HWHEEL) and anything else pass through
     * untouched. */
    if (event->code == INPUT_REL_X) {
        data->last_x = event->value;
    } else if (event->code == INPUT_REL_Y) {
        data->last_y = event->value;
    } else {
        return ZMK_INPUT_PROC_CONTINUE;
    }

    int32_t x = data->last_x;
    int32_t y = data->last_y;

    uint32_t magnitude = isqrt32((uint32_t)(x * x + y * y));

    int32_t mult_fp = 1000 + (int32_t)((magnitude * 1000) / (uint32_t)cfg->divisor);
    mult_fp = CLAMP(mult_fp, cfg->min_multiplier, cfg->max_multiplier);

    event->value = (int32_t)(event->value * mult_fp / 1000);

    /* sync marks the end of this input report. Reset so a stale axis value
     * from this report never leaks into the next one (see the comment on
     * struct magnitude_scaler_data above). */
    if (event->sync) {
        data->last_x = 0;
        data->last_y = 0;
    }

    return ZMK_INPUT_PROC_CONTINUE;
}

static struct zmk_input_processor_driver_api magnitude_scaler_driver_api = {
    .handle_event = magnitude_scaler_handle_event,
};

#define MAGNITUDE_SCALER_INST(n)                                                                  \
    static struct magnitude_scaler_data magnitude_scaler_data_##n = {                              \
        .last_x = 0,                                                                              \
        .last_y = 0,                                                                              \
    };                                                                                             \
    static const struct magnitude_scaler_config magnitude_scaler_config_##n = {                    \
        .min_multiplier = DT_INST_PROP_OR(n, min_multiplier, 1000),                                \
        .max_multiplier = DT_INST_PROP_OR(n, max_multiplier, 1500),                                \
        .divisor = DT_INST_PROP_OR(n, divisor, 10),                                                \
    };                                                                                             \
    DEVICE_DT_INST_DEFINE(n, NULL, NULL, &magnitude_scaler_data_##n,                               \
                          &magnitude_scaler_config_##n, POST_KERNEL,                                \
                          CONFIG_KERNEL_INIT_PRIORITY_DEFAULT, &magnitude_scaler_driver_api);

DT_INST_FOREACH_STATUS_OKAY(MAGNITUDE_SCALER_INST)
