/*
 * Copyright (c) 2025 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 *
 * Port of the old tyamanak/zmk-pmw3610-driver fork's
 * CONFIG_PMW3610_MOVEMENT_THRESHOLD (only report movement once accumulated
 * motion exceeds a threshold) to a ZMK input processor, so the auto-mouse
 * layer (zip_temp_layer) is not endlessly re-triggered by tiny cursor drift
 * while a hand rests on the trackball.
 *
 * While the configured `layer` is active, this processor is a pure
 * passthrough: no deadzone is applied there, so precise cursor movement
 * inside the auto-mouse layer is never degraded.
 *
 * While that layer is inactive, REL_X/REL_Y values are accumulated
 * per-axis. Events are dropped (ZMK_INPUT_PROC_STOP) until the accumulated
 * |x|+|y| exceeds `threshold`, at which point the gate "unlocks" for the
 * remainder of the current input report: the (possibly multi-event)
 * accumulated value for the axis being processed is flushed out and the
 * axis accumulator is cleared. This means no motion is ever lost -- it is
 * only delayed until enough of it has built up.
 */

#define DT_DRV_COMPAT zmk_input_processor_movement_gate

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/sys/util.h>
#include <drivers/input_processor.h>

#include <stdlib.h> /* abs() -- see app/src/rgb_underglow.c for the same pairing */

#include <zephyr/logging/log.h>

#include <zmk/keymap.h>

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

struct movement_gate_config {
    int32_t threshold;
    uint8_t layer;
};

/* Per-instance state. acc_x/acc_y persist across input reports (sync
 * events) so that sub-threshold drift keeps accumulating instead of being
 * discarded -- only `unlocked` is reset at each report boundary. */
struct movement_gate_data {
    int32_t acc_x;
    int32_t acc_y;
    bool unlocked;
};

static int movement_gate_handle_event(const struct device *dev, struct input_event *event,
                                      uint32_t param1, uint32_t param2,
                                      struct zmk_input_processor_state *state) {
    const struct movement_gate_config *cfg = dev->config;
    struct movement_gate_data *data = dev->data;

    if (event->type != INPUT_EV_REL ||
        (event->code != INPUT_REL_X && event->code != INPUT_REL_Y)) {
        return ZMK_INPUT_PROC_CONTINUE;
    }

    if (zmk_keymap_layer_active(cfg->layer)) {
        /* Auto-mouse layer is already active: no deadzone, don't let stale
         * accumulator state leak into the next time the layer deactivates. */
        data->acc_x = 0;
        data->acc_y = 0;
        data->unlocked = false;
        return ZMK_INPUT_PROC_CONTINUE;
    }

    int32_t *acc = (event->code == INPUT_REL_X) ? &data->acc_x : &data->acc_y;
    *acc += event->value;

    int ret;

    if (data->unlocked || (abs(data->acc_x) + abs(data->acc_y) > cfg->threshold)) {
        /* Either already unlocked earlier in this report, or the threshold
         * was just crossed: flush this axis' accumulated value and clear
         * it. `unlocked` stays set for the rest of the report so that a
         * sibling axis event (e.g. Y following X in the same report) is
         * also let through -- see the ZMK_INPUT_PROC_STOP note below for
         * why that matters. */
        data->unlocked = true;
        event->value = *acc;
        *acc = 0;
        ret = ZMK_INPUT_PROC_CONTINUE;
    } else {
        /* Below threshold: drop the event entirely. ZMK_INPUT_PROC_STOP
         * makes the input listener return without reporting anything for
         * this event (see apply_config()/input_handler() in
         * app/src/pointing/input_listener.c), so nothing is emitted --
         * but the value we already added to *acc above is preserved, so
         * the motion is not lost, only delayed until it crosses
         * threshold.
         *
         * `unlocked` exists because a single input report can carry more
         * than one axis event ending in a sync (e.g. REL_X then REL_Y with
         * sync=true). If we let REL_X through here but then STOPped the
         * following REL_Y, the listener would return before ever seeing
         * evt->sync, so the REL_X we already let through would never be
         * flushed to the host until some later report happens to carry a
         * sync of its own. Marking `unlocked` once we cross threshold
         * ensures every subsequent event in the same report (including
         * the one carrying sync) is also let through.
         */
        ret = ZMK_INPUT_PROC_STOP;
    }

    if (event->sync) {
        /* Report boundary: `unlocked` only applies within a single report. */
        data->unlocked = false;
    }

    return ret;
}

static struct zmk_input_processor_driver_api movement_gate_driver_api = {
    .handle_event = movement_gate_handle_event,
};

#define MOVEMENT_GATE_INST(n)                                                                    \
    static struct movement_gate_data movement_gate_data_##n = {                                  \
        .acc_x = 0,                                                                              \
        .acc_y = 0,                                                                              \
        .unlocked = false,                                                                       \
    };                                                                                            \
    static const struct movement_gate_config movement_gate_config_##n = {                         \
        .threshold = DT_INST_PROP_OR(n, threshold, 5),                                            \
        .layer = DT_INST_PROP(n, layer),                                                          \
    };                                                                                             \
    DEVICE_DT_INST_DEFINE(n, NULL, NULL, &movement_gate_data_##n,                                  \
                          &movement_gate_config_##n, POST_KERNEL,                                  \
                          CONFIG_KERNEL_INIT_PRIORITY_DEFAULT, &movement_gate_driver_api);

DT_INST_FOREACH_STATUS_OKAY(MOVEMENT_GATE_INST)
