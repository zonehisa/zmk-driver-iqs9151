/*
 * Input processor that converts IQS9151 pinch wheel events into zoom key taps.
 */

#define DT_DRV_COMPAT zmk_input_processor_pinch_zoom

#include <stdlib.h>

#include <zephyr/dt-bindings/input/input-event-codes.h>
#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <drivers/input_processor.h>

#include <zephyr/logging/log.h>

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

#include <zmk/behavior.h>
#include <zmk/keymap.h>
#include <zmk/virtual_key_position.h>

struct pinch_zoom_config {
    uint8_t index;
    uint16_t trigger_code;
    uint16_t wheel_code;
    struct zmk_behavior_binding zoom_in;
    struct zmk_behavior_binding zoom_out;
};

struct pinch_zoom_data {
    bool active;
};

static int pinch_zoom_tap_binding(const struct pinch_zoom_config *cfg,
                                  const struct zmk_behavior_binding *binding,
                                  struct zmk_input_processor_state *state) {
    struct zmk_behavior_binding_event event = {
        .position = ZMK_VIRTUAL_KEY_POSITION_BEHAVIOR_INPUT_PROCESSOR(
            state->input_device_index, cfg->index),
        .timestamp = k_uptime_get(),
#if IS_ENABLED(CONFIG_ZMK_SPLIT)
        .source = ZMK_POSITION_STATE_CHANGE_SOURCE_LOCAL,
#endif
    };

    int ret = zmk_behavior_invoke_binding(binding, event, true);
    if (ret < 0) {
        return ret;
    }

    event.timestamp = k_uptime_get();
    return zmk_behavior_invoke_binding(binding, event, false);
}

static void pinch_zoom_consume_event(struct input_event *event) {
    event->type = INPUT_EV_KEY;
    event->code = INPUT_BTN_8;
    event->value = 0;
}

static int pinch_zoom_handle_event(const struct device *dev, struct input_event *event,
                                   uint32_t param1, uint32_t param2,
                                   struct zmk_input_processor_state *state) {
    ARG_UNUSED(param1);
    ARG_UNUSED(param2);

    const struct pinch_zoom_config *cfg = dev->config;
    struct pinch_zoom_data *data = dev->data;

    if (event->type == INPUT_EV_KEY && event->code == cfg->trigger_code) {
        data->active = event->value != 0;
        pinch_zoom_consume_event(event);
        return ZMK_INPUT_PROC_CONTINUE;
    }

    if (!data->active || event->type != INPUT_EV_REL || event->code != cfg->wheel_code) {
        return ZMK_INPUT_PROC_CONTINUE;
    }

    if (event->value != 0) {
        const struct zmk_behavior_binding *binding =
            event->value > 0 ? &cfg->zoom_in : &cfg->zoom_out;
        int taps = MIN(abs(event->value), 3);

        for (int i = 0; i < taps; i++) {
            int ret = pinch_zoom_tap_binding(cfg, binding, state);
            if (ret < 0) {
                return ret;
            }
        }
    }

    pinch_zoom_consume_event(event);
    return ZMK_INPUT_PROC_CONTINUE;
}

static struct zmk_input_processor_driver_api pinch_zoom_driver_api = {
    .handle_event = pinch_zoom_handle_event,
};

static int pinch_zoom_init(const struct device *dev) { return 0; }

#define PINCH_ZOOM_INST(n)                                                                         \
    static const struct zmk_behavior_binding pinch_zoom_bindings_##n[] = {                         \
        LISTIFY(DT_INST_PROP_LEN(n, bindings), ZMK_KEYMAP_EXTRACT_BINDING, (, ), DT_DRV_INST(n))}; \
    BUILD_ASSERT(ARRAY_SIZE(pinch_zoom_bindings_##n) == 2,                                         \
                 "pinch zoom needs exactly two bindings: zoom-in, zoom-out");                     \
    static const struct pinch_zoom_config pinch_zoom_config_##n = {                                \
        .index = n,                                                                                \
        .trigger_code = DT_INST_PROP(n, trigger_code),                                             \
        .wheel_code = DT_INST_PROP(n, wheel_code),                                                 \
        .zoom_in = pinch_zoom_bindings_##n[0],                                                     \
        .zoom_out = pinch_zoom_bindings_##n[1],                                                    \
    };                                                                                             \
    static struct pinch_zoom_data pinch_zoom_data_##n = {};                                        \
    DEVICE_DT_INST_DEFINE(n, &pinch_zoom_init, NULL, &pinch_zoom_data_##n,                         \
                          &pinch_zoom_config_##n, POST_KERNEL,                                    \
                          CONFIG_KERNEL_INIT_PRIORITY_DEFAULT, &pinch_zoom_driver_api);

DT_INST_FOREACH_STATUS_OKAY(PINCH_ZOOM_INST)
