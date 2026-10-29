/*
 * SPDX-License-Identifier: MIT
 *
 * &ble_host_link: step through or set the host link parameters of the active
 * BLE profile and the pointer report interval, hand them back to the
 * automatic choice, and show or type what is in effect.
 *
 * NEXT / PREV step from the value in effect -- fixed by hand, or chosen
 * automatically -- to the next larger / smaller preset, round the list: after
 * the largest comes the smallest. AUTO is not on the list: the ..._AUTOMATIC
 * commands (or SET ... AUTO) and BLE_HOST_LINK_AUTO hand a value back.
 */

#define DT_DRV_COMPAT zmk_behavior_ble_host_link

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <drivers/behavior.h>
#include <zmk/behavior.h>

#include <dt-bindings/zmk/ble_host_link.h>
#include <zmk_ble_host_link/ble_host_link.h>

LOG_MODULE_DECLARE(zmk_ble_host_link, CONFIG_ZMK_BLE_HOST_LINK_LOG_LEVEL);

#if DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT)

struct presets {
    const uint32_t *values;
    size_t count;
};

struct behavior_ble_host_link_config {
    struct presets interval;
    struct presets latency;
    struct presets ball_report;
};

#if IS_ENABLED(CONFIG_ZMK_BEHAVIOR_METADATA)

static const struct behavior_parameter_value_metadata no_arg_values[] = {
    {.display_name = "Host interval: next",
     .type = BEHAVIOR_PARAMETER_VALUE_TYPE_VALUE,
     .value = BLE_HOST_INTERVAL_NEXT_CMD},
    {.display_name = "Host interval: previous",
     .type = BEHAVIOR_PARAMETER_VALUE_TYPE_VALUE,
     .value = BLE_HOST_INTERVAL_PREV_CMD},
    {.display_name = "Host latency: next",
     .type = BEHAVIOR_PARAMETER_VALUE_TYPE_VALUE,
     .value = BLE_HOST_LATENCY_NEXT_CMD},
    {.display_name = "Host latency: previous",
     .type = BEHAVIOR_PARAMETER_VALUE_TYPE_VALUE,
     .value = BLE_HOST_LATENCY_PREV_CMD},
    {.display_name = "Ball BLE report: next",
     .type = BEHAVIOR_PARAMETER_VALUE_TYPE_VALUE,
     .value = BALL_BLE_REPORT_NEXT_CMD},
    {.display_name = "Ball BLE report: previous",
     .type = BEHAVIOR_PARAMETER_VALUE_TYPE_VALUE,
     .value = BALL_BLE_REPORT_PREV_CMD},
    {.display_name = "Host interval: automatic",
     .type = BEHAVIOR_PARAMETER_VALUE_TYPE_VALUE,
     .value = BLE_HOST_INTERVAL_AUTOMATIC_CMD},
    {.display_name = "Host latency: automatic",
     .type = BEHAVIOR_PARAMETER_VALUE_TYPE_VALUE,
     .value = BLE_HOST_LATENCY_AUTOMATIC_CMD},
    {.display_name = "Ball BLE report: automatic",
     .type = BEHAVIOR_PARAMETER_VALUE_TYPE_VALUE,
     .value = BALL_BLE_REPORT_AUTOMATIC_CMD},
    {.display_name = "All automatic, search again",
     .type = BEHAVIOR_PARAMETER_VALUE_TYPE_VALUE,
     .value = BLE_HOST_LINK_AUTO_CMD},
    {.display_name = "Send the request again",
     .type = BEHAVIOR_PARAMETER_VALUE_TYPE_VALUE,
     .value = BLE_HOST_LINK_REQUEST_AGAIN_CMD},
    {.display_name = "Show on the LED",
     .type = BEHAVIOR_PARAMETER_VALUE_TYPE_VALUE,
     .value = BLE_HOST_LINK_SHOW_CMD},
    {.display_name = "Type a report",
     .type = BEHAVIOR_PARAMETER_VALUE_TYPE_VALUE,
     .value = BLE_HOST_LINK_TYPE_REPORT_CMD},
};

static const struct behavior_parameter_metadata_set no_args_set = {
    .param1_values = no_arg_values,
    .param1_values_len = ARRAY_SIZE(no_arg_values),
};

static const struct behavior_parameter_value_metadata interval_set_cmd[] = {
    {.display_name = "Host interval: set",
     .type = BEHAVIOR_PARAMETER_VALUE_TYPE_VALUE,
     .value = BLE_HOST_INTERVAL_SET_CMD},
};
/* One range per "set": ZMK Studio edits a range or picks from values, not
   both -- "automatic" is a command of its own above. */
static const struct behavior_parameter_value_metadata interval_set_value[] = {
    {.display_name = "Interval (1.25 ms units)",
     .type = BEHAVIOR_PARAMETER_VALUE_TYPE_RANGE,
     .range = {.min = 6, .max = CONFIG_ZMK_BLE_HOST_LINK_SLOWEST_SETTABLE_INTERVAL}},
};
static const struct behavior_parameter_metadata_set interval_set = {
    .param1_values = interval_set_cmd,
    .param1_values_len = ARRAY_SIZE(interval_set_cmd),
    .param2_values = interval_set_value,
    .param2_values_len = ARRAY_SIZE(interval_set_value),
};

static const struct behavior_parameter_value_metadata latency_set_cmd[] = {
    {.display_name = "Host latency: set",
     .type = BEHAVIOR_PARAMETER_VALUE_TYPE_VALUE,
     .value = BLE_HOST_LATENCY_SET_CMD},
};
static const struct behavior_parameter_value_metadata latency_set_value[] = {
    {.display_name = "Latency (connection events)",
     .type = BEHAVIOR_PARAMETER_VALUE_TYPE_RANGE,
     .range = {.min = 0, .max = 499}},
};
static const struct behavior_parameter_metadata_set latency_set = {
    .param1_values = latency_set_cmd,
    .param1_values_len = ARRAY_SIZE(latency_set_cmd),
    .param2_values = latency_set_value,
    .param2_values_len = ARRAY_SIZE(latency_set_value),
};

static const struct behavior_parameter_value_metadata ball_set_cmd[] = {
    {.display_name = "Ball BLE report: set",
     .type = BEHAVIOR_PARAMETER_VALUE_TYPE_VALUE,
     .value = BALL_BLE_REPORT_SET_CMD},
};
static const struct behavior_parameter_value_metadata ball_set_value[] = {
    {.display_name = "Report interval (ms, 0 = every frame)",
     .type = BEHAVIOR_PARAMETER_VALUE_TYPE_RANGE,
     .range = {.min = 0, .max = CONFIG_ZMK_BLE_HOST_LINK_SLOWEST_SETTABLE_BALL_REPORT_MS}},
};
static const struct behavior_parameter_metadata_set ball_set = {
    .param1_values = ball_set_cmd,
    .param1_values_len = ARRAY_SIZE(ball_set_cmd),
    .param2_values = ball_set_value,
    .param2_values_len = ARRAY_SIZE(ball_set_value),
};

static const struct behavior_parameter_metadata_set metadata_sets[] = {
    no_args_set,
    interval_set,
    latency_set,
    ball_set,
};

static const struct behavior_parameter_metadata metadata = {
    .sets_len = ARRAY_SIZE(metadata_sets),
    .sets = metadata_sets,
};

#endif /* IS_ENABLED(CONFIG_ZMK_BEHAVIOR_METADATA) */

/* The next larger (up) or smaller preset than `current`, round the list. */
static uint32_t step(const struct presets *p, uint32_t current, bool up) {
    bool found = false;
    uint32_t best = 0;

    for (size_t i = 0; i < p->count; i++) {
        const uint32_t v = p->values[i];
        const bool beyond = up ? v > current : v < current;
        const bool nearer = !found || (up ? v < best : v > best);

        if (beyond && nearer) {
            best = v;
            found = true;
        }
    }
    if (found) {
        return best;
    }
    /* Past the end: round to the other end. */
    best = p->values[0];
    for (size_t i = 1; i < p->count; i++) {
        if (up ? p->values[i] < best : p->values[i] > best) {
            best = p->values[i];
        }
    }
    return best;
}

/* The value in effect: fixed by hand -- as set, not as asked for after the
   spec's limits -- or what is asked for (in use, for the ball). */
static uint32_t interval_in_effect(const struct zmk_ble_host_link_status *s) {
    return s->interval_pinned ? s->pinned_interval : s->asked_interval;
}

static uint32_t latency_in_effect(const struct zmk_ble_host_link_status *s) {
    return s->latency_pinned ? s->pinned_latency : s->asked_latency;
}

static uint32_t ball_in_effect(const struct zmk_ble_host_link_status *s) {
    return s->ball_report_pinned ? s->pinned_ball_report_ms : s->ball_report_ms;
}

static void set_interval(uint32_t value) {
    if (value > 0xFFFF) {
        LOG_ERR("Host interval %u is out of range: one exact interval, in 1.25 ms units", value);
        return;
    }
    zmk_ble_host_link_set_interval((uint16_t)value);
}

static void set_latency(uint32_t value) {
    if (value > 0xFFFF) {
        LOG_ERR("Host latency %u is out of range", value);
        return;
    }
    zmk_ble_host_link_set_latency((uint16_t)value);
}

static void set_ball(uint32_t value) {
    if (value > BALL_BLE_REPORT_AUTO) {
        LOG_ERR("Ball report interval %u ms is out of range", value);
        return;
    }
    zmk_ble_host_link_set_ball_report((uint8_t)value);
}

static int on_keymap_binding_pressed(struct zmk_behavior_binding *binding,
                                     struct zmk_behavior_binding_event event) {
    ARG_UNUSED(event);

    const struct device *dev = zmk_behavior_get_binding(binding->behavior_dev);
    const struct behavior_ble_host_link_config *cfg = dev->config;
    const uint32_t cmd = binding->param1;
    const uint32_t value = binding->param2;
    struct zmk_ble_host_link_status s;

    switch (cmd) {
    case BLE_HOST_INTERVAL_SET_CMD:
        set_interval(value);
        break;
    case BLE_HOST_LATENCY_SET_CMD:
        set_latency(value);
        break;
    case BALL_BLE_REPORT_SET_CMD:
        set_ball(value);
        break;
    case BLE_HOST_INTERVAL_AUTOMATIC_CMD:
        set_interval(BLE_HOST_INTERVAL_AUTO);
        break;
    case BLE_HOST_LATENCY_AUTOMATIC_CMD:
        set_latency(BLE_HOST_LATENCY_AUTO);
        break;
    case BALL_BLE_REPORT_AUTOMATIC_CMD:
        set_ball(BALL_BLE_REPORT_AUTO);
        break;

    case BLE_HOST_INTERVAL_NEXT_CMD:
    case BLE_HOST_INTERVAL_PREV_CMD:
        if (zmk_ble_host_link_get_status(&s) == 0) {
            set_interval(step(&cfg->interval, interval_in_effect(&s),
                              cmd == BLE_HOST_INTERVAL_NEXT_CMD));
        }
        break;
    case BLE_HOST_LATENCY_NEXT_CMD:
    case BLE_HOST_LATENCY_PREV_CMD:
        if (zmk_ble_host_link_get_status(&s) == 0) {
            set_latency(
                step(&cfg->latency, latency_in_effect(&s), cmd == BLE_HOST_LATENCY_NEXT_CMD));
        }
        break;
    case BALL_BLE_REPORT_NEXT_CMD:
    case BALL_BLE_REPORT_PREV_CMD:
        if (zmk_ble_host_link_get_status(&s) == 0) {
            set_ball(step(&cfg->ball_report, ball_in_effect(&s), cmd == BALL_BLE_REPORT_NEXT_CMD));
        }
        break;

    case BLE_HOST_LINK_AUTO_CMD:
        zmk_ble_host_link_auto();
        break;
    case BLE_HOST_LINK_REQUEST_AGAIN_CMD:
        zmk_ble_host_link_request_again();
        break;
    case BLE_HOST_LINK_SHOW_CMD:
        zmk_ble_host_link_show();
        break;
    case BLE_HOST_LINK_TYPE_REPORT_CMD:
        zmk_ble_host_link_type_report();
        break;

    default:
        LOG_ERR("Unknown &ble_host_link command %u", cmd);
        return -ENOTSUP;
    }

    return ZMK_BEHAVIOR_OPAQUE;
}

static int on_keymap_binding_released(struct zmk_behavior_binding *binding,
                                      struct zmk_behavior_binding_event event) {
    ARG_UNUSED(binding);
    ARG_UNUSED(event);
    return ZMK_BEHAVIOR_OPAQUE;
}

static int behavior_ble_host_link_init(const struct device *dev) {
    ARG_UNUSED(dev);
    return 0;
}

static const struct behavior_driver_api behavior_ble_host_link_driver_api = {
    .binding_pressed = on_keymap_binding_pressed,
    .binding_released = on_keymap_binding_released,
    .locality = BEHAVIOR_LOCALITY_CENTRAL,
#if IS_ENABLED(CONFIG_ZMK_BEHAVIOR_METADATA)
    .parameter_metadata = &metadata,
#endif
};

/* Preset checks at build time: exact values only, AUTO is not a preset. */
#define INTERVAL_PRESET_OK(node_id, prop, idx)                                                     \
    &&(DT_PROP_BY_IDX(node_id, prop, idx) >= 6) &&                                                 \
        (DT_PROP_BY_IDX(node_id, prop, idx) <= CONFIG_ZMK_BLE_HOST_LINK_SLOWEST_SETTABLE_INTERVAL)
#define LATENCY_PRESET_OK(node_id, prop, idx) &&(DT_PROP_BY_IDX(node_id, prop, idx) <= 499)
#define BALL_PRESET_OK(node_id, prop, idx)                                                         \
    &&(DT_PROP_BY_IDX(node_id, prop, idx) <=                                                       \
       CONFIG_ZMK_BLE_HOST_LINK_SLOWEST_SETTABLE_BALL_REPORT_MS)

#define BLE_HOST_LINK_INST(n)                                                                      \
    BUILD_ASSERT(DT_INST_PROP_LEN(n, ble_host_interval_presets) > 0 &&                             \
                     DT_INST_PROP_LEN(n, ble_host_latency_presets) > 0 &&                          \
                     DT_INST_PROP_LEN(n, ball_ble_report_presets) > 0,                             \
                 "&ble_host_link needs at least one preset of each kind");                         \
    BUILD_ASSERT(1 DT_INST_FOREACH_PROP_ELEM(n, ble_host_interval_presets, INTERVAL_PRESET_OK),    \
                 "ble-host-interval-presets: 6..ZMK_BLE_HOST_LINK_SLOWEST_SETTABLE_INTERVAL "       \
                 "(1.25 ms units), no AUTO");                                                      \
    BUILD_ASSERT(1 DT_INST_FOREACH_PROP_ELEM(n, ble_host_latency_presets, LATENCY_PRESET_OK),      \
                 "ble-host-latency-presets: 0..499, no AUTO");                                     \
    BUILD_ASSERT(1 DT_INST_FOREACH_PROP_ELEM(n, ball_ble_report_presets, BALL_PRESET_OK),          \
                 "ball-ble-report-presets: 0..ZMK_BLE_HOST_LINK_SLOWEST_SETTABLE_BALL_REPORT_MS, "  \
                 "no AUTO");                                                                       \
    static const uint32_t ble_host_link_interval_##n[] =                                           \
        DT_INST_PROP(n, ble_host_interval_presets);                                                \
    static const uint32_t ble_host_link_latency_##n[] = DT_INST_PROP(n, ble_host_latency_presets); \
    static const uint32_t ble_host_link_ball_##n[] = DT_INST_PROP(n, ball_ble_report_presets);     \
    static const struct behavior_ble_host_link_config ble_host_link_config_##n = {                 \
        .interval = {ble_host_link_interval_##n, ARRAY_SIZE(ble_host_link_interval_##n)},          \
        .latency = {ble_host_link_latency_##n, ARRAY_SIZE(ble_host_link_latency_##n)},             \
        .ball_report = {ble_host_link_ball_##n, ARRAY_SIZE(ble_host_link_ball_##n)},               \
    };                                                                                             \
    BEHAVIOR_DT_INST_DEFINE(n, behavior_ble_host_link_init, NULL, NULL,                            \
                            &ble_host_link_config_##n, POST_KERNEL,                                \
                            CONFIG_KERNEL_INIT_PRIORITY_DEFAULT,                                   \
                            &behavior_ble_host_link_driver_api);

DT_INST_FOREACH_STATUS_OKAY(BLE_HOST_LINK_INST)

#endif /* DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT) */
