/*
 * SPDX-License-Identifier: MIT
 *
 * &ble_host_link: step through or set the host link parameters of the active
 * BLE profile and the pointer report interval, hand them back to the
 * automatic choice, and show or type what is in effect.
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

/* An interval preset or _SET value: min | max << 16, max 0 meaning min. */
static uint16_t interval_min_of(uint32_t v) { return (uint16_t)(v & 0xFFFF); }
static uint16_t interval_max_of(uint32_t v) {
    return (v >> 16) ? (uint16_t)(v >> 16) : (uint16_t)(v & 0xFFFF);
}

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
static const struct behavior_parameter_value_metadata interval_set_value[] = {
    {.display_name = "Automatic",
     .type = BEHAVIOR_PARAMETER_VALUE_TYPE_VALUE,
     .value = BLE_HOST_INTERVAL_AUTO},
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
    {.display_name = "Automatic",
     .type = BEHAVIOR_PARAMETER_VALUE_TYPE_VALUE,
     .value = BLE_HOST_LATENCY_AUTO},
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
    {.display_name = "Automatic",
     .type = BEHAVIOR_PARAMETER_VALUE_TYPE_VALUE,
     .value = BALL_BLE_REPORT_AUTO},
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

static int find_interval(const struct presets *p, uint16_t min, uint16_t max) {
    for (size_t i = 0; i < p->count; i++) {
        if (interval_min_of(p->values[i]) == min && interval_max_of(p->values[i]) == max) {
            return (int)i;
        }
    }
    return -1;
}

static int find_value(const struct presets *p, uint32_t value) {
    for (size_t i = 0; i < p->count; i++) {
        if (p->values[i] == value) {
            return (int)i;
        }
    }
    return -1;
}

/* Wrapping step from position i; a value not in the list steps onto its ends. */
static uint32_t step(const struct presets *p, int i, bool forward) {
    if (i < 0) {
        return p->values[forward ? 0 : p->count - 1];
    }
    return p->values[forward ? ((size_t)i + 1) % p->count : ((size_t)i + p->count - 1) % p->count];
}

/* Position of the value fixed by hand -- as set, not as asked for after the
   spec's limits -- or of the AUTO marker. */
static int interval_position(const struct presets *p, const struct zmk_ble_host_link_status *s) {
    return s->interval_pinned
               ? find_interval(p, s->pinned_interval_min, s->pinned_interval_max)
               : find_value(p, BLE_HOST_INTERVAL_AUTO);
}

static int latency_position(const struct presets *p, const struct zmk_ble_host_link_status *s) {
    return find_value(p, s->latency_pinned ? s->pinned_latency : BLE_HOST_LATENCY_AUTO);
}

static int ball_position(const struct presets *p, const struct zmk_ble_host_link_status *s) {
    return find_value(p, s->ball_report_pinned ? s->pinned_ball_report_ms : BALL_BLE_REPORT_AUTO);
}

static void set_interval(const struct presets *p, uint32_t value) {
    const uint16_t min = interval_min_of(value);
    const uint16_t max = interval_max_of(value);

    if (zmk_ble_host_link_set_interval(min, max) == 0) {
        zmk_ble_host_link_count_out(ZMK_BLE_HOST_LINK_COUNT_INTERVAL,
                                    (uint8_t)(find_interval(p, min, max) + 1));
    }
}

static void set_latency(const struct presets *p, uint32_t value) {
    if (value > 0xFFFF) {
        LOG_ERR("Host latency %u is out of range", value);
        return;
    }
    if (zmk_ble_host_link_set_latency((uint16_t)value) == 0) {
        zmk_ble_host_link_count_out(ZMK_BLE_HOST_LINK_COUNT_LATENCY,
                                    (uint8_t)(find_value(p, value) + 1));
    }
}

static void set_ball(const struct presets *p, uint32_t value) {
    if (value > BALL_BLE_REPORT_AUTO) {
        LOG_ERR("Ball report interval %u ms is out of range", value);
        return;
    }
    if (zmk_ble_host_link_set_ball_report((uint8_t)value) == 0) {
        zmk_ble_host_link_count_out(ZMK_BLE_HOST_LINK_COUNT_BALL_REPORT,
                                    (uint8_t)(find_value(p, value) + 1));
    }
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
        set_interval(&cfg->interval, value);
        break;
    case BLE_HOST_LATENCY_SET_CMD:
        set_latency(&cfg->latency, value);
        break;
    case BALL_BLE_REPORT_SET_CMD:
        set_ball(&cfg->ball_report, value);
        break;

    case BLE_HOST_INTERVAL_NEXT_CMD:
    case BLE_HOST_INTERVAL_PREV_CMD:
        if (zmk_ble_host_link_get_status(&s) == 0) {
            set_interval(&cfg->interval,
                         step(&cfg->interval, interval_position(&cfg->interval, &s),
                              cmd == BLE_HOST_INTERVAL_NEXT_CMD));
        }
        break;
    case BLE_HOST_LATENCY_NEXT_CMD:
    case BLE_HOST_LATENCY_PREV_CMD:
        if (zmk_ble_host_link_get_status(&s) == 0) {
            set_latency(&cfg->latency, step(&cfg->latency, latency_position(&cfg->latency, &s),
                                            cmd == BLE_HOST_LATENCY_NEXT_CMD));
        }
        break;
    case BALL_BLE_REPORT_NEXT_CMD:
    case BALL_BLE_REPORT_PREV_CMD:
        if (zmk_ble_host_link_get_status(&s) == 0) {
            set_ball(&cfg->ball_report,
                     step(&cfg->ball_report, ball_position(&cfg->ball_report, &s),
                          cmd == BALL_BLE_REPORT_NEXT_CMD));
        }
        break;

    case BLE_HOST_LINK_AUTO_CMD:
        zmk_ble_host_link_auto();
        break;
    case BLE_HOST_LINK_REQUEST_AGAIN_CMD:
        zmk_ble_host_link_request_again();
        break;
    case BLE_HOST_LINK_SHOW_CMD:
        if (zmk_ble_host_link_get_status(&s) == 0) {
            /* The verdict first: if the LED queue overflows, the counts are
               what is lost, not the answer. */
            zmk_ble_host_link_show();
            zmk_ble_host_link_count_out(ZMK_BLE_HOST_LINK_COUNT_INTERVAL,
                                        (uint8_t)(interval_position(&cfg->interval, &s) + 1));
            zmk_ble_host_link_count_out(ZMK_BLE_HOST_LINK_COUNT_LATENCY,
                                        (uint8_t)(latency_position(&cfg->latency, &s) + 1));
            zmk_ble_host_link_count_out(ZMK_BLE_HOST_LINK_COUNT_BALL_REPORT,
                                        (uint8_t)(ball_position(&cfg->ball_report, &s) + 1));
        }
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

/* Preset checks at build time. */

/* SHOW queues a verdict (up to 2 blinks) and one blink per preset position,
   up to the length of each list, into the LED widget's queue. */
#if IS_ENABLED(CONFIG_ZMK_BLE_HOST_LINK_LED_FEEDBACK)
#include <zmk_rgbled_widget/widget.h>
#define LED_INDICATIONS_SHOW_MAY_QUEUE RGBLED_WIDGET_INDICATION_QUEUE_LENGTH
#else
#define LED_INDICATIONS_SHOW_MAY_QUEUE 0
#endif
#define SHOW_VERDICT_BLINKS 2
#define PRESET_MIN(v) ((v)&0xFFFF)
#define PRESET_MAX(v) (((v) >> 16) ? ((v) >> 16) : ((v)&0xFFFF))
#define INTERVAL_PRESET_OK(node_id, prop, idx)                                                     \
    &&((DT_PROP_BY_IDX(node_id, prop, idx) == BLE_HOST_INTERVAL_AUTO) ||                           \
       ((PRESET_MIN(DT_PROP_BY_IDX(node_id, prop, idx)) >= 6) &&                                   \
        (PRESET_MAX(DT_PROP_BY_IDX(node_id, prop, idx)) >=                                         \
         PRESET_MIN(DT_PROP_BY_IDX(node_id, prop, idx))) &&                                        \
        (PRESET_MAX(DT_PROP_BY_IDX(node_id, prop, idx)) <=                                         \
         CONFIG_ZMK_BLE_HOST_LINK_SLOWEST_SETTABLE_INTERVAL)))
#define LATENCY_PRESET_OK(node_id, prop, idx)                                                      \
    &&((DT_PROP_BY_IDX(node_id, prop, idx) == BLE_HOST_LATENCY_AUTO) ||                            \
       (DT_PROP_BY_IDX(node_id, prop, idx) <= 499))
#define BALL_PRESET_OK(node_id, prop, idx)                                                         \
    &&((DT_PROP_BY_IDX(node_id, prop, idx) == BALL_BLE_REPORT_AUTO) ||                             \
       (DT_PROP_BY_IDX(node_id, prop, idx) <=                                                      \
        CONFIG_ZMK_BLE_HOST_LINK_SLOWEST_SETTABLE_BALL_REPORT_MS))

#define BLE_HOST_LINK_INST(n)                                                                      \
    BUILD_ASSERT(DT_INST_PROP_LEN(n, ble_host_interval_presets) > 0 &&                             \
                     DT_INST_PROP_LEN(n, ble_host_latency_presets) > 0 &&                          \
                     DT_INST_PROP_LEN(n, ball_ble_report_presets) > 0,                             \
                 "&ble_host_link needs at least one preset of each kind");                         \
    BUILD_ASSERT(!IS_ENABLED(CONFIG_ZMK_BLE_HOST_LINK_LED_FEEDBACK) ||                             \
                     DT_INST_PROP_LEN(n, ble_host_interval_presets) +                              \
                             DT_INST_PROP_LEN(n, ble_host_latency_presets) +                       \
                             DT_INST_PROP_LEN(n, ball_ble_report_presets) +                        \
                             SHOW_VERDICT_BLINKS <=                                                \
                         LED_INDICATIONS_SHOW_MAY_QUEUE,                                           \
                 "&ble_host_link: too many presets for SHOW to count out on the LED");             \
    BUILD_ASSERT(1 DT_INST_FOREACH_PROP_ELEM(n, ble_host_interval_presets, INTERVAL_PRESET_OK),    \
                 "ble-host-interval-presets: BLE_HOST_INTERVAL_AUTO, or 6 <= min <= max <= "       \
                 "ZMK_BLE_HOST_LINK_SLOWEST_SETTABLE_INTERVAL (1.25 ms units)");                   \
    BUILD_ASSERT(1 DT_INST_FOREACH_PROP_ELEM(n, ble_host_latency_presets, LATENCY_PRESET_OK),      \
                 "ble-host-latency-presets: BLE_HOST_LATENCY_AUTO, or 0..499");                    \
    BUILD_ASSERT(1 DT_INST_FOREACH_PROP_ELEM(n, ball_ble_report_presets, BALL_PRESET_OK),          \
                 "ball-ble-report-presets: BALL_BLE_REPORT_AUTO, or 0.."                           \
                 "ZMK_BLE_HOST_LINK_SLOWEST_SETTABLE_BALL_REPORT_MS");                             \
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
