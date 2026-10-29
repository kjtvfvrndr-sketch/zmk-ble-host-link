/*
 * SPDX-License-Identifier: MIT
 *
 * A debugging watch on the keyboard's advertising -- its call to the hosts
 * (ZMK_BLE_HOST_LINK_LOG_ADVERTISING_OUT_OF_STEP). It logs; with
 * ZMK_BLE_HOST_LINK_RESTART_ADVERTISING_WHEN_NO_HOST_ANSWERS it also restarts
 * the advertising once when no host answers it.
 *
 * ZMK advertises while the active profile is open or its host is away, and
 * keeps its own note of whether it is advertising: it does not ask the
 * stack. Its connect and disconnect handlers, which run on different
 * threads, both update that note around a start that waits for the radio.
 * If they cross, the note and the stack part: ZMK stops calling while it
 * believes it calls. No host can connect any more, the links already up
 * keep working, and a restart clears it.
 *
 * Once a second this file compares what ZMK's rule asks for with what the
 * stack does, and logs when they part for longer than ZMK takes to catch
 * up, and when they meet again. It also checks that a connectable advertiser
 * has its connection object waiting: without it the stack turns away the
 * host that answers.
 *
 * The stack can also advertise, by its own account, while no host gets
 * through: neither a host of another profile, which comes the moment the
 * keyboard calls, nor one pairing anew -- and the links already up keep
 * working. When advertising in step has gone NO_HOST_ANSWERS_AFTER_MS with
 * no host connecting, this file logs it with the whole Bluetooth snapshot,
 * once for that stretch -- a host simply away or asleep looks the same, so
 * the line says so. With the restart option it then stops and starts the
 * advertising through ZMK's own call for a new device name (the same name:
 * nothing is stored), and logs whether a host comes within
 * AFTER_RESTART_WAIT_MS: one that comes within RESTART_FREED_IT_WITHIN_MS --
 * as a host waiting for the call does -- says the advertiser was stuck and
 * the restart freed it; one that comes later was more likely brought by
 * hand, and tells nothing; none says the fault is below the advertiser.
 * With no profile paired to a host, no host comes by itself: nothing is
 * watched then.
 */

#include <stdio.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/init.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/logging/log.h>

#include <zmk/ble.h>

/* The stack's internal header (subsys/bluetooth/host): bt_dev.adv. */
#include "hci_core.h"

#include "advertising_out_of_step_log.h"
#include "host_connections_log.h"

LOG_MODULE_DECLARE(zmk_ble_host_link, CONFIG_ZMK_BLE_HOST_LINK_LOG_LEVEL);

#define CHECK_EVERY_MS 1000
/* ZMK restarts advertising straight from its handlers: a part longer than
   this is not ZMK catching up. */
#define OUT_OF_STEP_AFTER_MS 3000
/* The stack is up and ZMK has advertised once by then. */
#define FIRST_CHECK_AFTER_MS 5000
/* A host of another profile, sent away before, answers within a second or
   two of the keyboard calling; a host pairing anew within a few. */
#define NO_HOST_ANSWERS_AFTER_MS 15000
#define AFTER_RESTART_WAIT_MS 15000
/* A host waiting for the keyboard's call answers within a second or two of
   it; later, someone more likely paired or woke one by hand. */
#define RESTART_FREED_IT_WITHIN_MS 3000

enum advertising_seen {
    ADVERTISING_OFF,
    ADVERTISING_PAUSED,
    ADVERTISING_ON,
    ADVERTISING_ON_WITHOUT_WAITING_CONNECTION,
};

struct advertising_look {
    bool zmk_rule_asks;
    enum advertising_seen stack;
    unsigned int connection_objects;
};

struct connection_objects_count {
    unsigned int in_use;
    bool advertiser_waits;
};

static void count_connection_objects(struct bt_conn *conn, void *data) {
    struct connection_objects_count *count = data;
    struct bt_conn_info info;

    if (bt_conn_get_info(conn, &info) != 0) {
        return;
    }
    count->in_use++;
    if (host_connections_log_is_advertisers_connection(conn)) {
        count->advertiser_waits = true;
    }
}

static struct advertising_look look_now(void) {
    struct connection_objects_count count = {0};

    bt_conn_foreach(BT_CONN_TYPE_LE, count_connection_objects, &count);

    struct advertising_look look = {
        .zmk_rule_asks =
            zmk_ble_active_profile_is_open() || !zmk_ble_active_profile_is_connected(),
        .connection_objects = count.in_use,
    };

    if (atomic_test_bit(bt_dev.adv.flags, BT_ADV_PAUSED)) {
        look.stack = ADVERTISING_PAUSED;
    } else if (!atomic_test_bit(bt_dev.adv.flags, BT_ADV_ENABLED)) {
        look.stack = ADVERTISING_OFF;
    } else if (atomic_test_bit(bt_dev.adv.flags, BT_ADV_CONNECTABLE) && !count.advertiser_waits) {
        look.stack = ADVERTISING_ON_WITHOUT_WAITING_CONNECTION;
    } else {
        look.stack = ADVERTISING_ON;
    }
    return look;
}

static bool in_step(const struct advertising_look *look) {
    return look->zmk_rule_asks ? look->stack == ADVERTISING_ON : look->stack == ADVERTISING_OFF;
}

static const char *stack_text(enum advertising_seen stack) {
    switch (stack) {
    case ADVERTISING_OFF:
        return "not advertising";
    case ADVERTISING_PAUSED:
        return "advertising paused by the stack";
    case ADVERTISING_ON:
        return "advertising, a connection object waits for a host";
    case ADVERTISING_ON_WITHOUT_WAITING_CONNECTION:
        return "advertising with no connection object waiting -- a host that answers is "
               "turned away";
    default:
        return "unknown";
    }
}

/* "profile 1 waits for its host (open)" / "profile 0's host is connected". */
static void rule_text(bool zmk_rule_asks, char *buf, size_t len) {
    const int profile = zmk_ble_active_profile_index();

    if (!zmk_rule_asks) {
        snprintf(buf, len, "profile %d's host is connected", profile);
    } else if (zmk_ble_active_profile_is_open()) {
        snprintf(buf, len, "profile %d waits for a host to pair (open)", profile);
    } else {
        snprintf(buf, len, "profile %d waits for its host (away)", profile);
    }
}

void advertising_out_of_step_log_state(void) {
    const struct advertising_look look = look_now();
    char rule[64];

    rule_text(look.zmk_rule_asks, rule, sizeof(rule));
    LOG_INF("  advertising: %s; ZMK's rule %s it (%s)%s", stack_text(look.stack),
            look.zmk_rule_asks ? "asks for" : "does not ask for", rule,
            in_step(&look) ? "" : " -- OUT OF STEP");
}

static int64_t out_of_step_since_ms = -1;
static bool out_of_step_logged;

/* ---- no host answers ---- */

/* k_uptime_get() of the last host link that came up; written from the
   Bluetooth thread, read on the system work queue. */
static atomic_t last_host_connected_ms;

static void on_connected(struct bt_conn *conn, uint8_t err) {
    struct bt_conn_info info;

    if (err == 0 && bt_conn_get_info(conn, &info) == 0 && info.role == BT_CONN_ROLE_PERIPHERAL) {
        atomic_set(&last_host_connected_ms, (atomic_val_t)k_uptime_get_32());
    }
}

BT_CONN_CB_DEFINE(no_host_answers_watch) = {
    .connected = on_connected,
};

enum no_host_answers_stage {
    NOT_ADVERTISING_IN_STEP,
    WAITING_FOR_A_HOST,
    LOGGED_NO_HOST,
    WAITING_AFTER_RESTART,
    LOGGED_STILL_NO_HOST,
};

static enum no_host_answers_stage no_host_stage = NOT_ADVERTISING_IN_STEP;
/* k_uptime_get_32() the stage's clock runs from. */
static uint32_t no_host_stage_since_ms;

static bool host_connected_since(uint32_t since_ms) {
    return (int32_t)((uint32_t)atomic_get(&last_host_connected_ms) - since_ms) >= 0;
}

static void restart_advertising(void) {
    char name[CONFIG_BT_DEVICE_NAME_MAX + 1];

    /* ZMK's own stop and start: with the name unchanged nothing is stored,
       and ZMK's note of its advertising stays right. */
    strncpy(name, bt_get_name(), sizeof(name) - 1);
    name[sizeof(name) - 1] = '\0';

    const int err = zmk_ble_set_device_name(name);

    if (err != 0) {
        LOG_WRN("Advertising restart failed (%d)", err);
    } else {
        LOG_INF("Advertising restarted: waiting %u s for a host", AFTER_RESTART_WAIT_MS / 1000);
    }
}

/* A host paired to some profile: one that may come by itself. */
static bool some_profile_has_a_host(void) {
    for (uint8_t p = 0; p < ZMK_BLE_PROFILE_COUNT; p++) {
        if (!zmk_ble_profile_is_open(p)) {
            return true;
        }
    }
    return false;
}

/* After a restart: whether a host has come, and what that says. Checked
   before anything else -- the active profile's host coming back ends the
   advertising at once, and with it the stretch being watched. */
static bool restart_answered(void) {
    if (no_host_stage != WAITING_AFTER_RESTART || !host_connected_since(no_host_stage_since_ms)) {
        return false;
    }

    const uint32_t after = (uint32_t)atomic_get(&last_host_connected_ms) - no_host_stage_since_ms;

    if (after <= RESTART_FREED_IT_WITHIN_MS) {
        LOG_INF("A host answered %u ms after the advertising was restarted -- the advertiser "
                "was stuck, the restart freed it",
                after);
    } else {
        LOG_INF("A host came %u s after the advertising was restarted -- too late to tell "
                "whether the restart freed it (brought by hand?)",
                after / 1000U);
    }
    return true;
}

static void watch_no_host_answers(const struct advertising_look *look, uint32_t now) {
    const bool calling = look->zmk_rule_asks && look->stack == ADVERTISING_ON &&
                         some_profile_has_a_host();

    if (restart_answered()) {
        no_host_stage = calling ? WAITING_FOR_A_HOST : NOT_ADVERTISING_IN_STEP;
        no_host_stage_since_ms = now;
        return;
    }
    if (!calling) {
        no_host_stage = NOT_ADVERTISING_IN_STEP;
        return;
    }
    if (no_host_stage == NOT_ADVERTISING_IN_STEP) {
        no_host_stage = WAITING_FOR_A_HOST;
        no_host_stage_since_ms = now;
        return;
    }
    if (host_connected_since(no_host_stage_since_ms)) {
        /* Connections get through: the clock starts again from this one. */
        no_host_stage = WAITING_FOR_A_HOST;
        no_host_stage_since_ms = now;
        return;
    }

    const uint32_t waited = now - no_host_stage_since_ms;

    if (no_host_stage == WAITING_FOR_A_HOST && waited >= NO_HOST_ANSWERS_AFTER_MS) {
        char rule[64];

        rule_text(look->zmk_rule_asks, rule, sizeof(rule));
        LOG_WRN("Advertising for %u s and no host has connected -- not even a host of another "
                "profile (fine if none is near and awake): %s",
                waited / 1000U, rule);
        host_connections_log_bluetooth_state("when no host answers");
        if (IS_ENABLED(CONFIG_ZMK_BLE_HOST_LINK_RESTART_ADVERTISING_WHEN_NO_HOST_ANSWERS)) {
            restart_advertising();
            no_host_stage = WAITING_AFTER_RESTART;
            no_host_stage_since_ms = now;
        } else {
            no_host_stage = LOGGED_NO_HOST;
        }
    } else if (no_host_stage == WAITING_AFTER_RESTART && waited >= AFTER_RESTART_WAIT_MS) {
        LOG_WRN("Still no host %u s after the advertising was restarted -- the fault is below "
                "the advertiser",
                waited / 1000U);
        no_host_stage = LOGGED_STILL_NO_HOST;
    }
}

static void check_cb(struct k_work *work);
static K_WORK_DELAYABLE_DEFINE(check_work, check_cb);

static void check_cb(struct k_work *work) {
    ARG_UNUSED(work);

    const struct advertising_look look = look_now();
    const int64_t now = k_uptime_get();

    watch_no_host_answers(&look, k_uptime_get_32());
    if (in_step(&look)) {
        if (out_of_step_logged) {
            LOG_INF("Advertising back in step after %u s: %s",
                    (uint32_t)((now - out_of_step_since_ms) / 1000), stack_text(look.stack));
        }
        out_of_step_since_ms = -1;
        out_of_step_logged = false;
    } else if (out_of_step_since_ms < 0) {
        out_of_step_since_ms = now;
    } else if (!out_of_step_logged && now - out_of_step_since_ms >= OUT_OF_STEP_AFTER_MS) {
        char rule[64];

        rule_text(look.zmk_rule_asks, rule, sizeof(rule));
        out_of_step_logged = true;
        if (look.zmk_rule_asks) {
            LOG_WRN("Advertising out of step for %u s: %s, but the keyboard is %s -- no host "
                    "can connect; %u of %d connection objects in use",
                    OUT_OF_STEP_AFTER_MS / 1000, rule, stack_text(look.stack),
                    look.connection_objects, CONFIG_BT_MAX_CONN);
        } else {
            LOG_WRN("Advertising out of step for %u s: %s, yet the keyboard is %s",
                    OUT_OF_STEP_AFTER_MS / 1000, rule, stack_text(look.stack));
        }
    }
    k_work_reschedule(&check_work, K_MSEC(CHECK_EVERY_MS));
}

static int advertising_out_of_step_log_init(void) {
    k_work_reschedule(&check_work, K_MSEC(FIRST_CHECK_AFTER_MS));
    return 0;
}

SYS_INIT(advertising_out_of_step_log_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);
