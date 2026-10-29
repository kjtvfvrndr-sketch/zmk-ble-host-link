/*
 * SPDX-License-Identifier: MIT
 *
 * zmk-ble-host-link: the parameters of the Bluetooth link to the computer.
 *
 * For every host (BLE profile) this finds the fastest interval the host
 * accepts and the latency that goes with it (search.c), keeps the result in
 * settings, and asks for it on every later connection. Any parameter can be
 * fixed by hand per profile instead; it replaces its part of the request. The
 * pointer report interval follows the interval the host gave, unless fixed by
 * hand too.
 *
 * Connection parameters can only be requested; the host decides. A request
 * is accepted when the link moves onto exactly what was asked
 * (hl_link_matches) before the answer deadline; anything else counts as a
 * refusal -- unless the link moves onto it late, before anything else is
 * asked. One request at a time: the next goes out once the last is answered
 * or its deadline has passed.
 *
 * Around that:
 *   - a stored result or a value fixed by hand the host refuses is asked for
 *     once more 5 s later; a stored result refused again makes the host be
 *     searched now (decisions.c: stored and candidate);
 *   - a change of the link made by someone else -- ZMK Studio, the host -- is
 *     undone, unless it is ZMK Studio's while open, or the host insists;
 *   - a link dropped after a request counts against it only if the host comes
 *     straight back, the link did not drop by itself lately, and the same
 *     request is blamed on two connections in a row (decisions.c);
 *   - a host that connects while another profile is active is asked for its
 *     own parameters once; optionally, only the active profile's host stays
 *     connected.
 *
 * Threading. The Bluetooth callbacks run in Bluetooth's own contexts -- the
 * RX thread, where the HCI command behind bt_conn_le_param_update() would
 * deadlock, or Zephyr's deferred connection work for a disconnect -- so they
 * only note what happened and schedule work. Everything else runs on the
 * system work queue under hl_lock -- the same place Zephyr's own automatic
 * update calls bt_conn_le_param_update() from.
 */

#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/init.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/util.h>
#include <zephyr/settings/settings.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/hci_types.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/logging/log.h>

#if IS_ENABLED(CONFIG_INPUT)
#include <zephyr/input/input.h>
#endif

#if IS_ENABLED(CONFIG_ZMK_BLE_HOST_LINK_LOG_RSSI)
#include <zephyr/bluetooth/hci.h>
#include <zephyr/net/buf.h>
#include <zephyr/sys/byteorder.h>
#endif

#include <zmk/ble.h>
#include <zmk/endpoints.h>
#include <zmk/event_manager.h>
#include <zmk/events/ble_active_profile_changed.h>
#include <zmk/events/endpoint_changed.h>
#include <zmk/events/keycode_state_changed.h>
#include <zmk/events/position_state_changed.h>
#include <dt-bindings/zmk/keys.h>

#include <zmk_ble_host_link/ble_host_link.h>

#include "decisions.h"
#include "host_connections_log.h"
#include "search.h"

#if IS_ENABLED(CONFIG_PMW3610_ALT_REPORT_INTERVAL_RUNTIME)
#include <zmk_pmw3610_alt/report_interval.h>
#define HAVE_BALL_REPORT 1
#else
#define HAVE_BALL_REPORT 0
#endif

#if IS_ENABLED(CONFIG_ZMK_BLE_HOST_LINK_LED_FEEDBACK)
#include <zmk_rgbled_widget/widget.h>
#endif

LOG_MODULE_REGISTER(zmk_ble_host_link, CONFIG_ZMK_BLE_HOST_LINK_LOG_LEVEL);

#define PROFILE_COUNT ZMK_BLE_PROFILE_COUNT
BUILD_ASSERT(PROFILE_COUNT <= 32, "the profile masks hold 32 profiles");

#define FASTEST_INTERVAL CONFIG_ZMK_BLE_HOST_LINK_FASTEST_INTERVAL
#define SLOWEST_INTERVAL CONFIG_ZMK_BLE_HOST_LINK_SLOWEST_INTERVAL
#define SUPERVISION_TIMEOUT (CONFIG_ZMK_BLE_HOST_LINK_SUPERVISION_TIMEOUT_MS / 10)
#define LATENCY_WHILE_SEARCHING CONFIG_ZMK_BLE_HOST_LINK_LATENCY_WHILE_SEARCHING_INTERVAL
#define IDLE_LISTEN_PERIOD_MS CONFIG_ZMK_BLE_HOST_LINK_IDLE_LISTEN_PERIOD_MS
#define RETRY_REFUSED_REQUEST_AFTER_MS CONFIG_ZMK_BLE_HOST_LINK_RETRY_REFUSED_REQUEST_AFTER_MS

BUILD_ASSERT(CONFIG_ZMK_BLE_HOST_LINK_SUPERVISION_TIMEOUT_MS % 10 == 0,
             "ZMK_BLE_HOST_LINK_SUPERVISION_TIMEOUT_MS must be a multiple of 10");
BUILD_ASSERT(SLOWEST_INTERVAL < 4 * SUPERVISION_TIMEOUT,
             "the supervision timeout is too short for ZMK_BLE_HOST_LINK_SLOWEST_INTERVAL");
BUILD_ASSERT(CONFIG_ZMK_BLE_HOST_LINK_SLOWEST_SETTABLE_INTERVAL < 4 * SUPERVISION_TIMEOUT,
             "the supervision timeout is too short for "
             "ZMK_BLE_HOST_LINK_SLOWEST_SETTABLE_INTERVAL");

/* Zephyr's own automatic request would go out besides this module's, five
   seconds into every connection, and undo it (Kconfig sets it n). */
BUILD_ASSERT(!IS_ENABLED(CONFIG_BT_GAP_AUTO_UPDATE_CONN_PARAMS),
             "zmk-ble-host-link sends every request of the host link: "
             "BT_GAP_AUTO_UPDATE_CONN_PARAMS must stay off");

/* A link that drops this soon after it moved onto a request counts as
   dropped by it: parameters the link cannot keep end in a supervision
   timeout, which takes the timeout itself to show -- the one the link runs,
   which the host may have set longer than asked -- plus this margin. */
#define DROP_MARGIN_AFTER_TIMEOUT_MS 2000
/* A request that could not be sent at all -- the controller busy with another
 * procedure, say, on this link or the split link -- is sent again: first
 * quickly, then after longer pauses. After that the operation ends, and the
 * host is held to nothing it did not get to answer. */
#define SEND_RETRIES_QUICK 3
#define SEND_RETRY_QUICK_MS 300
#define SEND_RETRIES_TOTAL 6
#define SEND_RETRY_SLOW_MS 5000
/* ZMK Studio's request seen this soon after a connection is its notification
 * subscription restored with the bond, not someone opening Studio now: the
 * module's own request is sent again over it. */
#define ZMK_STUDIO_RESTORED_AT_CONNECT_MS 15000
/* While ZMK Studio has the link, whether it is still subscribed is checked
 * this often: a host may refuse the request Studio sends when it closes. */
#define ZMK_STUDIO_CHECK_MS 30000

/* Interval in 1.25 ms units, printed as "%u.%02u" ms. */
#define MS2(u) ((uint32_t)(u) * 125U / 100U), ((uint32_t)(u) * 125U % 100U)
/* A time in ms, printed as "%u.%u" s. */
#define S1(ms) ((uint32_t)(ms) / 1000U), ((uint32_t)(ms) % 1000U / 100U)

/* ------------------------------------------------------------- records */

/* 6: one exact interval everywhere, stored result and candidate. Records of
   another format or size are dropped on load and the host searched again. */
#define RECORD_FORMAT 6

struct hl_record {
    uint8_t format;
    /* Pointer report interval + 1 (ms); 0: derived from the host interval. */
    uint8_t pinned_ball_report_plus1;
    /* 0: chosen automatically. */
    uint16_t pinned_interval;
    /* Latency + 1; 0: chosen automatically. */
    uint16_t pinned_latency_plus1;
    /* The host the values belong to; BT_ADDR_LE_ANY when set on an open slot. */
    bt_addr_le_t peer;
    /* What the host taught: the stored result, the candidate, drop limits. */
    struct hl_learned learned;
};

static struct hl_record recs[PROFILE_COUNT];
static uint32_t dirty;  /* profiles to write */
static uint32_t stored; /* profiles that have a record in settings */
/* Profiles whose stored record was of no use (another format): deleted. */
static uint32_t stale;
/* Records of profiles this build does not have (ZMK_BLE_PROFILE_COUNT went
   down): deleted too. */
static uint32_t stale_beyond_profiles;
static bool loaded;
static K_MUTEX_DEFINE(hl_lock);

/* What is known about each profile's host since boot, kept out of flash. */
struct hl_session {
    /* Drops of its link, and the requests they blame. */
    struct hl_drop_watch drops;
    /* The connection the host was last searched on -- its bt_conn_index()
       and conn_up_ms -- and what that search found: selected again on the
       same connection, the profile does not start over. A search after the
       stored result was refused twice: what it found is asked for, not the
       stored result again. Nothing found: the host keeps its own. */
    bool searched;
    bool searched_after_refusal;
    uint8_t searched_conn_index;
    uint32_t searched_connection;
    bool search_found;
    struct hl_params search_result;
    /* The slot's host this session is about: another one starts it over. */
    bt_addr_le_t peer;
};

static struct hl_session sessions[PROFILE_COUNT];

static void save_work_cb(struct k_work *work);
static K_WORK_DELAYABLE_DEFINE(save_work, save_work_cb);

static bool rec_is_default(const struct hl_record *r) {
    static const struct hl_learned nothing_learned;

    return r->pinned_ball_report_plus1 == 0 && r->pinned_interval == 0 &&
           r->pinned_latency_plus1 == 0 &&
           memcmp(&r->learned, &nothing_learned, sizeof(nothing_learned)) == 0;
}

/* Changes by key wait for ZMK's settings debounce; results of a search are
   rare and written at once. */
static void mark_dirty(uint8_t p, bool now) {
    dirty |= BIT(p);
    k_work_reschedule(&save_work, now ? K_NO_WAIT : K_MSEC(CONFIG_ZMK_SETTINGS_SAVE_DEBOUNCE));
}

/* The record of profile p, checked against the host now bonded to the slot.
   Lock held. */
static struct hl_record *rec_get(uint8_t p) {
    struct hl_record *r = &recs[p];
    const bt_addr_le_t *peer = zmk_ble_profile_address(p);

    /* Drops seen with one host say nothing about the next -- a record with
       nothing in it included. */
    if (bt_addr_le_cmp(&sessions[p].peer, peer) != 0) {
        memset(&sessions[p], 0, sizeof(sessions[p]));
        bt_addr_le_copy(&sessions[p].peer, peer);
    }

    if (rec_is_default(r) || bt_addr_le_cmp(&r->peer, peer) == 0) {
        return r;
    }

    if (bt_addr_le_cmp(&r->peer, BT_ADDR_LE_ANY) == 0) {
        bt_addr_le_copy(&r->peer, peer);
        LOG_INF("Profile %u: values set before pairing now belong to its host", p);
    } else if (bt_addr_le_cmp(peer, BT_ADDR_LE_ANY) == 0) {
        memset(r, 0, sizeof(*r));
        LOG_INF("Profile %u: bond cleared, values reset", p);
    } else {
        memset(r, 0, sizeof(*r));
        LOG_INF("Profile %u: another host is bonded now, values reset", p);
    }
    mark_dirty(p, false);
    return r;
}

/* After a change: bind to the slot's host, or clear a record back at default. */
static void rec_changed(uint8_t p, bool now) {
    struct hl_record *r = &recs[p];

    if (rec_is_default(r)) {
        memset(r, 0, sizeof(*r));
    } else {
        bt_addr_le_copy(&r->peer, zmk_ble_profile_address(p));
    }
    mark_dirty(p, now);
}

/* The slot has a host: what a host taught belongs to it. A search or a drop
   judged just as the slot was cleared teaches nothing. */
static bool slot_bonded(uint8_t p) {
    return bt_addr_le_cmp(zmk_ble_profile_address(p), BT_ADDR_LE_ANY) != 0;
}

static bool interval_pinned(const struct hl_record *r) { return r->pinned_interval != 0; }
static bool latency_pinned(const struct hl_record *r) { return r->pinned_latency_plus1 != 0; }
static bool has_pin(const struct hl_record *r) { return interval_pinned(r) || latency_pinned(r); }
static bool has_stored_result(const struct hl_record *r) {
    return hl_result_exists(&r->learned.stored);
}

/* What a search of this profile's host works with. */
static void search_limits(const struct hl_record *r, struct hl_search_limits *limits) {
    *limits = (struct hl_search_limits){
        .fastest_interval = FASTEST_INTERVAL,
        .slowest_interval = SLOWEST_INTERVAL,
        .latency_while_searching_interval = LATENCY_WHILE_SEARCHING,
        .idle_listen_period_ms = IDLE_LISTEN_PERIOD_MS,
        .timeout = SUPERVISION_TIMEOUT,
        .pinned_latency =
            latency_pinned(r) ? r->pinned_latency_plus1 - 1 : HL_LATENCY_NOT_PINNED,
        .interval_floor = r->learned.interval_floor,
        .latency_ceiling = hl_learned_latency_ceiling(&r->learned),
    };
}

/* The latency chosen automatically at an interval: the idle listening period
   wanted, under the ceiling learned from drops. */
static uint16_t auto_latency_at(const struct hl_record *r, uint16_t interval) {
    return hl_latency_under_ceiling(
        hl_latency_for_idle_listen_period(interval, IDLE_LISTEN_PERIOD_MS, SUPERVISION_TIMEOUT),
        hl_learned_latency_ceiling(&r->learned));
}

/*
 * What to ask the host for when not searching: the interval fixed by hand,
 * else the stored result's, else the secure request's (the slowest
 * interval). The latency fixed by hand, else the stored result's, else the
 * one that goes with the interval. Lock held.
 */
static void compose_request(const struct hl_record *r, struct hl_params *p) {
    const struct hl_learned *l = &r->learned;
    struct hl_search_limits limits;

    search_limits(r, &limits);
    p->timeout = SUPERVISION_TIMEOUT;

    if (interval_pinned(r)) {
        p->interval = r->pinned_interval;
    } else if (has_stored_result(r)) {
        p->interval = l->stored.interval;
    } else {
        p->interval = SLOWEST_INTERVAL;
    }

    if (latency_pinned(r)) {
        p->latency = hl_latency_allowed(r->pinned_latency_plus1 - 1, p->interval, p->timeout);
    } else if (interval_pinned(r)) {
        p->latency = auto_latency_at(r, p->interval);
    } else if (has_stored_result(r)) {
        /* Within what the timeout configured now allows. */
        p->latency = hl_latency_under_ceiling(
            hl_latency_allowed(l->stored.latency, p->interval, p->timeout),
            limits.latency_ceiling);
    } else {
        p->latency = hl_search_latency_at(&limits, p->interval);
    }
}

/* What the values fixed by hand are, for the log. */
static const char *pinned_values(const struct hl_record *r) {
    if (interval_pinned(r) && latency_pinned(r)) {
        return "the values fixed by hand";
    }
    return interval_pinned(r) ? "the interval fixed by hand" : "the latency fixed by hand";
}

/* A latency fixed by hand the spec does not allow at the interval asked. */
static void log_latency_trim(uint8_t profile, const struct hl_record *r, uint16_t latency_asked) {
    if (latency_pinned(r) && latency_asked < r->pinned_latency_plus1 - 1) {
        LOG_INF("Profile %u: latency %u trimmed to %u: the %u.%u s timeout must outlast two "
                "listening periods",
                profile, r->pinned_latency_plus1 - 1, latency_asked,
                S1(CONFIG_ZMK_BLE_HOST_LINK_SUPERVISION_TIMEOUT_MS));
    }
}

/* ------------------------------------------------------------------ LED */

enum verdict {
    VERDICT_ACCEPTED,
    VERDICT_REFUSED,
    VERDICT_NOT_CONNECTED_OR_SEARCHING,
};

#if IS_ENABLED(CONFIG_ZMK_BLE_HOST_LINK_LED_FEEDBACK)

/* The host link's parameters are the whole keyboard's: shown on every half
   of a split where the widget can. */
static void blink(uint32_t rgb, uint8_t count, uint16_t on_ms) {
#if IS_ENABLED(CONFIG_RGBLED_WIDGET_BLINK_ON_ALL_HALVES_API)
    rgbled_widget_blink_on_all_halves(rgb, count, on_ms, CONFIG_ZMK_BLE_HOST_LINK_LED_BLINK_OFF_MS);
#else
    rgbled_widget_blink(rgb, count, on_ms, CONFIG_ZMK_BLE_HOST_LINK_LED_BLINK_OFF_MS);
#endif
}

static void indicate(enum verdict v) {
    const uint16_t on = CONFIG_ZMK_BLE_HOST_LINK_LED_BLINK_ON_MS;

    switch (v) {
    case VERDICT_ACCEPTED:
        blink(CONFIG_ZMK_BLE_HOST_LINK_LED_RGB_HOST_ACCEPTED, 1, on * 2);
        break;
    case VERDICT_REFUSED:
        blink(CONFIG_ZMK_BLE_HOST_LINK_LED_RGB_HOST_REFUSED, 3, on);
        break;
    case VERDICT_NOT_CONNECTED_OR_SEARCHING:
        blink(CONFIG_ZMK_BLE_HOST_LINK_LED_RGB_NOT_CONNECTED_OR_SEARCHING, 1, on * 2);
        break;
    }
}

#else

static inline void indicate(enum verdict v) { ARG_UNUSED(v); }

#endif /* IS_ENABLED(CONFIG_ZMK_BLE_HOST_LINK_LED_FEEDBACK) */

/* -------------------------------------------------------------- links */

/* conn_up_ms: k_uptime_get_32() at the connect of each host link, by
   bt_conn_index(). Written in the RX context, read in work handlers. */
static atomic_t conn_up_ms[CONFIG_BT_MAX_CONN];

static bool is_host_link(struct bt_conn *conn, struct bt_conn_info *info) {
    return bt_conn_get_info(conn, info) == 0 && info->type == BT_CONN_TYPE_LE &&
           info->role == BT_CONN_ROLE_PERIPHERAL;
}

/* The active profile's host link, referenced, or NULL. */
static struct bt_conn *active_conn(void) {
    const bt_addr_le_t *addr = zmk_ble_active_profile_addr();

    if (bt_addr_le_cmp(addr, BT_ADDR_LE_ANY) == 0) {
        return NULL;
    }

    struct bt_conn *conn = bt_conn_lookup_addr_le(BT_ID_DEFAULT, addr);
    if (conn == NULL) {
        return NULL;
    }

    struct bt_conn_info info;
    if (bt_conn_get_info(conn, &info) != 0 || info.state != BT_CONN_STATE_CONNECTED) {
        bt_conn_unref(conn);
        return NULL;
    }
    return conn;
}

static bool read_link(struct bt_conn *conn, struct hl_link *link) {
    struct bt_conn_info info;

    if (conn == NULL || bt_conn_get_info(conn, &info) != 0 ||
        info.state != BT_CONN_STATE_CONNECTED) {
        return false;
    }
    link->interval = info.le.interval;
    link->latency = info.le.latency;
    link->timeout = info.le.timeout;
    return true;
}

static bool same_link(const struct hl_link *a, const struct hl_link *b) {
    return a->interval == b->interval && a->latency == b->latency && a->timeout == b->timeout;
}

/* How often the keyboard listens on this link with nothing to send (ms). */
static uint32_t idle_listen_period_ms(const struct hl_link *link) {
    return DIV_ROUND_UP((uint32_t)link->interval * 125U * (link->latency + 1U), 100U);
}

/* How long the link has been up (ms). */
static uint32_t link_up_ms(struct bt_conn *conn) {
    return k_uptime_get_32() - (uint32_t)atomic_get(&conn_up_ms[bt_conn_index(conn)]);
}

/* What is left of Zephyr's hold on the first request of a connection. */
static uint32_t request_hold_left_ms(struct bt_conn *conn) {
    const uint32_t up = link_up_ms(conn);

    return up < CONFIG_BT_CONN_PARAM_UPDATE_TIMEOUT ? CONFIG_BT_CONN_PARAM_UPDATE_TIMEOUT - up : 0;
}

/*
 * One request at a time on each link: the last one sent, its answer
 * deadline, and the connection it was sent on (its conn_up_ms), by
 * bt_conn_index(). The link moving onto it answers it at once
 * (answers_work); a refusal shows only as the deadline passing. Lock held.
 */
static struct {
    uint32_t connection;
    int64_t deadline_ms;
    struct hl_params asked;
} answer_due[CONFIG_BT_MAX_CONN];

static void answer_due_at(struct bt_conn *conn, int64_t deadline_ms,
                          const struct hl_params *asked) {
    const uint8_t i = bt_conn_index(conn);

    answer_due[i].connection = (uint32_t)atomic_get(&conn_up_ms[i]);
    answer_due[i].deadline_ms = deadline_ms;
    answer_due[i].asked = *asked;
}

static void answer_came(struct bt_conn *conn) { answer_due[bt_conn_index(conn)].deadline_ms = 0; }

/* How long the last request on the link may still be answered. */
static uint32_t answer_due_left_ms(struct bt_conn *conn) {
    const uint8_t i = bt_conn_index(conn);
    const int64_t now = k_uptime_get();

    if (answer_due[i].connection != (uint32_t)atomic_get(&conn_up_ms[i]) ||
        answer_due[i].deadline_ms <= now) {
        return 0;
    }
    return (uint32_t)(answer_due[i].deadline_ms - now);
}

/* When a request sent now must be answered by: on a slow link the keyboard
   hears the answer only when it listens. */
static int64_t answer_deadline(const struct hl_link *link) {
    return k_uptime_get() + CONFIG_ZMK_BLE_HOST_LINK_HOST_ANSWER_TIMEOUT_MS +
           2 * idle_listen_period_ms(link);
}

static void log_params(const char *what, const struct hl_params *p) {
    LOG_INF("%s %u.%02u ms, latency %u, timeout %u ms", what, MS2(p->interval), p->latency,
            p->timeout * 10U);
}

/* -------------------------------------------------------------- RSSI */

#if IS_ENABLED(CONFIG_ZMK_BLE_HOST_LINK_LOG_RSSI)

/* How often the active link's signal is read while nothing else reads it:
   what the log shows next to a drop. */
#define RSSI_SAMPLE_PERIOD_MS 30000

/* The last signal read on each host link: dBm, when (k_uptime_get(), 0
   none) and on which connection (its conn_up_ms). By bt_conn_index(). Lock
   held. */
static struct {
    int8_t dbm;
    int64_t at_ms;
    uint32_t connection;
} rssi_seen[CONFIG_BT_MAX_CONN];

/* The controller's RSSI of the link (BT_CTLR_CONN_RSSI). Lock held. */
static bool read_rssi(struct bt_conn *conn, int8_t *dbm) {
    uint16_t handle;
    struct net_buf *buf;
    struct net_buf *rsp = NULL;
    struct bt_hci_cp_read_rssi *cp;

    if (bt_hci_get_conn_handle(conn, &handle) != 0) {
        return false;
    }
    buf = bt_hci_cmd_create(BT_HCI_OP_READ_RSSI, sizeof(*cp));
    if (buf == NULL) {
        return false;
    }
    cp = net_buf_add(buf, sizeof(*cp));
    cp->handle = sys_cpu_to_le16(handle);
    if (bt_hci_cmd_send_sync(BT_HCI_OP_READ_RSSI, buf, &rsp) != 0 || rsp == NULL) {
        LOG_DBG("RSSI read failed -- is BT_CTLR_CONN_RSSI on?");
        return false;
    }
    *dbm = ((struct bt_hci_rp_read_rssi *)rsp->data)->rssi;
    net_buf_unref(rsp);

    const uint8_t i = bt_conn_index(conn);

    rssi_seen[i].dbm = *dbm;
    rssi_seen[i].at_ms = k_uptime_get();
    rssi_seen[i].connection = (uint32_t)atomic_get(&conn_up_ms[i]);
    return true;
}

/* The signal now, for the log. Lock held. */
static void log_rssi(struct bt_conn *conn) {
    int8_t dbm;

    if (read_rssi(conn, &dbm)) {
        LOG_INF("   host link RSSI: %d dBm", dbm);
    }
}

/* The last signal read before a link went down. Lock held. */
static void log_rssi_before_drop(uint8_t i, uint32_t connection, int profile) {
    if (rssi_seen[i].at_ms == 0 || rssi_seen[i].connection != connection) {
        return;
    }
    LOG_INF("Profile %d: last host link RSSI %d dBm, %u s before the link went down", profile,
            rssi_seen[i].dbm, (uint32_t)((k_uptime_get() - rssi_seen[i].at_ms) / 1000));
}

static void rssi_work_cb(struct k_work *work);
static K_WORK_DELAYABLE_DEFINE(rssi_work, rssi_work_cb);

#else

static inline void log_rssi(struct bt_conn *conn) { ARG_UNUSED(conn); }
static inline void log_rssi_before_drop(uint8_t i, uint32_t connection, int profile) {
    ARG_UNUSED(i);
    ARG_UNUSED(connection);
    ARG_UNUSED(profile);
}

#endif /* IS_ENABLED(CONFIG_ZMK_BLE_HOST_LINK_LOG_RSSI) */

/*
 * ZMK Studio over Bluetooth asks for its own parameters while it is open --
 * the peripheral preferred interval with a lower latency -- and for the
 * preferred ones with the usual latency when it closes
 * (zmk/app/src/studio/gatt_rpc_transport.c), from the callback of its RPC
 * characteristic's subscription. Studio has the link when the link runs the
 * first and the host is subscribed to that characteristic.
 */
#if IS_ENABLED(CONFIG_ZMK_STUDIO_TRANSPORT_BLE) &&                                                 \
    defined(CONFIG_ZMK_STUDIO_TRANSPORT_BLE_PREF_LATENCY) &&                                       \
    defined(CONFIG_BT_PERIPHERAL_PREF_LATENCY) && defined(CONFIG_BT_PERIPHERAL_PREF_TIMEOUT)
#if CONFIG_ZMK_STUDIO_TRANSPORT_BLE_PREF_LATENCY < CONFIG_BT_PERIPHERAL_PREF_LATENCY
#define ZMK_STUDIO_ASKS_FOR_PARAMS 1
#endif
#endif

#if defined(ZMK_STUDIO_ASKS_FOR_PARAMS)

/* ZMK Studio is recognised by its parameters: the module's own requests must
   never look like them. */
BUILD_ASSERT(SUPERVISION_TIMEOUT > CONFIG_BT_PERIPHERAL_PREF_TIMEOUT + 1 ||
                 SUPERVISION_TIMEOUT + 1 < CONFIG_BT_PERIPHERAL_PREF_TIMEOUT,
             "ZMK_BLE_HOST_LINK_SUPERVISION_TIMEOUT_MS must differ from ZMK Studio's "
             "BT_PERIPHERAL_PREF_TIMEOUT by more than 10 ms");

/* ZMK_STUDIO_BT_RPC_CHRC_UUID (zmk/app/src/studio/uuid.h, private to ZMK). */
static const struct bt_uuid_128 zmk_studio_rpc_uuid = BT_UUID_INIT_128(
    BT_UUID_128_ENCODE(0x00000001, 0x0196, 0x6107, 0xc967, 0xc5cfb1c2482a));

static bool zmk_studio_subscribed(struct bt_conn *conn) {
    static const struct bt_gatt_attr *rpc;

    if (rpc == NULL) {
        rpc = bt_gatt_find_by_uuid(NULL, 0, &zmk_studio_rpc_uuid.uuid);
    }
    return rpc != NULL && bt_gatt_is_subscribed(conn, rpc, BT_GATT_CCC_INDICATE);
}

static bool is_zmk_studio_params(const struct hl_link *link) {
    return link->interval >= CONFIG_BT_PERIPHERAL_PREF_MIN_INT &&
           link->interval <= CONFIG_BT_PERIPHERAL_PREF_MAX_INT &&
           link->latency == CONFIG_ZMK_STUDIO_TRANSPORT_BLE_PREF_LATENCY &&
           link->timeout == CONFIG_BT_PERIPHERAL_PREF_TIMEOUT;
}

#else

static bool is_zmk_studio_params(const struct hl_link *link) {
    ARG_UNUSED(link);
    return false;
}

static bool zmk_studio_subscribed(struct bt_conn *conn) {
    ARG_UNUSED(conn);
    return false;
}

#endif /* ZMK_STUDIO_ASKS_FOR_PARAMS */

/* The host was subscribed to ZMK Studio already in the first seconds of the
   link: the subscription that came back with the bond, not Studio opened.
   Cleared once it is seen gone. By bt_conn_index(). */
static atomic_t zmk_studio_subscribed_at_connect[CONFIG_BT_MAX_CONN];

static void note_zmk_studio_subscription(struct bt_conn *conn) {
    const uint8_t i = bt_conn_index(conn);

    if (!zmk_studio_subscribed(conn)) {
        atomic_set(&zmk_studio_subscribed_at_connect[i], 0);
    } else if (link_up_ms(conn) < ZMK_STUDIO_RESTORED_AT_CONNECT_MS) {
        atomic_set(&zmk_studio_subscribed_at_connect[i], 1);
    }
}

/* ZMK Studio has the link, or only seems to: its subscription back with the
   bond at connect, its parameters fixed by hand. Lock held. */
static bool zmk_studio_on_link(struct bt_conn *conn, const struct hl_link *link) {
    note_zmk_studio_subscription(conn);
    return hl_zmk_studio_has_link(
        is_zmk_studio_params(link), zmk_studio_subscribed(conn),
        atomic_get(&zmk_studio_subscribed_at_connect[bt_conn_index(conn)]) != 0,
        link_up_ms(conn), ZMK_STUDIO_RESTORED_AT_CONNECT_MS);
}

/* ZMK Studio's request at connect -- its subscription back with the bond --
   may land on top of the module's own. */
static bool zmk_studio_request_at_connect(struct bt_conn *conn, const struct hl_link *link) {
    return is_zmk_studio_params(link) && zmk_studio_subscribed(conn) &&
           link_up_ms(conn) < ZMK_STUDIO_RESTORED_AT_CONNECT_MS;
}

/* ------------------------------------------------------- ball report */

uint16_t zmk_ble_host_link_ball_report_for_interval(uint16_t interval) {
    /* the host interval in us, rounded up to the 4 ms frame grid */
    const uint32_t interval_us = (uint32_t)interval * 1250U;

    return (uint16_t)MAX(4U, DIV_ROUND_UP(interval_us, 4000U) * 4U);
}

#if HAVE_BALL_REPORT
/* The report interval for the active profile: fixed by hand (0 too, every
   frame), derived from the link, or the driver default while unconnected. */
static uint16_t ball_report_wanted(void) {
    struct hl_record *r = rec_get(zmk_ble_active_profile_index());
    struct hl_link link;

    if (r->pinned_ball_report_plus1) {
        return r->pinned_ball_report_plus1 - 1;
    }

    struct bt_conn *conn = active_conn();
    const bool connected = read_link(conn, &link);

    if (conn) {
        bt_conn_unref(conn);
    }
    return connected ? zmk_ble_host_link_ball_report_for_interval(link.interval)
                     : pmw3610_alt_get_ble_report_interval_default();
}
#endif

/* Lock held. */
static void apply_ball_report(void) {
#if HAVE_BALL_REPORT
    pmw3610_alt_set_ble_report_interval(ball_report_wanted());
#endif
}

/* ------------------------------------------------------------- input */

/* k_uptime_get_32() of the last key or pointer event: search requests and
   undos wait for a pause in use. */
static atomic_t last_input_ms;

#if IS_ENABLED(CONFIG_INPUT)
static void on_input(struct input_event *evt) {
    ARG_UNUSED(evt);
    atomic_set(&last_input_ms, (atomic_val_t)k_uptime_get_32());
}
INPUT_CALLBACK_DEFINE(NULL, on_input);
#endif

static uint32_t input_quiet_left_ms(void) {
    const uint32_t since = k_uptime_get_32() - (uint32_t)atomic_get(&last_input_ms);
    const uint32_t need = CONFIG_ZMK_BLE_HOST_LINK_INPUT_QUIET_BEFORE_SEARCH_REQUEST_MS;

    return since < need ? need - since : 0;
}

/* --------------------------------------------- where the module left it */

/*
 * The active link as the module last left it -- after its own request was
 * answered -- so that a change made by anyone else shows. Only the active
 * profile's link is kept: one that stops being active is forgotten, and when
 * its profile is selected again it is taken on afresh (a search cut short by
 * the switch starts over). `conn` is only compared, never dereferenced;
 * settled_conn, its atomic twin, is cleared by the disconnect callback.
 */
static struct {
    struct bt_conn *conn;
    struct hl_link link;
    /* The request in effect on the link -- the stored result, what a search
       found, a value fixed by hand: what an undo asks for. */
    bool has_wanted;
    struct hl_params wanted;
    /* ZMK Studio has the link: left alone until it closes. */
    bool zmk_studio_open;
    /* The host keeps its own parameters until the next connection: it
       changed the link back after an undo, refused what was fixed by hand,
       or accepted nothing in a search. */
    bool host_insists;
    /* k_uptime_get() of the last undo; 0 none. */
    int64_t undone_at_ms;
} settled;

static atomic_ptr_t settled_conn;

/* The module left this very link -- not an earlier one whose struct bt_conn
   the pool has handed out again. Lock held. */
static bool is_settled(struct bt_conn *conn) {
    return conn != NULL && conn == settled.conn && atomic_ptr_get(&settled_conn) == conn;
}

static bool is_active_profile_link(struct bt_conn *conn) {
    return bt_addr_le_cmp(bt_conn_get_dst(conn), zmk_ble_active_profile_addr()) == 0;
}

/* Lock held. */
static void forget_settled(void) {
    settled.conn = NULL;
    atomic_ptr_set(&settled_conn, NULL);
}

/* The link as the module leaves it: `seen`, what its last answer showed, or
   the link now. Lock held. */
static void settle(struct bt_conn *conn, const struct hl_link *seen) {
    struct hl_link link;

    if (!is_active_profile_link(conn) || !read_link(conn, &link)) {
        if (is_settled(conn)) {
            forget_settled();
        }
        return;
    }
    if (!is_settled(conn)) {
        settled.conn = conn;
        settled.has_wanted = false;
        settled.zmk_studio_open = false;
        settled.host_insists = false;
        settled.undone_at_ms = 0;
        atomic_ptr_set(&settled_conn, conn);
    }
    settled.link = seen ? *seen : link;
}

/* The host was searched on this very connection: what that search left.
   Lock held. */
static bool searched_on_this_connection(uint8_t profile, struct bt_conn *conn) {
    const struct hl_session *ses = &sessions[profile];
    const uint8_t i = bt_conn_index(conn);

    return ses->searched && ses->searched_conn_index == i &&
           ses->searched_connection == (uint32_t)atomic_get(&conn_up_ms[i]);
}

/* A search after the stored result was refused twice found this on this
   very connection: it stands for the connection. Lock held. */
static bool connection_result(uint8_t profile, struct bt_conn *conn, struct hl_params *p) {
    const struct hl_session *ses = &sessions[profile];

    if (has_pin(rec_get(profile)) || !searched_on_this_connection(profile, conn) ||
        !ses->searched_after_refusal || !ses->search_found) {
        return false;
    }
    *p = ses->search_result;
    return true;
}

/* What the link should run: the request in effect when the module left it,
   what a search found on this connection, or what the profile asks for.
   Lock held. */
static void intended_request(struct bt_conn *conn, uint8_t profile, struct hl_params *p) {
    if (is_settled(conn) && settled.has_wanted) {
        *p = settled.wanted;
    } else if (!connection_result(profile, conn, p)) {
        compose_request(rec_get(profile), p);
    }
}

/* While ZMK Studio has the link, whether it is still there. */
static void zmk_studio_check_work_cb(struct k_work *work);
static K_WORK_DELAYABLE_DEFINE(zmk_studio_check_work, zmk_studio_check_work_cb);

/* Lock held, after settle(). */
static void zmk_studio_took_link(void) {
    settled.zmk_studio_open = true;
    k_work_reschedule(&zmk_studio_check_work, K_MSEC(ZMK_STUDIO_CHECK_MS));
}

/* The active link changed while the module had nothing on air. */
static void drift_work_cb(struct k_work *work);
static K_WORK_DELAYABLE_DEFINE(drift_work, drift_work_cb);

/* The link back on what it should run, a while after someone else changed
   it. */
static void undo_work_cb(struct k_work *work);
static K_WORK_DELAYABLE_DEFINE(undo_work, undo_work_cb);

/* ------------------------------------------------------- the operation */

/*
 * One operation at a time, on the active profile's link: a single request
 * (APPLY) or a search (SEARCH). A newer one replaces it -- though its first
 * request waits for the answer to the last one sent.
 */
enum op_kind {
    OP_NONE,
    OP_APPLY,
    OP_SEARCH,
};

enum apply_purpose {
    /* One request, its answer final: REQUEST_AGAIN, an undo. */
    APPLY_ONCE,
    /* The stored search result: asked once more after a refusal; refused
       again, the host is searched now. */
    APPLY_STORED_RESULT,
    /* With a value fixed by hand: asked once more after a refusal; refused
       again, left until the next connection. */
    APPLY_PINNED,
};

enum search_reason {
    SEARCH_FIRST,
    SEARCH_BY_USER,
    SEARCH_AFTER_STORED_REFUSED,
};

static struct {
    enum op_kind kind;
    struct bt_conn *conn; /* referenced */
    uint8_t profile;
    /* Started by a key: the final verdict shows on the LED. */
    bool user_initiated;
    /* Nothing fixed by hand in what it asks: a drop of the link may count
       against it. */
    bool blamable;
    /* What a single request is (OP_APPLY), for the log. */
    const char *asked_for;
    enum apply_purpose purpose;
    /* APPLY_STORED_RESULT, APPLY_PINNED: refused once already. */
    bool refused_once;
    /* APPLY_STORED_RESULT refused twice: the host is searched at
       escalate_at_ms, unless it answers late before. */
    bool escalation_pending;
    int64_t escalate_at_ms;

    struct hl_params asked;
    /* op.asked waits to go out at send_at_ms: Zephyr's hold at connect,
       the answer to an earlier request, a retry. */
    bool send_pending;
    int64_t send_at_ms;
    /* ...waiting only for the answer to the last request on the link: the
       answer coming sends it at once. */
    bool send_waits_for_answer;
    /* Attempts to get the current request sent at all. */
    uint8_t send_retries;
    /* On air, not judged yet. */
    bool awaiting;
    int64_t deadline_ms;
    /* When the last verdict came; INT64_MAX: none yet. The next search
       request waits a pause after it -- time for a late answer too. */
    int64_t last_verdict_at_ms;
    /* The link as the last request went out, and as the operation began
       (have_link_at_begin). */
    struct hl_link link_at_send;
    bool have_link_at_begin;
    struct hl_link link_at_begin;
    /* The last request was judged refused, and nothing was asked since: the
       link moving onto it now is a late acceptance. */
    bool refused_awaiting_late_answer;
    /* ZMK Studio's request at connect replaced this one once already. */
    bool resent_over_zmk_studio;
    /* What the last verdict showed. */
    bool have_seen;
    struct hl_link seen_link;

    /* The request the link last moved onto, and when: what a drop soon
       after is about (hl_drop_blame()). Or one already in effect when it
       was to be asked (taken_moved_link false): it holding through the
       window breaks the row of a request blamed before, but a drop in it is
       the link's own -- nothing was changed. */
    bool have_taken;
    bool taken_moved_link;
    bool taken_held;
    struct hl_result taken;
    int64_t taken_at_ms;
    uint32_t taken_window_ms;

    /* The operation ends with this request in effect (settled.wanted). */
    bool ends_on_wanted;
    struct hl_params wanted;
    /* ...or with the link where the host keeps it until the next
       connection (settled.host_insists). */
    bool ends_with_host_insisting;

    struct hl_search search;
    enum search_reason search_reason;
} op;

/* The link the operation runs on, for the RX callbacks. */
static atomic_ptr_t op_conn;
static atomic_t answer_arrived;
/* A search runs (zmk_ble_host_link_is_searching()). */
static atomic_t search_running;

static void op_work_cb(struct k_work *work);
static K_WORK_DELAYABLE_DEFINE(op_work, op_work_cb);

bool zmk_ble_host_link_is_searching(void) { return atomic_get(&search_running) != 0; }

static struct hl_result result_of(const struct hl_params *p) {
    return (struct hl_result){.interval = p->interval, .latency = p->latency};
}

static void op_end(void) {
    if (op.conn) {
        struct hl_link now;
        const bool have_now = read_link(op.conn, &now);

        settle(op.conn, op.have_seen ? &op.seen_link : NULL);
        if (is_settled(op.conn) && op.ends_on_wanted) {
            settled.has_wanted = true;
            settled.wanted = op.wanted;
        }
        if (is_settled(op.conn) && op.ends_with_host_insisting) {
            settled.host_insists = true;
        } else if (is_settled(op.conn) && op.ends_on_wanted && op.user_initiated) {
            /* Asked for by a key: what the host insisted on before is
               history -- a later change is undone again. */
            settled.host_insists = false;
            settled.undone_at_ms = 0;
        }
        if (is_settled(op.conn) && op.ends_on_wanted && have_now &&
            !hl_link_matches(&op.wanted, &now)) {
            /* The host moved the link off it meanwhile: put back. */
            LOG_INF("The link is at %u.%02u ms, latency %u, not on profile %u's parameters: "
                    "asking for them again in %u s",
                    MS2(now.interval), now.latency, op.profile,
                    CONFIG_ZMK_BLE_HOST_LINK_RESTORE_AFTER_FOREIGN_CHANGE_MS / 1000);
            settled.link = now;
            k_work_reschedule(&undo_work,
                              K_MSEC(CONFIG_ZMK_BLE_HOST_LINK_RESTORE_AFTER_FOREIGN_CHANGE_MS));
        } else if (is_settled(op.conn) && op.have_seen && have_now &&
                   !same_link(&now, &op.seen_link)) {
            /* The link moved after the last verdict -- the host, ZMK Studio
               -- while the operation still had it: looked at now. */
            k_work_reschedule(&drift_work, K_MSEC(50));
        }
        bt_conn_unref(op.conn);
    }
    atomic_ptr_set(&op_conn, NULL);
    atomic_set(&search_running, 0);
    op.conn = NULL;
    op.kind = OP_NONE;
    op.awaiting = false;
    op.send_pending = false;
    op.escalation_pending = false;
    /* op_work is left to run: an update that came as the operation ended
       goes to drift_work from there. */
}

/* ------------------------------------------------------------- drops */

/* A blame confirmed: what it teaches goes into the record. Lock held. */
static void blame_confirmed(uint8_t p, const struct hl_result *blamed) {
    struct hl_record *r = rec_get(p);

    if (!slot_bonded(p)) {
        return;
    }

    const unsigned int changed =
        hl_blame_confirmed(&r->learned, blamed, LATENCY_WHILE_SEARCHING, SLOWEST_INTERVAL);

    LOG_WRN("Profile %u: the link dropped after %u.%02u ms, latency %u on %u connections in a "
            "row",
            p, MS2(blamed->interval), blamed->latency, HL_DROPS_THAT_CONFIRM_BLAME);
    if (changed & HL_BLAME_LOWERS_LATENCY) {
        LOG_WRN("Profile %u: latency stays below %u on this host from now on", p,
                blamed->latency);
    }
    if (changed & HL_BLAME_RAISES_FLOOR) {
        LOG_WRN("Profile %u: intervals up to %u.%02u ms are skipped on this host from now on", p,
                MS2(blamed->interval));
    }
    if (changed & HL_BLAME_CLEARS_STORED) {
        LOG_WRN("Profile %u: the stored result goes -- the host is searched again", p);
    }
    if (changed & HL_BLAME_CLEARS_CANDIDATE) {
        LOG_WRN("Profile %u: the candidate goes", p);
    }
    rec_changed(p, true);
}

/*
 * A host link that went down, noted by the disconnect callback for
 * link_events_work: whether the operation or the probation was on it then.
 * And a host link that came up: its host is back. By bt_conn_index().
 */
static struct {
    atomic_t down;
    atomic_t up;
    struct bt_conn *conn; /* only compared */
    uint32_t connection;  /* its conn_up_ms */
    int profile;
    uint8_t reason;
    uint32_t link_up_ms;
    bool op_was_on_it;
    bool probation_was_on_it;
    int profile_up;
} link_events[CONFIG_BT_MAX_CONN];

/*
 * After an operation ends, the request the link last moved onto stays on
 * probation for its drop window: parameters the link cannot keep show as a
 * drop only after the supervision timeout. By bt_conn_index(); `probation_conn`
 * is the link, for the disconnect callback (only compared).
 */
static struct {
    uint8_t profile;
    struct hl_result request;
    bool blamable;
    /* false: the request was in effect already -- a drop is the link's own. */
    bool link_moved;
    int64_t until_ms;
} probations[CONFIG_BT_MAX_CONN];
static atomic_ptr_t probation_conn[CONFIG_BT_MAX_CONN];

static void probation_over_work_cb(struct k_work *work);
static K_WORK_DELAYABLE_DEFINE(probation_over_work, probation_over_work_cb);

static void probation_over_work_cb(struct k_work *work) {
    ARG_UNUSED(work);

    k_mutex_lock(&hl_lock, K_FOREVER);
    const int64_t now = k_uptime_get();
    int64_t next = INT64_MAX;

    for (uint8_t i = 0; i < CONFIG_BT_MAX_CONN; i++) {
        if (atomic_ptr_get(&probation_conn[i]) == NULL) {
            continue;
        }
        if (atomic_get(&link_events[i].down)) {
            /* Dropped: link_went_down() judges it. */
            continue;
        }
        if (now < probations[i].until_ms) {
            next = MIN(next, probations[i].until_ms);
            continue;
        }
        /* No drop: the request holds. */
        hl_request_held(&sessions[probations[i].profile].drops, &probations[i].request);
        atomic_ptr_set(&probation_conn[i], NULL);
    }
    if (next != INT64_MAX) {
        k_work_reschedule(&probation_over_work, K_MSEC(next - now));
    }
    k_mutex_unlock(&hl_lock);
}

/* The request the link last moved onto stays on probation. Lock held. */
static void start_probation(void) {
    const int64_t until = op.taken_at_ms + op.taken_window_ms;
    const uint8_t i = bt_conn_index(op.conn);

    if (!op.have_taken) {
        return;
    }
    if (k_uptime_get() >= until) {
        /* Through its window already: the request holds. */
        if (!op.taken_held) {
            op.taken_held = true;
            hl_request_held(&sessions[op.profile].drops, &op.taken);
        }
        return;
    }
    probations[i].profile = op.profile;
    probations[i].request = op.taken;
    probations[i].blamable = op.blamable;
    probations[i].link_moved = op.taken_moved_link;
    probations[i].until_ms = until;
    atomic_ptr_set(&probation_conn[i], op.conn);
    k_work_reschedule(&probation_over_work, K_NO_WAIT);
}

/* A request asked of a host whose profile is not the active one: a drop of
   its link in the request's drop window is not counted -- nor taken for the
   link dropping by itself. By bt_conn_index(); the connection is its
   conn_up_ms. Lock held. */
static struct {
    uint32_t connection;
    int64_t until_ms;
} inactive_request[CONFIG_BT_MAX_CONN];

/* The link moved onto op.asked now -- or, link_moved false, it ran op.asked
   already. Its drop window: the supervision timeout the link runs (or the
   one asked, if longer) and a margin. */
static void note_taken(bool link_moved) {
    struct hl_link link;
    uint32_t timeout_ms = CONFIG_ZMK_BLE_HOST_LINK_SUPERVISION_TIMEOUT_MS;

    if (read_link(op.conn, &link)) {
        timeout_ms = MAX(timeout_ms, (uint32_t)link.timeout * 10U);
    }
    op.have_taken = true;
    op.taken_moved_link = link_moved;
    op.taken_held = false;
    op.taken = result_of(&op.asked);
    op.taken_at_ms = k_uptime_get();
    op.taken_window_ms = timeout_ms + DROP_MARGIN_AFTER_TIMEOUT_MS;
}


/* What a drop of a link has to do with a request. */
enum drop_involves {
    /* No request: the link dropped by itself. */
    DROP_INVOLVES_NOTHING,
    /* A request the drop counts against (*blamed). */
    DROP_BLAMES_REQUEST,
    /* A request with a value fixed by hand, or one asked of a host whose
       profile is not the active one: not counted at all. */
    DROP_INVOLVES_UNBLAMABLE_REQUEST,
};

/* Which request a drop of the operation's link is about: one on air, or
   judged refused but still open to a late answer, or the one the link last
   moved onto. Lock held. */
static enum drop_involves op_drop_involves(struct hl_result *blamed) {
    const int64_t now = k_uptime_get();
    const bool late_answer_window =
        op.refused_awaiting_late_answer &&
        now <= op.last_verdict_at_ms + CONFIG_ZMK_BLE_HOST_LINK_PAUSE_BETWEEN_SEARCH_REQUESTS_MS;
    const bool request_out = op.awaiting || late_answer_window;

    switch (hl_drop_blame(request_out, op.have_taken, now - op.taken_at_ms,
                          op.taken_window_ms)) {
    case HL_DROP_BLAMES_REQUEST_OUT:
        *blamed = result_of(&op.asked);
        break;
    case HL_DROP_BLAMES_REQUEST_TAKEN:
        if (!op.taken_moved_link) {
            return DROP_INVOLVES_NOTHING;
        }
        *blamed = op.taken;
        break;
    case HL_DROP_BLAMES_NOTHING:
        return DROP_INVOLVES_NOTHING;
    }
    return op.blamable ? DROP_BLAMES_REQUEST : DROP_INVOLVES_UNBLAMABLE_REQUEST;
}

/* A host link went down. Lock held. */
static void link_went_down(uint8_t i) {
    struct bt_conn *conn = link_events[i].conn;
    const int p = link_events[i].profile;
    const uint8_t reason = link_events[i].reason;
    const int64_t now = k_uptime_get();
    struct hl_result blamed;
    enum drop_involves involves = DROP_INVOLVES_NOTHING;

    if (reason != HL_DROP_REASON_LOCAL_HOST) {
        log_rssi_before_drop(i, link_events[i].connection, p);
    }

    if (link_events[i].op_was_on_it && op.kind != OP_NONE && op.conn == conn) {
        involves = op_drop_involves(&blamed);
        LOG_INF("Profile %u: link down (reason 0x%02x), %s abandoned", op.profile, reason,
                op.kind == OP_SEARCH ? "the search" : "the request");
        op_end();
    }
    if (link_events[i].probation_was_on_it && atomic_ptr_get(&probation_conn[i]) == conn) {
        if (involves == DROP_INVOLVES_NOTHING && now <= probations[i].until_ms &&
            probations[i].link_moved) {
            blamed = probations[i].request;
            involves = probations[i].blamable ? DROP_BLAMES_REQUEST
                                              : DROP_INVOLVES_UNBLAMABLE_REQUEST;
        }
        atomic_ptr_set(&probation_conn[i], NULL);
    }
    if (involves == DROP_INVOLVES_NOTHING &&
        inactive_request[i].connection == link_events[i].connection &&
        now <= inactive_request[i].until_ms) {
        involves = DROP_INVOLVES_UNBLAMABLE_REQUEST;
    }
    if (p < 0 || p >= PROFILE_COUNT) {
        return;
    }
    if (involves == DROP_INVOLVES_UNBLAMABLE_REQUEST) {
        LOG_DBG("Profile %u: link down (reason 0x%02x) after a request that is not counted", p,
                reason);
        return;
    }

    struct hl_drop_watch *watch = &sessions[p].drops;
    struct hl_result confirmed;
    const bool have_blamed = involves == DROP_BLAMES_REQUEST;

    (void)rec_get((uint8_t)p);
    switch (hl_drop_seen(watch, reason, have_blamed ? &blamed : NULL, link_events[i].link_up_ms,
                         CONFIG_ZMK_BLE_HOST_LINK_STABLE_LINK_MS, now, &confirmed)) {
    case HL_DROP_SEEN_IGNORED:
        break;
    case HL_DROP_SEEN_UNPROVOKED:
        LOG_DBG("Profile %u: link down (reason 0x%02x), no request to blame", p, reason);
        break;
    case HL_DROP_SEEN_NOT_LOST_ON_AIR:
        LOG_DBG("Profile %u: link down (reason 0x%02x), not lost on air -- not a drop", p,
                reason);
        break;
    case HL_DROP_SEEN_HOST_UNSTABLE:
        LOG_INF("Profile %u: the link dropped after %u.%02u ms, latency %u (reason 0x%02x) -- "
                "not counted, the link dropped by itself lately",
                p, MS2(blamed.interval), blamed.latency, reason);
        break;
    case HL_DROP_SEEN_WAITS_FOR_HOST:
        LOG_WRN("Profile %u: the link dropped after %u.%02u ms, latency %u (reason 0x%02x) -- it "
                "counts if the host comes straight back",
                p, MS2(blamed.interval), blamed.latency, reason);
        break;
    case HL_DROP_SEEN_SUSPECTED:
        LOG_WRN("Profile %u: the host dropped the link over %u.%02u ms, latency %u (reason "
                "0x%02x) -- counted once",
                p, MS2(blamed.interval), blamed.latency, reason);
        break;
    case HL_DROP_SEEN_CONFIRMED:
        blame_confirmed((uint8_t)p, &confirmed);
        break;
    }
}

/* A host connected: a drop it left behind is judged now. Lock held. */
static void host_came_back(uint8_t p) {
    struct hl_drop_watch *watch = &sessions[p].drops;
    const int64_t away_ms = k_uptime_get() - watch->pending_at_ms;
    const uint32_t away_s = (uint32_t)CLAMP(away_ms / 1000, 0, (int64_t)UINT32_MAX);
    const struct hl_result pending = watch->pending_request;
    const bool request_blamed = watch->pending_blames_request;
    struct hl_result confirmed;

    switch (hl_host_returned(watch, k_uptime_get(),
                             CONFIG_ZMK_BLE_HOST_LINK_DROP_COUNTS_IF_HOST_RETURNS_WITHIN_MS,
                             &confirmed)) {
    case HL_RETURN_NOTHING_PENDING:
        break;
    case HL_RETURN_LATE:
        if (request_blamed) {
            LOG_INF("Profile %u: the host is back after %u s -- it slept or was switched off, "
                    "the drop does not count",
                    p, away_s);
        }
        break;
    case HL_RETURN_UNSTABLE:
        LOG_INF("Profile %u: the host is back %u s after the link dropped by itself -- drops "
                "count against requests again once a link holds %u s",
                p, away_s, CONFIG_ZMK_BLE_HOST_LINK_STABLE_LINK_MS / 1000);
        break;
    case HL_RETURN_SUSPECTED:
        LOG_WRN("Profile %u: the host is back %u s after the link dropped after %u.%02u ms, "
                "latency %u -- counted once, again on the next connection confirms it",
                p, away_s, MS2(pending.interval), pending.latency);
        break;
    case HL_RETURN_CONFIRMED:
        blame_confirmed(p, &confirmed);
        break;
    }
}

/* Links that went down, then links that came up: a host may come back on
   another slot. Lock held. */
static void process_link_events(void) {
    for (uint8_t i = 0; i < CONFIG_BT_MAX_CONN; i++) {
        if (atomic_cas(&link_events[i].down, 1, 0)) {
            link_went_down(i);
        }
    }
    for (uint8_t i = 0; i < CONFIG_BT_MAX_CONN; i++) {
        if (atomic_cas(&link_events[i].up, 1, 0)) {
            const int p = link_events[i].profile_up;

            if (p >= 0 && p < PROFILE_COUNT) {
                (void)rec_get((uint8_t)p);
                host_came_back((uint8_t)p);
            }
        }
    }
}

static void link_events_work_cb(struct k_work *work) {
    ARG_UNUSED(work);

    k_mutex_lock(&hl_lock, K_FOREVER);
    if (loaded) {
        process_link_events();
    }
    k_mutex_unlock(&hl_lock);
}

static K_WORK_DEFINE(link_events_work, link_events_work_cb);

/* ------------------------------------------------------- the operation */

static void op_begin(enum op_kind kind, struct bt_conn *conn, uint8_t profile, bool user,
                     bool blamable) {
    op_end();
    op.kind = kind;
    op.conn = bt_conn_ref(conn);
    op.profile = profile;
    op.user_initiated = user;
    op.blamable = blamable;
    op.asked_for = NULL;
    op.purpose = APPLY_ONCE;
    op.refused_once = false;
    op.escalation_pending = false;
    op.send_pending = false;
    op.send_waits_for_answer = false;
    op.send_retries = 0;
    op.last_verdict_at_ms = INT64_MAX;
    op.refused_awaiting_late_answer = false;
    op.resent_over_zmk_studio = false;
    op.have_seen = false;
    op.have_link_at_begin = read_link(conn, &op.link_at_begin);
    op.have_taken = false;
    op.ends_on_wanted = false;
    op.ends_with_host_insisting = false;
    atomic_set(&answer_arrived, 0);
    atomic_set(&search_running, kind == OP_SEARCH);
    atomic_ptr_set(&op_conn, conn);
    /* A new request on the link: the last one's probation is over. */
    atomic_ptr_cas(&probation_conn[bt_conn_index(conn)], conn, NULL);
}

/* What the request on air is, for the log. */
static const char *asked_for(void) {
    if (op.kind != OP_SEARCH) {
        return op.asked_for ? op.asked_for : "a request";
    }
    switch (op.search.asked_step) {
    case HL_STEP_SECURE:
        return "search: the secure request";
    case HL_STEP_LADDER:
        return "search: a ladder step";
    case HL_STEP_LATENCY:
        return "search: a latency step";
    }
    return "search";
}

static void op_verdict(bool accepted, bool on_air);

/* op.asked waits to go out after `delay_ms`. */
static void send_later(uint32_t delay_ms) {
    op.send_pending = true;
    op.send_waits_for_answer = false;
    op.send_at_ms = k_uptime_get() + delay_ms;
    k_work_reschedule(&op_work, K_MSEC(delay_ms));
}

/* Send op.asked. Lock held; may end the operation. */
static void op_send(void) {
    /* In the first BT_CONN_PARAM_UPDATE_TIMEOUT of a connection Zephyr would
       hold the request and send it later from its own work, telling no one
       if that fails -- or drop it if the host's own update happens to meet
       it. So the request waits that out here, and goes out at once. And it
       waits for the answer to the last request sent on the link. */
    const uint32_t hold_ms = request_hold_left_ms(op.conn);
    const uint32_t answer_ms = answer_due_left_ms(op.conn);
    const uint32_t wait_ms = MAX(hold_ms > 0 ? hold_ms + 100 : 0, answer_ms);

    if (wait_ms > 0) {
        send_later(wait_ms);
        op.send_waits_for_answer = answer_ms > 0;
        return;
    }

    struct hl_link link;

    op.send_pending = false;
    op.refused_awaiting_late_answer = false;
    if (!read_link(op.conn, &link)) {
        /* Going down: link_events_work ends the operation. */
        return;
    }
    op.link_at_send = link;
    if (hl_link_matches(&op.asked, &link)) {
        /* The link runs it already: nothing to send. */
        LOG_DBG("Already in effect: %u.%02u ms, latency %u", MS2(link.interval), link.latency);
        op_verdict(true, false);
        return;
    }

    const struct bt_le_conn_param p =
        BT_LE_CONN_PARAM_INIT(op.asked.interval, op.asked.interval, op.asked.latency,
                              op.asked.timeout);

    atomic_set(&answer_arrived, 0);

    const int err = bt_conn_le_param_update(op.conn, &p);

    if (err == 0) {
        op.awaiting = true;
        op.deadline_ms = answer_deadline(&link);
        answer_due_at(op.conn, op.deadline_ms, &op.asked);
        LOG_INF("Asking for %s:", asked_for());
        log_params("  ", &op.asked);
        log_rssi(op.conn);
        k_work_reschedule(&op_work, K_MSEC(op.deadline_ms - k_uptime_get()));
        return;
    }
    if (err == -EALREADY) {
        /* Zephyr finds the link on it already (the link moved meanwhile). */
        op_verdict(true, false);
        return;
    }

    /* Not sent at all -- the controller busy with another procedure, on this
       link or the split link. Not the host's answer: try again, and in the
       end give up without holding the host to anything. */
    if (op.send_retries < SEND_RETRIES_TOTAL) {
        const uint32_t delay_ms =
            op.send_retries < SEND_RETRIES_QUICK ? SEND_RETRY_QUICK_MS : SEND_RETRY_SLOW_MS;

        op.send_retries++;
        LOG_WRN("Request not sent (%d), trying again in %u ms", err, delay_ms);
        send_later(delay_ms);
        return;
    }
    LOG_WRN("Request could not be sent (%d): given up, nothing learned from it", err);
    if (op.user_initiated) {
        indicate(VERDICT_REFUSED);
    }
    op_end();
}

static void op_apply(struct bt_conn *conn, uint8_t profile, const struct hl_params *p, bool user,
                     bool blamable, enum apply_purpose purpose, const char *what) {
    op_begin(OP_APPLY, conn, profile, user, blamable);
    op.purpose = purpose;
    op.asked_for = what;
    op.asked = *p;
    op_send();
}

static void op_search(struct bt_conn *conn, uint8_t profile, bool user,
                      enum search_reason reason);

/* Decide what a link needs and start it. Lock held. */
static void start_for_link(struct bt_conn *conn, uint8_t profile, bool user) {
    struct hl_record *r = rec_get(profile);
    const struct hl_learned *l = &r->learned;
    struct hl_params p;

    LOG_DBG("Profile %u: stored %u.%02u ms latency %u, candidate %u.%02u ms latency %u, "
            "interval floor %u.%02u ms, latency ceiling %d",
            profile, MS2(l->stored.interval), l->stored.latency, MS2(l->candidate.interval),
            l->candidate.latency, MS2(l->interval_floor), (int)l->latency_ceiling_plus1 - 1);

    /* What was in effect belongs to the settings before: the request about
       to go out says anew what is wanted. */
    settled.has_wanted = false;

    if (!has_pin(r) && searched_on_this_connection(profile, conn)) {
        /* The host was searched on this connection already. */
        if (connection_result(profile, conn, &p)) {
            op_apply(conn, profile, &p, user, true, APPLY_ONCE,
                     "what the search found on this connection");
            return;
        }
        if (sessions[profile].search_found) {
            /* A first search: its result is the stored one. */
            goto ask_stored;
        }
        LOG_INF("Profile %u: the host accepted nothing on this connection -- it keeps its own "
                "parameters",
                profile);
        op_end();
        settle(conn, NULL);
        if (is_settled(conn)) {
            settled.host_insists = true;
        }
        if (user) {
            indicate(VERDICT_REFUSED);
        }
        return;
    }

    if (!interval_pinned(r) && !has_stored_result(r)) {
        op_search(conn, profile, user, SEARCH_FIRST);
        return;
    }

ask_stored:
    compose_request(r, &p);
    log_latency_trim(profile, r, p.latency);
    if (has_pin(r)) {
        op_apply(conn, profile, &p, user, false, APPLY_PINNED, pinned_values(r));
    } else {
        op_apply(conn, profile, &p, user, true, APPLY_STORED_RESULT, "the stored search result");
    }
}

static void search_finish(void) {
    struct hl_record *r = rec_get(op.profile);
    const struct hl_search *s = &op.search;
    struct hl_result found = {0};

    if (s->have_result) {
        found = result_of(&s->result);
        /* Searched at a latency fixed by hand: the one chosen automatically
           is stored, for when it is handed back. */
        if (latency_pinned(r)) {
            found.latency = auto_latency_at(r, found.interval);
        }
    }

    {
        struct hl_session *ses = &sessions[op.profile];
        const uint8_t i = bt_conn_index(op.conn);

        ses->searched = true;
        ses->searched_after_refusal = op.search_reason == SEARCH_AFTER_STORED_REFUSED;
        ses->searched_conn_index = i;
        ses->searched_connection = (uint32_t)atomic_get(&conn_up_ms[i]);
        ses->search_found = s->have_result;
        ses->search_result = s->result;
    }

    if (!slot_bonded(op.profile)) {
        /* The slot was cleared meanwhile: nothing to keep. */
    } else if (op.search_reason == SEARCH_AFTER_STORED_REFUSED) {
        switch (hl_search_after_stored_refused(&r->learned, s->have_result ? &found : NULL)) {
        case HL_RESEARCH_FOUND_NOTHING:
            LOG_INF("Profile %u: the host accepted nothing -- the stored result stays", op.profile);
            break;
        case HL_RESEARCH_FOUND_STORED:
            LOG_INF("Profile %u: found the stored result itself -- it stays", op.profile);
            rec_changed(op.profile, true);
            break;
        case HL_RESEARCH_CANDIDATE_STORED:
            LOG_INF("Profile %u: found the candidate again -- it is the stored result now",
                    op.profile);
            rec_changed(op.profile, true);
            break;
        case HL_RESEARCH_NEW_CANDIDATE:
            LOG_INF("Profile %u: found something else -- used on this connection and kept as the "
                    "candidate; the stored result stays",
                    op.profile);
            rec_changed(op.profile, true);
            break;
        }
    } else if (hl_first_search_finished(&r->learned, s->have_result ? &found : NULL)) {
        rec_changed(op.profile, true);
    }

    if (s->have_result) {
        LOG_INF("Search on profile %u done after %u requests:", op.profile, s->requests);
        log_params("  host link", &s->result);
        op.ends_on_wanted = true;
        op.wanted = s->result;
    } else {
        LOG_WRN("Search on profile %u done after %u requests: the host accepted nothing -- %s",
                op.profile, s->requests,
                op.search_reason == SEARCH_AFTER_STORED_REFUSED
                    ? "it keeps its own parameters on this connection"
                    : "searched again on the next connection");
        op.ends_with_host_insisting = true;
    }

    /* The request the link last moved onto stays on probation. */
    start_probation();

    if (op.user_initiated) {
        struct hl_link now;
        const bool on_result =
            s->have_result && read_link(op.conn, &now) && hl_link_matches(&s->result, &now);

        indicate(on_result ? VERDICT_ACCEPTED : VERDICT_REFUSED);
    }
    apply_ball_report();
    op_end();
}

/*
 * A host just paired is busy with the new device for a while -- Windows
 * installs its drivers -- and may refuse a request it would take a minute
 * later: Windows refused 7.5 ms ten seconds after pairing, and the profile
 * kept 8.75 ms. The search sends its first, secure request at once and the
 * rest from SEARCH_WAITS_AFTER_PAIRING_MS after the pairing.
 *
 * By bt_conn_index(): the connection (its conn_up_ms) the host paired on, and
 * when. From the pairing callback; read under the lock.
 */
static atomic_t paired_connection[CONFIG_BT_MAX_CONN];
static atomic_t paired_at_ms[CONFIG_BT_MAX_CONN];

static void on_pairing_complete(struct bt_conn *conn, bool bonded) {
    struct bt_conn_info info;

    ARG_UNUSED(bonded);
    if (!is_host_link(conn, &info)) {
        return;
    }

    const uint8_t i = bt_conn_index(conn);

    atomic_set(&paired_at_ms[i], (atomic_val_t)k_uptime_get_32());
    atomic_set(&paired_connection[i], atomic_get(&conn_up_ms[i]));
}

static struct bt_conn_auth_info_cb pairing_seen = {
    .pairing_complete = on_pairing_complete,
};

static int pairing_seen_init(void) {
    return bt_conn_auth_info_cb_register(&pairing_seen);
}

SYS_INIT(pairing_seen_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);

/* How long the search still waits for a host paired on this connection. */
static uint32_t settling_after_pairing_left_ms(struct bt_conn *conn) {
    const uint8_t i = bt_conn_index(conn);

    if (atomic_get(&paired_connection[i]) != atomic_get(&conn_up_ms[i])) {
        return 0;
    }

    const uint32_t since = k_uptime_get_32() - (uint32_t)atomic_get(&paired_at_ms[i]);
    const uint32_t need = CONFIG_ZMK_BLE_HOST_LINK_SEARCH_WAITS_AFTER_PAIRING_MS;

    return since < need ? need - since : 0;
}

/* The next request of the search, once the pause and a lull in use allow. */
static void search_step(void) {
    const int64_t now = k_uptime_get();
    int64_t ready_at = op.last_verdict_at_ms == INT64_MAX
                           ? now
                           : op.last_verdict_at_ms +
                                 CONFIG_ZMK_BLE_HOST_LINK_PAUSE_BETWEEN_SEARCH_REQUESTS_MS;

    /* The request the link moved onto gets its whole drop window before the
       next goes out: a drop in it is then about that request only. */
    if (op.have_taken) {
        ready_at = MAX(ready_at, op.taken_at_ms + op.taken_window_ms);
    }
    if (now < ready_at) {
        k_work_reschedule(&op_work, K_MSEC(ready_at - now));
        return;
    }
    /* A host just paired is still setting the new device up: the whole
       search, its first request too, waits until it is done. */
    static uint32_t settling_logged_for;
    const uint32_t settling = settling_after_pairing_left_ms(op.conn);

    if (settling > 0) {
        const uint32_t connection = (uint32_t)atomic_get(&conn_up_ms[bt_conn_index(op.conn)]);

        if (settling_logged_for != connection) {
            settling_logged_for = connection;
            LOG_INF("Profile %u's host paired just now: the search starts in %u s", op.profile,
                    DIV_ROUND_UP(settling, 1000U));
        }
        k_work_reschedule(&op_work, K_MSEC(settling));
        return;
    }
    if (op.search.requests > 0) {
        const uint32_t quiet = input_quiet_left_ms();

        if (quiet > 0) {
            k_work_reschedule(&op_work, K_MSEC(quiet));
            return;
        }
    }
    if (op.have_taken && !op.taken_held) {
        op.taken_held = true;
        hl_request_held(&sessions[op.profile].drops, &op.taken);
    }

    struct hl_params req;

    /* The next step is chosen: no late answer to the last one now. */
    op.refused_awaiting_late_answer = false;
    if (!hl_search_next(&op.search, &req)) {
        search_finish();
        return;
    }
    op.asked = req;
    op.send_retries = 0;
    op_send();
}

static void op_search(struct bt_conn *conn, uint8_t profile, bool user,
                      enum search_reason reason) {
    const struct hl_record *r = rec_get(profile);
    struct hl_search_limits limits;

    search_limits(r, &limits);
    op_begin(OP_SEARCH, conn, profile, user, !latency_pinned(r));
    op.search_reason = reason;
    hl_search_start(&op.search, &limits);
    LOG_INF("Searching the host link parameters of profile %u", profile);
    log_latency_trim(profile, r, hl_search_latency_at(&limits, SLOWEST_INTERVAL));
    if (user) {
        indicate(VERDICT_NOT_CONNECTED_OR_SEARCHING);
    }
    search_step();
}

/* ------------------------------------------------------- the answers */

/* A single request's final verdict, or the one more after a refusal. Lock
   held; may end the operation or start the next. */
static void apply_verdict(bool accepted) {
    if (accepted) {
        op.ends_on_wanted = true;
        op.wanted = op.asked;
        if (op.purpose == APPLY_STORED_RESULT &&
            hl_stored_result_accepted(&rec_get(op.profile)->learned)) {
            LOG_DBG("Profile %u: the stored result accepted -- the candidate goes", op.profile);
            rec_changed(op.profile, false);
        }
        if (op.user_initiated) {
            indicate(VERDICT_ACCEPTED);
        }
        start_probation();
        op_end();
        return;
    }

    if (op.purpose != APPLY_ONCE && !op.refused_once) {
        op.refused_once = true;
        LOG_INF("The host did not take %s: asking once more in %u s", asked_for(),
                RETRY_REFUSED_REQUEST_AFTER_MS / 1000);
        op.send_retries = 0;
        send_later(RETRY_REFUSED_REQUEST_AFTER_MS);
        return;
    }

    switch (op.purpose) {
    case APPLY_STORED_RESULT:
        /* The search waits the pause first: a late answer may come yet. */
        LOG_INF("Profile %u: the host refused the stored result twice -- searching it in %u s",
                op.profile, CONFIG_ZMK_BLE_HOST_LINK_PAUSE_BETWEEN_SEARCH_REQUESTS_MS / 1000);
        op.escalation_pending = true;
        op.escalate_at_ms =
            k_uptime_get() + CONFIG_ZMK_BLE_HOST_LINK_PAUSE_BETWEEN_SEARCH_REQUESTS_MS;
        k_work_reschedule(&op_work,
                          K_MSEC(CONFIG_ZMK_BLE_HOST_LINK_PAUSE_BETWEEN_SEARCH_REQUESTS_MS));
        return;
    case APPLY_PINNED:
        LOG_WRN("Profile %u: the host refused %s", op.profile, op.asked_for);
        indicate(VERDICT_REFUSED);
        op.ends_with_host_insisting = true;
        op_end();
        return;
    case APPLY_ONCE:
        if (op.user_initiated) {
            indicate(VERDICT_REFUSED);
        }
        op_end();
        return;
    }
}

/* The verdict on op.asked: the link moved onto it (accepted), or the
   deadline passed without it. on_air: it was sent (not already in effect).
   Lock held; may end the operation or start the next. */
static void op_verdict(bool accepted, bool on_air) {
    struct hl_link link;

    op.awaiting = false;
    op.last_verdict_at_ms = k_uptime_get();
    if (on_air && accepted) {
        answer_came(op.conn);
    }
    if (!read_link(op.conn, &link)) {
        /* Going down: link_events_work judges the request -- still on air
           -- and ends the operation. */
        op.awaiting = on_air;
        return;
    }
    op.have_seen = true;
    op.seen_link = link;

    if (accepted) {
        if (on_air) {
            LOG_INF("Host accepted: link at %u.%02u ms, latency %u, timeout %u ms",
                    MS2(link.interval), link.latency, link.timeout * 10U);
        }
        if (!same_link(&op.link_at_send, &link)) {
            note_taken(true);
        } else if (!on_air && op.kind == OP_APPLY) {
            /* In effect already: it holding breaks the row of drops blamed
               on it before. (A search moves on without waiting for it.) The
               link moved onto it since the operation began -- the answer to
               a request this one waited for -- is a move all the same. */
            note_taken(op.have_link_at_begin && !same_link(&op.link_at_begin, &link));
        }
    } else {
        LOG_INF("Host did NOT accept: link at %u.%02u ms%s, latency %u%s, timeout %u ms",
                MS2(link.interval), link.interval == op.asked.interval ? "" : " (asked otherwise)",
                link.latency, link.latency == op.asked.latency ? "" : " (asked otherwise)",
                link.timeout * 10U);

        if (zmk_studio_request_at_connect(op.conn, &link) && !op.resent_over_zmk_studio) {
            /* ZMK Studio's subscription came back with the bond, and its
               request -- sent by Zephyr as the connection settled -- landed
               on top of this one. Once more; after that, it is the host's
               answer. */
            op.resent_over_zmk_studio = true;
            LOG_INF("ZMK Studio's request at connect landed on top of this one: asking again");
            op_send();
            return;
        }
        if (zmk_studio_on_link(op.conn, &link)) {
            /* Not the host's answer: ZMK Studio has taken the link over.
               Taken up afresh when it closes (drift_work,
               zmk_studio_check_work). */
            LOG_INF("ZMK Studio has the link: %s",
                    op.kind == OP_SEARCH ? "the search starts over when it closes"
                                         : "the request waits until it closes");
            apply_ball_report();
            op_end();
            zmk_studio_took_link();
            return;
        }
        op.refused_awaiting_late_answer = true;
    }
    apply_ball_report();

    if (op.kind == OP_SEARCH) {
        hl_search_answer(&op.search, accepted);
        search_step();
        return;
    }
    apply_verdict(accepted);
}

/* The link moved onto the request judged refused, before anything else was
   asked: the host answered late. Lock held. */
static void late_acceptance(const struct hl_link *link) {
    LOG_INF("Host accepted late: link at %u.%02u ms, latency %u", MS2(link->interval),
            link->latency);
    op.refused_awaiting_late_answer = false;
    op.send_pending = false;
    op.escalation_pending = false;
    op.last_verdict_at_ms = k_uptime_get();
    op.have_seen = true;
    op.seen_link = *link;
    note_taken(true);
    apply_ball_report();
    if (op.kind == OP_SEARCH) {
        hl_search_late_answer(&op.search);
        search_step();
        return;
    }
    apply_verdict(true);
}

/* The link moved while nothing was on air: a late answer, ZMK Studio, or the
   host on its own. Returns whether the operation still goes on as it was.
   Lock held. */
static bool op_link_moved_between_requests(void) {
    struct hl_link link;

    if (!read_link(op.conn, &link)) {
        return false;
    }
    if (zmk_studio_on_link(op.conn, &link)) {
        LOG_INF("ZMK Studio has the link: %s",
                op.kind == OP_SEARCH ? "the search starts over when it closes"
                                     : "the request waits until it closes");
        op.have_seen = true;
        op.seen_link = link;
        apply_ball_report();
        op_end();
        zmk_studio_took_link();
        return false;
    }
    if (op.refused_awaiting_late_answer && hl_link_matches(&op.asked, &link)) {
        late_acceptance(&link);
        return false;
    }
    LOG_DBG("Link moved between requests: %u.%02u ms, latency %u", MS2(link.interval),
            link.latency);
    apply_ball_report();
    return true;
}

static void op_work_cb(struct k_work *work) {
    ARG_UNUSED(work);

    k_mutex_lock(&hl_lock, K_FOREVER);

    const int64_t now = k_uptime_get();
    const bool moved = atomic_set(&answer_arrived, 0) != 0;

    if (op.kind == OP_NONE) {
        /* The link moved as the operation ended: looked at as anyone's
           change. */
        if (moved) {
            k_work_reschedule(&drift_work, K_MSEC(50));
        }
        goto out;
    }
    if (op.awaiting) {
        struct hl_link link;

        if (!read_link(op.conn, &link)) {
            /* Going down: link_events_work ends the operation. */
            goto out;
        }
        if (hl_link_matches(&op.asked, &link)) {
            op_verdict(true, true);
        } else if (now >= op.deadline_ms) {
            op_verdict(false, true);
        } else {
            /* Not the answer -- a refusal comes as the old parameters, but
               so may another update completing first: until the deadline,
               the answer may still come. */
            k_work_reschedule(&op_work, K_MSEC(op.deadline_ms - now));
        }
        goto out;
    }
    if (moved && !op_link_moved_between_requests()) {
        goto out;
    }
    if (op.escalation_pending) {
        if (now < op.escalate_at_ms) {
            k_work_reschedule(&op_work, K_MSEC(op.escalate_at_ms - now));
            goto out;
        }

        struct bt_conn *conn = bt_conn_ref(op.conn);

        LOG_INF("Profile %u: searching the host now", op.profile);
        op_search(conn, op.profile, op.user_initiated, SEARCH_AFTER_STORED_REFUSED);
        bt_conn_unref(conn);
        goto out;
    }
    if (op.send_pending) {
        const bool answered = op.send_waits_for_answer && answer_due_left_ms(op.conn) == 0;

        if (now < op.send_at_ms && !answered) {
            k_work_reschedule(&op_work, K_MSEC(op.send_at_ms - now));
            goto out;
        }
        /* Due -- or the answer it waited for came: sent now, unless
           Zephyr's hold at connect is still on. */
        op_send();
        goto out;
    }
    if (op.kind == OP_SEARCH) {
        search_step();
    }
out:
    k_mutex_unlock(&hl_lock);
}

/* ----------------------------------------- following the active link */

static void link_work_cb(struct k_work *work) {
    ARG_UNUSED(work);

    k_mutex_lock(&hl_lock, K_FOREVER);
    if (!loaded) {
        goto out;
    }

    /* Drops and returns first: what they teach decides what is asked. */
    process_link_events();

    /* A slot cleared (BT_CLR, BT_CLR_ALL) or bonded to another host: its
       values are reset now, while the slot shows it -- not when the slot is
       next looked at, by which time the same host may have bonded again. */
    for (uint8_t p = 0; p < PROFILE_COUNT; p++) {
        (void)rec_get(p);
    }

    apply_ball_report();

    struct bt_conn *conn = active_conn();

    /* A link no longer the active profile's is forgotten: taken on afresh
       when its profile is selected again. */
    if (settled.conn != NULL && !is_settled(conn)) {
        forget_settled();
    }

    if (conn == NULL) {
        if (op.kind != OP_NONE) {
            op_end();
        }
        goto out;
    }

    /* A link the module has not seen yet: a new connection, or the profile
       of a link that was connected all along now selected. */
    if ((op.kind == OP_NONE || op.conn != conn) && !is_settled(conn)) {
        const int profile = zmk_ble_profile_index(bt_conn_get_dst(conn));

        /* A host still pairing has no profile yet: pairing completes with a
           profile change, which brings this back. */
        if (profile >= 0 && profile < PROFILE_COUNT) {
            start_for_link(conn, (uint8_t)profile, false);
        }
    }
    bt_conn_unref(conn);
out:
    k_mutex_unlock(&hl_lock);
}

static K_WORK_DELAYABLE_DEFINE(link_work, link_work_cb);

static void ball_work_cb(struct k_work *work) {
    ARG_UNUSED(work);

    k_mutex_lock(&hl_lock, K_FOREVER);
    if (loaded) {
        apply_ball_report();
    }
    k_mutex_unlock(&hl_lock);
}

static K_WORK_DELAYABLE_DEFINE(ball_work, ball_work_cb);

#if IS_ENABLED(CONFIG_ZMK_BLE_HOST_LINK_LOG_RSSI)
/* The active link's signal, read quietly now and then: the last one shows in
   the log when the link goes down. */
static void rssi_work_cb(struct k_work *work) {
    ARG_UNUSED(work);

    k_mutex_lock(&hl_lock, K_FOREVER);
    struct bt_conn *conn = loaded ? active_conn() : NULL;
    int8_t dbm;

    if (conn != NULL) {
        (void)read_rssi(conn, &dbm);
        bt_conn_unref(conn);
    }
    k_mutex_unlock(&hl_lock);
    k_work_reschedule(&rssi_work, K_MSEC(RSSI_SAMPLE_PERIOD_MS));
}
#endif

/* A change by key reaches the host once the presses have settled. */
static void key_change_work_cb(struct k_work *work) {
    ARG_UNUSED(work);

    k_mutex_lock(&hl_lock, K_FOREVER);
    struct bt_conn *conn = active_conn();

    if (conn == NULL) {
        indicate(VERDICT_NOT_CONNECTED_OR_SEARCHING);
    } else {
        start_for_link(conn, zmk_ble_active_profile_index(), true);
        bt_conn_unref(conn);
    }
    k_mutex_unlock(&hl_lock);
}

static K_WORK_DELAYABLE_DEFINE(key_change_work, key_change_work_cb);

/* ------------------------------- ZMK Studio's subscription at connect */

/* A few seconds into a connection, while the module would still resend over
   ZMK Studio's request: a subscription there now came back with the bond. */
static void zmk_studio_connect_work_cb(struct k_work *work) {
    ARG_UNUSED(work);

    k_mutex_lock(&hl_lock, K_FOREVER);
    struct bt_conn *conn = loaded ? active_conn() : NULL;

    if (conn != NULL) {
        note_zmk_studio_subscription(conn);
        bt_conn_unref(conn);
    }
    k_mutex_unlock(&hl_lock);
}

static K_WORK_DELAYABLE_DEFINE(zmk_studio_connect_work, zmk_studio_connect_work_cb);

/* -------------------------------- changes of the link made by others */

/* The active link changed while the module had nothing on air: ZMK Studio,
   the host, or the host's late answer to a request given up on. */
static void drift_work_cb(struct k_work *work) {
    ARG_UNUSED(work);

    k_mutex_lock(&hl_lock, K_FOREVER);
    struct bt_conn *conn = loaded && op.kind == OP_NONE ? active_conn() : NULL;
    struct hl_link link;

    if (conn == NULL) {
        goto out;
    }
    if (!is_settled(conn) || !read_link(conn, &link) || same_link(&link, &settled.link)) {
        goto out_unref;
    }

    const uint8_t profile = zmk_ble_active_profile_index();
    struct hl_params wanted;

    settled.link = link;
    if (zmk_studio_on_link(conn, &link)) {
        k_work_cancel_delayable(&undo_work);
        zmk_studio_took_link();
        LOG_INF("ZMK Studio asked for its own link parameters: left alone while it is open");
        goto out_unref;
    }
    if (settled.zmk_studio_open) {
        settled.zmk_studio_open = false;
        LOG_INF("ZMK Studio closed: asking for profile %u's parameters again", profile);
        start_for_link(conn, profile, false);
        goto out_unref;
    }
    intended_request(conn, profile, &wanted);
    if (hl_link_matches(&wanted, &link)) {
        /* What the link should run: a late answer to the module's own
           request, say. The host no longer keeps its own. */
        if (settled.host_insists) {
            LOG_INF("Host link now runs what profile %u asks for: the host took it late",
                    profile);
        } else {
            LOG_DBG("Host link now runs what profile %u asks for", profile);
        }
        settled.host_insists = false;
        k_work_cancel_delayable(&undo_work);
        goto out_unref;
    }
    if (settled.host_insists) {
        LOG_DBG("Host link now %u.%02u ms, latency %u: the host's choice, left alone",
                MS2(link.interval), link.latency);
        goto out_unref;
    }
    if (settled.undone_at_ms != 0 &&
        k_uptime_get() - settled.undone_at_ms < CONFIG_ZMK_BLE_HOST_LINK_FOREIGN_CHANGE_GIVE_UP_MS) {
        settled.host_insists = true;
        k_work_cancel_delayable(&undo_work);
        LOG_WRN("The host changed the link again (%u.%02u ms, latency %u): left as it wants "
                "until the next connection",
                MS2(link.interval), link.latency);
        goto out_unref;
    }
    LOG_INF("Host link changed by someone else (%u.%02u ms, latency %u, timeout %u ms): "
            "asking for profile %u's parameters again in %u s",
            MS2(link.interval), link.latency, link.timeout * 10U, profile,
            CONFIG_ZMK_BLE_HOST_LINK_RESTORE_AFTER_FOREIGN_CHANGE_MS / 1000);
    k_work_reschedule(&undo_work, K_MSEC(CONFIG_ZMK_BLE_HOST_LINK_RESTORE_AFTER_FOREIGN_CHANGE_MS));

out_unref:
    bt_conn_unref(conn);
out:
    k_mutex_unlock(&hl_lock);
}

static void undo_work_cb(struct k_work *work) {
    ARG_UNUSED(work);

    k_mutex_lock(&hl_lock, K_FOREVER);
    struct bt_conn *conn = loaded && op.kind == OP_NONE ? active_conn() : NULL;

    if (conn == NULL) {
        goto out;
    }
    if (is_settled(conn) && !settled.host_insists && !settled.zmk_studio_open) {
        const uint32_t quiet = input_quiet_left_ms();

        if (quiet > 0) {
            k_work_reschedule(&undo_work, K_MSEC(quiet));
        } else {
            const uint8_t profile = zmk_ble_active_profile_index();
            struct hl_params p;

            intended_request(conn, profile, &p);
            settled.undone_at_ms = k_uptime_get();
            op_apply(conn, profile, &p, false, !has_pin(rec_get(profile)), APPLY_ONCE,
                     "the link back on the profile's parameters");
        }
    }
    bt_conn_unref(conn);
out:
    k_mutex_unlock(&hl_lock);
}

static void zmk_studio_check_work_cb(struct k_work *work) {
    ARG_UNUSED(work);

    k_mutex_lock(&hl_lock, K_FOREVER);
    struct bt_conn *conn = loaded ? active_conn() : NULL;

    if (conn == NULL) {
        goto out;
    }
    if (is_settled(conn) && settled.zmk_studio_open) {
        if (zmk_studio_subscribed(conn)) {
            k_work_reschedule(&zmk_studio_check_work, K_MSEC(ZMK_STUDIO_CHECK_MS));
        } else if (op.kind != OP_NONE) {
            /* Looked at again once the operation is over. */
            k_work_reschedule(&zmk_studio_check_work,
                              K_MSEC(CONFIG_ZMK_BLE_HOST_LINK_RESTORE_AFTER_FOREIGN_CHANGE_MS));
        } else {
            /* Closed, and the host refused Studio's closing request -- or
               it went unnoticed. */
            note_zmk_studio_subscription(conn);
            settled.zmk_studio_open = false;
            LOG_INF("ZMK Studio is no longer subscribed: asking for profile %u's parameters again",
                    zmk_ble_active_profile_index());
            start_for_link(conn, zmk_ble_active_profile_index(), false);
        }
    }
    bt_conn_unref(conn);
out:
    k_mutex_unlock(&hl_lock);
}

/* --------------------------------------- the hosts of other profiles */

/*
 * A host that connects while another profile is active is asked for its own
 * parameters once -- the stored result, or the secure request -- so that it
 * does not run whatever it opened with. One that was the active one before
 * keeps what it has. With KEEP_ONLY_ACTIVE_HOST_CONNECTED, once the active
 * profile's host is connected, every other one is disconnected.
 *
 * By bt_conn_index(): asked on this connection (or it was the active one,
 * or given up on after INACTIVE_SEND_RETRIES failures).
 */
static atomic_t inactive_host_asked[CONFIG_BT_MAX_CONN];
static atomic_t inactive_host_send_failures[CONFIG_BT_MAX_CONN];
#define INACTIVE_SEND_RETRIES 3

struct inactive_hosts {
    int active_profile;
    struct bt_conn *active;
    struct bt_conn *conns[CONFIG_BT_MAX_CONN];
    uint8_t profiles[CONFIG_BT_MAX_CONN];
    size_t count;
};

static void collect_inactive_host(struct bt_conn *conn, void *data) {
    struct inactive_hosts *hosts = data;
    struct bt_conn_info info;

    if (conn == hosts->active || !is_host_link(conn, &info) ||
        info.state != BT_CONN_STATE_CONNECTED || hosts->count >= ARRAY_SIZE(hosts->conns)) {
        return;
    }

    /* A host that is no profile's -- one pairing on an open slot -- may
       become the active one: left alone. */
    const int profile = zmk_ble_profile_index(info.le.dst);

    if (profile < 0 || profile >= PROFILE_COUNT || profile == hosts->active_profile) {
        return;
    }
    hosts->profiles[hosts->count] = (uint8_t)profile;
    hosts->conns[hosts->count++] = bt_conn_ref(conn);
}

static void others_work_cb(struct k_work *work);
static K_WORK_DELAYABLE_DEFINE(others_work, others_work_cb);

/* Ask an inactive host for its own parameters. Lock held. */
static void ask_inactive_host(struct bt_conn *conn, uint8_t profile) {
    const uint8_t i = bt_conn_index(conn);
    const uint32_t hold_ms = request_hold_left_ms(conn);
    const uint32_t wait_ms = MAX(hold_ms > 0 ? hold_ms + 100 : 0, answer_due_left_ms(conn));
    struct hl_params p;
    struct hl_link link;

    if (wait_ms > 0) {
        /* Sent once Zephyr's hold on a new link is over, so that a failure
           shows here. */
        k_work_reschedule(&others_work, K_MSEC(wait_ms));
        return;
    }
    if (!read_link(conn, &link)) {
        return;
    }
    compose_request(rec_get(profile), &p);
    if (hl_link_matches(&p, &link)) {
        atomic_set(&inactive_host_asked[i], 1);
        return;
    }

    const struct bt_le_conn_param param =
        BT_LE_CONN_PARAM_INIT(p.interval, p.interval, p.latency, p.timeout);
    const int err = bt_conn_le_param_update(conn, &param);

    if (err == 0 || err == -EALREADY) {
        const int64_t deadline = answer_deadline(&link);

        atomic_set(&inactive_host_asked[i], 1);
        if (err == 0) {
            answer_due_at(conn, deadline, &p);
        }
        /* A drop in this request's window is not counted. */
        atomic_ptr_cas(&probation_conn[i], conn, NULL);
        inactive_request[i].connection = (uint32_t)atomic_get(&conn_up_ms[i]);
        inactive_request[i].until_ms = deadline + MAX(CONFIG_ZMK_BLE_HOST_LINK_SUPERVISION_TIMEOUT_MS,
                                                      (uint32_t)link.timeout * 10U) +
                                       DROP_MARGIN_AFTER_TIMEOUT_MS;
        LOG_INF("Profile %u's host connected while profile %u is active: asked for its own "
                "parameters",
                profile, zmk_ble_active_profile_index());
        log_params("  ", &p);
        log_rssi(conn);
    } else if (atomic_inc(&inactive_host_send_failures[i]) < INACTIVE_SEND_RETRIES) {
        LOG_WRN("Profile %u's host: request not sent (%d), trying again in %u ms", profile, err,
                SEND_RETRY_SLOW_MS);
        k_work_reschedule(&others_work, K_MSEC(SEND_RETRY_SLOW_MS));
    } else {
        atomic_set(&inactive_host_asked[i], 1);
        LOG_WRN("Profile %u's host: request not sent (%d), given up on this link", profile, err);
    }
}

#if IS_ENABLED(CONFIG_ZMK_BLE_HOST_LINK_KEEP_ONLY_ACTIVE_HOST_CONNECTED)
/*
 * An inactive host is sent away only once its link is calm: encrypted, no
 * request of this module waiting for its answer, and up for at least
 * SEND_AWAY_AFTER_LINK_UP_MS. A disconnect in the middle of the encryption
 * leaves the host with a failed encryption on its bond; one behind a pending
 * parameter change took a whole supervision timeout to get through, and the
 * hosts sent away so did not come back. A link that is not calm
 * SEND_AWAY_WAIT_LIMIT_MS after the first try is sent away anyway.
 */
#define SEND_AWAY_AFTER_LINK_UP_MS 3000
#define SEND_AWAY_WAIT_LIMIT_MS 15000
/* A disconnect normally goes through within a few connection events. */
#define SEND_AWAY_SLOW_MS 1000

/* By bt_conn_index(): the connection (its conn_up_ms) the first try to send
   the host away was on, and when; and when the disconnect was asked for (0:
   not asked). */
static uint32_t send_away_connection[CONFIG_BT_MAX_CONN];
static int64_t send_away_first_try_ms[CONFIG_BT_MAX_CONN];
static atomic_t sent_away_at_ms[CONFIG_BT_MAX_CONN];

/* How long until the link is calm; 0: calm now. *why says what it waits for.
   Lock held. */
static uint32_t calm_in_ms(struct bt_conn *conn, const char **why) {
    struct bt_conn_info info;
    const uint32_t up = link_up_ms(conn);
    uint32_t wait = 0;

    *why = NULL;
    if (up < SEND_AWAY_AFTER_LINK_UP_MS) {
        wait = SEND_AWAY_AFTER_LINK_UP_MS - up;
        *why = "just connected";
    }
    if (answer_due_left_ms(conn) > wait) {
        wait = answer_due_left_ms(conn);
        *why = "a parameter request waits for its answer";
    }
    if (bt_conn_get_info(conn, &info) == 0 && info.security.level < BT_SECURITY_L2) {
        /* Looked at again soon: the encryption takes a few events. */
        wait = MAX(wait, 500U);
        *why = "not encrypted yet";
    }
    return wait;
}

/* Sends an inactive host away, or says when to try again (0: sent, or no
   need). Lock held. */
static uint32_t send_away_inactive_host(struct bt_conn *conn, uint8_t profile) {
    const uint8_t i = bt_conn_index(conn);
    const uint32_t connection = (uint32_t)atomic_get(&conn_up_ms[i]);
    const int64_t now = k_uptime_get();
    const char *why;

    if (atomic_get(&sent_away_at_ms[i]) != 0 && send_away_connection[i] == connection) {
        return 0; /* asked already; the disconnect is on its way */
    }
    if (send_away_connection[i] != connection) {
        send_away_connection[i] = connection;
        send_away_first_try_ms[i] = now;
        atomic_set(&sent_away_at_ms[i], 0);
    }

    const uint32_t wait = calm_in_ms(conn, &why);

    if (wait > 0 && now - send_away_first_try_ms[i] < SEND_AWAY_WAIT_LIMIT_MS) {
        LOG_DBG("Profile %u's host: sent away once its link is calm (%s)", profile, why);
        return MIN(wait, (uint32_t)(send_away_first_try_ms[i] + SEND_AWAY_WAIT_LIMIT_MS - now));
    }

    const int err = bt_conn_disconnect(conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);

    if (err != 0 && err != -ENOTCONN) {
        LOG_WRN("Profile %u's host: disconnect not sent (%d), trying again in %u ms", profile,
                err, SEND_RETRY_QUICK_MS);
        return SEND_RETRY_QUICK_MS;
    }
    atomic_set(&sent_away_at_ms[i], (atomic_val_t)MAX(k_uptime_get_32(), 1U));
    if (wait > 0) {
        LOG_INF("Profile %u's host disconnected: only the active profile's host stays "
                "connected -- not calm after %u s (%s), sent away anyway (%d)",
                profile, SEND_AWAY_WAIT_LIMIT_MS / 1000, why, err);
    } else {
        LOG_INF("Profile %u's host disconnected: only the active profile's host stays "
                "connected (%d)",
                profile, err);
    }
    return 0;
}

/* From the disconnected callback: how long the host took to go. */
static void note_sent_away_host_gone(struct bt_conn *conn, int profile) {
    const uint8_t i = bt_conn_index(conn);
    const uint32_t asked_at = (uint32_t)atomic_set(&sent_away_at_ms[i], 0);

    if (asked_at == 0) {
        return;
    }

    const uint32_t took = k_uptime_get_32() - asked_at;

    if (took > SEND_AWAY_SLOW_MS) {
        LOG_WRN("Profile %d's host took %u ms to go after it was sent away", profile, took);
    }
}
#endif

static void others_work_cb(struct k_work *work) {
    ARG_UNUSED(work);

    k_mutex_lock(&hl_lock, K_FOREVER);
    if (!loaded) {
        goto out;
    }

    struct inactive_hosts hosts = {
        .active_profile = zmk_ble_active_profile_index(),
        .active = active_conn(),
    };

    if (hosts.active != NULL) {
        /* The operation takes care of it; once inactive again, it keeps
           what it has. */
        atomic_set(&inactive_host_asked[bt_conn_index(hosts.active)], 1);
    }
    bt_conn_foreach(BT_CONN_TYPE_LE, collect_inactive_host, &hosts);

    uint32_t look_again_ms = 0;

    for (size_t i = 0; i < hosts.count; i++) {
        struct bt_conn *conn = hosts.conns[i];

#if IS_ENABLED(CONFIG_ZMK_BLE_HOST_LINK_KEEP_ONLY_ACTIVE_HOST_CONNECTED)
        if (hosts.active != NULL) {
            /* ZMK stops advertising now, so it does not come back. Until
               then -- a profile just selected whose host is away, say --
               the others stay: sent away now, they would only connect again
               at once. */
            const uint32_t again = send_away_inactive_host(conn, hosts.profiles[i]);

            if (again > 0 && (look_again_ms == 0 || again < look_again_ms)) {
                look_again_ms = again;
            }
            bt_conn_unref(conn);
            continue;
        }
        /* Staying for now: the wait for a calm link starts afresh once the
           active profile's host is back. */
        send_away_connection[bt_conn_index(conn)] = 0;
#endif
        if (!atomic_get(&inactive_host_asked[bt_conn_index(conn)])) {
            ask_inactive_host(conn, hosts.profiles[i]);
        }
        bt_conn_unref(conn);
    }
    if (look_again_ms > 0) {
        k_work_reschedule(&others_work, K_MSEC(look_again_ms));
    }
    if (hosts.active) {
        bt_conn_unref(hosts.active);
    }
out:
    k_mutex_unlock(&hl_lock);
}

static void others_changed(void) { k_work_reschedule(&others_work, K_MSEC(100)); }

/* ------------------------------------------- answers to the last request */

/* A host link moved, by bt_conn_index(): set by the update callback. */
static atomic_t link_moved_flag[CONFIG_BT_MAX_CONN];

/* The link moved onto the last request asked on it: that request is
   answered, and whatever waits for it goes now. Lock held. */
static void check_answer(struct bt_conn *conn, void *data) {
    ARG_UNUSED(data);
    struct bt_conn_info info;
    struct hl_link link;

    if (!is_host_link(conn, &info)) {
        return;
    }

    const uint8_t i = bt_conn_index(conn);

    if (!atomic_cas(&link_moved_flag[i], 1, 0) || answer_due_left_ms(conn) == 0 ||
        !read_link(conn, &link) || !hl_link_matches(&answer_due[i].asked, &link)) {
        return;
    }
    answer_came(conn);
    if (op.kind != OP_NONE && op.conn == conn && op.send_pending && op.send_waits_for_answer) {
        k_work_reschedule(&op_work, K_NO_WAIT);
    } else if (!atomic_get(&inactive_host_asked[i])) {
        others_changed();
    }
}

static void answers_work_cb(struct k_work *work) {
    ARG_UNUSED(work);

    k_mutex_lock(&hl_lock, K_FOREVER);
    if (loaded) {
        bt_conn_foreach(BT_CONN_TYPE_LE, check_answer, NULL);
    }
    k_mutex_unlock(&hl_lock);
}

static K_WORK_DEFINE(answers_work, answers_work_cb);

/* ------------------------------------------- Bluetooth callbacks */

static void on_connected(struct bt_conn *conn, uint8_t err) {
    struct bt_conn_info info;

    if (err != 0 || !is_host_link(conn, &info)) {
        return;
    }

    const uint8_t i = bt_conn_index(conn);

    atomic_set(&conn_up_ms[i], (atomic_val_t)k_uptime_get_32());

    /* What the host chose on its own: the starting point of everything. */
    LOG_INF("Host opened the link at %u.%02u ms, latency %u, timeout %u ms",
            MS2(info.le.interval), info.le.latency, info.le.timeout * 10U);
    atomic_set(&zmk_studio_subscribed_at_connect[i], 0);
    atomic_set(&inactive_host_asked[i], 0);
    atomic_set(&inactive_host_send_failures[i], 0);

    /* Its host is back: a drop it left behind is judged. */
    link_events[i].profile_up = zmk_ble_profile_index(info.le.dst);
    atomic_set(&link_events[i].up, 1);
    k_work_submit(&link_events_work);

    k_work_reschedule(&link_work, K_MSEC(50));
    /* By then a subscription to ZMK Studio restored with the bond is back. */
    k_work_reschedule(&zmk_studio_connect_work, K_MSEC(ZMK_STUDIO_RESTORED_AT_CONNECT_MS * 2 / 3));
    others_changed();
}

static void on_disconnected(struct bt_conn *conn, uint8_t reason) {
    struct bt_conn_info info;

    if (!is_host_link(conn, &info)) {
        return;
    }

    const uint8_t i = bt_conn_index(conn);

    atomic_ptr_cas(&settled_conn, conn, NULL);

    /* An operation ending puts its last request on probation a moment
       before it lets go of the link -- a drop then may be for both; the
       operation's blame goes first. A new operation on the link ends the
       probation (op_begin()). */
    link_events[i].conn = conn;
    link_events[i].connection = (uint32_t)atomic_get(&conn_up_ms[i]);
    link_events[i].profile = zmk_ble_profile_index(info.le.dst);
    link_events[i].reason = reason ? reason : 0xFF;
    link_events[i].link_up_ms = link_up_ms(conn);
    link_events[i].op_was_on_it = atomic_ptr_get(&op_conn) == conn;
    link_events[i].probation_was_on_it = atomic_ptr_get(&probation_conn[i]) == conn;
    atomic_set(&link_events[i].down, 1);
    k_work_submit(&link_events_work);
#if IS_ENABLED(CONFIG_ZMK_BLE_HOST_LINK_KEEP_ONLY_ACTIVE_HOST_CONNECTED)
    note_sent_away_host_gone(conn, link_events[i].profile);
#endif

    k_work_reschedule(&ball_work, K_MSEC(20));
}

static void on_le_param_updated(struct bt_conn *conn, uint16_t interval, uint16_t latency,
                                uint16_t timeout) {
    struct bt_conn_info info;

    if (!is_host_link(conn, &info)) {
        return;
    }
    /* DBG: the split link ladder logs every host link update at INF too. */
    LOG_DBG("Host link now %u.%02u ms, latency %u, timeout %u ms", MS2(interval), latency,
            timeout * 10U);

    /* Also called on a refusal over the Link Layer, with the old values. */
    atomic_set(&link_moved_flag[bt_conn_index(conn)], 1);
    k_work_submit(&answers_work);
    if (atomic_ptr_get(&op_conn) == conn) {
        atomic_set(&answer_arrived, 1);
        k_work_reschedule(&op_work, K_NO_WAIT);
    } else if (atomic_ptr_get(&settled_conn) == conn) {
        k_work_reschedule(&drift_work, K_MSEC(50));
    }
    k_work_reschedule(&ball_work, K_MSEC(20));
}

BT_CONN_CB_DEFINE(zmk_ble_host_link_conn_callbacks) = {
    .connected = on_connected,
    .disconnected = on_disconnected,
    .le_param_updated = on_le_param_updated,
};

/* ----------------------------------------------------------- ZMK events */

static int ble_host_link_listener(const zmk_event_t *eh) {
    if (as_zmk_position_state_changed(eh) != NULL) {
        atomic_set(&last_input_ms, (atomic_val_t)k_uptime_get_32());
        return ZMK_EV_EVENT_BUBBLE;
    }
    if (as_zmk_ble_active_profile_changed(eh) != NULL) {
        /* Selected, its host connected or gone, a slot paired or cleared. */
        k_work_reschedule(&link_work, K_MSEC(20));
        others_changed();
        return ZMK_EV_EVENT_BUBBLE;
    }
    if (as_zmk_endpoint_changed(eh) != NULL) {
        k_work_reschedule(&ball_work, K_MSEC(20));
    }
    return ZMK_EV_EVENT_BUBBLE;
}

ZMK_LISTENER(zmk_ble_host_link, ble_host_link_listener);
ZMK_SUBSCRIPTION(zmk_ble_host_link, zmk_ble_active_profile_changed);
ZMK_SUBSCRIPTION(zmk_ble_host_link, zmk_endpoint_changed);
ZMK_SUBSCRIPTION(zmk_ble_host_link, zmk_position_state_changed);

/* -------------------------------------------------------------- settings */

static int hl_settings_set(const char *name, size_t len, settings_read_cb read_cb, void *cb_arg) {
    char *end;
    const unsigned long p = strtoul(name, &end, 10);
    struct hl_record r;

    if (end == name || *end != '\0') {
        return -ENOENT;
    }
    if (p >= PROFILE_COUNT) {
        LOG_WRN("Dropping the stored values of profile %lu, there are %d", p, PROFILE_COUNT);
        if (p < 32) {
            k_mutex_lock(&hl_lock, K_FOREVER);
            stale_beyond_profiles |= BIT(p);
            k_mutex_unlock(&hl_lock);
        }
        return 0;
    }
    if (len != sizeof(r)) {
        LOG_WRN("Dropping the stored values of profile %lu: an older format (%u bytes, now %u)",
                p, (unsigned int)len, (unsigned int)sizeof(r));
        k_mutex_lock(&hl_lock, K_FOREVER);
        stale |= BIT(p);
        k_mutex_unlock(&hl_lock);
        return 0;
    }

    const ssize_t rc = read_cb(cb_arg, &r, sizeof(r));

    if (rc < 0) {
        return (int)rc;
    }
    if (r.format != RECORD_FORMAT) {
        LOG_WRN("Dropping the stored values of profile %lu: format %u, now %u", p, r.format,
                RECORD_FORMAT);
        k_mutex_lock(&hl_lock, K_FOREVER);
        stale |= BIT(p);
        k_mutex_unlock(&hl_lock);
        return 0;
    }

    k_mutex_lock(&hl_lock, K_FOREVER);
    recs[p] = r;
    stored |= BIT(p);
    stale &= ~BIT(p);
    k_mutex_unlock(&hl_lock);
    return 0;
}

static int hl_settings_commit(void) {
    k_mutex_lock(&hl_lock, K_FOREVER);
    loaded = true;
    /* Records of no use are deleted with the next write. */
    for (uint8_t p = 0; p < PROFILE_COUNT; p++) {
        if ((stale & BIT(p)) && rec_is_default(&recs[p])) {
            stored |= BIT(p);
            mark_dirty(p, false);
        }
    }
    stale = 0;
    if (stale_beyond_profiles) {
        k_work_reschedule(&save_work, K_MSEC(CONFIG_ZMK_SETTINGS_SAVE_DEBOUNCE));
    }
    k_mutex_unlock(&hl_lock);

    /* All handlers have run: the BLE profiles are known too. */
    k_work_reschedule(&link_work, K_NO_WAIT);
    others_changed();
#if IS_ENABLED(CONFIG_ZMK_BLE_HOST_LINK_LOG_RSSI)
    k_work_reschedule(&rssi_work, K_MSEC(RSSI_SAMPLE_PERIOD_MS));
#endif
    return 0;
}

SETTINGS_STATIC_HANDLER_DEFINE(zmk_ble_host_link, "ble_host_link", NULL, hl_settings_set,
                               hl_settings_commit, NULL);

static void save_work_cb(struct k_work *work) {
    ARG_UNUSED(work);

    struct hl_record snapshot[PROFILE_COUNT];

    k_mutex_lock(&hl_lock, K_FOREVER);
    const uint32_t todo = dirty;
    const uint32_t beyond = stale_beyond_profiles;
    uint32_t have = stored;

    dirty = 0;
    stale_beyond_profiles = 0;
    memcpy(snapshot, recs, sizeof(snapshot));
    k_mutex_unlock(&hl_lock);

    for (uint8_t p = PROFILE_COUNT; p < 32; p++) {
        if (beyond & BIT(p)) {
            char key[32];

            snprintf(key, sizeof(key), "ble_host_link/%u", p);
            settings_delete(key);
        }
    }

    for (uint8_t p = 0; p < PROFILE_COUNT; p++) {
        if (!(todo & BIT(p))) {
            continue;
        }

        char key[32];
        int err;

        snprintf(key, sizeof(key), "ble_host_link/%u", p);
        if (rec_is_default(&snapshot[p])) {
            if (!(have & BIT(p))) {
                continue;
            }
            err = settings_delete(key);
            if (err == 0) {
                have &= ~BIT(p);
            }
        } else {
            snapshot[p].format = RECORD_FORMAT;
            /* NVS skips a write identical to what it holds. */
            err = settings_save_one(key, &snapshot[p], sizeof(snapshot[p]));
            if (err == 0) {
                have |= BIT(p);
            }
        }
        if (err) {
            LOG_ERR("Saving %s failed: %d", key, err);
        }
    }

    k_mutex_lock(&hl_lock, K_FOREVER);
    stored = (stored & ~todo) | (have & todo);
    k_mutex_unlock(&hl_lock);
}

/* ------------------------------------------------------------------ API */

static int lock_loaded(void) {
    k_mutex_lock(&hl_lock, K_FOREVER);
    if (!loaded) {
        k_mutex_unlock(&hl_lock);
        return -EAGAIN;
    }
    return 0;
}

static void schedule_key_change(void) {
    k_work_reschedule(&key_change_work, K_MSEC(CONFIG_ZMK_BLE_HOST_LINK_APPLY_AFTER_LAST_KEY_MS));
}

/* What is asked of the active profile's host now. Lock held. */
static void asked_now(uint8_t p, struct bt_conn *conn, struct hl_params *asked) {
    if (op.kind == OP_SEARCH && op.profile == p) {
        *asked = op.search.have_result ? op.search.result : op.asked;
    } else if (op.kind == OP_APPLY && op.profile == p) {
        *asked = op.asked;
    } else if (conn != NULL) {
        intended_request(conn, p, asked);
    } else {
        compose_request(rec_get(p), asked);
    }
}

int zmk_ble_host_link_get_status(struct zmk_ble_host_link_status *s) {
    if (lock_loaded()) {
        return -EAGAIN;
    }

    const uint8_t p = zmk_ble_active_profile_index();
    struct hl_record *r = rec_get(p);
    struct hl_params asked;
    struct hl_link link = {0};
    struct bt_conn *conn = active_conn();

    *s = (struct zmk_ble_host_link_status){0};
    s->interval_pinned = interval_pinned(r);
    s->latency_pinned = latency_pinned(r);
    s->ball_report_pinned = r->pinned_ball_report_plus1 != 0;
    s->pinned_interval = r->pinned_interval;
    s->pinned_latency = latency_pinned(r) ? r->pinned_latency_plus1 - 1 : 0;
    s->pinned_ball_report_ms = r->pinned_ball_report_plus1 ? r->pinned_ball_report_plus1 - 1 : 0;

    asked_now(p, conn, &asked);
    s->asked_interval = asked.interval;
    s->asked_latency = asked.latency;
    s->asked_timeout = asked.timeout;

    s->connected = read_link(conn, &link);
    s->link_interval = link.interval;
    s->link_latency = link.latency;
    s->link_timeout = link.timeout;
    s->searching = op.kind == OP_SEARCH && op.profile == p;
#if HAVE_BALL_REPORT
    s->ball_report_ms = pmw3610_alt_get_ble_report_interval();
#endif
    if (conn) {
        bt_conn_unref(conn);
    }
    k_mutex_unlock(&hl_lock);
    return 0;
}

int zmk_ble_host_link_set_interval(uint16_t interval) {
    if (interval != BLE_HOST_INTERVAL_AUTO &&
        (interval < HL_SPEC_FASTEST_INTERVAL ||
         interval > CONFIG_ZMK_BLE_HOST_LINK_SLOWEST_SETTABLE_INTERVAL)) {
        LOG_ERR("Host interval %u is outside %u..%u (1.25 ms units)", interval,
                HL_SPEC_FASTEST_INTERVAL, CONFIG_ZMK_BLE_HOST_LINK_SLOWEST_SETTABLE_INTERVAL);
        indicate(VERDICT_REFUSED);
        return -EINVAL;
    }
    if (lock_loaded()) {
        return -EAGAIN;
    }

    const uint8_t p = zmk_ble_active_profile_index();
    struct hl_record *r = rec_get(p);

    r->pinned_interval = interval;
    rec_changed(p, false);
    settled.has_wanted = false;
    if (interval != BLE_HOST_INTERVAL_AUTO) {
        LOG_INF("Profile %u: host interval fixed at %u.%02u ms", p, MS2(interval));
    } else if (has_stored_result(r)) {
        LOG_INF("Profile %u: host interval chosen automatically (%u.%02u ms)", p,
                MS2(r->learned.stored.interval));
    } else {
        LOG_INF("Profile %u: host interval chosen automatically (by a search)", p);
    }
    schedule_key_change();
    k_mutex_unlock(&hl_lock);
    return 0;
}

int zmk_ble_host_link_set_latency(uint16_t latency) {
    if (latency != BLE_HOST_LATENCY_AUTO && latency > HL_SPEC_MAX_LATENCY) {
        LOG_ERR("Host latency %u is above %u", latency, HL_SPEC_MAX_LATENCY);
        indicate(VERDICT_REFUSED);
        return -EINVAL;
    }
    if (lock_loaded()) {
        return -EAGAIN;
    }

    const uint8_t p = zmk_ble_active_profile_index();
    struct hl_record *r = rec_get(p);

    r->pinned_latency_plus1 = latency == BLE_HOST_LATENCY_AUTO ? 0 : latency + 1;
    rec_changed(p, false);
    settled.has_wanted = false;
    if (latency != BLE_HOST_LATENCY_AUTO) {
        LOG_INF("Profile %u: host latency fixed at %u", p, latency);
    } else if (interval_pinned(r) || has_stored_result(r)) {
        struct hl_params asked;

        compose_request(r, &asked);
        LOG_INF("Profile %u: host latency chosen automatically (%u)", p, asked.latency);
    } else {
        LOG_INF("Profile %u: host latency chosen automatically (by a search)", p);
    }
    schedule_key_change();
    k_mutex_unlock(&hl_lock);
    return 0;
}

int zmk_ble_host_link_set_ball_report(uint8_t ms) {
    if (ms != BALL_BLE_REPORT_AUTO &&
        ms > CONFIG_ZMK_BLE_HOST_LINK_SLOWEST_SETTABLE_BALL_REPORT_MS) {
        LOG_ERR("Ball report interval %u ms is above %u", ms,
                CONFIG_ZMK_BLE_HOST_LINK_SLOWEST_SETTABLE_BALL_REPORT_MS);
        indicate(VERDICT_REFUSED);
        return -EINVAL;
    }
    if (!HAVE_BALL_REPORT) {
        LOG_WRN("No pointer driver takes a report interval");
        return -ENOTSUP;
    }
    if (lock_loaded()) {
        return -EAGAIN;
    }

    const uint8_t p = zmk_ble_active_profile_index();
    struct hl_record *r = rec_get(p);

    r->pinned_ball_report_plus1 = ms == BALL_BLE_REPORT_AUTO ? 0 : ms + 1;
    rec_changed(p, false);
    apply_ball_report();
    if (ms != BALL_BLE_REPORT_AUTO) {
        LOG_INF("Profile %u: ball report interval fixed at %u ms", p, ms);
    } else {
#if HAVE_BALL_REPORT
        LOG_INF("Profile %u: ball report interval chosen automatically (%u ms)", p,
                pmw3610_alt_get_ble_report_interval());
#endif
    }
    /* Nothing to ask of the host: in effect at once. */
    indicate(VERDICT_ACCEPTED);
    k_mutex_unlock(&hl_lock);
    return 0;
}

int zmk_ble_host_link_auto(void) {
    if (lock_loaded()) {
        return -EAGAIN;
    }

    const uint8_t p = zmk_ble_active_profile_index();
    struct hl_record *r = rec_get(p);
    const bt_addr_le_t peer = r->peer;

    memset(r, 0, sizeof(*r));
    r->peer = peer;
    memset(&sessions[p], 0, sizeof(sessions[p]));
    bt_addr_le_copy(&sessions[p].peer, zmk_ble_profile_address(p));
    rec_changed(p, false);
    settled.has_wanted = false;
    apply_ball_report();
    LOG_INF("Profile %u: everything back to automatic, searching again", p);

    struct bt_conn *conn = active_conn();

    if (conn == NULL) {
        indicate(VERDICT_NOT_CONNECTED_OR_SEARCHING);
    } else {
        op_search(conn, p, true, SEARCH_BY_USER);
        bt_conn_unref(conn);
    }
    k_mutex_unlock(&hl_lock);
    return 0;
}

int zmk_ble_host_link_request_again(void) {
    if (lock_loaded()) {
        return -EAGAIN;
    }

    const uint8_t p = zmk_ble_active_profile_index();
    struct bt_conn *conn = active_conn();

    if (conn == NULL) {
        indicate(VERDICT_NOT_CONNECTED_OR_SEARCHING);
    } else if (op.conn == conn && (op.kind == OP_SEARCH || op.escalation_pending)) {
        /* Nothing to ask again while the search asks: it would only cut the
           search short. */
        LOG_INF("Profile %u: asking again ignored -- a search is running", p);
        indicate(VERDICT_NOT_CONNECTED_OR_SEARCHING);
    } else {
        const struct hl_record *r = rec_get(p);
        struct hl_params req;

        compose_request(r, &req);
        op_apply(conn, p, &req, true, !has_pin(r), APPLY_ONCE, "asked again by a key");
        bt_conn_unref(conn);
    }
    k_mutex_unlock(&hl_lock);
    return 0;
}

void zmk_ble_host_link_show(void) {
    struct zmk_ble_host_link_status s;
    enum verdict v;

    if (zmk_ble_host_link_get_status(&s)) {
        return;
    }

    LOG_INF("Profile %u: interval %s, latency %s, ball report %s", zmk_ble_active_profile_index(),
            s.interval_pinned ? "fixed" : "auto", s.latency_pinned ? "fixed" : "auto",
            s.ball_report_pinned ? "fixed" : "auto");
    LOG_INF("Asked: %u.%02u ms, latency %u, timeout %u ms", MS2(s.asked_interval),
            s.asked_latency, s.asked_timeout * 10U);
    if (s.connected) {
        LOG_INF("Link:  %u.%02u ms, latency %u, timeout %u ms", MS2(s.link_interval),
                s.link_latency, s.link_timeout * 10U);

        k_mutex_lock(&hl_lock, K_FOREVER);
        struct bt_conn *conn = active_conn();

        if (conn != NULL) {
            log_rssi(conn);
            bt_conn_unref(conn);
        }
        k_mutex_unlock(&hl_lock);
    } else {
        LOG_INF("Link:  not connected");
    }
    LOG_INF("Ball report over BLE: %u ms%s", s.ball_report_ms, s.searching ? "; searching" : "");

    if (!s.connected || s.searching) {
        v = VERDICT_NOT_CONNECTED_OR_SEARCHING;
    } else {
        const struct hl_params asked = {s.asked_interval, s.asked_latency, s.asked_timeout};
        const struct hl_link link = {s.link_interval, s.link_latency, s.link_timeout};

        v = hl_link_matches(&asked, &link) ? VERDICT_ACCEPTED : VERDICT_REFUSED;
    }
    indicate(v);
    host_connections_log_bluetooth_state("on request");
}

/* ------------------------------------------------------- typed report */

/*
 * Only keys that type the same character in the Latin and the Cyrillic
 * layouts of Windows and macOS: digits, underscore, parentheses. So the
 * report reads the same whichever layout the computer is on.
 *
 *   granted(asked)_granted(asked)_granted(asked)_ball
 *   interval        latency        timeout        report
 *
 * Interval in 1.25 ms units, timeout in 10 ms units, ball report in ms.
 * Granted is 0 while the profile is not connected.
 */
static char typed[64];
static size_t typed_len;
static size_t typed_pos;
static bool typed_key_down;
static uint32_t typed_hold_ms;

static uint32_t keycode_for(char c) {
    switch (c) {
    case '0':
        return N0;
    case '1':
        return N1;
    case '2':
        return N2;
    case '3':
        return N3;
    case '4':
        return N4;
    case '5':
        return N5;
    case '6':
        return N6;
    case '7':
        return N7;
    case '8':
        return N8;
    case '9':
        return N9;
    case '_':
        return UNDERSCORE;
    case '(':
        return LPAR;
    case ')':
        return RPAR;
    default:
        return 0;
    }
}

static void type_work_cb(struct k_work *work);
static K_WORK_DELAYABLE_DEFINE(type_work, type_work_cb);

static void type_work_cb(struct k_work *work) {
    ARG_UNUSED(work);

    if (typed_pos >= typed_len) {
        typed_len = 0;
        return;
    }

    const uint32_t keycode = keycode_for(typed[typed_pos]);

    raise_zmk_keycode_state_changed_from_encoded(keycode, !typed_key_down, k_uptime_get());
    if (typed_key_down) {
        typed_pos++;
    }
    typed_key_down = !typed_key_down;
    k_work_reschedule(&type_work, K_MSEC(typed_hold_ms));
}

int zmk_ble_host_link_type_report(void) {
    struct zmk_ble_host_link_status s;

    if (typed_len) {
        return -EBUSY;
    }
    if (zmk_ble_host_link_get_status(&s)) {
        return -EAGAIN;
    }

    const int n = snprintf(typed, sizeof(typed), "%u(%u)_%u(%u)_%u(%u)_%u", s.link_interval,
                           s.asked_interval, s.link_latency, s.asked_latency, s.link_timeout,
                           s.asked_timeout, s.ball_report_ms);

    /* Over Bluetooth one report per connection event at least: on a slow
       link faster typing would overrun ZMK's report queue and lose keys. */
    typed_hold_ms = CONFIG_ZMK_BLE_HOST_LINK_TYPED_REPORT_KEY_HOLD_MS;
    if (zmk_endpoints_selected().transport == ZMK_TRANSPORT_BLE && s.connected) {
        typed_hold_ms = MAX(typed_hold_ms, DIV_ROUND_UP((uint32_t)s.link_interval * 125U, 100U));
    }

    typed_len = n > 0 ? MIN((size_t)n, sizeof(typed) - 1) : 0;
    typed_pos = 0;
    typed_key_down = false;
    LOG_INF("Typing: %s", typed);
    k_work_reschedule(&type_work, K_NO_WAIT);
    return 0;
}
