/*
 * SPDX-License-Identifier: MIT
 *
 * zmk-ble-host-link: the parameters of the Bluetooth link to the computer.
 *
 * For every host (BLE profile) this finds the fastest interval the host
 * accepts and the latency that goes with it (search.c), keeps the result in
 * settings, and asks for it on every later connection. Any parameter can be
 * fixed by hand per profile instead. The pointer report interval follows the
 * interval the host gave, unless fixed by hand too.
 *
 * Connection parameters can only be requested; the host decides. Every
 * request is checked against what the link then runs (hl_link_matches), and
 * anything else counts as a refusal.
 *
 * Around that:
 *   - a stored result the host refuses is asked for once more later, and the
 *     host searched again only after refusals on two connections in a row;
 *   - a change of the link made by someone else -- ZMK Studio, the host -- is
 *     undone, unless it is ZMK Studio's while open, or the host insists;
 *   - a link dropped after a search request, or soon after a stored result
 *     took effect, counts against that request only if the host comes
 *     straight back (it disliked it) rather than late (it slept);
 *   - optionally, only the active profile's host stays connected.
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

BUILD_ASSERT(CONFIG_ZMK_BLE_HOST_LINK_SUPERVISION_TIMEOUT_MS % 10 == 0,
             "ZMK_BLE_HOST_LINK_SUPERVISION_TIMEOUT_MS must be a multiple of 10");
BUILD_ASSERT(SLOWEST_INTERVAL < 4 * SUPERVISION_TIMEOUT,
             "the supervision timeout is too short for ZMK_BLE_HOST_LINK_SLOWEST_INTERVAL");
BUILD_ASSERT(CONFIG_ZMK_BLE_HOST_LINK_SLOWEST_SETTABLE_INTERVAL < 4 * SUPERVISION_TIMEOUT,
             "the supervision timeout is too short for "
             "ZMK_BLE_HOST_LINK_SLOWEST_SETTABLE_INTERVAL");

#if IS_ENABLED(CONFIG_ZMK_BLE_HOST_LINK_KEEP_ONLY_ACTIVE_HOST_CONNECTED)
#define BACKGROUND_TIMEOUT (CONFIG_ZMK_BLE_HOST_LINK_BACKGROUND_SUPERVISION_TIMEOUT_MS / 10)
BUILD_ASSERT(CONFIG_ZMK_BLE_HOST_LINK_BACKGROUND_SUPERVISION_TIMEOUT_MS % 10 == 0,
             "ZMK_BLE_HOST_LINK_BACKGROUND_SUPERVISION_TIMEOUT_MS must be a multiple of 10");
/* Core spec: timeout * 4 > (1 + latency) * interval_max (10 ms / 1.25 ms units). */
BUILD_ASSERT(4 * BACKGROUND_TIMEOUT > (1 + CONFIG_ZMK_BLE_HOST_LINK_BACKGROUND_LATENCY) *
                                          CONFIG_ZMK_BLE_HOST_LINK_BACKGROUND_INTERVAL_MAX,
             "the background supervision timeout is too short for its interval and latency");
#endif

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

/* ------------------------------------------------------------- records */

/* 4: what a host taught, as struct hl_learned (decisions.h). 5: with the
   fallback's search memory. Records of an
   older format are dropped on load and the host searched again. */
#define RECORD_FORMAT 5

#define REC_SEARCHED HL_LEARNED_SEARCHED
#define REC_FOUND HL_LEARNED_FOUND
#define REC_FASTEST_REFUSED HL_LEARNED_FASTEST_REFUSED
#define REC_NO_SEARCHING HL_LEARNED_NO_SEARCHING
#define REC_HAS_FALLBACK HL_LEARNED_HAS_FALLBACK

struct hl_record {
    uint8_t format;
    /* Pointer report interval + 1 (ms); 0: derived from the host interval. */
    uint8_t pinned_ball_report_plus1;
    /* 0: chosen automatically. */
    uint16_t pinned_interval_min;
    uint16_t pinned_interval_max;
    /* Latency + 1; 0: chosen automatically. */
    uint16_t pinned_latency_plus1;
    /* The host the values belong to; BT_ADDR_LE_ANY when set on an open slot. */
    bt_addr_le_t peer;
    /* What the host taught: search results, refusals, drops. */
    struct hl_learned learned;
};

static struct hl_record recs[PROFILE_COUNT];
static uint32_t dirty;  /* profiles to write */
static uint32_t stored; /* profiles that have a record in settings */
/* Profiles whose stored record was of no use (an old format): deleted. */
static uint32_t stale;
/* Records of profiles this build does not have (ZMK_BLE_PROFILE_COUNT went
   down): deleted too. */
static uint32_t stale_beyond_profiles;
static bool loaded;
static K_MUTEX_DEFINE(hl_lock);

/* What is known about each profile's host since boot, kept out of flash. */
struct hl_session {
    /* conn_up_ms of the connection last counted in refused_connections: one
       count per connection, however often the stored result is asked for on
       it. */
    uint32_t refusal_counted_on_connect_ms;
    bool refusal_counted_once;

    /* Counted drops in a row. */
    uint8_t drop_strikes;
    /* A drop after a request, waiting to see whether the host comes straight
       back, and the request it followed. */
    bool drop_suspected;
    int64_t drop_at_ms;
    struct hl_dropped_request drop_request;

    /* The slot's host this session is about: another one starts it over. */
    bt_addr_le_t peer;
};

static struct hl_session sessions[PROFILE_COUNT];

static void save_work_cb(struct k_work *work);
static K_WORK_DELAYABLE_DEFINE(save_work, save_work_cb);

static bool rec_is_default(const struct hl_record *r) {
    const struct hl_learned *l = &r->learned;

    /* What goes with `found` or `fallback` alone means nothing without them. */
    const uint8_t flags = l->flags & ~(HL_LEARNED_WITH_FOUND | HL_LEARNED_WITH_FALLBACK);

    return flags == 0 && r->pinned_ball_report_plus1 == 0 && r->pinned_interval_min == 0 &&
           r->pinned_latency_plus1 == 0 && l->latency_ceiling_plus1 == 0 &&
           l->fallback_candidates_tried == 0 && l->refused_connections == 0;
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

    /* Drops counted against one host say nothing about the next -- a record
       with nothing in it included. */
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

static bool interval_pinned(const struct hl_record *r) { return r->pinned_interval_min != 0; }
static bool latency_pinned(const struct hl_record *r) { return r->pinned_latency_plus1 != 0; }

/* Below a latency the link dropped at, if one was learned. */
static uint16_t under_latency_ceiling(const struct hl_record *r, uint16_t latency) {
    if (r->learned.latency_ceiling_plus1 == 0) {
        return latency;
    }
    const uint16_t ceiling = r->learned.latency_ceiling_plus1 - 1;

    return MIN(latency, ceiling > 0 ? ceiling - 1 : 0);
}

/* The request every search starts with, and the fallback for a refused
   stored result: exactly the slowest acceptable interval, at the latency a
   search would ask it with. */
static void secure_request(const struct hl_record *r, struct hl_params *p) {
    p->interval_min = SLOWEST_INTERVAL;
    p->interval_max = SLOWEST_INTERVAL;
    p->timeout = SUPERVISION_TIMEOUT;
    if (latency_pinned(r)) {
        p->latency = hl_latency_allowed(r->pinned_latency_plus1 - 1, SLOWEST_INTERVAL,
                                        SUPERVISION_TIMEOUT, false);
    } else {
        p->latency = under_latency_ceiling(
            r, hl_latency_allowed(LATENCY_WHILE_SEARCHING, SLOWEST_INTERVAL, SUPERVISION_TIMEOUT,
                                  true));
    }
}

/* Searched, and the host accepted nothing: nothing is asked of it, its own
   parameters stay (start_for_link). */
static bool asks_nothing(const struct hl_record *r) {
    return hl_learned_asks_nothing(&r->learned) && !interval_pinned(r);
}

/* What to ask the host for when not searching. Lock held. */
static void compose_request(const struct hl_record *r, struct hl_params *p) {
    const struct hl_learned *l = &r->learned;
    const bool apple_rules = l->flags & REC_FASTEST_REFUSED;

    if (interval_pinned(r)) {
        p->interval_min = r->pinned_interval_min;
        p->interval_max = r->pinned_interval_max;
        p->timeout = SUPERVISION_TIMEOUT;
    } else if (l->flags & REC_FOUND) {
        *p = l->found;
        /* The timeout as configured now, not as stored. */
        p->timeout = SUPERVISION_TIMEOUT;
    } else {
        secure_request(r, p);
    }

    if (latency_pinned(r)) {
        /* By hand: only the spec's limit, so that Apple's can be tested. */
        p->latency =
            hl_latency_allowed(r->pinned_latency_plus1 - 1, p->interval_max, p->timeout, false);
    } else if (!interval_pinned(r) && (l->flags & REC_FOUND)) {
        /* Found at the timeout configured then: within what the timeout
           now allows (the spec's limit, and Apple's for a host playing by
           them). */
        p->latency = under_latency_ceiling(
            r, hl_latency_allowed(l->found.latency, p->interval_max, p->timeout, apple_rules));
    } else if (!interval_pinned(r)) {
        /* secure_request() has it. */
    } else {
        p->latency = under_latency_ceiling(
            r, hl_latency_for_idle_listen_period(p->interval_max,
                                                 CONFIG_ZMK_BLE_HOST_LINK_IDLE_LISTEN_PERIOD_MS,
                                                 p->timeout, apple_rules));
    }
}

/* The fallback as it is asked for: with a latency fixed by hand, or under a
   ceiling learned since. */
static void fallback_request(const struct hl_record *r, struct hl_params *p) {
    *p = r->learned.fallback;
    p->timeout = SUPERVISION_TIMEOUT;
    /* Found at the timeout configured then: within what the timeout now
       allows, by the rules the host played by in that search. */
    p->latency = latency_pinned(r)
                     ? hl_latency_allowed(r->pinned_latency_plus1 - 1, p->interval_max, p->timeout,
                                          false)
                     : under_latency_ceiling(
                           r, hl_latency_allowed(p->latency, p->interval_max, p->timeout,
                                                 r->learned.fallback_fastest_refused));
}

/* ------------------------------------------------------------------ LED */

enum verdict {
    VERDICT_NONE,
    VERDICT_ACCEPTED,
    VERDICT_REFUSED,
    VERDICT_NOT_CONNECTED_OR_SEARCHING,
};

#if IS_ENABLED(CONFIG_ZMK_BLE_HOST_LINK_LED_FEEDBACK)

static void blink(uint32_t rgb, uint8_t count, uint16_t on_ms) {
    rgbled_widget_blink(rgb, count, on_ms, CONFIG_ZMK_BLE_HOST_LINK_LED_BLINK_OFF_MS);
}

static void indicate(enum verdict v) {
    const uint16_t on = CONFIG_ZMK_BLE_HOST_LINK_LED_BLINK_ON_MS;

    switch (v) {
    case VERDICT_ACCEPTED:
        blink(CONFIG_ZMK_BLE_HOST_LINK_LED_RGB_HOST_ACCEPTED, 1, on * 2);
        break;
    case VERDICT_REFUSED:
        blink(CONFIG_ZMK_BLE_HOST_LINK_LED_RGB_HOST_REFUSED, 2, on);
        break;
    case VERDICT_NOT_CONNECTED_OR_SEARCHING:
        blink(CONFIG_ZMK_BLE_HOST_LINK_LED_RGB_NOT_CONNECTED_OR_SEARCHING, 1, on * 2);
        break;
    default:
        break;
    }
}

void zmk_ble_host_link_count_out(enum zmk_ble_host_link_count what, uint8_t count) {
    const uint32_t rgb = what == ZMK_BLE_HOST_LINK_COUNT_INTERVAL
                             ? CONFIG_ZMK_BLE_HOST_LINK_LED_RGB_COUNT_INTERVAL
                         : what == ZMK_BLE_HOST_LINK_COUNT_LATENCY
                             ? CONFIG_ZMK_BLE_HOST_LINK_LED_RGB_COUNT_LATENCY
                             : CONFIG_ZMK_BLE_HOST_LINK_LED_RGB_COUNT_BALL_REPORT;

    if (count == 0) {
        blink(rgb, 1, CONFIG_ZMK_BLE_HOST_LINK_LED_BLINK_ON_MS * 4);
    } else {
        blink(rgb, count, CONFIG_ZMK_BLE_HOST_LINK_LED_BLINK_ON_MS);
    }
}

#else

static inline void indicate(enum verdict v) { ARG_UNUSED(v); }

void zmk_ble_host_link_count_out(enum zmk_ble_host_link_count what, uint8_t count) {
    ARG_UNUSED(what);
    ARG_UNUSED(count);
}

#endif /* IS_ENABLED(CONFIG_ZMK_BLE_HOST_LINK_LED_FEEDBACK) */

/* -------------------------------------------------------------- links */

/* conn_up_ms: k_uptime_get_32() at the connect of each host link, by
   bt_conn_index(). Written in the RX context, read in work handlers. */
static atomic_t conn_up_ms[CONFIG_BT_MAX_CONN];

/* The parameters each host link runs by the host's own choice -- as opened,
   or as the host changed them before this module asked for anything on the
   link and before ZMK Studio showed up on it -- by bt_conn_index(). Written
   in the RX thread and in work handlers, hence the spinlock. */
static struct hl_link hosts_own[CONFIG_BT_MAX_CONN];
static struct k_spinlock hosts_own_lock;
/* This module (or the background request) asked for something on the link. */
static atomic_t module_asked_on_link[CONFIG_BT_MAX_CONN];
/* The link ran ZMK Studio's parameters at some point: what follows -- the
   host's answer to Studio's closing request -- is no choice of the host's. */
static atomic_t zmk_studio_was_on_link[CONFIG_BT_MAX_CONN];

static void note_hosts_own(struct bt_conn *conn, uint16_t interval, uint16_t latency,
                           uint16_t timeout) {
    const k_spinlock_key_t key = k_spin_lock(&hosts_own_lock);

    hosts_own[bt_conn_index(conn)] =
        (struct hl_link){.interval = interval, .latency = latency, .timeout = timeout};
    k_spin_unlock(&hosts_own_lock, key);
}

static struct hl_link hosts_own_link(struct bt_conn *conn) {
    const k_spinlock_key_t key = k_spin_lock(&hosts_own_lock);
    const struct hl_link own = hosts_own[bt_conn_index(conn)];

    k_spin_unlock(&hosts_own_lock, key);
    return own;
}

/* Whether the host's own choice can still be taken from the link. */
static bool hosts_own_still_shows(struct bt_conn *conn) {
    const uint8_t i = bt_conn_index(conn);

    return !atomic_get(&module_asked_on_link[i]) && !atomic_get(&zmk_studio_was_on_link[i]);
}

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

/* What is left of Zephyr's hold on the first request of a connection. */
static uint32_t request_hold_left_ms(struct bt_conn *conn) {
    const uint32_t up = k_uptime_get_32() - (uint32_t)atomic_get(&conn_up_ms[bt_conn_index(conn)]);

    return up < CONFIG_BT_CONN_PARAM_UPDATE_TIMEOUT ? CONFIG_BT_CONN_PARAM_UPDATE_TIMEOUT - up : 0;
}

static void log_params(const char *what, const struct hl_params *p) {
    if (p->interval_min == p->interval_max) {
        LOG_INF("%s %u.%02u ms, latency %u, timeout %u ms", what, MS2(p->interval_min),
                p->latency, p->timeout * 10U);
    } else {
        LOG_INF("%s %u.%02u-%u.%02u ms, latency %u, timeout %u ms", what, MS2(p->interval_min),
                MS2(p->interval_max), p->latency, p->timeout * 10U);
    }
}

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
   never look like them -- not even with the timeout moved by one to get past
   Zephyr (hl_timeout_that_reaches_the_host()). */
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

/* How long the link has been up (ms). */
static uint32_t link_up_ms(struct bt_conn *conn) {
    return k_uptime_get_32() - (uint32_t)atomic_get(&conn_up_ms[bt_conn_index(conn)]);
}

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

/* ------------------------------------------------------- ball report */

uint16_t zmk_ble_host_link_ball_report_for_interval(uint16_t interval) {
    /* half the interval in us, rounded up to the 4 ms frame grid */
    const uint32_t half_us = (uint32_t)interval * 625U;

    return (uint16_t)MAX(4U, DIV_ROUND_UP(half_us, 4000U) * 4U);
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

/* k_uptime_get_32() of the last key or pointer event: searches and restores
   wait for a pause in use. */
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
    const uint32_t need = CONFIG_ZMK_BLE_HOST_LINK_INPUT_QUIET_BEFORE_PROBE_MS;

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
    /* The request in effect on the link -- the stored result, the fallback,
       the secure request, the host's own: what an undo asks for. */
    bool has_wanted;
    struct hl_params wanted;
    /* ZMK Studio has the link: left alone until it closes. */
    bool zmk_studio_open;
    /* The host changed the link back after the module undid its change. */
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

/* ------------------------------------------------------- the operation */

/*
 * One operation at a time, on the active profile's link: a single request
 * (APPLY) or a search (SEARCH). A newer one replaces it.
 */
enum op_kind {
    OP_NONE,
    OP_APPLY,
    OP_SEARCH,
};

enum apply_purpose {
    /* One request, its answer final: a value fixed by hand, the secure
       request, the fallback, the host's own back, an undo, REQUEST_AGAIN. */
    APPLY_ONCE,
    /* A stored search result: asked once more after a refusal, then -- as
       hl_stored_result_refused() decides -- the fallback, the secure
       request or a search. */
    APPLY_STORED_RESULT,
};

enum search_reason {
    SEARCH_FIRST,
    SEARCH_BY_USER,
    SEARCH_AGAIN_STORED_REFUSED,
};

static struct {
    enum op_kind kind;
    struct bt_conn *conn; /* referenced */
    uint8_t profile;
    /* Started by a key: the first verdict shows on the LED. */
    bool user_initiated;
    /* Started by a key -- or followed on its own from what a key started
       (the stored result refused): stays set when user_initiated is spent
       on the first verdict. */
    bool started_by_user;
    /* The request in op.asked went on air (bt_conn_le_param_update()
       took it): the host may answer it yet. */
    bool asked_went_on_air;
    /* What a single request is (OP_APPLY), for the log. */
    const char *asked_for;

    enum apply_purpose purpose;
    /* APPLY_STORED_RESULT: 0 the first request, 1 the one more. */
    uint8_t stored_attempt;
    /* k_uptime_get() to send again at; INT64_MAX none. */
    int64_t send_again_at_ms;

    struct hl_params asked;
    bool awaiting;
    /* The answer is the link as it is: nothing was sent (-EALREADY). */
    bool answer_without_request;
    bool resend;
    /* Attempts to get the current request sent at all. */
    uint8_t send_retries;
    /* ZMK Studio's request at connect replaced this one once already. */
    bool resent_over_zmk_studio;
    int64_t deadline_ms;
    /* When the last request went on air; INT64_MAX: nothing sent yet. */
    int64_t on_air_at_ms;
    /* When the host last answered, or the deadline passed; 0 not yet. */
    int64_t answered_at_ms;
    /* The link as the last request went out. */
    struct hl_link link_at_send;
    /* What the last answer showed. */
    bool have_seen;
    struct hl_link seen_link;

    /* The request the link last moved onto, and when: what a drop soon
       after is about (hl_drop_blame()). */
    bool have_taken;
    struct hl_dropped_request taken;
    int64_t taken_at_ms;
    uint32_t taken_window_ms;
    /* hl_search.taken_count when last looked at. */
    uint8_t search_taken_seen;

    /* The operation ends with this request in effect (settled.wanted). */
    bool ends_on_wanted;
    struct hl_params wanted;
    /* ...or with the link where the host keeps putting it: left there until
       the next connection (settled.host_insists). */
    bool ends_with_host_insisting;

    struct hl_search search;
    enum search_reason search_reason;
} op;

/* The link the operation runs on, for the RX callbacks. */
static atomic_ptr_t op_conn;
static atomic_t answer_arrived;
static atomic_t link_lost_reason; /* 0: none */
/* A search runs (zmk_ble_host_link_is_searching()). */
static atomic_t search_running;

static void op_work_cb(struct k_work *work);
static K_WORK_DELAYABLE_DEFINE(op_work, op_work_cb);

bool zmk_ble_host_link_is_searching(void) { return atomic_get(&search_running) != 0; }

static void op_end(void) {
    if (op.conn) {
        struct hl_link now;

        settle(op.conn, op.have_seen ? &op.seen_link : NULL);
        if (is_settled(op.conn) && op.ends_on_wanted) {
            settled.has_wanted = true;
            settled.wanted = op.wanted;
        }
        if (is_settled(op.conn) && op.ends_with_host_insisting) {
            settled.host_insists = true;
        } else if (is_settled(op.conn) && op.ends_on_wanted && op.started_by_user) {
            /* Asked for by a key and got: what the host insisted on before
               is history -- a later change is undone again. */
            settled.host_insists = false;
            settled.undone_at_ms = 0;
        }
        /* The link moved after the last answer -- the host, ZMK Studio --
           while the operation still had it: looked at now. */
        if (is_settled(op.conn) && op.have_seen && read_link(op.conn, &now) &&
            !same_link(&now, &op.seen_link)) {
            k_work_reschedule(&drift_work, K_MSEC(50));
        }
        bt_conn_unref(op.conn);
    }
    atomic_ptr_set(&op_conn, NULL);
    atomic_set(&search_running, 0);
    op.conn = NULL;
    op.kind = OP_NONE;
    op.awaiting = false;
    op.resend = false;
    op.send_again_at_ms = INT64_MAX;
    k_work_cancel_delayable(&op_work);
}

/* ------------------------------------------- drops after a request */

/* A counted drop, into the record. Lock held. */
static void count_drop(uint8_t p) {
    struct hl_session *ses = &sessions[p];
    struct hl_record *r = rec_get(p);
    const unsigned int changed = hl_count_drop(&r->learned, &ses->drop_strikes,
                                               &ses->drop_request);

    if (changed & HL_DROP_SKIPS_FASTEST) {
        LOG_WRN("Profile %u: the fastest interval is skipped on this host from now on", p);
    }
    if (changed & HL_DROP_SKIPS_CANDIDATE) {
        LOG_WRN("Profile %u: that candidate is skipped on this host from now on", p);
    }
    if (changed & HL_DROP_LOWERS_LATENCY) {
        LOG_WRN("Profile %u: latency stays below %u on this host from now on", p,
                ses->drop_request.params.latency);
    }
    if (changed & HL_DROP_CLEARS_STORED) {
        LOG_WRN("Profile %u: the stored result does not hold -- searched again", p);
    }
    if (changed & HL_DROP_CLEARS_FALLBACK) {
        LOG_WRN("Profile %u: the fallback does not hold -- dropped", p);
    }
    if (changed & HL_DROP_STOPS_SEARCHING) {
        LOG_WRN("Profile %u: dropped after requests %u times in a row -- no more searching on "
                "this host until BLE_HOST_LINK_AUTO",
                p, ses->drop_strikes);
    }
    rec_changed(p, true);
}

/* The link dropped soon after a request took effect. Lock held. */
static void request_dropped(uint8_t p, const struct hl_dropped_request *req, uint8_t reason) {
    struct hl_session *ses = &sessions[p];

    switch (hl_drop_reason(reason)) {
    case HL_DROP_NOT_THE_HOSTS:
        return;
    case HL_DROP_COUNTS_NOW:
        ses->drop_request = *req;
        LOG_WRN("Profile %u: the host dropped the link over the parameters asked for "
                "(reason 0x%02x)",
                p, reason);
        count_drop(p);
        return;
    case HL_DROP_COUNTS_IF_HOST_RETURNS:
        ses->drop_request = *req;
        ses->drop_suspected = true;
        ses->drop_at_ms = k_uptime_get();
        LOG_WRN("Profile %u: the link dropped soon after a request (reason 0x%02x) -- it counts "
                "if the host comes straight back",
                p, reason);
        return;
    }
}

/* The host of profile p is back: a drop it left behind counts if it is back
   soon -- it was awake and disliked the request -- not if it slept. Lock held. */
static void judge_drop_on_return(uint8_t p) {
    struct hl_session *ses = &sessions[p];

    if (!ses->drop_suspected) {
        return;
    }
    ses->drop_suspected = false;

    const int64_t away_ms = k_uptime_get() - ses->drop_at_ms;
    const uint32_t away_s = (uint32_t)MIN(away_ms / 1000, (int64_t)UINT32_MAX);

    if (hl_drop_counts_on_return(away_ms,
                                 CONFIG_ZMK_BLE_HOST_LINK_DROP_COUNTS_IF_HOST_RETURNS_WITHIN_MS)) {
        LOG_WRN("Profile %u: the host is back %u s after dropping the link after a request -- "
                "it counts against that request",
                p, away_s);
        count_drop(p);
    } else {
        LOG_INF("Profile %u: the host is back after %u s -- it slept or was switched off, "
                "the drop does not count",
                p, away_s);
    }
}

/*
 * After an operation ends, the request the link last moved onto stays on
 * probation for its drop window: parameters the link cannot keep show as a
 * drop only after the supervision timeout. A request that comes through it
 * clears the drops counted before (drop_strikes): only drops in a row stop
 * the searching.
 */
static struct {
    uint8_t profile;
    struct hl_dropped_request request;
    int64_t until_ms;
} probation;
static atomic_ptr_t probation_conn;
static atomic_t probation_lost_reason;

static void probation_over_work_cb(struct k_work *work) {
    ARG_UNUSED(work);

    k_mutex_lock(&hl_lock, K_FOREVER);
    if (atomic_ptr_get(&probation_conn) != NULL && k_uptime_get() >= probation.until_ms) {
        /* No drop: the request holds. */
        sessions[probation.profile].drop_strikes = 0;
        atomic_ptr_set(&probation_conn, NULL);
    }
    k_mutex_unlock(&hl_lock);
}

static K_WORK_DELAYABLE_DEFINE(probation_over_work, probation_over_work_cb);

static void probation_work_cb(struct k_work *work) {
    ARG_UNUSED(work);

    k_mutex_lock(&hl_lock, K_FOREVER);
    const uint8_t reason = (uint8_t)atomic_set(&probation_lost_reason, 0);

    if (reason != 0 && atomic_ptr_get(&probation_conn) != NULL &&
        k_uptime_get() <= probation.until_ms) {
        request_dropped(probation.profile, &probation.request, reason);
    }
    atomic_ptr_set(&probation_conn, NULL);
    k_mutex_unlock(&hl_lock);
}

static K_WORK_DEFINE(probation_work, probation_work_cb);

/* The request the link last moved onto stays on probation. Lock held. */
static void start_probation(void) {
    const int64_t until = op.taken_at_ms + op.taken_window_ms;

    if (!op.have_taken) {
        return;
    }
    if (k_uptime_get() >= until) {
        /* Through its window already, watched by the operation (a search
           waits each one out): the request holds. */
        sessions[op.profile].drop_strikes = 0;
        return;
    }
    probation.profile = op.profile;
    probation.request = op.taken;
    probation.until_ms = until;
    atomic_set(&probation_lost_reason, 0);
    atomic_ptr_set(&probation_conn, op.conn);
    k_work_reschedule(&probation_over_work, K_MSEC(until - k_uptime_get()));
}

/* The link moved onto this request now. Its drop window: the supervision
   timeout the link runs (or the one asked, if longer) and a margin. */
static void note_taken(const struct hl_dropped_request *req) {
    struct hl_link link;
    uint32_t timeout_ms = CONFIG_ZMK_BLE_HOST_LINK_SUPERVISION_TIMEOUT_MS;

    if (read_link(op.conn, &link)) {
        timeout_ms = MAX(timeout_ms, (uint32_t)link.timeout * 10U);
    }
    op.have_taken = true;
    op.taken = *req;
    op.taken_at_ms = k_uptime_get();
    op.taken_window_ms = timeout_ms + DROP_MARGIN_AFTER_TIMEOUT_MS;
}

/* The search moved the link onto a request (accepted, or answered late). */
static void note_search_taken(void) {
    if (op.search.taken_count == op.search_taken_seen) {
        return;
    }
    op.search_taken_seen = op.search.taken_count;

    const struct hl_dropped_request req = {
        .stored_result = false,
        .kind = op.search.taken_kind,
        .candidate = op.search.taken_candidate,
        .params = op.search.taken,
    };

    note_taken(&req);
}

/* The request the operation has out, as a drop would be judged by. */
static struct hl_dropped_request op_request_out(void) {
    if (op.kind == OP_SEARCH) {
        return (struct hl_dropped_request){
            .stored_result = false,
            .kind = op.search.asked_kind,
            .candidate = op.search.asked_candidate,
            .params = op.search.asked,
        };
    }
    return (struct hl_dropped_request){
        .stored_result = true,
        .kind = HL_ASK_RESTORE,
        .params = op.asked,
    };
}

/* The link the operation ran on went down. Lock held. */
static void op_link_lost(uint8_t reason) {
    const int64_t now = k_uptime_get();
    /* Only a judged request on air counts: a link lost while nothing was
       out (pairing gone wrong, say) says nothing about it. */
    const bool judged = (op.kind == OP_SEARCH && op.search.asked_kind != HL_ASK_RESTORE) ||
                        (op.kind == OP_APPLY && op.purpose == APPLY_STORED_RESULT);
    const bool request_out = judged && op.awaiting && op.on_air_at_ms != INT64_MAX;

    switch (hl_drop_blame(request_out, op.have_taken, now - op.taken_at_ms,
                          op.taken_window_ms)) {
    case HL_DROP_BLAMES_REQUEST_OUT: {
        const struct hl_dropped_request req = op_request_out();

        request_dropped(op.profile, &req, reason);
        break;
    }
    case HL_DROP_BLAMES_REQUEST_TAKEN:
        request_dropped(op.profile, &op.taken, reason);
        break;
    case HL_DROP_BLAMES_NOTHING:
        LOG_INF("Profile %u: link down, request abandoned", op.profile);
        break;
    }
    op_end();
}

/* A loss the disconnect callback reported but op_work has not judged yet. */
static void judge_pending_loss(void) {
    const uint8_t lost = (uint8_t)atomic_set(&link_lost_reason, 0);

    if (lost && op.kind != OP_NONE) {
        op_link_lost(lost);
    }
}

/* Set while an operation started by a key starts its follow-up on its own:
   the follow-up is still the key's (op.started_by_user). */
static bool next_op_follows_user_op;

static void op_begin(enum op_kind kind, struct bt_conn *conn, uint8_t profile, bool user) {
    op_end();
    op.kind = kind;
    op.conn = bt_conn_ref(conn);
    op.profile = profile;
    op.user_initiated = user;
    op.started_by_user = user || next_op_follows_user_op;
    next_op_follows_user_op = false;
    op.asked_went_on_air = false;
    op.purpose = APPLY_ONCE;
    op.stored_attempt = 0;
    op.send_again_at_ms = INT64_MAX;
    op.send_retries = 0;
    op.resent_over_zmk_studio = false;
    op.on_air_at_ms = INT64_MAX;
    op.answered_at_ms = 0;
    op.have_seen = false;
    op.have_taken = false;
    op.search_taken_seen = 0;
    op.ends_on_wanted = false;
    op.ends_with_host_insisting = false;
    atomic_set(&answer_arrived, 0);
    atomic_set(&link_lost_reason, 0);
    atomic_set(&search_running, kind == OP_SEARCH);
    atomic_ptr_set(&op_conn, conn);
    /* A new request on the link: the last one's probation is over. */
    atomic_ptr_cas(&probation_conn, conn, NULL);
}

/* What the request on air is, for the log. */
static const char *asked_for(void) {
    static char candidate[32];

    if (op.kind != OP_SEARCH) {
        return op.asked_for ? op.asked_for : "a request";
    }
    switch (op.search.asked_kind) {
    case HL_ASK_SECURE:
        return "search: the secure request";
    case HL_ASK_FASTEST:
        return "search: the fastest interval";
    case HL_ASK_FALLBACK_CANDIDATE:
        snprintf(candidate, sizeof(candidate), "search: fallback candidate %u",
                 op.search.asked_candidate);
        return candidate;
    case HL_ASK_LATENCY:
        return "search: a latency step";
    case HL_ASK_RESTORE:
        return "search: back on the best found";
    }
    return "search";
}

/* Send op.asked. Lock held; may end the operation. */
static void op_send(void) {
    /* In the first BT_CONN_PARAM_UPDATE_TIMEOUT of a connection Zephyr would
       hold the request and send it later from its own work, telling no one
       if that fails -- or drop it if the host's own update happens to meet
       it. So the request waits that out here, and goes out at once. */
    const uint32_t hold_ms = request_hold_left_ms(op.conn);

    op.answer_without_request = false;
    if (hold_ms > 0) {
        op.resend = true;
        op.awaiting = false;
        k_work_reschedule(&op_work, K_MSEC(hold_ms + 100));
        return;
    }

    struct hl_link link = {0};
    const bool have_link = read_link(op.conn, &link);
    const uint16_t timeout =
        have_link ? hl_timeout_that_reaches_the_host(&op.asked, &link) : op.asked.timeout;
    const struct bt_le_conn_param p = BT_LE_CONN_PARAM_INIT(
        op.asked.interval_min, op.asked.interval_max, op.asked.latency, timeout);

    atomic_set(&answer_arrived, 0);
    op.resend = false;
    op.link_at_send = link;

    const int err = bt_conn_le_param_update(op.conn, &p);
    const int64_t now = k_uptime_get();

    if (err == 0) {
        /* On a slow link the keyboard hears the answer only when it listens. */
        const uint32_t listen_ms = have_link ? idle_listen_period_ms(&link) : 0;
        struct hl_params sent = op.asked;

        atomic_set(&module_asked_on_link[bt_conn_index(op.conn)], 1);
        sent.timeout = timeout;
        op.awaiting = true;
        op.asked_went_on_air = true;
        op.on_air_at_ms = now;
        op.deadline_ms = now + CONFIG_ZMK_BLE_HOST_LINK_HOST_ANSWER_TIMEOUT_MS + 2 * listen_ms;
        LOG_INF("Asking for %s:", asked_for());
        log_params("  ", &sent);
        k_work_reschedule(&op_work, K_MSEC(op.deadline_ms - now));
        return;
    }

    if (err == -EALREADY) {
        /* The link runs it already: answered at once, nothing was sent. */
        LOG_DBG("Already in effect");
        op.awaiting = true;
        op.answer_without_request = true;
        op.deadline_ms = now;
        atomic_set(&answer_arrived, 1);
        k_work_reschedule(&op_work, K_NO_WAIT);
        return;
    }

    /* Not sent at all -- the controller busy with another procedure, on this
       link or the split link. Not the host's answer: try again, and in the
       end give up without holding the host to anything. */
    if (op.send_retries < SEND_RETRIES_TOTAL) {
        const uint32_t delay_ms =
            op.send_retries < SEND_RETRIES_QUICK ? SEND_RETRY_QUICK_MS : SEND_RETRY_SLOW_MS;

        op.send_retries++;
        op.resend = true;
        LOG_WRN("Request not sent (%d), trying again in %u ms", err, delay_ms);
        k_work_reschedule(&op_work, K_MSEC(delay_ms));
        return;
    }
    LOG_WRN("Request could not be sent (%d): given up, nothing learned from it", err);
    if (op.user_initiated) {
        indicate(VERDICT_REFUSED);
    }
    op_end();
}

static void op_apply(struct bt_conn *conn, uint8_t profile, const struct hl_params *p, bool user,
                     enum apply_purpose purpose, const char *asked_for) {
    judge_pending_loss();
    op_begin(OP_APPLY, conn, profile, user);
    op.purpose = purpose;
    op.asked_for = asked_for;
    op.asked = *p;
    op_send();
}

static void op_search(struct bt_conn *conn, uint8_t profile, bool user,
                      enum search_reason reason);

/* The host's own parameters, as a request. */
static struct hl_params hosts_own_request(struct bt_conn *conn) {
    const struct hl_link own = hosts_own_link(conn);

    return (struct hl_params){own.interval, own.interval, own.latency, own.timeout};
}

/* Decide what a link needs and start it. Lock held. */
static void start_for_link(struct bt_conn *conn, uint8_t profile, bool user) {
    struct hl_record *r = rec_get(profile);
    struct hl_params p;

    LOG_DBG("Profile %u: flags 0x%02x (searched %u, found %u, fastest refused %u, no searching "
            "%u, fallback %u, searched again %u), refused connections %u, latency ceiling %d",
            profile, r->learned.flags, !!(r->learned.flags & HL_LEARNED_SEARCHED),
            !!(r->learned.flags & HL_LEARNED_FOUND),
            !!(r->learned.flags & HL_LEARNED_FASTEST_REFUSED),
            !!(r->learned.flags & HL_LEARNED_NO_SEARCHING),
            !!(r->learned.flags & HL_LEARNED_HAS_FALLBACK),
            !!(r->learned.flags & HL_LEARNED_SEARCHED_AGAIN), r->learned.refused_connections,
            (int)r->learned.latency_ceiling_plus1 - 1);
    if (r->learned.flags & HL_LEARNED_FOUND) {
        LOG_DBG("Profile %u: stored result %u.%02u-%u.%02u ms, latency %u", profile,
                MS2(r->learned.found.interval_min), MS2(r->learned.found.interval_max),
                r->learned.found.latency);
    }
    if (r->learned.flags & HL_LEARNED_HAS_FALLBACK) {
        LOG_DBG("Profile %u: fallback %u.%02u-%u.%02u ms, latency %u", profile,
                MS2(r->learned.fallback.interval_min), MS2(r->learned.fallback.interval_max),
                r->learned.fallback.latency);
    }

    /* What was in effect belongs to the settings before: the request about
       to go out says anew what is wanted. */
    settled.has_wanted = false;

    compose_request(r, &p);

    if (interval_pinned(r)) {
        op_apply(conn, profile, &p, user, APPLY_ONCE, "the interval fixed by hand");
    } else if (r->learned.flags & REC_FOUND) {
        op_apply(conn, profile, &p, user, APPLY_STORED_RESULT, "the stored search result");
    } else if (asks_nothing(r)) {
        /* Searched, and the host accepted nothing -- not even the secure
           request, or not as asked (an Apple host scaling 15 ms to 30 ms):
           asking again would only make it worse. Its own parameters stay --
           and if the link left them for someone else's (the background
           parameters, ZMK Studio's), they are asked for back. */
        const struct hl_link own = hosts_own_link(conn);
        struct hl_link link;

        if (read_link(conn, &link) && own.interval != 0 && !same_link(&link, &own)) {
            const struct hl_params back = hosts_own_request(conn);

            LOG_INF("Profile %u: the host accepted nothing when searched -- asking for its own "
                    "link parameters back",
                    profile);
            op_apply(conn, profile, &back, user, APPLY_ONCE, "the host's own parameters back");
            return;
        }
        LOG_INF("Profile %u: the host accepted nothing when searched -- its own link parameters "
                "are left as they are",
                profile);
        if (user) {
            indicate(VERDICT_ACCEPTED);
        }
        judge_pending_loss();
        op_end();
        settle(conn, NULL);
    } else if (r->learned.flags & REC_NO_SEARCHING) {
        op_apply(conn, profile, &p, user, APPLY_ONCE,
                 "the secure request (no searching on this host)");
    } else {
        op_search(conn, profile, user, SEARCH_FIRST);
    }
}

static void search_finish(void) {
    struct hl_record *r = rec_get(op.profile);
    const struct hl_search *s = &op.search;
    const bool again = op.search_reason == SEARCH_AGAIN_STORED_REFUSED;
    const enum hl_search_outcome outcome =
        hl_search_finished(&r->learned, s, again, SUPERVISION_TIMEOUT);

    rec_changed(op.profile, true);

    switch (outcome) {
    case HL_SEARCH_OUTCOME_STORED:
        break;
    case HL_SEARCH_OUTCOME_REPLACED_STORED:
        LOG_INF("Profile %u: searched again, found no worse -- it is the stored result now",
                op.profile);
        break;
    case HL_SEARCH_OUTCOME_FALLBACK:
        LOG_INF("Profile %u: searched again, found only slower -- kept as the fallback, the "
                "stored result stays",
                op.profile);
        break;
    case HL_SEARCH_OUTCOME_NOTHING_CHANGED:
        LOG_INF("Profile %u: searched again, the host accepted nothing -- everything stays",
                op.profile);
        break;
    }

    if (s->host_keeps_moving) {
        LOG_WRN("Profile %u: the host keeps moving the link to its own parameters during the "
                "search -- left there, the search ends on what it found",
                op.profile);
    }
    /* Where the link is now decides, not how it got there: a late answer
       may have put it back on the best, the host may have moved it off. */
    struct hl_link now;
    const bool on_best =
        s->have_best && read_link(op.conn, &now) && hl_link_matches(&s->best, &now);

    if (s->have_best) {
        struct hl_params best = s->best;

        best.timeout = SUPERVISION_TIMEOUT;
        LOG_INF("Search on profile %u done after %u requests%s:", op.profile, s->requests,
                on_best ? "" : " (the link is not on it: the host keeps its own until the next "
                               "connection)");
        log_params("  host link", &best);
        if (on_best) {
            op.ends_on_wanted = true;
            op.wanted = s->best;
        } else {
            op.ends_with_host_insisting = true;
        }
    } else {
        LOG_WRN("Search on profile %u done after %u requests: the host accepted nothing",
                op.profile, s->requests);
    }

    /* The request the link last moved onto stays on probation. */
    start_probation();

    if (op.user_initiated) {
        indicate(on_best ? VERDICT_ACCEPTED : VERDICT_REFUSED);
    }
    apply_ball_report();
    op_end();
}

/* The next request of the search, once the pause and a lull in use allow. */
static void search_step(void) {
    const int64_t now = k_uptime_get();
    int64_t pause_end =
        op.on_air_at_ms == INT64_MAX
            ? now
            : op.on_air_at_ms + CONFIG_ZMK_BLE_HOST_LINK_PAUSE_BETWEEN_PROBES_MS;

    /* The request the link moved onto gets its whole drop window before the
       next goes out: a drop in it is then about that request only. */
    if (op.have_taken) {
        pause_end = MAX(pause_end, op.taken_at_ms + op.taken_window_ms);
    }

    /* Putting the link back after a bad pick does not wait: until then it
       runs worse than anything the search would keep. */
    const bool gated = op.search.requests > 0 && !op.search.restoring;

    if (gated && now < pause_end) {
        k_work_reschedule(&op_work, K_MSEC(pause_end - now));
        return;
    }
    if (gated) {
        const uint32_t quiet = input_quiet_left_ms();

        if (quiet > 0) {
            k_work_reschedule(&op_work, K_MSEC(quiet));
            return;
        }
    }

    struct hl_params req;

    if (!hl_search_next(&op.search, &req)) {
        search_finish();
        return;
    }
    op.asked = req;
    op.asked_went_on_air = false;
    op.send_retries = 0;
    op_send();
}

static void op_search(struct bt_conn *conn, uint8_t profile, bool user,
                      enum search_reason reason) {
    judge_pending_loss();

    struct hl_record *r = rec_get(profile);
    const struct hl_learned *l = &r->learned;
    const struct hl_search_limits limits = {
        .fastest_interval = FASTEST_INTERVAL,
        .slowest_interval = SLOWEST_INTERVAL,
        .latency_while_searching_interval = LATENCY_WHILE_SEARCHING,
        .idle_listen_period_ms = CONFIG_ZMK_BLE_HOST_LINK_IDLE_LISTEN_PERIOD_MS,
        .timeout = SUPERVISION_TIMEOUT,
        .max_requests = CONFIG_ZMK_BLE_HOST_LINK_MAX_PROBES_PER_CONNECTION,
        .pinned_latency =
            latency_pinned(r) ? r->pinned_latency_plus1 - 1 : HL_LATENCY_NOT_PINNED,
    };
    const bool found = l->flags & REC_FOUND;
    const struct hl_search_memory memory = {
        .fastest_refused = l->flags & REC_FASTEST_REFUSED,
        .fallback_candidates_tried = l->fallback_candidates_tried,
        .protected_fallback_candidate =
            found ? hl_fallback_candidate_index(&limits, l->found.interval_min,
                                                l->found.interval_max)
                  : HL_NO_FALLBACK_CANDIDATE,
        .fastest_protected = found && l->found.interval_min == FASTEST_INTERVAL &&
                             l->found.interval_max == FASTEST_INTERVAL,
        .latency_ceiling =
            l->latency_ceiling_plus1 ? l->latency_ceiling_plus1 - 1 : HL_NO_LATENCY_CEILING,
    };
    struct hl_link opened = {0};

    read_link(conn, &opened);
    op_begin(OP_SEARCH, conn, profile, user);
    op.search_reason = reason;
    hl_search_start(&op.search, &limits, &memory, &opened);
    op.search_taken_seen = op.search.taken_count;
    LOG_INF("Searching the host link parameters of profile %u", profile);
    if (user) {
        indicate(VERDICT_NOT_CONNECTED_OR_SEARCHING);
    }
    search_step();
}

/* ------------------------------------------------------- the answers */

/* APPLY_STORED_RESULT refused. Lock held. */
static void stored_result_refused(void) {
    struct hl_session *ses = &sessions[op.profile];
    struct hl_record *r = rec_get(op.profile);

    if (op.stored_attempt == 0) {
        op.stored_attempt = 1;
        op.send_again_at_ms = k_uptime_get() + CONFIG_ZMK_BLE_HOST_LINK_STORED_RESULT_RETRY_MS;
        LOG_INF("The host did not take the stored result: asking once more in %u s",
                CONFIG_ZMK_BLE_HOST_LINK_STORED_RESULT_RETRY_MS / 1000);
        k_work_reschedule(&op_work, K_MSEC(CONFIG_ZMK_BLE_HOST_LINK_STORED_RESULT_RETRY_MS));
        return;
    }

    /* The first refusal on this connection is the one that counts. */
    const uint32_t connected_at = (uint32_t)atomic_get(&conn_up_ms[bt_conn_index(op.conn)]);
    const bool first = !ses->refusal_counted_once ||
                       ses->refusal_counted_on_connect_ms != connected_at;
    const struct hl_learned before = r->learned;

    ses->refusal_counted_once = true;
    ses->refusal_counted_on_connect_ms = connected_at;

    const enum hl_stored_refusal_step step = hl_stored_result_refused(&r->learned, first);

    if (memcmp(&before, &r->learned, sizeof(before)) != 0) {
        /* A replaced or dropped stored result at once, a count with the
           settings debounce. */
        rec_changed(op.profile, step == HL_STORED_REFUSED_FALLBACK_REPLACES_IT ||
                                    step == HL_STORED_REFUSED_DROPPED);
    }

    struct bt_conn *conn = bt_conn_ref(op.conn);
    const uint8_t profile = op.profile;
    struct hl_params next;

    next_op_follows_user_op = op.started_by_user;
    switch (step) {
    case HL_STORED_REFUSED_FALLBACK_REPLACES_IT:
        LOG_INF("Profile %u: the stored result was refused on %u connections in a row -- the "
                "fallback replaces it",
                profile, HL_STORED_REFUSALS_THAT_REPLACE_IT);
        compose_request(r, &next);
        op_apply(conn, profile, &next, false, APPLY_ONCE, "the fallback, now the stored result");
        break;
    case HL_STORED_REFUSED_SEARCH_AGAIN:
        LOG_INF("Profile %u: the stored result was refused on %u connections in a row -- "
                "searching again",
                profile, r->learned.refused_connections);
        op_search(conn, profile, false, SEARCH_AGAIN_STORED_REFUSED);
        break;
    case HL_STORED_REFUSED_DROPPED:
        LOG_INF("Profile %u: the stored result keeps being refused, and searching found nothing "
                "better -- dropped, the host keeps its own parameters",
                profile);
        start_for_link(conn, profile, false);
        break;
    case HL_STORED_REFUSED_ASK_FALLBACK:
        fallback_request(r, &next);
        LOG_INF("The host refused the stored result twice: asking for the fallback");
        op_apply(conn, profile, &next, false, APPLY_ONCE, "the fallback");
        break;
    case HL_STORED_REFUSED_ASK_SECURE:
        secure_request(r, &next);
        LOG_INF("The host refused the stored result twice: asking for the secure request");
        op_apply(conn, profile, &next, false, APPLY_ONCE, "the secure request");
        break;
    }
    next_op_follows_user_op = false;
    bt_conn_unref(conn);
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

/*
 * The link moved under a search request that never went out -- waiting to
 * be sent again: built for a search that has got on since -- a late answer
 * taken, the search over, the link to be put back on the best first, or a
 * restore no longer needed -- it is dropped (hl_search_request_not_sent()).
 */
static bool unsent_search_request_outdated(uint8_t taken_before) {
    const bool restore = op.search.asked_kind == HL_ASK_RESTORE;

    return op.search.taken_count != taken_before || op.search.phase == HL_PHASE_DONE ||
           op.search.restoring != restore;
}

/*
 * The link moved under the search: ZMK Studio's request (its parameters, and
 * the host subscribed to it -- at connect, its subscription back with the
 * bond) is not the host's move; anything else is. The search caps the moves
 * it takes for not the host's.
 */
static void search_link_moved(const struct hl_link *link) {
    const uint8_t taken_before = op.search.taken_count;
    const bool by_zmk_studio = is_zmk_studio_params(link) && zmk_studio_subscribed(op.conn);

    if (by_zmk_studio) {
        hl_search_link_moved_not_by_host(&op.search, link);
    } else {
        hl_search_link_moved(&op.search, link);
    }

    if (op.search.taken_count != taken_before) {
        LOG_DBG("Search: the host's late answer -- link at %u.%02u ms, latency %u, taken as that "
                "request's answer",
                MS2(link->interval), link->latency);
    } else if (op.search.restoring) {
        LOG_DBG("Search: the link moved off the best (%u.%02u ms, latency %u, by %s, %u in a "
                "row): putting it back",
                MS2(link->interval), link->latency, by_zmk_studio ? "ZMK Studio" : "the host",
                op.search.host_moves_in_a_row);
    }
}

static void op_answered(void) {
    struct hl_link link;
    /* Nothing came by the deadline: an L2CAP refusal, which Zephyr only
       logs -- or an answer still on its way. */
    const bool host_silent = !op.answer_without_request && !atomic_get(&answer_arrived);

    if (!read_link(op.conn, &link)) {
        /* Going down: the disconnect callback judges the request -- still
           awaited -- through op_link_lost(). */
        k_work_reschedule(&op_work, K_SECONDS(2));
        return;
    }

    /* From here on, a change of the link is news (op_work_cb). */
    atomic_set(&answer_arrived, 0);
    op.awaiting = false;
    op.answered_at_ms = k_uptime_get();
    op.have_seen = true;
    op.seen_link = link;

    const bool accepted = hl_link_matches(&op.asked, &link);
    const bool moved = !same_link(&op.link_at_send, &link);

    if (accepted) {
        LOG_INF("Host accepted: link at %u.%02u ms, latency %u, timeout %u ms",
                MS2(link.interval), link.latency, link.timeout * 10U);
    } else {
        const bool interval_ok = link.interval == op.asked.interval_min;

        LOG_INF("Host did NOT accept%s: link at %u.%02u ms%s, latency %u%s, timeout %u ms",
                host_silent ? " (no answer)" : "", MS2(link.interval),
                interval_ok ? "" : " (asked otherwise)", link.latency,
                link.latency == op.asked.latency ? "" : " (asked otherwise)",
                link.timeout * 10U);
    }

    if (!accepted && is_zmk_studio_params(&link) && zmk_studio_subscribed(op.conn) &&
        link_up_ms(op.conn) < ZMK_STUDIO_RESTORED_AT_CONNECT_MS && !op.resent_over_zmk_studio) {
        /* ZMK Studio's subscription came back with the bond, and its
           request -- sent by Zephyr as the connection settled -- landed on
           top of this one. Once more; after that, it is the host's answer. */
        op.resent_over_zmk_studio = true;
        LOG_INF("ZMK Studio's request at connect landed on top of this one: asking again");
        if (op.kind == OP_SEARCH) {
            /* The link as it is now, before the request goes out again --
               ZMK Studio's doing, not the host's. The request was on air:
               it goes out again as it is, whatever the search learned
               meanwhile -- the host may still answer it. */
            search_link_moved(&link);
            note_search_taken();
            if (op.search.phase == HL_PHASE_DONE) {
                /* That move ended the search: nothing goes out again. */
                search_step();
                return;
            }
        }
        op_send();
        return;
    }
    if (!accepted && zmk_studio_on_link(op.conn, &link)) {
        /* Not the host's answer: ZMK Studio has taken the link over. Taken
           up afresh when it closes (drift_work, zmk_studio_check_work). */
        LOG_INF("ZMK Studio has the link: %s waits until it closes",
                op.kind == OP_SEARCH ? "the search starts over when it" : "the request");
        apply_ball_report();
        op_end();
        zmk_studio_took_link();
        return;
    }

    /* A single request, and the link moved somewhere else entirely -- an
       update still queued in the controller (the background parameters,
       just after switching to this host) completing first: not the answer.
       It may still come, until the deadline. */
    if (op.kind == OP_APPLY && !accepted && !host_silent && !op.answer_without_request &&
        moved && op.answered_at_ms < op.deadline_ms) {
        LOG_DBG("The link moved, but not to the request: waiting for the answer");
        op.awaiting = true;
        k_work_reschedule(&op_work, K_MSEC(op.deadline_ms - op.answered_at_ms));
        return;
    }

    /* The first verdict of something started by a key shows; what follows
       on its own (one more request, a search) does not. */
    if (op.user_initiated && op.kind == OP_APPLY) {
        indicate(accepted ? VERDICT_ACCEPTED : VERDICT_REFUSED);
        op.user_initiated = false;
    }

    if (op.kind == OP_SEARCH) {
        const bool forced_before = op.search.host_forces_latency_0;

        hl_search_answer(&op.search, &link, host_silent);
        if (op.search.asked.latency != op.asked.latency) {
            LOG_DBG("Search: the interval taken at the host's latency %u", op.search.asked.latency);
        }
        if (op.search.host_forces_latency_0 && !forced_before) {
            LOG_INF("The host gives latency 0 to what is asked with latency: the search ends on "
                    "the slowest interval it took -- at latency 0 every interval is a wake-up");
        }
        note_search_taken();
        apply_ball_report();
        search_step();
        return;
    }

    /* OP_APPLY */
    apply_ball_report();
    if (accepted) {
        op.ends_on_wanted = true;
        op.wanted = op.asked;
        if (moved) {
            const struct hl_dropped_request req = {
                .stored_result = op.purpose == APPLY_STORED_RESULT,
                .kind = HL_ASK_RESTORE,
                .params = op.asked,
            };

            if (op.purpose == APPLY_STORED_RESULT) {
                note_taken(&req);
            }
        }
    }
    if (op.purpose == APPLY_STORED_RESULT) {
        if (!accepted) {
            stored_result_refused();
            return;
        }
        struct hl_record *r = rec_get(op.profile);

        if (hl_stored_result_accepted(&r->learned)) {
            LOG_DBG("Profile %u: the stored result accepted -- the refusal count starts over",
                    op.profile);
            rec_changed(op.profile, false);
        }
        start_probation();
    }
    op_end();
}

static void op_work_cb(struct k_work *work) {
    ARG_UNUSED(work);

    k_mutex_lock(&hl_lock, K_FOREVER);

    const uint8_t lost = (uint8_t)atomic_set(&link_lost_reason, 0);

    if (op.kind == OP_NONE) {
        goto out;
    }
    if (lost) {
        op_link_lost(lost);
        goto out;
    }
    if (op.resend) {
        struct hl_link link;

        /* The link moved while the request waited to go out (the host's own
           update as the connection settles): the search sees it first, so
           that the answer is judged against the link as it is. */
        if (op.kind == OP_SEARCH && atomic_set(&answer_arrived, 0) && read_link(op.conn, &link)) {
            const uint8_t taken_before = op.search.taken_count;

            op.have_seen = true;
            op.seen_link = link;
            if (zmk_studio_on_link(op.conn, &link)) {
                LOG_INF("ZMK Studio has the link: the search starts over when it closes");
                apply_ball_report();
                op_end();
                zmk_studio_took_link();
                goto out;
            }
            if (op.asked_went_on_air && link.interval >= op.asked.interval_min &&
                link.interval <= op.asked.interval_max && link.latency <= op.asked.latency) {
                /* The host answered the request that was on air (ZMK
                   Studio's landed on top of it) while it waited to go out
                   again: its answer -- judged as any answer, a slow pick
                   from a range or a lowered latency included. */
                LOG_INF("Host answered while the request waited to go out again: link at "
                        "%u.%02u ms, latency %u",
                        MS2(link.interval), link.latency);
                op.resend = false;
                hl_search_answer(&op.search, &link, false);
                note_search_taken();
                apply_ball_report();
                search_step();
                goto out;
            }
            search_link_moved(&link);
            note_search_taken();
            /* A request that went on air before (sent again after ZMK
               Studio's landed on top) may still be answered: it goes out
               again as it is -- unless the search is over. */
            if (op.search.phase == HL_PHASE_DONE ||
                (!op.asked_went_on_air && unsent_search_request_outdated(taken_before))) {
                if (!op.asked_went_on_air) {
                    hl_search_request_not_sent(&op.search);
                }
                op.resend = false;
                search_step();
                goto out;
            }
        }
        op_send();
        goto out;
    }
    if (op.awaiting) {
        const int64_t now = k_uptime_get();

        if (!atomic_get(&answer_arrived) && now < op.deadline_ms) {
            k_work_reschedule(&op_work, K_MSEC(op.deadline_ms - now));
            goto out;
        }
        op_answered();
        goto out;
    }
    if (op.send_again_at_ms != INT64_MAX) {
        const int64_t now = k_uptime_get();

        if (now < op.send_again_at_ms) {
            k_work_reschedule(&op_work, K_MSEC(op.send_again_at_ms - now));
            goto out;
        }
        struct hl_link link;

        op.send_again_at_ms = INT64_MAX;
        if (read_link(op.conn, &link) && zmk_studio_on_link(op.conn, &link)) {
            /* ZMK Studio opened meanwhile: the stored result waits until it
               closes. */
            LOG_INF("ZMK Studio has the link: the stored result waits until it closes");
            op.have_seen = true;
            op.seen_link = link;
            op_end();
            zmk_studio_took_link();
            goto out;
        }
        op.send_retries = 0;
        op_send();
        goto out;
    }
    if (op.kind == OP_SEARCH) {
        struct hl_link link;

        /* The link moved between two requests: a late answer, ZMK Studio,
           or the host on its own. */
        if (atomic_set(&answer_arrived, 0) && read_link(op.conn, &link)) {
            op.have_seen = true;
            op.seen_link = link;
            if (zmk_studio_on_link(op.conn, &link)) {
                LOG_INF("ZMK Studio has the link: the search starts over when it closes");
                apply_ball_report();
                op_end();
                zmk_studio_took_link();
                goto out;
            }
            search_link_moved(&link);
            note_search_taken();
            apply_ball_report();
        }
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
            /* The link went down: judge it here if the disconnect callback's
               own wake-up has not run yet. */
            const uint8_t lost = (uint8_t)atomic_set(&link_lost_reason, 0);

            if (lost) {
                op_link_lost(lost);
            } else {
                op_end();
            }
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
            judge_drop_on_return((uint8_t)profile);
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

static void undo_work_cb(struct k_work *work);
static K_WORK_DELAYABLE_DEFINE(undo_work, undo_work_cb);

/* What the link should run, for the undo: the request in effect when the
   module left it, the host's own for a profile that asks nothing, or what
   the profile asks for. Lock held. */
static void wanted_on_link(struct bt_conn *conn, uint8_t profile, struct hl_params *p) {
    const struct hl_record *r = rec_get(profile);

    if (asks_nothing(r)) {
        *p = hosts_own_request(conn);
    } else if (settled.has_wanted) {
        *p = settled.wanted;
    } else {
        compose_request(r, p);
    }
}

/* The active link changed while the module had nothing on air: ZMK Studio,
   the host, or the host's answer to a request given up on. */
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
    const struct hl_record *r = rec_get(profile);
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
    if (asks_nothing(r) && hosts_own_still_shows(conn) && !is_zmk_studio_params(&link)) {
        /* The host's own choice on a link nothing was asked of: its own
           parameters now. */
        note_hosts_own(conn, link.interval, link.latency, link.timeout);
        goto out_unref;
    }
    wanted_on_link(conn, profile, &wanted);
    if (hl_link_matches(&wanted, &link)) {
        /* What the link should run: a late answer to the module's own
           request, say. */
        k_work_cancel_delayable(&undo_work);
        LOG_DBG("Host link now runs what profile %u asks for", profile);
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

            wanted_on_link(conn, profile, &p);
            settled.undone_at_ms = k_uptime_get();
            op_apply(conn, profile, &p, false, APPLY_ONCE, "undoing someone else's change");
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
        } else if (op.kind == OP_NONE) {
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

/* ------------------------------------- only the active host connected */

#if IS_ENABLED(CONFIG_ZMK_BLE_HOST_LINK_KEEP_ONLY_ACTIVE_HOST_CONNECTED)

/* The background parameters were asked of the host link with this
   bt_conn_index() since it connected or was last the active one (or given
   up on after BACKGROUND_SEND_RETRIES failures). */
static atomic_t background_asked[CONFIG_BT_MAX_CONN];
static atomic_t background_send_failures[CONFIG_BT_MAX_CONN];
#define BACKGROUND_SEND_RETRIES 3

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
        /* Once inactive again it gets the background parameters again. */
        atomic_set(&background_asked[bt_conn_index(hosts.active)], 0);
        atomic_set(&background_send_failures[bt_conn_index(hosts.active)], 0);
    }
    bt_conn_foreach(BT_CONN_TYPE_LE, collect_inactive_host, &hosts);

    /*
     * With the active profile's host connected, every other goes: ZMK stops
     * advertising then, so it does not come back. Until then -- a profile
     * just selected whose host is away, say -- they stay, on background
     * parameters: sent away now, they would only connect again at once.
     */
    for (size_t i = 0; i < hosts.count; i++) {
        struct bt_conn *conn = hosts.conns[i];

        if (hosts.active != NULL) {
            const int err = bt_conn_disconnect(conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);

            LOG_INF("Profile %u's host disconnected: only the active profile's host stays "
                    "connected (%d)",
                    hosts.profiles[i], err);
        } else if (!atomic_get(&background_asked[bt_conn_index(conn)])) {
            const uint32_t hold_ms = request_hold_left_ms(conn);

            if (hold_ms > 0) {
                /* Sent once Zephyr's hold on a new link is over, so that a
                   failure shows here. */
                k_work_reschedule(&others_work, K_MSEC(hold_ms + 100));
            } else {
                const struct bt_le_conn_param p = BT_LE_CONN_PARAM_INIT(
                    CONFIG_ZMK_BLE_HOST_LINK_BACKGROUND_INTERVAL_MIN,
                    CONFIG_ZMK_BLE_HOST_LINK_BACKGROUND_INTERVAL_MAX,
                    CONFIG_ZMK_BLE_HOST_LINK_BACKGROUND_LATENCY, BACKGROUND_TIMEOUT);
                const int err = bt_conn_le_param_update(conn, &p);

                if (err == 0 || err == -EALREADY) {
                    atomic_set(&background_asked[bt_conn_index(conn)], 1);
                    atomic_set(&module_asked_on_link[bt_conn_index(conn)], 1);
                    /* Its own request's probation is over: a drop now is
                       about these. */
                    atomic_ptr_cas(&probation_conn, conn, NULL);
                    LOG_INF("Profile %u's host kept while the active one is away: asked for "
                            "the background parameters",
                            hosts.profiles[i]);
                } else if (atomic_inc(&background_send_failures[bt_conn_index(conn)]) <
                           BACKGROUND_SEND_RETRIES) {
                    LOG_WRN("Profile %u's host: background parameters not sent (%d), trying "
                            "again in %u ms",
                            hosts.profiles[i], err, SEND_RETRY_SLOW_MS);
                    k_work_reschedule(&others_work, K_MSEC(SEND_RETRY_SLOW_MS));
                } else {
                    atomic_set(&background_asked[bt_conn_index(conn)], 1);
                    LOG_WRN("Profile %u's host: background parameters not sent (%d), given up "
                            "on this link",
                            hosts.profiles[i], err);
                }
            }
        }
        bt_conn_unref(conn);
    }
    if (hosts.active) {
        bt_conn_unref(hosts.active);
    }
out:
    k_mutex_unlock(&hl_lock);
}

static void others_changed(void) { k_work_reschedule(&others_work, K_MSEC(100)); }

static void others_connected(struct bt_conn *conn) {
    atomic_set(&background_asked[bt_conn_index(conn)], 0);
    atomic_set(&background_send_failures[bt_conn_index(conn)], 0);
    others_changed();
}

#else

static inline void others_changed(void) {}
static inline void others_connected(struct bt_conn *conn) { ARG_UNUSED(conn); }

#endif /* IS_ENABLED(CONFIG_ZMK_BLE_HOST_LINK_KEEP_ONLY_ACTIVE_HOST_CONNECTED) */

/* ------------------------------------------- Bluetooth callbacks */

static void on_connected(struct bt_conn *conn, uint8_t err) {
    struct bt_conn_info info;

    if (err != 0 || !is_host_link(conn, &info)) {
        return;
    }
    atomic_set(&conn_up_ms[bt_conn_index(conn)], (atomic_val_t)k_uptime_get_32());

    /* What the host chose on its own: the starting point of everything. */
    LOG_INF("Host opened the link at %u.%02u ms, latency %u, timeout %u ms",
            MS2(info.le.interval), info.le.latency, info.le.timeout * 10U);
    atomic_set(&module_asked_on_link[bt_conn_index(conn)], 0);
    atomic_set(&zmk_studio_was_on_link[bt_conn_index(conn)], 0);
    atomic_set(&zmk_studio_subscribed_at_connect[bt_conn_index(conn)], 0);
    note_hosts_own(conn, info.le.interval, info.le.latency, info.le.timeout);

    k_work_reschedule(&link_work, K_MSEC(50));
    /* By then a subscription to ZMK Studio restored with the bond is back. */
    k_work_reschedule(&zmk_studio_connect_work, K_MSEC(ZMK_STUDIO_RESTORED_AT_CONNECT_MS * 2 / 3));
    others_connected(conn);
}

static void on_disconnected(struct bt_conn *conn, uint8_t reason) {
    struct bt_conn_info info;

    if (!is_host_link(conn, &info)) {
        return;
    }
    atomic_ptr_cas(&settled_conn, conn, NULL);
    if (atomic_ptr_get(&op_conn) == conn) {
        atomic_set(&link_lost_reason, reason ? reason : 0xFF);
        k_work_reschedule(&op_work, K_NO_WAIT);
    }
    /* Not `else`: an operation ending puts its last request on probation a
       moment before it lets go of the link -- a drop then is for both, and
       the operation, already over, ignores it. A new operation on the link
       ends the probation first (op_begin()), so it is never both else. */
    if (atomic_ptr_get(&probation_conn) == conn) {
        atomic_set(&probation_lost_reason, reason ? reason : 0xFF);
        k_work_submit(&probation_work);
    }
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

    /* The host's own choice while nothing was asked of it on this link yet
       and ZMK Studio never had it: what an untouched link would run. */
    const struct hl_link now = {interval, latency, timeout};

    if (is_zmk_studio_params(&now)) {
        atomic_set(&zmk_studio_was_on_link[bt_conn_index(conn)], 1);
    } else if (hosts_own_still_shows(conn)) {
        note_hosts_own(conn, interval, latency, timeout);
    }

    /* Also called on a refusal over the Link Layer, with the old values. */
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
        LOG_WRN("Dropping the stored values of profile %lu: %u bytes, expected %u", p,
                (unsigned int)len, (unsigned int)sizeof(r));
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
        LOG_WRN("Dropping the stored values of profile %lu: format %u, expected %u", p, r.format,
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
    for (uint8_t p = 0; p < PROFILE_COUNT; p++) {
        struct hl_learned *l = &recs[p].learned;
        /* The keyboard asks latency 0 itself only fixed by hand, under a
           ceiling of 0 or with no latency while searching. */
        const bool asks_latency_0_itself = recs[p].pinned_latency_plus1 == 1 ||
                                           under_latency_ceiling(&recs[p], 1) == 0 ||
                                           LATENCY_WHILE_SEARCHING == 0;

        if ((l->flags & HL_LEARNED_FOUND) && l->found.latency == 0 &&
            !(l->flags & HL_LEARNED_FOUND_LATENCY_0_BY_HOST) && !asks_latency_0_itself &&
            l->found.interval_min < SLOWEST_INTERVAL) {
            /* Stored before latency 0 counted as worse than a slower
               interval: the host's 0 at a faster one. Searched again. */
            l->flags &= ~(HL_LEARNED_SEARCHED | HL_LEARNED_FOUND | HL_LEARNED_WITH_FOUND |
                          HL_LEARNED_HAS_FALLBACK | HL_LEARNED_WITH_FALLBACK |
                          HL_LEARNED_SEARCHED_AGAIN);
            l->refused_connections = 0;
            LOG_INF("Profile %u: stored result at latency 0 from before -- dropped, %s", p,
                    (l->flags & HL_LEARNED_NO_SEARCHING)
                        ? "the secure request asked instead (no searching on this host)"
                        : "searched again");
            mark_dirty(p, false);
        }
    }
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
    s->pinned_interval_min = r->pinned_interval_min;
    s->pinned_interval_max = r->pinned_interval_max;
    s->pinned_latency = latency_pinned(r) ? r->pinned_latency_plus1 - 1 : 0;
    s->pinned_ball_report_ms = r->pinned_ball_report_plus1 ? r->pinned_ball_report_plus1 - 1 : 0;

    if (op.kind != OP_NONE && op.profile == p) {
        asked = op.kind == OP_SEARCH && op.search.have_best ? op.search.best : op.asked;
    } else if (asks_nothing(r)) {
        s->asks_nothing = true;
        asked = (struct hl_params){0};
    } else {
        compose_request(r, &asked);
    }
    s->asked_interval_min = asked.interval_min;
    s->asked_interval_max = asked.interval_max;
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

int zmk_ble_host_link_set_interval(uint16_t interval_min, uint16_t interval_max) {
    if (interval_min != BLE_HOST_INTERVAL_AUTO &&
        (interval_min < HL_SPEC_FASTEST_INTERVAL || interval_max < interval_min ||
         interval_max > CONFIG_ZMK_BLE_HOST_LINK_SLOWEST_SETTABLE_INTERVAL)) {
        LOG_ERR("Host interval %u-%u is outside %u..%u (1.25 ms units)", interval_min,
                interval_max, HL_SPEC_FASTEST_INTERVAL,
                CONFIG_ZMK_BLE_HOST_LINK_SLOWEST_SETTABLE_INTERVAL);
        indicate(VERDICT_REFUSED);
        return -EINVAL;
    }
    if (lock_loaded()) {
        return -EAGAIN;
    }

    const uint8_t p = zmk_ble_active_profile_index();
    struct hl_record *r = rec_get(p);

    r->pinned_interval_min = interval_min;
    r->pinned_interval_max = interval_min ? interval_max : 0;
    rec_changed(p, false);
    settled.has_wanted = false;
    if (interval_min) {
        LOG_INF("Profile %u: host interval fixed at %u.%02u-%u.%02u ms", p, MS2(interval_min),
                MS2(interval_max));
    } else {
        LOG_INF("Profile %u: host interval chosen automatically", p);
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
    /* A search result belongs to the latency it was found with. */
    r->learned.flags &= ~(REC_SEARCHED | REC_FOUND | HL_LEARNED_WITH_FOUND | REC_HAS_FALLBACK |
                          HL_LEARNED_WITH_FALLBACK | HL_LEARNED_SEARCHED_AGAIN);
    r->learned.refused_connections = 0;
    rec_changed(p, false);
    settled.has_wanted = false;
    if (latency == BLE_HOST_LATENCY_AUTO) {
        LOG_INF("Profile %u: host latency chosen automatically", p);
    } else {
        LOG_INF("Profile %u: host latency fixed at %u", p, latency);
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
    if (ms == BALL_BLE_REPORT_AUTO) {
        LOG_INF("Profile %u: ball report interval follows the host interval", p);
    } else {
        LOG_INF("Profile %u: ball report interval fixed at %u ms", p, ms);
    }
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
    } else {
        const struct hl_record *r = rec_get(p);
        struct hl_params req;

        /* A profile that asks nothing: the host's own parameters. */
        if (asks_nothing(r)) {
            req = hosts_own_request(conn);
        } else {
            compose_request(r, &req);
        }
        op_apply(conn, p, &req, true, APPLY_ONCE, "asked again by a key");
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
    LOG_INF("Asked: %u.%02u-%u.%02u ms, latency %u, timeout %u ms", MS2(s.asked_interval_min),
            MS2(s.asked_interval_max), s.asked_latency, s.asked_timeout * 10U);
    if (s.connected) {
        LOG_INF("Link:  %u.%02u ms, latency %u, timeout %u ms", MS2(s.link_interval),
                s.link_latency, s.link_timeout * 10U);
    } else {
        LOG_INF("Link:  not connected");
    }
    LOG_INF("Ball report over BLE: %u ms%s", s.ball_report_ms, s.searching ? "; searching" : "");

    if (!s.connected || s.searching) {
        v = VERDICT_NOT_CONNECTED_OR_SEARCHING;
    } else if (s.asks_nothing) {
        /* Nothing asked: the link runs the host's own parameters, as meant. */
        LOG_INF("Nothing asked of this host: it accepted nothing when searched");
        v = VERDICT_ACCEPTED;
    } else {
        const struct hl_params asked = {s.asked_interval_min, s.asked_interval_max,
                                        s.asked_latency, s.asked_timeout};
        const struct hl_link link = {s.link_interval, s.link_latency, s.link_timeout};

        v = hl_link_matches(&asked, &link) ? VERDICT_ACCEPTED : VERDICT_REFUSED;
    }
    indicate(v);
}

/* ------------------------------------------------------- typed report */

/*
 * Only keys that type the same character in the Latin and the Cyrillic
 * layouts of Windows and macOS: digits, minus, underscore, parentheses. So
 * the report reads the same whichever layout the computer is on.
 *
 *   granted(asked)_granted(asked)_granted(asked)_ball
 *   interval        latency        timeout        report
 *
 * Interval in 1.25 ms units ("min-max" for a range), timeout in 10 ms units,
 * ball report in ms. Granted is 0 while the profile is not connected.
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
    case '-':
        return MINUS;
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

    char asked_interval[16];

    if (s.asked_interval_min == s.asked_interval_max) {
        snprintf(asked_interval, sizeof(asked_interval), "%u", s.asked_interval_min);
    } else {
        snprintf(asked_interval, sizeof(asked_interval), "%u-%u", s.asked_interval_min,
                 s.asked_interval_max);
    }

    const int n = snprintf(typed, sizeof(typed), "%u(%s)_%u(%u)_%u(%u)_%u", s.link_interval,
                           asked_interval, s.link_latency, s.asked_latency, s.link_timeout,
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
