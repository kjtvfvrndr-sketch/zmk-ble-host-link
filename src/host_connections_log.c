/*
 * SPDX-License-Identifier: MIT
 *
 * What happens to the computers' connections, in the log
 * (ZMK_BLE_HOST_LINK_LOG_HOST_CONNECTIONS). Only logs; nothing here changes
 * what the keyboard does.
 *
 *   - the profile selected, what its host is doing, and which hosts are
 *     connected then;
 *   - every host that connects: whose profile it is -- or that no profile
 *     has it -- and the address it came with over the air, against the
 *     identity the stack resolved it to;
 *   - the encryption each host link reaches, or why it failed;
 *   - every host that goes: after how long and why;
 *   - pairings, failed pairings and deleted bonds;
 *   - at start and with BLE_HOST_SHOW, a snapshot: the profiles' addresses
 *     against the bonds the stack holds, every connection object with its
 *     state, and at start why the keyboard started.
 *
 * Together they show where a host that cannot come back stops: before the
 * keyboard sees it at all (no line), at its address, at the encryption, or
 * later; and whether connection objects stay taken after their link went.
 *
 * Host links only: the keyboard is the peripheral on them. The links to the
 * other half of a split, where it is central, show only in the snapshot.
 */

#include <stdio.h>

#include <zephyr/kernel.h>
#include <zephyr/init.h>
#include <zephyr/sys/util.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/addr.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/hci_types.h>
#include <zephyr/logging/log.h>

#if IS_ENABLED(CONFIG_HWINFO)
#include <zephyr/drivers/hwinfo.h>
#endif

#include <zmk/ble.h>
#include <zmk/event_manager.h>
#include <zmk/events/ble_active_profile_changed.h>

#include "host_connections_log.h"
#include "own_pairing_state_check.h"
#include "advertising_out_of_step_log.h"

LOG_MODULE_DECLARE(zmk_ble_host_link, CONFIG_ZMK_BLE_HOST_LINK_LOG_LEVEL);

#define PROFILE_COUNT ZMK_BLE_PROFILE_COUNT

/* Settings, bonds and ZMK's profiles are loaded by then, and the boot
   connections have had their first seconds. */
#define SNAPSHOT_AT_START_AFTER_MS 5000

/* "Profile 1's host" / "Host 11:22:33:44:55:66 (random)" */
#define HOST_NAME_LEN (BT_ADDR_LE_STR_LEN + 8)

/* ------------------------------------------------------------ wording */

/* "11:22:33:44:55:66 (public)", "…(random, resolvable private)" */
#define ADDRESS_TEXT_LEN (BT_ADDR_STR_LEN + 40)

static void address_text(const bt_addr_le_t *addr, char *buf, size_t len) {
    char a[BT_ADDR_STR_LEN];
    const char *kind;

    bt_addr_to_str(&addr->a, a, sizeof(a));
    if (addr->type == BT_ADDR_LE_PUBLIC || addr->type == BT_ADDR_LE_PUBLIC_ID) {
        kind = "public";
    } else if (BT_ADDR_IS_RPA(&addr->a)) {
        kind = "random, resolvable private";
    } else if (BT_ADDR_IS_STATIC(&addr->a)) {
        kind = "random static";
    } else {
        kind = "random, non-resolvable private";
    }
    snprintf(buf, len, "%s (%s)", a, kind);
}

static const char *disconnect_reason(uint8_t reason) {
    switch (reason) {
    case BT_HCI_ERR_AUTH_FAIL:
        return "authentication failure";
    case BT_HCI_ERR_PIN_OR_KEY_MISSING:
        return "key missing";
    case BT_HCI_ERR_CONN_TIMEOUT:
        return "supervision timeout, lost on air";
    case BT_HCI_ERR_CONN_LIMIT_EXCEEDED:
        return "connection limit exceeded";
    case BT_HCI_ERR_INSUFFICIENT_SECURITY:
        return "insufficient security";
    case BT_HCI_ERR_REMOTE_USER_TERM_CONN:
        return "ended by the other side";
    case BT_HCI_ERR_REMOTE_LOW_RESOURCES:
        return "the other side ran out of resources";
    case BT_HCI_ERR_REMOTE_POWER_OFF:
        return "the other side powered off";
    case BT_HCI_ERR_LOCALHOST_TERM_CONN:
        return "ended by this keyboard";
    case BT_HCI_ERR_UNSUPP_REMOTE_FEATURE:
        return "unsupported remote feature";
    case BT_HCI_ERR_UNSPECIFIED:
        return "unspecified";
    case BT_HCI_ERR_LL_RESP_TIMEOUT:
        return "link layer response timeout";
    case BT_HCI_ERR_LL_PROC_COLLISION:
        return "link layer procedure collision";
    case BT_HCI_ERR_INSTANT_PASSED:
        return "instant passed";
    case BT_HCI_ERR_PAIRING_NOT_SUPPORTED:
        return "pairing not supported";
    case BT_HCI_ERR_DIFF_TRANS_COLLISION:
        return "procedure collision";
    case BT_HCI_ERR_UNACCEPT_CONN_PARAM:
        return "unacceptable connection parameters";
    case BT_HCI_ERR_TERM_DUE_TO_MIC_FAIL:
        return "encrypted packet failed its check (MIC)";
    case BT_HCI_ERR_CONN_FAIL_TO_ESTAB:
        return "failed to be established";
    default:
        return "other";
    }
}

static const char *security_error(enum bt_security_err err) {
    switch (err) {
    case BT_SECURITY_ERR_SUCCESS:
        return "none";
    case BT_SECURITY_ERR_AUTH_FAIL:
        return "authentication failed";
    case BT_SECURITY_ERR_PIN_OR_KEY_MISSING:
        return "key missing";
    case BT_SECURITY_ERR_OOB_NOT_AVAILABLE:
        return "OOB data not available";
    case BT_SECURITY_ERR_AUTH_REQUIREMENT:
        return "authentication requirements not met";
    case BT_SECURITY_ERR_PAIR_NOT_SUPPORTED:
        return "pairing not supported";
    case BT_SECURITY_ERR_PAIR_NOT_ALLOWED:
        return "pairing not allowed";
    case BT_SECURITY_ERR_INVALID_PARAM:
        return "invalid parameters";
    case BT_SECURITY_ERR_KEY_REJECTED:
        return "key rejected";
    case BT_SECURITY_ERR_UNSPECIFIED:
    default:
        return "unspecified";
    }
}

static const char *conn_state(enum bt_conn_state state) {
    switch (state) {
    case BT_CONN_STATE_DISCONNECTED:
        return "disconnected, still held";
    case BT_CONN_STATE_CONNECTING:
        return "connecting";
    case BT_CONN_STATE_CONNECTED:
        return "connected";
    case BT_CONN_STATE_DISCONNECTING:
        return "disconnecting";
    default:
        return "unknown state";
    }
}

static const char *profile_state(uint8_t profile) {
    if (zmk_ble_profile_is_open(profile)) {
        return "open, no host paired";
    }
    return zmk_ble_profile_is_connected(profile) ? "its host is connected"
                                                 : "its host is away";
}

/* "Profile 1's host", or the address for a host no profile has. */
static void host_name(const bt_addr_le_t *identity, char *buf, size_t len) {
    const int profile = zmk_ble_profile_index(identity);

    if (profile >= 0 && profile < PROFILE_COUNT) {
        snprintf(buf, len, "Profile %d's host", profile);
    } else {
        char addr[BT_ADDR_LE_STR_LEN];

        bt_addr_le_to_str(identity, addr, sizeof(addr));
        snprintf(buf, len, "Host %s", addr);
    }
}

static bool is_host_link(struct bt_conn *conn, struct bt_conn_info *info) {
    return bt_conn_get_info(conn, info) == 0 && info->type == BT_CONN_TYPE_LE &&
           info->role == BT_CONN_ROLE_PERIPHERAL;
}

/* ------------------------------------------------- connections */

/* k_uptime_get_32() at each host link's connect, by bt_conn_index(). The
   Bluetooth callbacks that use it all run in the same context. */
static uint32_t connected_at_ms[CONFIG_BT_MAX_CONN];

static void on_connected(struct bt_conn *conn, uint8_t err) {
    struct bt_conn_info info;

    if (!is_host_link(conn, &info)) {
        return;
    }

    char identity[BT_ADDR_LE_STR_LEN];

    bt_addr_le_to_str(info.le.dst, identity, sizeof(identity));
    if (err != 0) {
        LOG_WRN("A host's connection failed (0x%02x, %s): %s", err, disconnect_reason(err),
                identity);
        return;
    }
    connected_at_ms[bt_conn_index(conn)] = k_uptime_get_32();

    const int profile = zmk_ble_profile_index(info.le.dst);
    const int active = zmk_ble_active_profile_index();

    if (profile == active) {
        LOG_INF("Profile %d's host connected -- the active profile", profile);
    } else if (profile >= 0 && profile < PROFILE_COUNT) {
        LOG_INF("Profile %d's host connected while profile %d is active", profile, active);
    } else if (zmk_ble_active_profile_is_open()) {
        LOG_INF("A host no profile has connected; profile %d is active and open: it may pair "
                "there",
                active);
    } else {
        LOG_INF("A host no profile has connected while profile %d is active", active);
    }

    char dst[ADDRESS_TEXT_LEN];

    address_text(info.le.dst, dst, sizeof(dst));
    if (bt_addr_le_cmp(info.le.dst, info.le.remote) != 0) {
        char remote[ADDRESS_TEXT_LEN];

        address_text(info.le.remote, remote, sizeof(remote));
        LOG_INF("  identity %s, came as %s", dst, remote);
    } else if (BT_ADDR_IS_RPA(&info.le.dst->a) && info.le.dst->type == BT_ADDR_LE_RANDOM) {
        /* The stack keeps a private address it could not resolve as it is:
           no bonded host's key matched it. */
        LOG_INF("  address %s, not resolved to any bonded host", dst);
    } else {
        LOG_INF("  address %s", dst);
    }
    own_pairing_state_log_address("a host connected", true);
}

static void on_disconnected(struct bt_conn *conn, uint8_t reason) {
    struct bt_conn_info info;

    if (!is_host_link(conn, &info)) {
        return;
    }

    char who[HOST_NAME_LEN];
    const uint32_t lasted_ms = k_uptime_get_32() - connected_at_ms[bt_conn_index(conn)];

    host_name(info.le.dst, who, sizeof(who));
    LOG_INF("%s disconnected after %u.%01u s: 0x%02x, %s", who, lasted_ms / 1000U,
            (lasted_ms % 1000U) / 100U, reason, disconnect_reason(reason));
}

static void on_security_changed(struct bt_conn *conn, bt_security_t level,
                                enum bt_security_err err) {
    struct bt_conn_info info;

    if (!is_host_link(conn, &info)) {
        return;
    }

    char who[HOST_NAME_LEN];

    host_name(info.le.dst, who, sizeof(who));
    if (err == BT_SECURITY_ERR_SUCCESS) {
        LOG_INF("%s: encrypted, security level %d", who, level);
    } else {
        LOG_WRN("%s: encryption failed: %s (%d), security level %d", who, security_error(err),
                err, level);
    }
}

BT_CONN_CB_DEFINE(zmk_ble_host_link_connections_log_callbacks) = {
    .connected = on_connected,
    .disconnected = on_disconnected,
    .security_changed = on_security_changed,
};

/* ------------------------------------------------- pairing and bonds */

#if IS_ENABLED(CONFIG_ZMK_BLE_HOST_LINK_CHECK_OWN_PAIRING_STATE)
/* The key pair check after a failed pairing: once the host has gone, and
   before it is likely to try again. */
#define CHECK_AFTER_FAILED_PAIRING_MS 1500

static void check_after_failed_pairing_cb(struct k_work *work) {
    ARG_UNUSED(work);
    own_pairing_state_check_key_pair("after a failed pairing");
}

static K_WORK_DELAYABLE_DEFINE(check_after_failed_pairing, check_after_failed_pairing_cb);
#endif

static void on_pairing_complete(struct bt_conn *conn, bool bonded) {
    struct bt_conn_info info;

    if (!is_host_link(conn, &info)) {
        return;
    }

    char addr[BT_ADDR_LE_STR_LEN];

    bt_addr_le_to_str(info.le.dst, addr, sizeof(addr));
    LOG_INF("Host %s paired%s; profile %d is active", addr, bonded ? ", bond kept" : ", no bond",
            zmk_ble_active_profile_index());
#if IS_ENABLED(CONFIG_ZMK_BLE_HOST_LINK_CHECK_OWN_PAIRING_STATE)
    char fingerprint[12];

    own_pairing_state_public_key_fingerprint(fingerprint, sizeof(fingerprint));
    LOG_INF("  with the keyboard's public key %s", fingerprint);
#endif
}

static void on_pairing_failed(struct bt_conn *conn, enum bt_security_err reason) {
    struct bt_conn_info info;

    if (!is_host_link(conn, &info)) {
        return;
    }

    char addr[BT_ADDR_LE_STR_LEN];

    bt_addr_le_to_str(info.le.dst, addr, sizeof(addr));
    LOG_WRN("Host %s: pairing failed: %s (%d)", addr, security_error(reason), reason);
#if IS_ENABLED(CONFIG_ZMK_BLE_HOST_LINK_CHECK_OWN_PAIRING_STATE)
    char fingerprint[12];

    own_pairing_state_public_key_fingerprint(fingerprint, sizeof(fingerprint));
    LOG_WRN("  with the keyboard's public key %s", fingerprint);
    own_pairing_state_log_address("a pairing failed", false);
    k_work_reschedule(&check_after_failed_pairing, K_MSEC(CHECK_AFTER_FAILED_PAIRING_MS));
#endif
}

/* From bt_unpair(): ZMK clears a profile's bond before the profile's address,
   so the profile still shows. */
static void on_bond_deleted(uint8_t id, const bt_addr_le_t *peer) {
    char addr[BT_ADDR_LE_STR_LEN];
    const int profile = zmk_ble_profile_index(peer);

    bt_addr_le_to_str(peer, addr, sizeof(addr));
    if (profile >= 0 && profile < PROFILE_COUNT) {
        LOG_INF("Bond deleted: profile %d's host %s (identity %u)", profile, addr, id);
    } else {
        LOG_INF("Bond deleted: %s, no profile's (identity %u)", addr, id);
    }
}

static struct bt_conn_auth_info_cb pairing_and_bonds_log = {
    .pairing_complete = on_pairing_complete,
    .pairing_failed = on_pairing_failed,
    .bond_deleted = on_bond_deleted,
};

/* ------------------------------------------------- the profile selected */

/* "0, 3" -- the profiles of the hosts connected now, "?" for one no profile
   has; "none". */
struct connected_hosts {
    char text[48];
    size_t len;
};

static void add_connected_host(struct bt_conn *conn, void *data) {
    struct connected_hosts *hosts = data;
    struct bt_conn_info info;

    if (!is_host_link(conn, &info) || info.state != BT_CONN_STATE_CONNECTED ||
        hosts->len >= sizeof(hosts->text) - 4) {
        return;
    }

    const int profile = zmk_ble_profile_index(info.le.dst);
    const int n = profile >= 0 ? snprintf(hosts->text + hosts->len,
                                          sizeof(hosts->text) - hosts->len, "%s%d",
                                          hosts->len ? ", " : "", profile)
                               : snprintf(hosts->text + hosts->len,
                                          sizeof(hosts->text) - hosts->len, "%s?",
                                          hosts->len ? ", " : "");

    if (n > 0) {
        hosts->len = MIN(hosts->len + (size_t)n, sizeof(hosts->text) - 1);
    }
}

static void connected_hosts_now(struct connected_hosts *hosts) {
    hosts->len = 0;
    hosts->text[0] = '\0';
    bt_conn_foreach(BT_CONN_TYPE_LE, add_connected_host, hosts);
    if (hosts->len == 0) {
        snprintf(hosts->text, sizeof(hosts->text), "none");
    }
}

/* The event also comes when the active profile's host connects or goes, and
   on a pairing: only a change of the profile is logged. The first one after
   start is the profile active from settings. */
static int last_logged_profile = -1;

static int on_active_profile_changed(const zmk_event_t *eh) {
    const struct zmk_ble_active_profile_changed *ev = as_zmk_ble_active_profile_changed(eh);

    if (ev == NULL || ev->index == last_logged_profile) {
        return ZMK_EV_EVENT_BUBBLE;
    }

    struct connected_hosts hosts;
    const bool first = last_logged_profile < 0;

    last_logged_profile = ev->index;
    connected_hosts_now(&hosts);
    LOG_INF("Profile %u %s: %s; hosts connected, by profile: %s", ev->index,
            first ? "active" : "selected", profile_state(ev->index), hosts.text);
    return ZMK_EV_EVENT_BUBBLE;
}

ZMK_LISTENER(zmk_ble_host_link_connections_log, on_active_profile_changed);
ZMK_SUBSCRIPTION(zmk_ble_host_link_connections_log, zmk_ble_active_profile_changed);

/* ------------------------------------------------- snapshot */

struct bond_lookup {
    const bt_addr_le_t *addr;
    bool found;
};

static void match_bond(const struct bt_bond_info *bond, void *data) {
    struct bond_lookup *lookup = data;

    if (bt_addr_le_cmp(&bond->addr, lookup->addr) == 0) {
        lookup->found = true;
    }
}

static bool stack_has_bond(const bt_addr_le_t *addr) {
    struct bond_lookup lookup = {.addr = addr};

    bt_foreach_bond(BT_ID_DEFAULT, match_bond, &lookup);
    return lookup.found;
}

/* The other half of a split: a bond the keyboard is central on. */
static bool is_other_half(const bt_addr_le_t *addr) {
    struct bt_conn *conn = bt_conn_lookup_addr_le(BT_ID_DEFAULT, addr);
    struct bt_conn_info info;
    bool central = false;

    if (conn != NULL) {
        central = bt_conn_get_info(conn, &info) == 0 && info.role == BT_CONN_ROLE_CENTRAL;
        bt_conn_unref(conn);
    }
    return central;
}

static void log_bond(const struct bt_bond_info *bond, void *data) {
    unsigned int *count = data;
    char addr[BT_ADDR_LE_STR_LEN];
    const int profile = zmk_ble_profile_index(&bond->addr);

    (*count)++;
    bt_addr_le_to_str(&bond->addr, addr, sizeof(addr));
    if (profile >= 0 && profile < PROFILE_COUNT) {
        LOG_INF("  bond %s: profile %d", addr, profile);
    } else if (is_other_half(&bond->addr)) {
        LOG_INF("  bond %s: the other half, connected", addr);
    } else {
        LOG_INF("  bond %s: no profile has it (the other half while away, or left over)",
                addr);
    }
}

bool host_connections_log_is_advertisers_connection(struct bt_conn *conn) {
    struct bt_conn_info info;

    return bt_conn_get_info(conn, &info) == 0 && info.type == BT_CONN_TYPE_LE &&
           info.state == BT_CONN_STATE_CONNECTING && bt_addr_le_eq(info.le.dst, BT_ADDR_LE_NONE);
}

static void log_conn(struct bt_conn *conn, void *data) {
    unsigned int *count = data;
    struct bt_conn_info info;

    if (bt_conn_get_info(conn, &info) != 0 || info.type != BT_CONN_TYPE_LE) {
        return;
    }
    (*count)++;

    char addr[BT_ADDR_LE_STR_LEN];

    bt_addr_le_to_str(info.le.dst, addr, sizeof(addr));
    if (host_connections_log_is_advertisers_connection(conn)) {
        LOG_INF("  connection %u: the advertiser's, waiting for a host to answer",
                bt_conn_index(conn));
    } else if (info.role == BT_CONN_ROLE_PERIPHERAL) {
        const int profile = zmk_ble_profile_index(info.le.dst);

        if (profile >= 0) {
            LOG_INF("  connection %u: profile %d's host %s, %s, security level %d",
                    bt_conn_index(conn), profile, addr, conn_state(info.state),
                    info.security.level);
        } else {
            LOG_INF("  connection %u: host %s no profile has, %s, security level %d",
                    bt_conn_index(conn), addr, conn_state(info.state), info.security.level);
        }
    } else {
        LOG_INF("  connection %u: other half %s, %s", bt_conn_index(conn), addr,
                conn_state(info.state));
    }
}

void host_connections_log_bluetooth_state(const char *occasion) {
    const int active = zmk_ble_active_profile_index();

    LOG_INF("Bluetooth state %s: profile %d active", occasion, active);
    for (uint8_t p = 0; p < PROFILE_COUNT; p++) {
        const bt_addr_le_t *addr = zmk_ble_profile_address(p);

        if (zmk_ble_profile_is_open(p)) {
            LOG_INF("  profile %u: open", p);
            continue;
        }

        char text[BT_ADDR_LE_STR_LEN];

        bt_addr_le_to_str(addr, text, sizeof(text));
        LOG_INF("  profile %u: %s, %s; bond in the stack: %s", p, text,
                zmk_ble_profile_is_connected(p) ? "connected" : "away",
                stack_has_bond(addr) ? "yes" : "MISSING");
    }

    unsigned int bonds = 0;

    bt_foreach_bond(BT_ID_DEFAULT, log_bond, &bonds);
    LOG_INF("  %u of %d bonds held", bonds, CONFIG_BT_MAX_PAIRED);

    unsigned int conns = 0;

    bt_conn_foreach(BT_CONN_TYPE_LE, log_conn, &conns);
    LOG_INF("  %u of %d connection objects in use", conns, CONFIG_BT_MAX_CONN);
    advertising_out_of_step_log_state();

    own_pairing_state_log_address(occasion, false);
    own_pairing_state_check_key_pair(occasion);
}

/* ------------------------------------------------- start */

#if IS_ENABLED(CONFIG_HWINFO)
/* Why the keyboard started. The record is cleared once read: the next start
   shows only its own cause. */
static void log_start_cause(void) {
    uint32_t cause;

    if (hwinfo_get_reset_cause(&cause) != 0) {
        LOG_INF("Started; the cause cannot be read");
        return;
    }
    (void)hwinfo_clear_reset_cause();

    if (cause == 0) {
        LOG_INF("Started with no reset recorded: power switched on (or the bootloader "
                "cleared the record)");
        return;
    }
    LOG_INF("Started after:%s%s%s%s%s%s%s%s (0x%08x)", (cause & RESET_PIN) ? " reset pin" : "",
            (cause & RESET_SOFTWARE) ? " software reset (a reboot key, a fatal error, the "
                                       "bootloader)"
                                     : "",
            (cause & RESET_WATCHDOG) ? " watchdog" : "",
            (cause & RESET_CPU_LOCKUP) ? " CPU lockup" : "",
            (cause & RESET_LOW_POWER_WAKE) ? " wake from system off" : "",
            (cause & RESET_DEBUG) ? " debugger" : "",
            (cause & RESET_BROWNOUT) ? " brownout" : "", (cause & RESET_POR) ? " power-on" : "",
            cause);
}
#endif

static void snapshot_at_start_cb(struct k_work *work) {
    ARG_UNUSED(work);

#if IS_ENABLED(CONFIG_HWINFO)
    log_start_cause();
#endif
    host_connections_log_bluetooth_state("at start");
}

static K_WORK_DELAYABLE_DEFINE(snapshot_at_start, snapshot_at_start_cb);

static int host_connections_log_init(void) {
    const int err = bt_conn_auth_info_cb_register(&pairing_and_bonds_log);

    if (err != 0) {
        LOG_WRN("Pairings and deleted bonds will not be logged (%d)", err);
    }
    k_work_reschedule(&snapshot_at_start, K_MSEC(SNAPSHOT_AT_START_AFTER_MS));
    return 0;
}

SYS_INIT(host_connections_log_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);
