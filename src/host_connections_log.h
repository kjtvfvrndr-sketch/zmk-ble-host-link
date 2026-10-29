/*
 * SPDX-License-Identifier: MIT
 *
 * The log of the hosts' connections (host_connections_log.c,
 * ZMK_BLE_HOST_LINK_LOG_HOST_CONNECTIONS).
 */

#pragma once

#include <stdbool.h>

#include <zephyr/sys/util.h>
#include <zephyr/bluetooth/conn.h>

#if IS_ENABLED(CONFIG_ZMK_BLE_HOST_LINK_LOG_HOST_CONNECTIONS)
/* Logs the Bluetooth state as it is now: the profiles against the bonds the
   stack holds, and every connection object. `occasion` heads it ("at start",
   "on request"). */
void host_connections_log_bluetooth_state(const char *occasion);

/* The connection object a connectable advertiser keeps for the host that
   answers it: connecting, no peer yet. */
bool host_connections_log_is_advertisers_connection(struct bt_conn *conn);
#else
static inline void host_connections_log_bluetooth_state(const char *occasion) {
    ARG_UNUSED(occasion);
}
#endif
