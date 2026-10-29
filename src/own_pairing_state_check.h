/*
 * SPDX-License-Identifier: MIT
 *
 * A debugging check of what this keyboard brings to a new pairing
 * (own_pairing_state_check.c, ZMK_BLE_HOST_LINK_CHECK_OWN_PAIRING_STATE).
 */

#pragma once

#include <stdbool.h>
#include <stddef.h>

#include <zephyr/sys/util.h>

#if IS_ENABLED(CONFIG_ZMK_BLE_HOST_LINK_CHECK_OWN_PAIRING_STATE)
/* Checks, off the work queues ZMK uses, that the keyboard's key pair for
   Secure Connections pairing still holds together, and logs the result.
   `occasion` -- a string literal -- heads it. Returns at once. */
void own_pairing_state_check_key_pair(const char *occasion);

/* Logs the address the stack advertises and connects with against the
   keyboard's identity: WRN when they differ, INF otherwise unless
   `only_if_differs`. */
void own_pairing_state_log_address(const char *occasion, bool only_if_differs);

/* "a1b2c3d4": the first bytes of the keyboard's public key, as sent to
   hosts; "none" before there is one. */
void own_pairing_state_public_key_fingerprint(char *buf, size_t len);
#else
static inline void own_pairing_state_check_key_pair(const char *occasion) {
    ARG_UNUSED(occasion);
}
static inline void own_pairing_state_log_address(const char *occasion, bool only_if_differs) {
    ARG_UNUSED(occasion);
    ARG_UNUSED(only_if_differs);
}
#endif
