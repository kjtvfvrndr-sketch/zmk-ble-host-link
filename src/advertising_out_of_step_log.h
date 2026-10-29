/*
 * SPDX-License-Identifier: MIT
 *
 * A debugging watch on the keyboard's advertising
 * (advertising_out_of_step_log.c, ZMK_BLE_HOST_LINK_LOG_ADVERTISING_OUT_OF_STEP).
 */

#pragma once

#include <zephyr/sys/util.h>

#if IS_ENABLED(CONFIG_ZMK_BLE_HOST_LINK_LOG_ADVERTISING_OUT_OF_STEP)
/* Logs one snapshot line: what the stack's advertiser does against what
   ZMK's rule asks for. */
void advertising_out_of_step_log_state(void);
#else
static inline void advertising_out_of_step_log_state(void) {}
#endif
