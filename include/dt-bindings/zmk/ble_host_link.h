/*
 * SPDX-License-Identifier: MIT
 *
 * &ble_host_link commands. In a keymap:
 *
 *   &ble_host_link BLE_HOST_INTERVAL_NEXT
 *   &ble_host_link BLE_HOST_INTERVAL_SET 9
 *   &ble_host_link BLE_HOST_INTERVAL_SET BLE_HOST_INTERVAL_RANGE(9, 21)
 *   &ble_host_link BLE_HOST_LATENCY_SET BLE_HOST_LATENCY_AUTO
 *   &ble_host_link BALL_BLE_REPORT_SET 8
 *
 * Units: host interval in Bluetooth's 1.25 ms steps (6 = 7.5 ms, 12 = 15 ms),
 * host latency in connection events, ball report interval in ms.
 */

#pragma once

#define BLE_HOST_INTERVAL_NEXT_CMD 0
#define BLE_HOST_INTERVAL_PREV_CMD 1
#define BLE_HOST_INTERVAL_SET_CMD 2
#define BLE_HOST_LATENCY_NEXT_CMD 3
#define BLE_HOST_LATENCY_PREV_CMD 4
#define BLE_HOST_LATENCY_SET_CMD 5
#define BALL_BLE_REPORT_NEXT_CMD 6
#define BALL_BLE_REPORT_PREV_CMD 7
#define BALL_BLE_REPORT_SET_CMD 8
#define BLE_HOST_LINK_AUTO_CMD 9
#define BLE_HOST_LINK_REQUEST_AGAIN_CMD 10
#define BLE_HOST_LINK_SHOW_CMD 11
#define BLE_HOST_LINK_TYPE_REPORT_CMD 12

/* Step through the presets of the behavior node, fixing the value by hand. */
#define BLE_HOST_INTERVAL_NEXT BLE_HOST_INTERVAL_NEXT_CMD 0
#define BLE_HOST_INTERVAL_PREV BLE_HOST_INTERVAL_PREV_CMD 0
#define BLE_HOST_LATENCY_NEXT BLE_HOST_LATENCY_NEXT_CMD 0
#define BLE_HOST_LATENCY_PREV BLE_HOST_LATENCY_PREV_CMD 0
#define BALL_BLE_REPORT_NEXT BALL_BLE_REPORT_NEXT_CMD 0
#define BALL_BLE_REPORT_PREV BALL_BLE_REPORT_PREV_CMD 0

/* Fix an exact value, given as the second parameter. */
#define BLE_HOST_INTERVAL_SET BLE_HOST_INTERVAL_SET_CMD
#define BLE_HOST_LATENCY_SET BLE_HOST_LATENCY_SET_CMD
#define BALL_BLE_REPORT_SET BALL_BLE_REPORT_SET_CMD

/* Drop everything fixed by hand on the active profile and search again. */
#define BLE_HOST_LINK_AUTO BLE_HOST_LINK_AUTO_CMD 0
/* Send the current request once more, ignoring what the search remembers. */
#define BLE_HOST_LINK_REQUEST_AGAIN BLE_HOST_LINK_REQUEST_AGAIN_CMD 0
/* Count the values out on the LED, then whether the link runs them. */
#define BLE_HOST_LINK_SHOW BLE_HOST_LINK_SHOW_CMD 0
/* Type the values into the focused text field, layout-independent. */
#define BLE_HOST_LINK_TYPE_REPORT BLE_HOST_LINK_TYPE_REPORT_CMD 0

/* Values for the _SET commands and the presets that hand a parameter back to
 * the automatic choice. */
#define BLE_HOST_INTERVAL_AUTO 0
#define BLE_HOST_LATENCY_AUTO 0xFFFF
#define BALL_BLE_REPORT_AUTO 255

/* A range of host intervals to offer, the host picks inside it. A plain
 * number asks for exactly that interval. */
#define BLE_HOST_INTERVAL_RANGE(min, max) (((max) << 16) | (min))
