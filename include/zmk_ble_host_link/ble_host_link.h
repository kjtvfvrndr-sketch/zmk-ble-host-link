/*
 * SPDX-License-Identifier: MIT
 *
 * The core of zmk-ble-host-link, for its behavior (and anything else that
 * wants it). Everything acts on the active BLE profile and is meant for the
 * system work queue, where ZMK runs behaviors. Until settings are loaded the
 * setters return -EAGAIN.
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#include <dt-bindings/zmk/ble_host_link.h>

struct zmk_ble_host_link_status {
    /* Fixed by hand on this profile, or chosen automatically. */
    bool interval_pinned;
    bool latency_pinned;
    bool ball_report_pinned;

    /* The values fixed by hand, as set (0 where not fixed). What is asked
       for may differ: a latency is kept within the spec's limit for the
       interval it goes with. */
    uint16_t pinned_interval;
    uint16_t pinned_latency;
    uint16_t pinned_ball_report_ms;

    /* What the module asks the host for now. */
    uint16_t asked_interval;
    uint16_t asked_latency;
    uint16_t asked_timeout; /* 10 ms units */

    /* What the link runs; all 0 while the profile is not connected. */
    bool connected;
    uint16_t link_interval;
    uint16_t link_latency;
    uint16_t link_timeout;

    /* A search is running on the link. */
    bool searching;

    /* The pointer report interval in use over BLE (ms); 0 without a pointer
       driver that takes one. */
    uint16_t ball_report_ms;
};

int zmk_ble_host_link_get_status(struct zmk_ble_host_link_status *status);

/* One exact interval; BLE_HOST_INTERVAL_AUTO hands the interval back. */
int zmk_ble_host_link_set_interval(uint16_t interval);
/* BLE_HOST_LATENCY_AUTO hands the latency back. */
int zmk_ble_host_link_set_latency(uint16_t latency);
/* BALL_BLE_REPORT_AUTO hands the report interval back. */
int zmk_ble_host_link_set_ball_report(uint8_t ms);

int zmk_ble_host_link_auto(void);
int zmk_ble_host_link_request_again(void);
void zmk_ble_host_link_show(void);
int zmk_ble_host_link_type_report(void);

/* A search of the host link's parameters runs: the link will change a few
   times more, one request every PAUSE_BETWEEN_SEARCH_REQUESTS_MS or so. For anything
   that follows the host link (the split link ladder), to wait for the end.
   Callable from any thread. */
bool zmk_ble_host_link_is_searching(void);

/* The pointer report interval derived from a host interval: the interval
   rounded up to the 4 ms frame grid of the sensor -- one report per
   connection event (7.5 ms -> 8, 11.25 ms -> 12, 15 ms -> 16). */
uint16_t zmk_ble_host_link_ball_report_for_interval(uint16_t interval);
