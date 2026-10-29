/*
 * SPDX-License-Identifier: MIT
 *
 * The automatic search for the host link's parameters, as plain logic: it
 * names the next request and is told whether the host accepted the last one.
 * No Zephyr in here, so tests/ can drive it against simulated hosts.
 *
 * The search, in order:
 *   1. the secure request: exactly the slowest interval worth having, at the
 *      latency every host takes;
 *   2. the ladder: every exact interval from the fastest up to one below the
 *      slowest, at that latency; the first one the host accepts ends it;
 *   3. the latency steps: towards the idle listening period wanted, at the
 *      interval found -- the target first, then halfway back after a refusal.
 * The result is the last request the host accepted: a ladder step if one was
 * accepted, else the secure request if it was, else nothing.
 *
 * Units are Bluetooth's own: intervals in 1.25 ms, the supervision timeout
 * in 10 ms, the latency in connection events.
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

/* The lowest interval the Bluetooth core spec allows (7.5 ms). */
#define HL_SPEC_FASTEST_INTERVAL 6
/* The highest peripheral latency the core spec allows. */
#define HL_SPEC_MAX_LATENCY 499

/* The latency steps stop after this many requests. */
#define HL_SEARCH_LATENCY_STEPS 3

/* What a request asks for: always one exact interval. */
struct hl_params {
    uint16_t interval;
    uint16_t latency;
    uint16_t timeout;
};

/* What the link runs. */
struct hl_link {
    uint16_t interval;
    uint16_t latency;
    uint16_t timeout;
};

/* No latency fixed by hand: the search picks it. */
#define HL_LATENCY_NOT_PINNED 0xFFFF
/* No latency ceiling learned. */
#define HL_NO_LATENCY_CEILING 0xFFFF
/* No interval floor learned. */
#define HL_NO_INTERVAL_FLOOR 0

struct hl_search_limits {
    /* The fastest interval worth asking for: where the ladder starts. */
    uint16_t fastest_interval;
    /* The slowest interval worth having (15 ms with a pointer): the secure
       request. The ladder ends one below it. */
    uint16_t slowest_interval;
    /* The latency the interval is searched at: one every host accepts. */
    uint16_t latency_while_searching_interval;
    /* The latency steps aim for interval * (latency + 1) near this. */
    uint16_t idle_listen_period_ms;
    uint16_t timeout;
    /* A latency fixed by hand, or HL_LATENCY_NOT_PINNED: every request
       carries it, and there are no latency steps. */
    uint16_t pinned_latency;
    /* Learned from drops: the ladder starts no lower than this interval
       (HL_NO_INTERVAL_FLOOR none), and every latency stays below the
       ceiling (HL_NO_LATENCY_CEILING none). */
    uint16_t interval_floor;
    uint16_t latency_ceiling;
};

enum hl_search_step {
    HL_STEP_SECURE,
    HL_STEP_LADDER,
    HL_STEP_LATENCY,
};

enum hl_search_phase {
    HL_PHASE_SECURE,
    HL_PHASE_LADDER,
    HL_PHASE_LATENCY,
    HL_PHASE_DONE,
};

struct hl_search {
    struct hl_search_limits limits;
    enum hl_search_phase phase;

    /* The last request the host accepted: the result so far. */
    bool have_result;
    struct hl_params result;

    /* The last request handed out, and what it was. */
    struct hl_params asked;
    enum hl_search_step asked_step;
    /* hl_search_answer() told the verdict on `asked`; the next
       hl_search_next() acts on it. Until then a refusal can still turn
       into an acceptance (hl_search_late_answer()). */
    bool verdict_pending;
    bool asked_accepted;

    uint16_t next_ladder_interval;

    uint16_t latency_refused_lowest; /* HL_NO_LATENCY_CEILING: none yet */
    uint8_t latency_steps;

    uint8_t requests;
};

/* The link runs what the request asked for: the interval and the latency
   exactly. A lower latency is not what was asked. The timeout is the host's
   to choose. */
bool hl_link_matches(const struct hl_params *asked, const struct hl_link *link);

/* The largest latency at most `wanted` the core spec allows at this interval
   and timeout: the timeout must outlast two listening periods,
   interval * (latency + 1) * 2 < timeout. */
uint16_t hl_latency_allowed(uint16_t wanted, uint16_t interval, uint16_t timeout);

/* The latency that makes interval * (latency + 1) come nearest the idle
   listen period without exceeding it, within the spec. */
uint16_t hl_latency_for_idle_listen_period(uint16_t interval, uint16_t idle_listen_period_ms,
                                           uint16_t timeout);

/* Below the ceiling, if there is one. */
uint16_t hl_latency_under_ceiling(uint16_t latency, uint16_t latency_ceiling);

/* The latency every request at this interval carries until the latency steps:
   the one fixed by hand, else the one searched at -- within the spec and
   under the ceiling. */
uint16_t hl_search_latency_at(const struct hl_search_limits *limits, uint16_t interval);

void hl_search_start(struct hl_search *search, const struct hl_search_limits *limits);

/* The next request to send, or false once the search is over (the result is
   final then). Acts on the verdict told since the last call. */
bool hl_search_next(struct hl_search *search, struct hl_params *request);

/* The host's verdict on the last request: accepted or not. */
void hl_search_answer(struct hl_search *search, bool accepted);

/* The link moved onto the last request after it was judged refused, before
   the next one went out: the host answered late, and accepted. Returns
   whether that changed the verdict. */
bool hl_search_late_answer(struct hl_search *search);
