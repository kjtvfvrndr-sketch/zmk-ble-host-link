/*
 * SPDX-License-Identifier: MIT
 *
 * The automatic search for the host link's parameters, as plain logic: it
 * names the next request, and is told what the link runs once the host has
 * answered. No Zephyr in here, so tests/ can drive it against simulated hosts.
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
/* Apple Accessory Design Guidelines (QA1931). */
#define HL_APPLE_MAX_LATENCY 30
/* An interval range must be at least 15 ms wide unless it is exactly 15-15. */
#define HL_APPLE_RANGE_WIDTH 12
/* The HID exception: "down to 11.25 ms may be accepted". */
#define HL_APPLE_HID_FASTEST_INTERVAL 9

/* Raising the latency stops after this many requests. */
#define HL_SEARCH_LATENCY_STEPS 3
/* The host moving the link off the best on its own this many times in a
   row -- with no search request answered in between (taken, refused with
   the link left as it was, or answered late; silence is no answer) -- ends
   the search there: it keeps imposing its own parameters (a power manager,
   say), and putting the link back would only go round and round. Moves with
   the search getting on between them (the host changing its mind now and
   then) do not add up: the requests a search may send bound those. */
#define HL_SEARCH_HOST_MOVES_TOLERATED 2
/* Moves off the best taken for not the host's (ZMK Studio's request) in one
   search; past these, they count as the host's: the search ends whatever
   the world around it does. */
#define HL_SEARCH_MOVES_NOT_BY_HOST_TAKEN 3

/* What a request asks for. */
struct hl_params {
    uint16_t interval_min;
    uint16_t interval_max;
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
/* No fallback candidate is protected (hl_search_memory). */
#define HL_NO_FALLBACK_CANDIDATE 0xFF
/* No latency ceiling learned (hl_search_memory). */
#define HL_NO_LATENCY_CEILING 0xFFFF

struct hl_search_limits {
    /* The fastest interval worth asking for. */
    uint16_t fastest_interval;
    /* The slowest interval the search counts as a success (15 ms with a
     * pointer). Also the "secure" request every search starts with. */
    uint16_t slowest_interval;
    /* The latency the interval is searched at: one every host accepts. */
    uint16_t latency_while_searching_interval;
    /* The latency search aims for interval * (latency + 1) near this. */
    uint16_t idle_listen_period_ms;
    uint16_t timeout;
    /* Requests per search, restores of a moved link not counted. */
    uint8_t max_requests;
    /* A latency fixed by hand, or HL_LATENCY_NOT_PINNED. */
    uint16_t pinned_latency;
};

/* What a search learns about a host and keeps for the next one. */
struct hl_search_memory {
    /* The host refused the fastest interval asked exactly. It is taken to play
     * by Apple's rules: the next search goes straight to the fallback
     * candidates, and the latency stays within Apple's limits. */
    bool fastest_refused;
    /* One bit per fallback candidate already tried (hl_fallback_candidate). */
    uint32_t fallback_candidates_tried;
    /* The candidate the stored result came from: a refusal of it does not
     * mark it tried -- a passing refusal must not lose a result the host
     * gave before. */
    uint8_t protected_fallback_candidate;
    /* Likewise when the stored result is the fastest interval: a refusal of
     * it does not set fastest_refused. */
    bool fastest_protected;
    /* The link dropped after asking for this much latency: ask below it. */
    uint16_t latency_ceiling;
};

enum hl_search_phase {
    HL_PHASE_SECURE,
    HL_PHASE_FASTEST,
    HL_PHASE_FALLBACK_CANDIDATES,
    HL_PHASE_LATENCY,
    HL_PHASE_DONE,
};

enum hl_request_kind {
    HL_ASK_SECURE,
    HL_ASK_FASTEST,
    HL_ASK_FALLBACK_CANDIDATE,
    HL_ASK_LATENCY,
    HL_ASK_RESTORE,
};

struct hl_search {
    struct hl_search_limits limits;
    struct hl_search_memory memory;
    enum hl_search_phase phase;

    /* The best request the host accepted so far, and the interval the link
     * got for it -- always the request's minimum (hl_link_matches). The link
     * is kept on it between requests. */
    bool have_best;
    struct hl_params best;
    uint16_t best_interval;
    /* The best's latency 0 is the host's: given to a request asked with
       latency (beats_best()). */
    bool best_latency_0_by_host;
    /* The same for the request just answered (hl_search_answer()). */
    bool asked_latency_0_by_host;

    /* The request out now. */
    struct hl_params asked;
    enum hl_request_kind asked_kind;
    uint8_t asked_candidate;

    /* The link before the last answer: a link that did not move was not
     * answered at all. */
    struct hl_link link_before;

    /* The last request the host left unanswered: silent past the deadline,
     * or the link moved somewhere else entirely (another request still
     * queued in the controller, say). A host can answer late -- the link
     * moves to it while the next request is out -- and then that is the
     * answer, not a refusal of the next one. */
    bool silent_pending;
    struct hl_params silent_asked;
    enum hl_request_kind silent_kind;
    uint8_t silent_candidate;
    /* Whether that silence was what set fastest_refused. */
    bool silent_set_fastest_refused;
    /* Whether it marked its candidate tried. */
    bool silent_marked_candidate;

    /* The request the link last moved onto -- accepted, or answered late --
     * and a count that changes with every such move: what a drop of the
     * link soon after is about. */
    uint8_t taken_count;
    struct hl_params taken;
    enum hl_request_kind taken_kind;
    uint8_t taken_candidate;

    /* The link left the best request: the next request puts it back. */
    bool restoring;
    /* That did not work either; the search stopped where the link was. */
    bool restore_failed;
    /* The host moved the link off the best on its own, so many times in a
       row: an answered search request starts the count over. */
    uint8_t host_moves_in_a_row;
    /* ...often enough to end the search (HL_SEARCH_HOST_MOVES_TOLERATED). */
    bool host_keeps_moving;
    /* Moves off the best taken for not the host's, so many in this search. */
    uint8_t moves_not_by_host;
    /* The host answered a request asked with latency at latency 0: the
       search ends once the link is on the best (beats_best() prefers the
       slower interval at 0). */
    bool host_forces_latency_0;

    uint8_t next_candidate;

    bool latency_started;
    uint16_t latency_accepted, latency_highest;
    uint8_t latency_steps;

    uint8_t requests;
};

/*
 * Whether the link runs what the request asked for: the interval equal to the
 * request's minimum, and the latency exactly.
 *
 * The minimum, not anywhere in the range: a range is only ever asked to meet
 * Apple's width rule, and its minimum is the interval wanted. A host that
 * picks higher in it has not given what was asked for.
 *
 * The latency too, because a refusal leaves the link on its previous
 * parameters, which may share the interval -- macOS keeps 15 ms at latency 0.
 *
 * Not the timeout: a host may set its own, and every timeout a link runs with
 * already meets the spec for its latency, which is all the timeout is for.
 */
bool hl_link_matches(const struct hl_params *asked, const struct hl_link *link);

/*
 * The timeout to send a request with. Zephyr's bt_conn_le_param_update()
 * sends nothing when the link already satisfies the request in its own sense
 * -- interval anywhere in the range, latency and timeout equal -- and returns
 * -EALREADY. When the link does not match in ours (its interval is not the
 * minimum), the request goes out with the timeout one step (10 ms) away, so
 * that the host has to answer it.
 */
uint16_t hl_timeout_that_reaches_the_host(const struct hl_params *request,
                                          const struct hl_link *link);

/* The largest latency at most `wanted` that the rules allow at this slowest
 * interval: the core spec always, Apple's guidelines when apple_rules. */
uint16_t hl_latency_allowed(uint16_t wanted, uint16_t interval_max, uint16_t timeout,
                            bool apple_rules);

/* The latency that makes interval * (latency + 1) come nearest the idle
 * listen period without exceeding it, within the rules. */
uint16_t hl_latency_for_idle_listen_period(uint16_t interval, uint16_t idle_listen_period_ms,
                                           uint16_t timeout, bool apple_rules);

/*
 * The fallback candidates, tried once the fastest interval is refused, in
 * order: the Apple-rule ranges (15 ms wide, from 11.25 ms up), then the exact
 * intervals between the fastest and the slowest. Returns false past the last.
 */
uint8_t hl_fallback_candidate_count(const struct hl_search_limits *limits);
bool hl_fallback_candidate(const struct hl_search_limits *limits, uint8_t index,
                           uint16_t *interval_min, uint16_t *interval_max);
/* The candidate with exactly this range, or HL_NO_FALLBACK_CANDIDATE. */
uint8_t hl_fallback_candidate_index(const struct hl_search_limits *limits, uint16_t interval_min,
                                    uint16_t interval_max);

/* `opened`: what the link runs as the search starts. */
void hl_search_start(struct hl_search *search, const struct hl_search_limits *limits,
                     const struct hl_search_memory *memory, const struct hl_link *opened);

/* The next request to send, or false once the search is over. */
bool hl_search_next(struct hl_search *search, struct hl_params *request);

/*
 * What the link runs after the host answered the last request, or after it
 * stayed silent past the answer deadline (host_silent), which counts as a
 * refusal until the host answers it late.
 *
 * A host that gives the interval but lowers the latency (Android and BlueZ
 * may) has accepted the interval: the request is taken with the latency the
 * host chose -- if the link moved; a link that did not move was refused.
 */
void hl_search_answer(struct hl_search *search, const struct hl_link *link, bool host_silent);

/*
 * The link moved while no request was out -- between two requests of the
 * search. The late answer to a request the host left silent is taken as
 * such (only the request exactly); any other move off
 * the best request makes the search put it back first, and counts towards
 * HL_SEARCH_HOST_MOVES_TOLERATED.
 */
void hl_search_link_moved(struct hl_search *search, const struct hl_link *link);

/* The same for a move that is not the host's doing (ZMK Studio's request at
   connect, say): put back, not counted against the host -- up to
   HL_SEARCH_MOVES_NOT_BY_HOST_TAKEN in a search, the host's after that. */
void hl_search_link_moved_not_by_host(struct hl_search *search, const struct hl_link *link);

/*
 * The request hl_search_next() gave was never sent (dropped while it waited
 * to go out, the search having got on): its step is the search's again --
 * the request limit, a fallback candidate's turn, a latency step. The next
 * hl_search_next() puts the link back first if it must, then asks it again
 * if it is still worth asking.
 */
void hl_search_request_not_sent(struct hl_search *search);
