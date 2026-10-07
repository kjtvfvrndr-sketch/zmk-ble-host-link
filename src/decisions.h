/*
 * SPDX-License-Identifier: MIT
 *
 * The decisions around the search, as plain logic like search.c: what a host
 * taught the module (kept in flash), what to do when the stored result is
 * refused, what a finished search changes, which drops of the link count and
 * what they change, and whether ZMK Studio has the link. No Zephyr in here,
 * so tests/ can drive it through tables of cases.
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "search.h"

/* ---------------------------------------------- what a host taught */

enum {
    /* A search ran to its end on this host. */
    HL_LEARNED_SEARCHED = 1u << 0,
    /* ...and the host accepted `found`. */
    HL_LEARNED_FOUND = 1u << 1,
    /* The host refused the fastest interval: Apple's rules. */
    HL_LEARNED_FASTEST_REFUSED = 1u << 2,
    /* The link dropped after requests twice in a row: no more searching on
       this host until BLE_HOST_LINK_AUTO. */
    HL_LEARNED_NO_SEARCHING = 1u << 3,
    /* A search run because the stored result was refused found only
       something slower: `fallback`, asked for while the stored result is
       refused. */
    HL_LEARNED_HAS_FALLBACK = 1u << 4,
    /* Such a search ran to its end since the refusals began. */
    HL_LEARNED_SEARCHED_AGAIN = 1u << 5,
    /* The latency 0 of `found` is the host's, given to a request asked with
       latency (hl_no_worse_than()). */
    HL_LEARNED_FOUND_LATENCY_0_BY_HOST = 1u << 6,
    /* The same for `fallback`. */
    HL_LEARNED_FALLBACK_LATENCY_0_BY_HOST = 1u << 7,
};

/* What goes with `found` (cleared with HL_LEARNED_FOUND) and with `fallback`
   (cleared with HL_LEARNED_HAS_FALLBACK). */
#define HL_LEARNED_WITH_FOUND HL_LEARNED_FOUND_LATENCY_0_BY_HOST
#define HL_LEARNED_WITH_FALLBACK HL_LEARNED_FALLBACK_LATENCY_0_BY_HOST

struct hl_learned {
    uint8_t flags;
    /* Connections in a row on which the stored result was refused twice,
       up to HL_STORED_REFUSALS_THAT_REPLACE_IT. */
    uint8_t refused_connections;
    /* The link dropped after asking for this latency: latency + 1, 0 none. */
    uint16_t latency_ceiling_plus1;
    /* Fallback candidates already tried on this host (hl_fallback_candidate). */
    uint32_t fallback_candidates_tried;
    /* With HL_LEARNED_FOUND. */
    struct hl_params found;
    /* With HL_LEARNED_HAS_FALLBACK. */
    struct hl_params fallback;
    /* What the search that found the fallback learned about the host,
       applied only once the fallback replaces the stored result: a search
       run while the host was merely busy must not spoil the memory. */
    bool fallback_fastest_refused;
    uint32_t fallback_candidates_tried_then;
};

/* A profile whose host accepted nothing when searched: nothing is asked of
   it, its own parameters stay. Unless an interval is fixed by hand. */
bool hl_learned_asks_nothing(const struct hl_learned *learned);

/* A search result no worse than another: a faster interval, or the same at
   no less latency -- except that latency 0 the host gave (`*_latency_0_by_host`)
   is worse than anything else, and between two such the slower interval is
   no worse (as in the search). */
bool hl_no_worse_than(const struct hl_params *a, bool a_latency_0_by_host,
                      const struct hl_params *b, bool b_latency_0_by_host);

/* ------------------------------------------ the stored result refused */

/* Connections in a row with the stored result refused twice: at the first
   of these counts (or any later one, until such a search has ended) the host
   is searched again; at the second a fallback takes the stored result's
   place -- the host has changed -- or, without one, the stored result is
   dropped and the host keeps its own. */
#define HL_STORED_REFUSALS_THAT_SEARCH_AGAIN 2
#define HL_STORED_REFUSALS_THAT_REPLACE_IT 5

enum hl_stored_refusal_step {
    /* Ask for the fallback result. */
    HL_STORED_REFUSED_ASK_FALLBACK,
    /* Ask for the secure request. */
    HL_STORED_REFUSED_ASK_SECURE,
    /* Search the host again. */
    HL_STORED_REFUSED_SEARCH_AGAIN,
    /* The fallback is the stored result now: ask for it. */
    HL_STORED_REFUSED_FALLBACK_REPLACES_IT,
    /* The stored result is dropped: nothing is asked any more, the host
       keeps its own parameters. */
    HL_STORED_REFUSED_DROPPED,
};

/* The stored result was refused twice on a connection. first_on_connection:
   the first time on this connection, the one that counts. Updates
   `learned`; its caller writes it when changed. */
enum hl_stored_refusal_step hl_stored_result_refused(struct hl_learned *learned,
                                                     bool first_on_connection);

/* The stored result was accepted. Returns whether `learned` changed. */
bool hl_stored_result_accepted(struct hl_learned *learned);

/* --------------------------------------------- a search ran to its end */

enum hl_search_outcome {
    /* A first search (or one by hand): its result is the stored one. */
    HL_SEARCH_OUTCOME_STORED,
    /* Searched again, found no worse: the new stored result. */
    HL_SEARCH_OUTCOME_REPLACED_STORED,
    /* Searched again, found only slower: the fallback (what the search
       learned waits with it). */
    HL_SEARCH_OUTCOME_FALLBACK,
    /* Searched again, the host accepted nothing: nothing changes, the
       search's memory included -- a busy host must not spoil it. */
    HL_SEARCH_OUTCOME_NOTHING_CHANGED,
};

/* A finished search into `learned`. again: run because the stored result was
   refused. timeout: the one results are stored with and asked at -- a search
   run again is weighed against the stored result as it is asked at it. */
enum hl_search_outcome hl_search_finished(struct hl_learned *learned,
                                          const struct hl_search *search, bool again,
                                          uint16_t timeout);

/* ------------------------------------------------------------- drops */

/* The request a dropped link followed. */
struct hl_dropped_request {
    /* A stored result that took effect, or a search request. */
    bool stored_result;
    enum hl_request_kind kind;
    uint8_t candidate;
    struct hl_params params;
};

enum hl_drop_reason_verdict {
    /* This keyboard's own doing (0x16): never counts. */
    HL_DROP_NOT_THE_HOSTS,
    /* "Unacceptable connection parameters" (0x3B): counts at once. */
    HL_DROP_COUNTS_NOW,
    /* Anything else -- the host asleep, Bluetooth off, a supervision
       timeout, a missed instant: counts if the host comes straight back. */
    HL_DROP_COUNTS_IF_HOST_RETURNS,
};

enum hl_drop_reason_verdict hl_drop_reason(uint8_t reason);

/* A host back away_ms after a drop: awake and disliking the request if soon,
   asleep or switched off if not. */
bool hl_drop_counts_on_return(int64_t away_ms, uint32_t within_ms);

/* What a counted drop changed (bits, for the log). */
enum {
    HL_DROP_SKIPS_FASTEST = 1u << 0,
    HL_DROP_SKIPS_CANDIDATE = 1u << 1,
    HL_DROP_LOWERS_LATENCY = 1u << 2,
    HL_DROP_CLEARS_STORED = 1u << 3,
    HL_DROP_STOPS_SEARCHING = 1u << 4,
    HL_DROP_CLEARS_FALLBACK = 1u << 5,
};

/* Two counted drops in a row stop the searching. */
#define HL_DROPS_THAT_STOP_SEARCHING 2

/*
 * A counted drop: a search request's kind is skipped on this host from now
 * on (the fastest interval, a candidate, a latency and above); a stored
 * result that took effect -- or a search request that ran the link at its
 * interval (the minimum: a range and an exact request alike) at no more
 * latency, which the stored result is no safer than -- goes, its
 * latency a ceiling, so the next connection searches again. A search
 * request that is so the fallback takes the fallback with it, so the next
 * refused connection searches again too. strikes: counted drops in a row.
 * Returns HL_DROP_* bits.
 */
unsigned int hl_count_drop(struct hl_learned *learned, uint8_t *strikes,
                           const struct hl_dropped_request *request);

/* Which request a drop now is about. */
enum hl_drop_blame {
    /* None: refused, or long enough ago. */
    HL_DROP_BLAMES_NOTHING,
    /* The request out now, not answered yet. */
    HL_DROP_BLAMES_REQUEST_OUT,
    /* The request the link last moved onto. */
    HL_DROP_BLAMES_REQUEST_TAKEN,
};

/*
 * request_out: a judged request is on air, not answered yet. taken: the link
 * moved onto a request since_taken_ms ago. A link that moved onto a request
 * breaks over it only after the supervision timeout, hence a window that
 * long and more (within_ms).
 */
enum hl_drop_blame hl_drop_blame(bool request_out, bool taken, int64_t since_taken_ms,
                                 uint32_t within_ms);

/* --------------------------------------------------------- ZMK Studio */

/*
 * ZMK Studio has the link: its parameters, the host subscribed to its RPC
 * characteristic, and not merely the subscription that came back with the
 * bond at connect -- a subscription there within the first
 * restored_window_ms counts only once it went away and came back.
 */
bool hl_zmk_studio_has_link(bool studio_params, bool subscribed, bool subscribed_at_connect,
                            uint32_t link_up_ms, uint32_t restored_window_ms);
