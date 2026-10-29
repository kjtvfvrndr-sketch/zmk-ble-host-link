/*
 * SPDX-License-Identifier: MIT
 *
 * The decisions around the search, as plain logic like search.c: what a host
 * taught the module (kept in flash), what a search run after the stored
 * result was refused changes, which drops of the link count against a
 * request and what a confirmed one changes, and whether ZMK Studio has the
 * link. No Zephyr in here, so tests/ can drive it through tables of cases.
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "search.h"

/* ---------------------------------------------- what a host taught */

/* One exact interval and its latency; interval 0: none. */
struct hl_result {
    uint16_t interval;
    uint16_t latency;
};

struct hl_learned {
    /* What is asked of the host on every connection. */
    struct hl_result stored;
    /* What a search found on a connection where the stored result was
       refused twice: it replaces the stored result once a search finds it
       again, and goes once the stored result is accepted. */
    struct hl_result candidate;
    /* Learned from drops (hl_blame_confirmed()): every latency stays below
       the ceiling -- latency + 1, 0 none -- and the ladder starts at the
       floor -- 0 none. */
    uint16_t latency_ceiling_plus1;
    uint16_t interval_floor;
};

bool hl_result_exists(const struct hl_result *result);
bool hl_same_result(const struct hl_result *a, const struct hl_result *b);
uint16_t hl_learned_latency_ceiling(const struct hl_learned *learned); /* HL_NO_LATENCY_CEILING */

/* The stored result was accepted: the candidate goes. Returns whether
   `learned` changed. */
bool hl_stored_result_accepted(struct hl_learned *learned);

/* A first search -- nothing stored -- ended; found: NULL if the host
   accepted nothing. Returns whether `learned` changed. */
bool hl_first_search_finished(struct hl_learned *learned, const struct hl_result *found);

enum hl_research_outcome {
    /* The host accepted nothing: nothing changes. */
    HL_RESEARCH_FOUND_NOTHING,
    /* Found the stored result itself: the host was only busy. The candidate
       goes. */
    HL_RESEARCH_FOUND_STORED,
    /* Found the candidate again: it is the stored result now. */
    HL_RESEARCH_CANDIDATE_STORED,
    /* Found something else: the new candidate, the stored result stays. */
    HL_RESEARCH_NEW_CANDIDATE,
};

/* A search run because the stored result was refused twice ended. */
enum hl_research_outcome hl_search_after_stored_refused(struct hl_learned *learned,
                                                        const struct hl_result *found);

/* ------------------------------------------------------------- drops */

/* This keyboard ended the link itself (0x16): never a drop to count. */
#define HL_DROP_REASON_LOCAL_HOST 0x16
/* "Unacceptable connection parameters" (0x3B): the parameters themselves,
   no need to wait for the host to return. */
#define HL_DROP_REASON_UNACCEPTABLE_PARAMS 0x3B

/* The link lost on air -- the only drops that show a link dropping by
   itself: supervision timeout (0x08), instant passed (0x28), MIC failure
   (0x3D), failed to establish (0x3E). A host that closes the link on purpose
   -- Bluetooth switched off (0x13), the adapter powered off (0x15) -- says
   nothing about the link. */
bool hl_drop_reason_is_loss_on_air(uint8_t reason);

/* The same request blamed for drops on this many connections in a row is
   confirmed. */
#define HL_DROPS_THAT_CONFIRM_BLAME 2

/* What drops have shown about a host since boot (kept out of flash). */
struct hl_drop_watch {
    /* The link drops by itself, with no request to blame, and the host comes
       straight back: drops say nothing about requests until a link stays
       up for the stable time. */
    bool unstable;

    /* A drop waiting for the host to come back. */
    bool pending;
    bool pending_blames_request;
    struct hl_result pending_request;
    int64_t pending_at_ms;

    /* The request blamed on the last connections, and on how many in a
       row. */
    struct hl_result suspect;
    uint8_t suspect_connections;
};

enum hl_drop_seen {
    /* 0x16, or nothing to watch: nothing to do. */
    HL_DROP_SEEN_IGNORED,
    /* No request to blame, lost on air: waits for the host -- straight back
       makes it unstable. */
    HL_DROP_SEEN_UNPROVOKED,
    /* No request to blame, and not lost on air -- the host closed it on
       purpose, say: nothing to do. */
    HL_DROP_SEEN_NOT_LOST_ON_AIR,
    /* A request to blame, but the host is unstable: not counted. */
    HL_DROP_SEEN_HOST_UNSTABLE,
    /* A request to blame: counts if the host comes straight back. */
    HL_DROP_SEEN_WAITS_FOR_HOST,
    /* 0x3B: counted now, once more needed. */
    HL_DROP_SEEN_SUSPECTED,
    /* 0x3B: counted now, and confirmed. */
    HL_DROP_SEEN_CONFIRMED,
};

/*
 * The link dropped. blamed: the request it followed (NULL none -- or one
 * fixed by hand, never blamed). link_up_ms: how long the link had been up; a
 * link up for stable_link_ms clears `unstable` first. On
 * HL_DROP_SEEN_CONFIRMED, *confirmed is the request.
 */
enum hl_drop_seen hl_drop_seen(struct hl_drop_watch *watch, uint8_t reason,
                               const struct hl_result *blamed, uint32_t link_up_ms,
                               uint32_t stable_link_ms, int64_t now_ms,
                               struct hl_result *confirmed);

enum hl_host_return {
    /* No drop waiting. */
    HL_RETURN_NOTHING_PENDING,
    /* Back late: it slept, or Bluetooth was off. Nothing counts. */
    HL_RETURN_LATE,
    /* Straight back after an unprovoked drop: unstable now. */
    HL_RETURN_UNSTABLE,
    /* Straight back: the request blamed once. */
    HL_RETURN_SUSPECTED,
    /* Straight back: the same request blamed on enough connections in a row
       -- *confirmed. */
    HL_RETURN_CONFIRMED,
};

/* The host connected again. */
enum hl_host_return hl_host_returned(struct hl_drop_watch *watch, int64_t now_ms,
                                     uint32_t within_ms, struct hl_result *confirmed);

/* A request held through its drop window: if it is the one blamed on the
   last connections, those blames are no longer in a row. Another request
   holding says nothing about it. */
void hl_request_held(struct hl_drop_watch *watch, const struct hl_result *held);

/* What a confirmed blame changed (bits, for the log). */
enum {
    HL_BLAME_LOWERS_LATENCY = 1u << 0,
    HL_BLAME_RAISES_FLOOR = 1u << 1,
    HL_BLAME_CLEARS_STORED = 1u << 2,
    HL_BLAME_CLEARS_CANDIDATE = 1u << 3,
};

/*
 * A confirmed blame. Above the latency the interval is searched at -- or at
 * the slowest interval, below which nothing is left -- the latency is what
 * the host cannot keep: a ceiling below it. Else the interval is: the ladder
 * starts above it. A stored result or candidate the new limits rule out
 * goes. Returns HL_BLAME_* bits.
 */
unsigned int hl_blame_confirmed(struct hl_learned *learned, const struct hl_result *blamed,
                                uint16_t latency_while_searching, uint16_t slowest_interval);

/* Which request a drop now is about. */
enum hl_drop_blame {
    /* None: nothing on air, nothing taken lately. */
    HL_DROP_BLAMES_NOTHING,
    /* The request out now, not answered yet. */
    HL_DROP_BLAMES_REQUEST_OUT,
    /* The request the link last moved onto. */
    HL_DROP_BLAMES_REQUEST_TAKEN,
};

/*
 * request_out: a request is on air, not answered yet. taken: the link moved
 * onto a request since_taken_ms ago. A link that moved onto a request breaks
 * over it only after the supervision timeout, hence a window that long and
 * more (within_ms).
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
