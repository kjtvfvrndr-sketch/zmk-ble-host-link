/*
 * SPDX-License-Identifier: MIT
 *
 * The decisions around the search. See decisions.h, and README.md.
 */

#include "decisions.h"

#include <stddef.h>

/* `request` ran the link at the interval `result` does, at no more latency:
   if it broke the link, `result` is no safer. A request taken runs the link
   at its minimum, so a range and an exact request with the same minimum run
   it alike. */
static bool no_safer_than(const struct hl_params *result, const struct hl_params *request) {
    return result->interval_min == request->interval_min && request->latency <= result->latency;
}

/* The stored result as it is asked for at `timeout`: its latency within
   what the timeout allows (by the rules the host plays by) and under the
   latency ceiling -- as the glue's compose_request() asks for it. */
static struct hl_params stored_as_asked(const struct hl_learned *learned, uint16_t timeout) {
    struct hl_params p = learned->found;

    p.timeout = timeout;
    p.latency = hl_latency_allowed(p.latency, p.interval_max, timeout,
                                   learned->flags & HL_LEARNED_FASTEST_REFUSED);
    if (learned->latency_ceiling_plus1 != 0) {
        const uint16_t ceiling = learned->latency_ceiling_plus1 - 1;
        const uint16_t below = ceiling > 0 ? ceiling - 1 : 0;

        if (p.latency > below) {
            p.latency = below;
        }
    }
    return p;
}

bool hl_learned_asks_nothing(const struct hl_learned *learned) {
    return (learned->flags & HL_LEARNED_SEARCHED) && !(learned->flags & HL_LEARNED_FOUND);
}

bool hl_no_worse_than(const struct hl_params *a, bool a_latency_0_by_host,
                      const struct hl_params *b, bool b_latency_0_by_host) {
    const bool a_zero = a_latency_0_by_host && a->latency == 0;
    const bool b_zero = b_latency_0_by_host && b->latency == 0;

    /* The host's latency 0 loses to anything else; between two such the
       slower interval wins (as in the search: beats_best()). */
    if (a_zero != b_zero) {
        return b_zero;
    }
    if (a_zero) {
        return a->interval_min >= b->interval_min;
    }
    return a->interval_min < b->interval_min ||
           (a->interval_min == b->interval_min && a->latency >= b->latency);
}

/* ------------------------------------------ the stored result refused */

enum hl_stored_refusal_step hl_stored_result_refused(struct hl_learned *learned,
                                                     bool first_on_connection) {
    const bool has_fallback = learned->flags & HL_LEARNED_HAS_FALLBACK;
    const bool searched_again = learned->flags & HL_LEARNED_SEARCHED_AGAIN;
    const bool no_searching = learned->flags & HL_LEARNED_NO_SEARCHING;

    if (first_on_connection &&
        learned->refused_connections < HL_STORED_REFUSALS_THAT_REPLACE_IT) {
        learned->refused_connections++;
    }

    const uint8_t refusals = learned->refused_connections;

    if (first_on_connection && refusals >= HL_STORED_REFUSALS_THAT_REPLACE_IT) {
        if (has_fallback) {
            /* Refused this long: the host has changed -- and what the search
               that found the fallback learned holds now too. */
            learned->found = learned->fallback;
            learned->flags &= ~(HL_LEARNED_FASTEST_REFUSED | HL_LEARNED_FOUND_LATENCY_0_BY_HOST);
            if (learned->flags & HL_LEARNED_FALLBACK_LATENCY_0_BY_HOST) {
                learned->flags |= HL_LEARNED_FOUND_LATENCY_0_BY_HOST;
            }
            if (learned->fallback_fastest_refused) {
                learned->flags |= HL_LEARNED_FASTEST_REFUSED;
            }
            learned->fallback_candidates_tried = learned->fallback_candidates_tried_then;
            learned->flags &=
                ~(HL_LEARNED_HAS_FALLBACK | HL_LEARNED_WITH_FALLBACK | HL_LEARNED_SEARCHED_AGAIN);
            learned->flags |= HL_LEARNED_FOUND;
            learned->refused_connections = 0;
            return HL_STORED_REFUSED_FALLBACK_REPLACES_IT;
        }
        if (searched_again || no_searching) {
            /* Searched again and the host took nothing, or no searching
               allowed: the host keeps its own. */
            learned->flags &= ~(HL_LEARNED_FOUND | HL_LEARNED_WITH_FOUND | HL_LEARNED_SEARCHED_AGAIN);
            learned->flags |= HL_LEARNED_SEARCHED;
            learned->refused_connections = 0;
            return HL_STORED_REFUSED_DROPPED;
        }
        return HL_STORED_REFUSED_SEARCH_AGAIN;
    }
    if (first_on_connection && refusals >= HL_STORED_REFUSALS_THAT_SEARCH_AGAIN && !has_fallback &&
        !searched_again && !no_searching) {
        return HL_STORED_REFUSED_SEARCH_AGAIN;
    }
    return has_fallback ? HL_STORED_REFUSED_ASK_FALLBACK : HL_STORED_REFUSED_ASK_SECURE;
}

bool hl_stored_result_accepted(struct hl_learned *learned) {
    if (learned->refused_connections == 0 && !(learned->flags & HL_LEARNED_SEARCHED_AGAIN)) {
        return false;
    }
    learned->refused_connections = 0;
    learned->flags &= ~HL_LEARNED_SEARCHED_AGAIN;
    return true;
}

/* --------------------------------------------- a search ran to its end */

static void set_found_latency_0_by_host(struct hl_learned *learned, bool by_host) {
    learned->flags &= ~HL_LEARNED_FOUND_LATENCY_0_BY_HOST;
    if (by_host) {
        learned->flags |= HL_LEARNED_FOUND_LATENCY_0_BY_HOST;
    }
}

static void keep_search_memory(struct hl_learned *learned, const struct hl_search *search) {
    learned->flags &= ~HL_LEARNED_FASTEST_REFUSED;
    if (search->memory.fastest_refused) {
        learned->flags |= HL_LEARNED_FASTEST_REFUSED;
    }
    learned->fallback_candidates_tried = search->memory.fallback_candidates_tried;
}

enum hl_search_outcome hl_search_finished(struct hl_learned *learned,
                                          const struct hl_search *search, bool again,
                                          uint16_t timeout) {
    struct hl_params best = search->best;

    best.timeout = timeout;

    if (again && (learned->flags & HL_LEARNED_FOUND)) {
        if (!search->have_best) {
            learned->flags |= HL_LEARNED_SEARCHED_AGAIN;
            return HL_SEARCH_OUTCOME_NOTHING_CHANGED;
        }
        const struct hl_params stored = stored_as_asked(learned, timeout);
        const bool stored_latency_0_by_host =
            (learned->flags & HL_LEARNED_FOUND_LATENCY_0_BY_HOST) && learned->found.latency == 0;

        if (hl_no_worse_than(&best, search->best_latency_0_by_host, &stored,
                             stored_latency_0_by_host)) {
            keep_search_memory(learned, search);
            learned->found = best;
            set_found_latency_0_by_host(learned, search->best_latency_0_by_host);
            learned->flags &= ~(HL_LEARNED_HAS_FALLBACK | HL_LEARNED_SEARCHED_AGAIN);
            learned->refused_connections = 0;
            return HL_SEARCH_OUTCOME_REPLACED_STORED;
        }
        learned->fallback = best;
        learned->flags &= ~HL_LEARNED_FALLBACK_LATENCY_0_BY_HOST;
        if (search->best_latency_0_by_host) {
            learned->flags |= HL_LEARNED_FALLBACK_LATENCY_0_BY_HOST;
        }
        learned->fallback_fastest_refused = search->memory.fastest_refused;
        learned->fallback_candidates_tried_then = search->memory.fallback_candidates_tried;
        learned->flags |= HL_LEARNED_HAS_FALLBACK | HL_LEARNED_SEARCHED_AGAIN;
        return HL_SEARCH_OUTCOME_FALLBACK;
    }

    keep_search_memory(learned, search);
    learned->flags |= HL_LEARNED_SEARCHED;
    learned->flags &= ~(HL_LEARNED_FOUND | HL_LEARNED_WITH_FOUND | HL_LEARNED_HAS_FALLBACK |
                        HL_LEARNED_WITH_FALLBACK | HL_LEARNED_SEARCHED_AGAIN);
    learned->refused_connections = 0;
    if (search->have_best) {
        learned->flags |= HL_LEARNED_FOUND;
        learned->found = best;
        set_found_latency_0_by_host(learned, search->best_latency_0_by_host);
    }
    return HL_SEARCH_OUTCOME_STORED;
}

/* ------------------------------------------------------------- drops */

/* HCI reasons (Bluetooth core spec, vol 1 part F). */
#define REASON_LOCAL_HOST_TERMINATED 0x16
#define REASON_UNACCEPTABLE_CONN_PARAMS 0x3B

enum hl_drop_reason_verdict hl_drop_reason(uint8_t reason) {
    switch (reason) {
    case REASON_LOCAL_HOST_TERMINATED:
        return HL_DROP_NOT_THE_HOSTS;
    case REASON_UNACCEPTABLE_CONN_PARAMS:
        return HL_DROP_COUNTS_NOW;
    default:
        return HL_DROP_COUNTS_IF_HOST_RETURNS;
    }
}

bool hl_drop_counts_on_return(int64_t away_ms, uint32_t within_ms) {
    return away_ms >= 0 && away_ms <= (int64_t)within_ms;
}

static bool lower_latency_ceiling(struct hl_learned *learned, uint16_t latency) {
    const uint16_t plus1 = latency + 1;

    if (learned->latency_ceiling_plus1 != 0 && plus1 >= learned->latency_ceiling_plus1) {
        return false;
    }
    learned->latency_ceiling_plus1 = plus1;
    return true;
}

unsigned int hl_count_drop(struct hl_learned *learned, uint8_t *strikes,
                           const struct hl_dropped_request *request) {
    unsigned int changed = 0;
    const bool stored = request->stored_result ||
                        ((learned->flags & HL_LEARNED_FOUND) &&
                         no_safer_than(&learned->found, &request->params));
    const bool fallback = !request->stored_result && (learned->flags & HL_LEARNED_HAS_FALLBACK) &&
                          no_safer_than(&learned->fallback, &request->params);

    if (*strikes < UINT8_MAX) {
        (*strikes)++;
    }

    if (request->stored_result) {
        if (lower_latency_ceiling(learned, request->params.latency)) {
            changed |= HL_DROP_LOWERS_LATENCY;
        }
    } else {
        switch (request->kind) {
        case HL_ASK_FASTEST:
            learned->flags |= HL_LEARNED_FASTEST_REFUSED;
            changed |= HL_DROP_SKIPS_FASTEST;
            break;
        case HL_ASK_FALLBACK_CANDIDATE:
            if (request->candidate < 32) {
                learned->fallback_candidates_tried |= 1u << request->candidate;
                changed |= HL_DROP_SKIPS_CANDIDATE;
            }
            break;
        case HL_ASK_LATENCY:
            if (lower_latency_ceiling(learned, request->params.latency)) {
                changed |= HL_DROP_LOWERS_LATENCY;
            }
            break;
        default:
            break;
        }
    }

    /* The stored result itself broke the link: it goes, and the next
       connection searches again with what this taught. */
    if (stored && (learned->flags & HL_LEARNED_FOUND)) {
        learned->flags &= ~(HL_LEARNED_SEARCHED | HL_LEARNED_FOUND | HL_LEARNED_WITH_FOUND |
                            HL_LEARNED_HAS_FALLBACK | HL_LEARNED_WITH_FALLBACK |
                            HL_LEARNED_SEARCHED_AGAIN);
        learned->refused_connections = 0;
        changed |= HL_DROP_CLEARS_STORED;
    }

    /* The fallback broke the link: it goes, and the next refused connection
       searches again -- with what this drop taught. */
    if (fallback && (learned->flags & HL_LEARNED_HAS_FALLBACK)) {
        learned->flags &=
            ~(HL_LEARNED_HAS_FALLBACK | HL_LEARNED_WITH_FALLBACK | HL_LEARNED_SEARCHED_AGAIN);
        changed |= HL_DROP_CLEARS_FALLBACK;
    }

    if (*strikes >= HL_DROPS_THAT_STOP_SEARCHING) {
        learned->flags |= HL_LEARNED_NO_SEARCHING;
        changed |= HL_DROP_STOPS_SEARCHING;
    }
    return changed;
}

enum hl_drop_blame hl_drop_blame(bool request_out, bool taken, int64_t since_taken_ms,
                                 uint32_t within_ms) {
    /* The link runs the request it moved onto: that one first. */
    if (taken && since_taken_ms >= 0 && since_taken_ms < (int64_t)within_ms) {
        return HL_DROP_BLAMES_REQUEST_TAKEN;
    }
    return request_out ? HL_DROP_BLAMES_REQUEST_OUT : HL_DROP_BLAMES_NOTHING;
}

/* --------------------------------------------------------- ZMK Studio */

bool hl_zmk_studio_has_link(bool studio_params, bool subscribed, bool subscribed_at_connect,
                            uint32_t link_up_ms, uint32_t restored_window_ms) {
    return studio_params && subscribed && !subscribed_at_connect &&
           link_up_ms >= restored_window_ms;
}
