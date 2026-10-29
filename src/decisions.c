/*
 * SPDX-License-Identifier: MIT
 */

#include <string.h>

#include "decisions.h"

/* ---------------------------------------------- what a host taught */

bool hl_result_exists(const struct hl_result *result) { return result->interval != 0; }

bool hl_same_result(const struct hl_result *a, const struct hl_result *b) {
    return a->interval == b->interval && a->latency == b->latency;
}

uint16_t hl_learned_latency_ceiling(const struct hl_learned *learned) {
    return learned->latency_ceiling_plus1 == 0 ? HL_NO_LATENCY_CEILING
                                               : learned->latency_ceiling_plus1 - 1;
}

static void clear_result(struct hl_result *result) { memset(result, 0, sizeof(*result)); }

bool hl_stored_result_accepted(struct hl_learned *learned) {
    if (!hl_result_exists(&learned->candidate)) {
        return false;
    }
    clear_result(&learned->candidate);
    return true;
}

bool hl_first_search_finished(struct hl_learned *learned, const struct hl_result *found) {
    if (found == NULL || !hl_result_exists(found)) {
        return false;
    }
    learned->stored = *found;
    clear_result(&learned->candidate);
    return true;
}

enum hl_research_outcome hl_search_after_stored_refused(struct hl_learned *learned,
                                                        const struct hl_result *found) {
    if (found == NULL || !hl_result_exists(found)) {
        return HL_RESEARCH_FOUND_NOTHING;
    }
    if (hl_same_result(found, &learned->stored)) {
        clear_result(&learned->candidate);
        return HL_RESEARCH_FOUND_STORED;
    }
    if (hl_result_exists(&learned->candidate) && hl_same_result(found, &learned->candidate)) {
        learned->stored = *found;
        clear_result(&learned->candidate);
        return HL_RESEARCH_CANDIDATE_STORED;
    }
    learned->candidate = *found;
    return HL_RESEARCH_NEW_CANDIDATE;
}

/* ------------------------------------------------------------- drops */

bool hl_drop_reason_is_loss_on_air(uint8_t reason) {
    switch (reason) {
    case 0x08: /* connection timeout */
    case 0x28: /* instant passed */
    case 0x3D: /* terminated due to MIC failure */
    case 0x3E: /* failed to be established */
        return true;
    default:
        return false;
    }
}

static bool counts_on_return(int64_t away_ms, uint32_t within_ms) {
    return away_ms >= 0 && away_ms <= (int64_t)within_ms;
}

/* One more connection the request was blamed on. Returns whether that
   confirms it (*confirmed then). */
static bool blame_once_more(struct hl_drop_watch *watch, const struct hl_result *request,
                            struct hl_result *confirmed) {
    if (watch->suspect_connections > 0 && hl_same_result(&watch->suspect, request)) {
        watch->suspect_connections++;
    } else {
        watch->suspect = *request;
        watch->suspect_connections = 1;
    }
    if (watch->suspect_connections < HL_DROPS_THAT_CONFIRM_BLAME) {
        return false;
    }
    *confirmed = watch->suspect;
    watch->suspect_connections = 0;
    clear_result(&watch->suspect);
    return true;
}

enum hl_drop_seen hl_drop_seen(struct hl_drop_watch *watch, uint8_t reason,
                               const struct hl_result *blamed, uint32_t link_up_ms,
                               uint32_t stable_link_ms, int64_t now_ms,
                               struct hl_result *confirmed) {
    if (reason == HL_DROP_REASON_LOCAL_HOST) {
        return HL_DROP_SEEN_IGNORED;
    }
    if (link_up_ms >= stable_link_ms) {
        watch->unstable = false;
    }

    watch->pending = false;
    if (blamed == NULL || !hl_result_exists(blamed)) {
        if (!hl_drop_reason_is_loss_on_air(reason)) {
            return HL_DROP_SEEN_NOT_LOST_ON_AIR;
        }
        watch->pending = true;
        watch->pending_blames_request = false;
        watch->pending_at_ms = now_ms;
        return HL_DROP_SEEN_UNPROVOKED;
    }
    if (watch->unstable) {
        return HL_DROP_SEEN_HOST_UNSTABLE;
    }
    if (reason == HL_DROP_REASON_UNACCEPTABLE_PARAMS) {
        return blame_once_more(watch, blamed, confirmed) ? HL_DROP_SEEN_CONFIRMED
                                                          : HL_DROP_SEEN_SUSPECTED;
    }
    watch->pending = true;
    watch->pending_blames_request = true;
    watch->pending_request = *blamed;
    watch->pending_at_ms = now_ms;
    return HL_DROP_SEEN_WAITS_FOR_HOST;
}

enum hl_host_return hl_host_returned(struct hl_drop_watch *watch, int64_t now_ms,
                                     uint32_t within_ms, struct hl_result *confirmed) {
    if (!watch->pending) {
        return HL_RETURN_NOTHING_PENDING;
    }
    watch->pending = false;
    if (!counts_on_return(now_ms - watch->pending_at_ms, within_ms)) {
        return HL_RETURN_LATE;
    }
    if (!watch->pending_blames_request) {
        watch->unstable = true;
        return HL_RETURN_UNSTABLE;
    }
    return blame_once_more(watch, &watch->pending_request, confirmed) ? HL_RETURN_CONFIRMED
                                                                      : HL_RETURN_SUSPECTED;
}

void hl_request_held(struct hl_drop_watch *watch, const struct hl_result *held) {
    if (watch->suspect_connections == 0 || !hl_same_result(&watch->suspect, held)) {
        return;
    }
    watch->suspect_connections = 0;
    clear_result(&watch->suspect);
}

/* A result the limits learned from drops rule out. */
static bool ruled_out(const struct hl_learned *learned, const struct hl_result *result) {
    const uint16_t ceiling = hl_learned_latency_ceiling(learned);

    return hl_result_exists(result) &&
           (result->interval < learned->interval_floor ||
            (ceiling != HL_NO_LATENCY_CEILING && result->latency >= ceiling));
}

unsigned int hl_blame_confirmed(struct hl_learned *learned, const struct hl_result *blamed,
                                uint16_t latency_while_searching, uint16_t slowest_interval) {
    unsigned int changed = 0;

    if (blamed->latency > latency_while_searching || blamed->interval >= slowest_interval) {
        const uint16_t plus1 = blamed->latency + 1;

        if (learned->latency_ceiling_plus1 == 0 || plus1 < learned->latency_ceiling_plus1) {
            learned->latency_ceiling_plus1 = plus1;
            changed |= HL_BLAME_LOWERS_LATENCY;
        }
    } else if (blamed->interval + 1 > learned->interval_floor) {
        learned->interval_floor = blamed->interval + 1;
        changed |= HL_BLAME_RAISES_FLOOR;
    }

    if (ruled_out(learned, &learned->stored)) {
        clear_result(&learned->stored);
        changed |= HL_BLAME_CLEARS_STORED;
    }
    if (ruled_out(learned, &learned->candidate)) {
        clear_result(&learned->candidate);
        changed |= HL_BLAME_CLEARS_CANDIDATE;
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
