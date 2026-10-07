/*
 * SPDX-License-Identifier: MIT
 *
 * The automatic search for the host link's parameters. See search.h, and
 * README.md, "How the search works".
 *
 * The order is what keeps it safe. A host that refuses a request leaves the
 * link on the parameters it already runs, so the search first secures one
 * that every host accepts -- the slowest acceptable interval, exactly, at a
 * latency every host takes -- and only then asks for anything faster. A
 * refusal after that costs nothing. A request the host accepts with a pick
 * that is no good (an Apple-rule range, answered above its minimum) does move
 * the link, and the search puts it back before going on.
 *
 * Every interval is asked for exactly, except where Apple's rules demand a
 * range: a host given a range may pick anywhere in it -- Windows takes the
 * slow end -- so a range would say nothing about how fast it can go.
 */

#include "search.h"

#include <stddef.h>

#define MIN(a, b) ((a) < (b) ? (a) : (b))
#define MAX(a, b) ((a) > (b) ? (a) : (b))

bool hl_link_matches(const struct hl_params *asked, const struct hl_link *link) {
    return link->interval == asked->interval_min && link->latency == asked->latency;
}

uint16_t hl_timeout_that_reaches_the_host(const struct hl_params *request,
                                          const struct hl_link *link) {
    const bool zephyr_skips = link->interval >= request->interval_min &&
                              link->interval <= request->interval_max &&
                              link->latency == request->latency &&
                              link->timeout == request->timeout;

    if (!zephyr_skips || hl_link_matches(request, link)) {
        return request->timeout;
    }
    /* Apple allows 2..6 s; the spec far more. Step inward from the edge. */
    return request->timeout < 600 ? request->timeout + 1 : request->timeout - 1;
}

/* Largest latency L with (L + 1) <= limit, or 0. */
static uint32_t below(uint32_t limit) { return limit > 0 ? limit - 1 : 0; }

uint16_t hl_latency_allowed(uint16_t wanted, uint16_t interval_max, uint16_t timeout,
                            bool apple_rules) {
    uint32_t latency = MIN(wanted, HL_SPEC_MAX_LATENCY);

    if (interval_max == 0) {
        return 0;
    }

    /* Core spec: timeout * 4 > (1 + latency) * interval_max, in 10 ms and
       1.25 ms units -- the timeout outlasts two listening periods. */
    latency = MIN(latency, below((4u * timeout - 1u) / interval_max));

    if (apple_rules) {
        latency = MIN(latency, HL_APPLE_MAX_LATENCY);
        /* interval_max * (latency + 1) <= 2 s */
        latency = MIN(latency, below(1600u / interval_max));
        /* interval_max * (latency + 1) * 3 < timeout */
        latency = MIN(latency, below((40u * timeout - 1u) / (15u * interval_max)));
    }

    return (uint16_t)latency;
}

uint16_t hl_latency_for_idle_listen_period(uint16_t interval, uint16_t idle_listen_period_ms,
                                           uint16_t timeout, bool apple_rules) {
    if (interval == 0) {
        return 0;
    }

    /* interval * 1.25 ms * (latency + 1) <= period */
    const uint32_t listens = (4u * idle_listen_period_ms) / (5u * interval);

    return hl_latency_allowed((uint16_t)MIN(below(listens), 0xFFFFu), interval, timeout,
                              apple_rules);
}

/* ------------------------------------------------- fallback candidates */

static uint16_t first_range_interval(const struct hl_search_limits *limits) {
    return MAX(HL_APPLE_HID_FASTEST_INTERVAL, limits->fastest_interval);
}

/* At most 16 of each kind, so that both fit one 32-bit mask. */
static uint8_t range_count(const struct hl_search_limits *limits) {
    const uint16_t first = first_range_interval(limits);

    return limits->slowest_interval <= first ? 0
                                             : (uint8_t)MIN(limits->slowest_interval - first, 16);
}

static uint8_t exact_count(const struct hl_search_limits *limits) {
    const uint16_t first = limits->fastest_interval + 1;

    return limits->slowest_interval <= first ? 0
                                             : (uint8_t)MIN(limits->slowest_interval - first, 16);
}

uint8_t hl_fallback_candidate_count(const struct hl_search_limits *limits) {
    return (uint8_t)(range_count(limits) + exact_count(limits));
}

bool hl_fallback_candidate(const struct hl_search_limits *limits, uint8_t index,
                           uint16_t *interval_min, uint16_t *interval_max) {
    const uint8_t ranges = range_count(limits);

    if (index < ranges) {
        *interval_min = first_range_interval(limits) + index;
        *interval_max = *interval_min + HL_APPLE_RANGE_WIDTH;
        return true;
    }
    if (index < ranges + exact_count(limits)) {
        *interval_min = limits->fastest_interval + 1 + (index - ranges);
        *interval_max = *interval_min;
        return true;
    }
    return false;
}

uint8_t hl_fallback_candidate_index(const struct hl_search_limits *limits, uint16_t interval_min,
                                    uint16_t interval_max) {
    const uint8_t count = hl_fallback_candidate_count(limits);

    for (uint8_t i = 0; i < count; i++) {
        uint16_t min, max;

        if (hl_fallback_candidate(limits, i, &min, &max) && min == interval_min &&
            max == interval_max) {
            return i;
        }
    }
    return HL_NO_FALLBACK_CANDIDATE;
}

/* ----------------------------------------------------------- the search */

void hl_search_start(struct hl_search *search, const struct hl_search_limits *limits,
                     const struct hl_search_memory *memory, const struct hl_link *opened) {
    *search = (struct hl_search){
        .limits = *limits,
        .memory = *memory,
        .phase = HL_PHASE_SECURE,
        .link_before = *opened,
    };
}

static bool latency_pinned(const struct hl_search *search) {
    return search->limits.pinned_latency != HL_LATENCY_NOT_PINNED;
}

/* Below a latency the link dropped at, if one was learned. */
static uint16_t under_ceiling(const struct hl_search *search, uint16_t latency) {
    const uint16_t ceiling = search->memory.latency_ceiling;

    if (latency_pinned(search) || ceiling == HL_NO_LATENCY_CEILING) {
        return latency;
    }
    return MIN(latency, ceiling > 0 ? ceiling - 1 : 0);
}

/* The latency an interval request goes out with. One fixed by hand is kept
   within the spec's limit only, as everywhere: Apple's are for testing. */
static uint16_t latency_while_searching(const struct hl_search *search, uint16_t interval_max,
                                        bool apple_rules) {
    if (latency_pinned(search)) {
        return hl_latency_allowed(search->limits.pinned_latency, interval_max,
                                  search->limits.timeout, false);
    }
    return under_ceiling(search,
                         hl_latency_allowed(search->limits.latency_while_searching_interval,
                                            interval_max, search->limits.timeout, apple_rules));
}

static void ask(struct hl_search *search, enum hl_request_kind kind, uint16_t interval_min,
                uint16_t interval_max, uint16_t latency, struct hl_params *request) {
    search->asked = (struct hl_params){
        .interval_min = interval_min,
        .interval_max = interval_max,
        .latency = latency,
        .timeout = search->limits.timeout,
    };
    search->asked_kind = kind;
    search->requests++;
    *request = search->asked;
}

/* The next fallback candidate worth a request, or false. */
static bool next_fallback_candidate(struct hl_search *search, uint16_t *interval_min,
                                    uint16_t *interval_max) {
    const uint8_t count = hl_fallback_candidate_count(&search->limits);

    while (search->next_candidate < count) {
        const uint8_t index = search->next_candidate++;
        uint16_t min, max;

        if (search->memory.fallback_candidates_tried & (1u << index)) {
            continue;
        }
        if (!hl_fallback_candidate(&search->limits, index, &min, &max)) {
            break;
        }
        /* Only a candidate that could beat what the link already has. */
        if (search->have_best && min >= search->best_interval) {
            continue;
        }

        search->asked_candidate = index;
        *interval_min = min;
        *interval_max = max;
        return true;
    }
    return false;
}

static enum hl_search_phase phase_after_secure(const struct hl_search *search) {
    if (search->limits.fastest_interval >= search->limits.slowest_interval) {
        return HL_PHASE_LATENCY;
    }
    return search->memory.fastest_refused ? HL_PHASE_FALLBACK_CANDIDATES : HL_PHASE_FASTEST;
}

bool hl_search_next(struct hl_search *search, struct hl_params *request) {
    const struct hl_search_limits *limits = &search->limits;

    if (search->phase == HL_PHASE_DONE) {
        return false;
    }

    /* Ahead of the request limit: a link left off its best request is put
     * back even when the search has no requests left. */
    if (search->restoring) {
        search->asked = search->best;
        search->asked_kind = HL_ASK_RESTORE;
        *request = search->asked;
        return true;
    }

    /* The host gives latency 0 to what is asked with latency: nothing
       faster is worth asking -- at 0 the slower interval is the better one,
       and the link is back on it. */
    if (search->host_forces_latency_0) {
        search->phase = HL_PHASE_DONE;
        return false;
    }

    if (search->requests >= limits->max_requests) {
        search->phase = HL_PHASE_DONE;
        return false;
    }

    for (;;) {
        switch (search->phase) {
        case HL_PHASE_SECURE: {
            const uint16_t slowest = limits->slowest_interval;

            ask(search, HL_ASK_SECURE, slowest, slowest,
                latency_while_searching(search, slowest, true), request);
            return true;
        }

        case HL_PHASE_FASTEST: {
            const uint16_t fastest = limits->fastest_interval;

            ask(search, HL_ASK_FASTEST, fastest, fastest,
                latency_while_searching(search, fastest, false), request);
            return true;
        }

        case HL_PHASE_FALLBACK_CANDIDATES: {
            uint16_t min, max;

            if (!next_fallback_candidate(search, &min, &max)) {
                search->phase = HL_PHASE_LATENCY;
                continue;
            }
            ask(search, HL_ASK_FALLBACK_CANDIDATE, min, max,
                latency_while_searching(search, max, true), request);
            return true;
        }

        case HL_PHASE_LATENCY: {
            if (!search->have_best || latency_pinned(search)) {
                search->phase = HL_PHASE_DONE;
                return false;
            }
            if (!search->latency_started) {
                /* A host that refused the fastest interval plays by Apple's
                   rules, and those cap the latency too. The period is
                   counted on the interval the link got, the rules on the
                   slowest the request allows. */
                const bool apple_rules = search->memory.fastest_refused;
                const uint16_t target = hl_latency_for_idle_listen_period(
                    search->best_interval, limits->idle_listen_period_ms, limits->timeout,
                    apple_rules);

                search->latency_started = true;
                search->latency_accepted = search->best.latency;
                search->latency_highest = under_ceiling(
                    search, hl_latency_allowed(target, search->best.interval_max, limits->timeout,
                                               apple_rules));
            }
            if (search->latency_accepted >= search->latency_highest ||
                search->latency_steps >= HL_SEARCH_LATENCY_STEPS) {
                search->phase = HL_PHASE_DONE;
                return false;
            }
            /* The target first; after a refusal, halfway between. */
            const uint16_t latency =
                search->latency_steps == 0
                    ? search->latency_highest
                    : (search->latency_accepted + search->latency_highest + 1) / 2;

            search->latency_steps++;
            /* The same interval request as the best: a range stays a range,
               which Apple's rules need. */
            ask(search, HL_ASK_LATENCY, search->best.interval_min, search->best.interval_max,
                latency, request);
            return true;
        }

        case HL_PHASE_DONE:
        default:
            return false;
        }
    }
}

static void note_taken(struct hl_search *search, const struct hl_params *request,
                       enum hl_request_kind kind, uint8_t candidate) {
    search->taken = *request;
    search->taken_kind = kind;
    search->taken_candidate = candidate;
    search->taken_count++;
}

static bool same_request(const struct hl_params *a, const struct hl_params *b) {
    return a->interval_min == b->interval_min && a->interval_max == b->interval_max &&
           a->latency == b->latency;
}

/*
 * A link on this request beats the best so far: a faster interval, or the
 * same at more latency. But latency 0 the host gave to a request asked with
 * latency (latency_0_by_host) loses to anything else whatever the interval,
 * and between two such the slower interval wins: at 0 every interval is a
 * wake-up, and the faster one only doubles them. Latency 0 the keyboard
 * asked for itself (fixed by hand, or under a latency ceiling) is compared
 * as any latency.
 */
static bool beats_best(const struct hl_search *search, const struct hl_link *link,
                       uint16_t latency, bool latency_0_by_host) {
    if (!search->have_best) {
        return true;
    }
    if (latency_0_by_host != search->best_latency_0_by_host) {
        return search->best_latency_0_by_host;
    }
    if (latency_0_by_host) {
        return link->interval > search->best_interval;
    }
    return link->interval < search->best_interval ||
           (link->interval == search->best_interval && latency > search->best.latency);
}

static void take_as_best(struct hl_search *search, const struct hl_link *link) {
    search->have_best = true;
    search->best = search->asked;
    search->best_interval = link->interval;
    search->best_latency_0_by_host = search->asked_latency_0_by_host;
    note_taken(search, &search->asked, search->asked_kind, search->asked_candidate);
    /* The same request asked again and answered: nothing left pending. */
    if (search->silent_pending && same_request(&search->silent_asked, &search->asked)) {
        search->silent_pending = false;
    }
}

/* An accepted request becomes the best only if it beats it: a request built
   before a late answer moved the best on (a resend, say) must not undo it.
   Otherwise the link is put back. */
static bool take_if_better(struct hl_search *search, const struct hl_link *link) {
    if (!beats_best(search, link, search->asked.latency, search->asked_latency_0_by_host)) {
        /* Not kept, but the link ran it until put back: a drop meanwhile is
           about it. */
        note_taken(search, &search->asked, search->asked_kind, search->asked_candidate);
        return false;
    }
    take_as_best(search, link);
    return true;
}

static bool same_link(const struct hl_link *a, const struct hl_link *b) {
    return a->interval == b->interval && a->latency == b->latency && a->timeout == b->timeout;
}

/* The latency phase from scratch, for a new best interval. */
static void restart_latency_phase(struct hl_search *search) {
    search->phase = HL_PHASE_LATENCY;
    search->latency_started = false;
    search->latency_steps = 0;
}

/*
 * The link moved to the request the host left unanswered earlier: its late
 * answer. What its silence was taken for is undone, and it counts as
 * accepted. The request out now is left unjudged -- the host has not
 * answered that one.
 */
static void take_late_answer(struct hl_search *search, const struct hl_link *link) {
    const struct hl_params late = search->silent_asked;
    const enum hl_request_kind kind = search->silent_kind;

    search->silent_pending = false;
    /* The search got on: the host's moves start counting over. */
    search->host_moves_in_a_row = 0;
    note_taken(search, &late, kind, search->silent_candidate);
    if (search->silent_set_fastest_refused) {
        search->memory.fastest_refused = false;
    }
    if (search->silent_marked_candidate) {
        search->memory.fallback_candidates_tried &= ~(1u << search->silent_candidate);
    }

    /* A late answer matches its request exactly: any latency 0 in it was
       asked for. */
    const bool better = beats_best(search, link, late.latency, false);

    if (better) {
        search->have_best = true;
        search->best = late;
        search->best_interval = link->interval;
        search->best_latency_0_by_host = false;
        if (late.latency > 0) {
            /* The host gives latency after all: the search may get on. */
            search->host_forces_latency_0 = false;
        }
        if (kind == HL_ASK_LATENCY) {
            search->latency_accepted = late.latency;
        } else if ((kind == HL_ASK_FASTEST || kind == HL_ASK_FALLBACK_CANDIDATE) &&
                   search->phase != HL_PHASE_DONE) {
            /* A search already over stays over: the link is on the better
               result, that is all. */
            restart_latency_phase(search);
        }
    }
    search->restoring = search->have_best && !hl_link_matches(&search->best, link);
    if (!search->restoring) {
        /* The link is on the best after all. */
        search->restore_failed = false;
        search->host_keeps_moving = false;
    }
}

void hl_search_answer(struct hl_search *search, const struct hl_link *link, bool host_silent) {
    const struct hl_link before = search->link_before;

    search->link_before = *link;
    search->asked_latency_0_by_host = false;
    /* An answer to a request sent after the search ended (it never should
       be) is taken for what it shows, but does not start the search again. */
    const bool over = search->phase == HL_PHASE_DONE;

    /* The link moved to a request the host left silent earlier, not to this
       one: that late answer, while this one stays unanswered. */
    if (!same_link(&before, link) && search->silent_pending &&
        !hl_link_matches(&search->asked, link) && hl_link_matches(&search->silent_asked, link)) {
        take_late_answer(search, link);
        /* The request out now is still unanswered: if the link moves onto
           it later, that is its answer too. */
        if (search->asked_kind != HL_ASK_RESTORE) {
            search->silent_pending = true;
            search->silent_asked = search->asked;
            search->silent_kind = search->asked_kind;
            search->silent_candidate = search->asked_candidate;
            search->silent_set_fastest_refused = false;
            search->silent_marked_candidate = false;
        }
        return;
    }

    bool accepted = hl_link_matches(&search->asked, link);

    /* The interval given, the latency lowered by the host: the interval is
       accepted, at the host's latency. Only for a link that moved -- a
       refusal leaves it where it was. */
    if (!accepted && search->asked_kind != HL_ASK_RESTORE &&
        link->interval == search->asked.interval_min && link->latency < search->asked.latency &&
        !same_link(&before, link) &&
        (search->asked_kind != HL_ASK_LATENCY || link->latency > search->latency_accepted)) {
        if (search->asked_kind == HL_ASK_LATENCY) {
            search->latency_highest = search->asked.latency - 1;
        }
        if (link->latency == 0) {
            /* Asked with latency, given 0: the host's policy, and every
               faster interval would only run at 0 too. */
            search->asked_latency_0_by_host = true;
            search->host_forces_latency_0 = true;
        }
        search->asked.latency = link->latency;
        accepted = true;
    }

    /* Unanswered: nothing came, or the link moved neither to this request
       nor stayed put -- it may still come. */
    const bool unanswered = !accepted && (host_silent || !same_link(&before, link));

    if (!unanswered && search->asked_kind != HL_ASK_RESTORE) {
        /* A search request answered -- taken, or refused with the link left
           as it was: the search got on, the host's moves start counting
           over. */
        search->host_moves_in_a_row = 0;
    }

    if (unanswered && search->asked_kind != HL_ASK_RESTORE) {
        search->silent_pending = true;
        search->silent_asked = search->asked;
        search->silent_kind = search->asked_kind;
        search->silent_candidate = search->asked_candidate;
        search->silent_set_fastest_refused = false;
        search->silent_marked_candidate = false;
    }

    switch (search->asked_kind) {
    case HL_ASK_RESTORE:
        search->restoring = false;
        if (!accepted) {
            search->restore_failed = true;
            search->phase = HL_PHASE_DONE;
        }
        return;

    case HL_ASK_SECURE:
        if (accepted) {
            take_if_better(search, link);
        }
        search->phase = phase_after_secure(search);
        break;

    case HL_ASK_FASTEST:
        if (accepted) {
            /* Kept or not (not: at latency 0 the host gave it, say), asked
               once is enough: the latency of the best is next. */
            take_if_better(search, link);
            search->phase = HL_PHASE_LATENCY;
        } else {
            if (!search->memory.fastest_protected && !search->memory.fastest_refused) {
                search->memory.fastest_refused = true;
                search->silent_set_fastest_refused = unanswered;
            }
            search->phase = HL_PHASE_FALLBACK_CANDIDATES;
        }
        break;

    case HL_ASK_FALLBACK_CANDIDATE:
        /* Accepted means the link got the candidate's minimum, and only
           candidates faster than the best are asked: the result. Not marked
           as tried -- it is what the next search on this host should get. */
        if (accepted) {
            if (take_if_better(search, link)) {
                search->phase = HL_PHASE_LATENCY;
            }
            break;
        }
        if (search->asked_candidate != search->memory.protected_fallback_candidate) {
            search->memory.fallback_candidates_tried |= 1u << search->asked_candidate;
            search->silent_marked_candidate = unanswered;
        }
        break;

    case HL_ASK_LATENCY:
        if (accepted) {
            if (take_if_better(search, link)) {
                search->latency_accepted = search->asked.latency;
            }
        } else {
            search->latency_highest = search->asked.latency - 1;
        }
        break;
    }

    /* Whatever the answer, the link must end up back on the best request --
       and needs no putting back if it is there. */
    search->restoring = search->have_best && !hl_link_matches(&search->best, link);
    if (over) {
        search->phase = HL_PHASE_DONE;
    }
}

static void link_moved(struct hl_search *search, const struct hl_link *link, bool by_host) {
    const struct hl_link before = search->link_before;

    search->link_before = *link;
    if (same_link(&before, link)) {
        return;
    }
    /* Only the request exactly: a host moving the link on its own to the
       interval it left unanswered, at a latency of its own (0, say), has
       not answered it. */
    if (search->silent_pending && hl_link_matches(&search->silent_asked, link)) {
        take_late_answer(search, link);
        return;
    }
    search->restoring = search->have_best && !hl_link_matches(&search->best, link);
    if (search->have_best && !search->restoring) {
        /* Back on the best, whoever put it there. */
        search->restore_failed = false;
        search->host_keeps_moving = false;
        return;
    }
    if (!by_host && search->restoring) {
        if (search->moves_not_by_host < HL_SEARCH_MOVES_NOT_BY_HOST_TAKEN) {
            search->moves_not_by_host++;
        } else {
            /* So many, the search no longer tells them from the host's. */
            by_host = true;
        }
    }
    if (by_host && search->restoring && search->phase != HL_PHASE_DONE &&
        ++search->host_moves_in_a_row >= HL_SEARCH_HOST_MOVES_TOLERATED) {
        /* The host keeps moving the link to its own: it is left there, and
           the search ends on what it found. */
        search->host_keeps_moving = true;
        search->restoring = false;
        search->phase = HL_PHASE_DONE;
    }
}

void hl_search_link_moved(struct hl_search *search, const struct hl_link *link) {
    link_moved(search, link, true);
}

void hl_search_link_moved_not_by_host(struct hl_search *search, const struct hl_link *link) {
    link_moved(search, link, false);
}

void hl_search_request_not_sent(struct hl_search *search) {
    if (search->asked_kind == HL_ASK_RESTORE || search->requests == 0) {
        /* A restore uses no step of the search. */
        return;
    }
    search->requests--;
    switch (search->asked_kind) {
    case HL_ASK_FALLBACK_CANDIDATE:
        /* Its turn again -- unless a late answer has taken the search past
           the candidates, where nothing reads it. */
        if (search->asked_candidate < search->next_candidate) {
            search->next_candidate = search->asked_candidate;
        }
        break;
    case HL_ASK_LATENCY:
        /* Unless a new best started the latency phase over. */
        if (search->latency_started && search->latency_steps > 0) {
            search->latency_steps--;
        }
        break;
    default:
        break;
    }
}
