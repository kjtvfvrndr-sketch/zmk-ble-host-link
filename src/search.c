/*
 * SPDX-License-Identifier: MIT
 */

#include <string.h>

#include "search.h"

#define MIN(a, b) ((a) < (b) ? (a) : (b))

bool hl_link_matches(const struct hl_params *asked, const struct hl_link *link) {
    return link->interval == asked->interval && link->latency == asked->latency;
}

/* Largest latency L with (L + 1) <= limit, or 0. */
static uint32_t below(uint32_t limit) { return limit > 0 ? limit - 1 : 0; }

uint16_t hl_latency_allowed(uint16_t wanted, uint16_t interval, uint16_t timeout) {
    if (interval == 0) {
        return 0;
    }
    /* timeout * 4 > (1 + latency) * interval, in 10 ms and 1.25 ms units */
    const uint32_t spec_limit = below((4u * timeout - 1u) / interval);

    return (uint16_t)MIN(MIN((uint32_t)wanted, (uint32_t)HL_SPEC_MAX_LATENCY), spec_limit);
}

uint16_t hl_latency_for_idle_listen_period(uint16_t interval, uint16_t idle_listen_period_ms,
                                           uint16_t timeout) {
    if (interval == 0) {
        return 0;
    }
    /* interval * 1.25 ms * (latency + 1) <= period */
    const uint32_t listens = (4u * idle_listen_period_ms) / (5u * interval);

    return hl_latency_allowed((uint16_t)MIN(below(listens), 0xFFFFu), interval, timeout);
}

uint16_t hl_latency_under_ceiling(uint16_t latency, uint16_t latency_ceiling) {
    if (latency_ceiling == HL_NO_LATENCY_CEILING) {
        return latency;
    }
    return MIN(latency, latency_ceiling > 0 ? latency_ceiling - 1 : 0);
}

uint16_t hl_search_latency_at(const struct hl_search_limits *limits, uint16_t interval) {
    if (limits->pinned_latency != HL_LATENCY_NOT_PINNED) {
        return hl_latency_allowed(limits->pinned_latency, interval, limits->timeout);
    }
    return hl_latency_under_ceiling(
        hl_latency_allowed(limits->latency_while_searching_interval, interval, limits->timeout),
        limits->latency_ceiling);
}

void hl_search_start(struct hl_search *search, const struct hl_search_limits *limits) {
    memset(search, 0, sizeof(*search));
    search->limits = *limits;
    search->phase = HL_PHASE_SECURE;
    search->next_ladder_interval = limits->fastest_interval > limits->interval_floor
                                       ? limits->fastest_interval
                                       : limits->interval_floor;
    search->latency_refused_lowest = HL_NO_LATENCY_CEILING;
}

static void ask(struct hl_search *search, enum hl_search_step step, uint16_t interval,
                uint16_t latency, struct hl_params *request) {
    search->asked = (struct hl_params){
        .interval = interval,
        .latency = latency,
        .timeout = search->limits.timeout,
    };
    search->asked_step = step;
    search->requests++;
    *request = search->asked;
}

static void take_verdict(struct hl_search *search) {
    if (!search->verdict_pending) {
        return;
    }
    search->verdict_pending = false;

    if (search->asked_accepted) {
        search->have_result = true;
        search->result = search->asked;
    }

    switch (search->asked_step) {
    case HL_STEP_SECURE:
        search->phase = HL_PHASE_LADDER;
        break;
    case HL_STEP_LADDER:
        if (search->asked_accepted) {
            /* The first ladder step taken is the fastest the host gives. */
            search->phase = HL_PHASE_LATENCY;
        } else {
            search->next_ladder_interval = search->asked.interval + 1;
        }
        break;
    case HL_STEP_LATENCY:
        if (!search->asked_accepted) {
            search->latency_refused_lowest = MIN(search->latency_refused_lowest,
                                                 search->asked.latency);
        }
        break;
    }
}

/* The next latency step, or false: the target first, then halfway between
   what was accepted and what was refused. */
static bool next_latency_step(struct hl_search *search, uint16_t *latency) {
    const struct hl_search_limits *limits = &search->limits;

    if (limits->pinned_latency != HL_LATENCY_NOT_PINNED ||
        search->latency_steps >= HL_SEARCH_LATENCY_STEPS) {
        return false;
    }

    const uint16_t accepted = search->result.latency;
    const uint16_t target = hl_latency_under_ceiling(
        hl_latency_for_idle_listen_period(search->result.interval, limits->idle_listen_period_ms,
                                          limits->timeout),
        limits->latency_ceiling);
    const uint16_t want = search->latency_refused_lowest == HL_NO_LATENCY_CEILING
                              ? target
                              : (uint16_t)((accepted + search->latency_refused_lowest) / 2u);

    if (want <= accepted || want >= search->latency_refused_lowest) {
        return false;
    }
    *latency = want;
    return true;
}

bool hl_search_next(struct hl_search *search, struct hl_params *request) {
    const struct hl_search_limits *limits = &search->limits;

    take_verdict(search);

    for (;;) {
        switch (search->phase) {
        case HL_PHASE_SECURE:
            ask(search, HL_STEP_SECURE, limits->slowest_interval,
                hl_search_latency_at(limits, limits->slowest_interval), request);
            return true;

        case HL_PHASE_LADDER:
            if (search->next_ladder_interval < limits->slowest_interval) {
                const uint16_t interval = search->next_ladder_interval;

                ask(search, HL_STEP_LADDER, interval, hl_search_latency_at(limits, interval),
                    request);
                return true;
            }
            search->phase = search->have_result ? HL_PHASE_LATENCY : HL_PHASE_DONE;
            break;

        case HL_PHASE_LATENCY: {
            uint16_t latency;

            if (!next_latency_step(search, &latency)) {
                search->phase = HL_PHASE_DONE;
                break;
            }
            search->latency_steps++;
            ask(search, HL_STEP_LATENCY, search->result.interval, latency, request);
            return true;
        }

        case HL_PHASE_DONE:
        default:
            return false;
        }
    }
}

void hl_search_answer(struct hl_search *search, bool accepted) {
    search->verdict_pending = true;
    search->asked_accepted = accepted;
}

bool hl_search_late_answer(struct hl_search *search) {
    if (!search->verdict_pending || search->asked_accepted) {
        return false;
    }
    search->asked_accepted = true;
    return true;
}
