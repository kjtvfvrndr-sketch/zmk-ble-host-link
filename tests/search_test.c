/*
 * SPDX-License-Identifier: MIT
 *
 * The search (src/search.c) against simulated hosts. Each host model decides
 * what a request does to the link, the way a real one would: refuse and keep
 * the link as it was, or accept and pick an interval inside the range.
 *
 *   make -C tests        builds and runs it on the build machine
 *
 * Checked for every model: the search ends; the link ends on the best request
 * the search found; that interval is never slower than the slowest acceptable
 * one; the request limit holds; a host that accepted the secure request never
 * ends on anything worse.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "../src/search.h"

#define TIMEOUT 400 /* 4 s */

typedef struct hl_link (*host_fn)(struct hl_link now, const struct hl_params *req);

/* ----------------------------------------------------------- host models */

static bool apple_rules_ok(const struct hl_params *r, bool hid_exception) {
    const uint16_t floor = hid_exception ? 9 : 12;
    const bool exact_15 = r->interval_min == 12 && r->interval_max == 12;
    const bool wide = r->interval_max >= r->interval_min + 12;

    if (r->interval_min < floor) {
        return false;
    }
    if (!hid_exception && r->interval_min % 12 != 0) {
        return false;
    }
    if (!exact_15 && !wide) {
        return false;
    }
    if (r->latency > 30) {
        return false;
    }
    if (r->interval_max * 5u * (r->latency + 1u) > 8000u) {
        return false;
    }
    if (15u * r->interval_max * (r->latency + 1u) >= 40u * r->timeout) {
        return false;
    }
    return r->timeout >= 200 && r->timeout <= 600;
}

static struct hl_link accept(const struct hl_params *r, uint16_t interval) {
    return (struct hl_link){.interval = interval, .latency = r->latency, .timeout = r->timeout};
}

/* macOS as observed: only the strict rules, picks the low end. */
static struct hl_link mac_strict(struct hl_link now, const struct hl_params *r) {
    return apple_rules_ok(r, false) ? accept(r, r->interval_min) : now;
}

/* macOS honouring the HID exception, picking the low end of a range. */
static struct hl_link mac_hid_low(struct hl_link now, const struct hl_params *r) {
    return apple_rules_ok(r, true) ? accept(r, r->interval_min) : now;
}

/* ...and picking the slow end -- the case that moves the link. */
static struct hl_link mac_hid_high(struct hl_link now, const struct hl_params *r) {
    return apple_rules_ok(r, true) ? accept(r, r->interval_max) : now;
}

/* An Apple device that "scales" 15-15 to 30 ms, as QA1931 warns. */
static struct hl_link apple_scaling(struct hl_link now, const struct hl_params *r) {
    if (!apple_rules_ok(r, false)) {
        return now;
    }
    return accept(r, r->interval_min == 12 && r->interval_max == 12 ? 24 : r->interval_max);
}

/* An Apple host that, given a range holding the interval it runs, keeps it. */
static struct hl_link mac_hid_keeps(struct hl_link now, const struct hl_params *r) {
    if (!apple_rules_ok(r, true)) {
        return now;
    }
    if (now.interval >= r->interval_min && now.interval <= r->interval_max) {
        return accept(r, now.interval);
    }
    return accept(r, r->interval_min);
}

/* Windows-like: anything valid, the fastest end. */
static struct hl_link pc_low(struct hl_link now, const struct hl_params *r) {
    (void)now;
    return accept(r, r->interval_min);
}

/* A host that always picks the slow end of a range. */
static struct hl_link pc_high(struct hl_link now, const struct hl_params *r) {
    (void)now;
    return accept(r, r->interval_max);
}

/* Takes nothing below 11.25 ms: gives 11.25 ms for anything faster, else the
   low end. */
static struct hl_link pc_min9(struct hl_link now, const struct hl_params *r) {
    if (r->interval_max < 9) {
        return now;
    }
    return accept(r, r->interval_min > 9 ? r->interval_min : 9);
}

/* Fast intervals fine, latency above 30 refused. */
static struct hl_link pc_lat30(struct hl_link now, const struct hl_params *r) {
    return r->latency > 30 ? now : accept(r, r->interval_min);
}

/* Latency above 45 refused. */
static struct hl_link pc_lat45(struct hl_link now, const struct hl_params *r) {
    return r->latency > 45 ? now : accept(r, r->interval_min);
}

/* Refuses everything. */
static struct hl_link refuses_all(struct hl_link now, const struct hl_params *r) {
    (void)r;
    return now;
}

/* Only ever 7.5 ms: refuses the secure 15-15 itself. */
static struct hl_link only_7_5(struct hl_link now, const struct hl_params *r) {
    return r->interval_min == 6 ? accept(r, 6) : now;
}

/* ------------------------------------------------------------- the runner */

static int failures;
/* Refusals over the Link Layer: the host answers, with the link as it was.
   Off: refusals are silent (L2CAP), as the runner normally has them. */
static bool refusals_answered;

#define CHECK(cond, ...)                                                                           \
    do {                                                                                           \
        if (!(cond)) {                                                                             \
            printf("    FAIL: ");                                                              \
            printf(__VA_ARGS__);                                                                   \
            printf("\n");                                                                          \
            failures++;                                                                            \
        }                                                                                          \
    } while (0)

static void print_params(const struct hl_params *p) {
    if (p->interval_min == p->interval_max) {
        printf("%u", p->interval_min);
    } else {
        printf("%u-%u", p->interval_min, p->interval_max);
    }
    printf("/%u", p->latency);
}

static bool same(const struct hl_link *a, const struct hl_link *b) {
    return a->interval == b->interval && a->latency == b->latency && a->timeout == b->timeout;
}

/* Gives the interval but caps the latency at 20 itself (as Android or BlueZ
   may), instead of refusing. */
static struct hl_link pc_lowers_latency(struct hl_link now, const struct hl_params *r) {
    (void)now;
    struct hl_link l = accept(r, r->interval_min);

    if (l.latency > 20) {
        l.latency = 20;
    }
    return l;
}

/* Gives every interval asked, always at latency 0. */
static struct hl_link pc_latency_0(struct hl_link now, const struct hl_params *r) {
    (void)now;
    struct hl_link l = accept(r, r->interval_min);

    l.latency = 0;
    return l;
}

/* Keeps the latency at 15 ms, gives anything faster only at latency 0. */
static struct hl_link pc_fast_latency_0(struct hl_link now, const struct hl_params *r) {
    (void)now;
    struct hl_link l = accept(r, r->interval_min);

    if (l.interval < 12) {
        l.latency = 0;
    }
    return l;
}

/* Zephyr's own test in bt_conn_le_param_update(): the link already
   satisfies the request, so nothing is sent (-EALREADY). */
static bool zephyr_skips(const struct hl_params *r, const struct hl_link *link) {
    return link->interval >= r->interval_min && link->interval <= r->interval_max &&
           link->latency == r->latency && link->timeout == r->timeout;
}

static struct hl_search run(const char *name, host_fn host, struct hl_link opened,
                            const struct hl_search_limits *limits,
                            struct hl_search_memory *memory) {
    struct hl_search s;
    struct hl_params req;
    struct hl_link link = opened;
    int sends = 0, restores = 0;

    hl_search_start(&s, limits, memory, &opened);
    printf("%-14s ", name);

    while (hl_search_next(&s, &req)) {
        const bool restore = s.asked_kind == HL_ASK_RESTORE;

        /* As the glue does: the timeout moved off the link's when Zephyr
           would otherwise keep a request that does not match. */
        req.timeout = hl_timeout_that_reaches_the_host(&req, &link);

        /* As Zephyr does: a request the link already satisfies is not sent,
           and the glue answers it at once with the link as is. */
        const bool already = zephyr_skips(&req, &link);

        const struct hl_link before = link;

        if (!already) {
            link = host(link, &req);
        }
        /* A host that leaves the link as it was said nothing the keyboard
           sees (an L2CAP refusal): silence until the deadline. */
        const bool silent = !already && same(&before, &link) && !refusals_answered;

        hl_search_answer(&s, &link, silent);
        sends++;
        restores += restore;
        CHECK(!already || hl_link_matches(&req, &link),
              "a request that does not match never reaches the host (-EALREADY)");

        printf("%s", restore ? "R:" : "");
        print_params(&req);
        if (req.timeout != limits->timeout) {
            printf("/t%u", req.timeout);
        }
        printf("%s ", already                          ? "="
                      : hl_link_matches(&req, &link)     ? "+"
                      : hl_link_matches(&s.asked, &link) ? "~" /* at the host's latency */
                                                         : "-");
        CHECK(sends < 64, "search does not end");
        if (sends >= 64) {
            break;
        }
    }

    printf("\n%14s -> ", "");
    if (s.have_best) {
        printf("link %u (%u.%02u ms) lat %u", link.interval, link.interval * 125 / 100,
               link.interval * 125 % 100, link.latency);
    } else {
        printf("nothing accepted, link %u lat %u", link.interval, link.latency);
    }
    printf("  [%d requests, %d restores%s%s]\n", sends - restores, restores,
           s.memory.fastest_refused ? ", fastest refused" : "",
           s.restore_failed ? ", RESTORE FAILED" : "");

    CHECK(sends - restores <= limits->max_requests, "request limit exceeded");
    if (s.have_best) {
        CHECK(hl_link_matches(&s.best, &link), "link is not on the best request");
        CHECK(s.best_interval == s.best.interval_min, "best interval is not the minimum asked");
        CHECK(s.best_interval <= limits->slowest_interval, "best slower than acceptable");
        CHECK(link.interval <= limits->slowest_interval, "link slower than acceptable");
        CHECK(s.best.timeout == limits->timeout, "best keeps a moved timeout");
    }
    *memory = s.memory;
    return s;
}

static struct hl_search_memory fresh_memory(void) {
    return (struct hl_search_memory){
        .protected_fallback_candidate = HL_NO_FALLBACK_CANDIDATE,
        .latency_ceiling = HL_NO_LATENCY_CEILING,
    };
}

int main(void) {
    const struct hl_search_limits limits = {
        .fastest_interval = 6,
        .slowest_interval = 12,
        .latency_while_searching_interval = 30,
        .idle_listen_period_ms = 465,
        .timeout = TIMEOUT,
        .max_requests = 12,
        .pinned_latency = HL_LATENCY_NOT_PINNED,
    };
    const struct hl_link mac_opened = {.interval = 12, .latency = 0, .timeout = 72};
    const struct hl_link pc_opened = {.interval = 24, .latency = 0, .timeout = 420};

    struct {
        const char *name;
        host_fn host;
        struct hl_link opened;
        uint16_t want_interval; /* 0: nothing must be accepted */
        uint16_t want_latency;
    } cases[] = {
        {"mac_strict", mac_strict, mac_opened, 12, 30},
        {"mac_hid_low", mac_hid_low, mac_opened, 9, 30},
        {"mac_hid_high", mac_hid_high, mac_opened, 12, 30},
        {"mac_hid_keeps", mac_hid_keeps, mac_opened, 12, 30},
        {"apple_scaling", apple_scaling, mac_opened, 0, 0},
        {"pc_low", pc_low, pc_opened, 6, 61},
        {"pc_high", pc_high, pc_opened, 6, 61},
        {"pc_min9", pc_min9, pc_opened, 9, 30},
        {"pc_lat30", pc_lat30, pc_opened, 6, 30},
        {"pc_lat45", pc_lat45, pc_opened, 6, 45},
        {"refuses_all", refuses_all, pc_opened, 0, 0},
        {"only_7_5", only_7_5, pc_opened, 6, 61},
        {"pc_lowers_lat", pc_lowers_latency, pc_opened, 6, 20},
    };

    printf("Requests: interval[-max]/latency[/t timeout], + accepted, - refused,\n"
           "          ~ interval accepted at a latency the host lowered,\n"
           "          = already in effect (never sent), R: restore\n\n");

    for (size_t i = 0; i < sizeof(cases) / sizeof(cases[0]); i++) {
        struct hl_search_memory memory = fresh_memory();
        const struct hl_search s = run(cases[i].name, cases[i].host, cases[i].opened, &limits,
                                       &memory);

        if (cases[i].want_interval == 0) {
            CHECK(!s.have_best || s.best_interval <= limits.slowest_interval,
                  "%s: unexpected result", cases[i].name);
        } else {
            CHECK(s.have_best, "%s: nothing found", cases[i].name);
            CHECK(s.best_interval == cases[i].want_interval, "%s: interval %u, wanted %u",
                  cases[i].name, s.best_interval, cases[i].want_interval);
            CHECK(s.best.latency == cases[i].want_latency, "%s: latency %u, wanted %u",
                  cases[i].name, s.best.latency, cases[i].want_latency);
        }

        /* The second search on the same host uses what the first learned. */
        struct hl_search_memory again = memory;
        char name[32];

        snprintf(name, sizeof(name), "  again");
        const struct hl_search s2 = run(name, cases[i].host, cases[i].opened, &limits, &again);

        if (s.have_best) {
            CHECK(s2.have_best && s2.best_interval <= s.best_interval,
                  "%s: second search worse than the first", cases[i].name);
        }
        printf("\n");
    }

    /* A latency fixed by hand: only the interval is searched, at that latency. */
    {
        struct hl_search_limits pinned = limits;
        struct hl_search_memory memory = fresh_memory();

        pinned.pinned_latency = 10;
        const struct hl_search s = run("pc_low lat=10", pc_low, pc_opened, &pinned, &memory);

        CHECK(s.have_best && s.best.latency == 10 && s.best_interval == 6,
              "pinned latency not kept");
        printf("\n");
    }

    /* A request limit too small to finish: still ends on the best so far. */
    {
        struct hl_search_limits tight = limits;
        struct hl_search_memory memory = fresh_memory();

        tight.max_requests = 2;
        run("mac_hid_high/2", mac_hid_high, mac_opened, &tight, &memory);
        printf("\n");
    }

    /* The link dropped at latency 40 once: the search stays below it. */
    {
        struct hl_search_memory memory = fresh_memory();

        memory.latency_ceiling = 40;
        const struct hl_search s = run("pc_low ceil40", pc_low, pc_opened, &limits, &memory);

        CHECK(s.have_best && s.best_interval == 6 && s.best.latency == 39,
              "latency ceiling not kept");
        printf("\n");
    }

    /* The stored result came from candidate 9-21; this time the host refuses
       everything. The candidate must not be marked tried. */
    {
        struct hl_search_memory memory = fresh_memory();
        const uint8_t index = hl_fallback_candidate_index(&limits, 9, 21);

        CHECK(index != HL_NO_FALLBACK_CANDIDATE, "candidate 9-21 not in the list");
        memory.fastest_refused = true;
        memory.protected_fallback_candidate = index;
        run("refuses, prot", refuses_all, mac_opened, &limits, &memory);
        CHECK(!(memory.fallback_candidates_tried & (1u << index)),
              "the protected candidate was marked tried");
        CHECK(memory.fallback_candidates_tried != 0, "other candidates not marked");

        /* ...and the next search on the host, now accepting again, gets it. */
        const struct hl_search s = run("  mac_hid_low", mac_hid_low, mac_opened, &limits, &memory);

        CHECK(s.have_best && s.best_interval == 9, "protected candidate not found again");
        printf("\n");
    }

    /* Windows takes the slow end of any range: exact requests only. */
    {
        struct hl_search_memory memory = fresh_memory();
        const struct hl_search s = run("pc_high only", pc_high, pc_opened, &limits, &memory);

        CHECK(s.requests <= 3, "a lazy host took %u requests", s.requests);
        printf("\n");
    }

    /* A Windows host answering 7.5 ms late: the link moves to it while the
       first fallback candidate is out. That is the answer to 7.5 ms. */
    {
        struct hl_search s;
        struct hl_params req;
        struct hl_link link = pc_opened;
        struct hl_search_memory memory = fresh_memory();

        printf("late answer    ");
        hl_search_start(&s, &limits, &memory, &link);
        hl_search_next(&s, &req); /* secure 12 */
        link = pc_low(link, &req);
        hl_search_answer(&s, &link, false);
        hl_search_next(&s, &req); /* fastest 6, no answer in time */
        CHECK(s.asked_kind == HL_ASK_FASTEST, "fastest not asked second");
        hl_search_answer(&s, &link, true);
        CHECK(s.memory.fastest_refused, "silence not taken as a refusal for now");
        hl_search_next(&s, &req); /* first candidate */
        CHECK(s.asked_kind == HL_ASK_FALLBACK_CANDIDATE, "no candidate after the silence");
        link = (struct hl_link){6, 30, TIMEOUT}; /* the late answer to 7.5 ms */
        hl_search_answer(&s, &link, false);
        CHECK(!s.memory.fastest_refused, "late answer did not undo fastest_refused");
        CHECK(s.have_best && s.best_interval == 6, "late answer not taken as the best");
        CHECK(!(s.memory.fallback_candidates_tried & 1u), "unanswered candidate marked tried");
        CHECK(!s.restoring, "restore after a late answer that is the best");
        CHECK(hl_search_next(&s, &req) && s.asked_kind == HL_ASK_LATENCY && req.latency == 61,
              "latency phase not on 7.5 ms after the late answer");
        printf("-> best %u lat %u\n\n", s.best_interval, s.best.latency);
    }

    /* A latency fixed by hand goes out as set, Apple's limits or not. */
    {
        struct hl_search_limits pinned = limits;
        struct hl_search_memory memory = fresh_memory();
        struct hl_search s;
        struct hl_params req;
        const struct hl_link opened = {12, 0, 72};

        pinned.pinned_latency = 60;
        hl_search_start(&s, &pinned, &memory, &opened);
        CHECK(hl_search_next(&s, &req) && req.latency == 60, "pinned latency capped (%u)",
              req.latency);
    }

    /* The stored result is the fastest interval; refused this time: it must
       not turn the host into an Apple one. */
    {
        struct hl_search_memory memory = fresh_memory();

        memory.fastest_protected = true;
        run("refuses, fast", refuses_all, pc_opened, &limits, &memory);
        CHECK(!memory.fastest_refused, "protected fastest marked refused");
        printf("\n");
    }

    /* The same Apple hosts refusing over the Link Layer: answered, not silent. */
    refusals_answered = true;
    {
        struct hl_search_memory memory = fresh_memory();
        const struct hl_search s = run("mac_strict LL", mac_strict, mac_opened, &limits, &memory);

        CHECK(s.have_best && s.best_interval == 12 && s.best.latency == 30, "mac_strict LL");
        memory = fresh_memory();
        const struct hl_search s2 = run("mac_hid_low LL", mac_hid_low, mac_opened, &limits,
                                        &memory);

        CHECK(s2.have_best && s2.best_interval == 9, "mac_hid_low LL");
        printf("\n");
    }
    refusals_answered = false;

    /* The late answer to 7.5 ms comes in the pause, and the next request
       then goes unanswered too: the move itself is the answer. */
    {
        struct hl_search s;
        struct hl_params req;
        struct hl_link link = pc_opened;
        struct hl_search_memory memory = fresh_memory();

        hl_search_start(&s, &limits, &memory, &link);
        hl_search_next(&s, &req); /* secure */
        link = pc_low(link, &req);
        hl_search_answer(&s, &link, false);
        hl_search_next(&s, &req); /* 6-6, silent */
        hl_search_answer(&s, &link, true);
        link = (struct hl_link){6, 30, TIMEOUT}; /* moves in the pause */
        hl_search_link_moved(&s, &link);
        CHECK(!s.memory.fastest_refused && s.have_best && s.best_interval == 6,
              "late answer in the pause lost");
        CHECK(hl_search_next(&s, &req) && s.asked_kind == HL_ASK_LATENCY && req.latency == 61,
              "no latency phase after a late answer in the pause");
    }

    /* A late answer while a restore is out is still taken. */
    {
        struct hl_search s;
        struct hl_params req;
        struct hl_link link = mac_opened;
        struct hl_search_memory memory = fresh_memory();

        hl_search_start(&s, &limits, &memory, &link);
        hl_search_next(&s, &req); /* secure 12/30 */
        link = mac_hid_low(link, &req);
        hl_search_answer(&s, &link, false);
        hl_search_next(&s, &req); /* 6-6 refused, silent */
        hl_search_answer(&s, &link, true);
        hl_search_next(&s, &req); /* 9-21 silent */
        CHECK(s.asked_kind == HL_ASK_FALLBACK_CANDIDATE, "candidate expected");
        hl_search_answer(&s, &link, true);
        link = (struct hl_link){20, 30, TIMEOUT}; /* the host moves the link itself */
        hl_search_link_moved(&s, &link);
        CHECK(s.restoring, "a move off the best is not put back");
        hl_search_next(&s, &req); /* restore 12-12 */
        CHECK(s.asked_kind == HL_ASK_RESTORE, "restore expected");
        link = (struct hl_link){9, 30, TIMEOUT}; /* ...late answer to 9-21 instead */
        hl_search_answer(&s, &link, false);
        CHECK(s.have_best && s.best_interval == 9 && !s.restore_failed,
              "late answer during a restore lost");
    }

    /* A request still queued in the controller (the background parameters,
       just after switching to that host) completes first: the link moves
       to neither the request nor stays put. Not a refusal yet -- the real
       answer comes after. */
    {
        struct hl_search s;
        struct hl_params req;
        struct hl_link link = {48, 25, 600};
        struct hl_search_memory memory = fresh_memory();

        hl_search_start(&s, &limits, &memory, &link);
        hl_search_next(&s, &req); /* secure 12/30 */
        link = (struct hl_link){60, 25, 600}; /* the queued background update lands */
        hl_search_answer(&s, &link, false);
        CHECK(!s.have_best && s.silent_pending, "a foreign move taken as the answer");
        link = (struct hl_link){12, 30, TIMEOUT}; /* ...then the real answer */
        hl_search_link_moved(&s, &link);
        CHECK(s.have_best && s.best_interval == 12, "the real answer after a foreign move lost");
        CHECK(s.taken_count > 0 && s.taken_kind == HL_ASK_SECURE,
              "the request the link moved onto not noted");
    }

    /* The late answer to the secure request comes while 7.5 ms is out; 7.5
       ms is accepted later still. Both are answers -- no restore to 15 ms
       in between, and 7.5 ms is what the link moved onto last. */
    {
        struct hl_search s;
        struct hl_params req;
        struct hl_link link = pc_opened;
        struct hl_search_memory memory = fresh_memory();

        hl_search_start(&s, &limits, &memory, &link);
        hl_search_next(&s, &req); /* secure, silent */
        hl_search_answer(&s, &link, true);
        hl_search_next(&s, &req); /* 6-6 out */
        CHECK(s.asked_kind == HL_ASK_FASTEST, "fastest expected");
        link = (struct hl_link){12, 30, TIMEOUT}; /* late answer to the secure one */
        hl_search_answer(&s, &link, false);
        CHECK(s.have_best && s.best_interval == 12, "late secure answer lost");
        CHECK(s.silent_pending && s.silent_kind == HL_ASK_FASTEST,
              "the request out is not kept pending after a late answer");
        link = (struct hl_link){6, 30, TIMEOUT}; /* ...then 7.5 ms, in the pause */
        hl_search_link_moved(&s, &link);
        CHECK(s.have_best && s.best_interval == 6 && !s.restoring,
              "7.5 ms after a late answer taken as a foreign move");
        CHECK(s.taken_kind == HL_ASK_FASTEST, "7.5 ms not what the link moved onto last");
        CHECK(!s.memory.fastest_refused, "7.5 ms marked refused");
    }

    /* The host changes the link on its own before a request goes out (the
       first seconds of a connection): seen first, a refusal after it is a
       plain refusal, not "unanswered". */
    {
        struct hl_search s;
        struct hl_params req;
        struct hl_link link = mac_opened;
        struct hl_search_memory memory = fresh_memory();

        hl_search_start(&s, &limits, &memory, &link);
        link = (struct hl_link){12, 0, 200}; /* the host's own update */
        hl_search_link_moved(&s, &link);
        hl_search_next(&s, &req); /* secure, refused: the link stays */
        hl_search_answer(&s, &link, false);
        CHECK(!s.silent_pending, "a plain refusal after a host's own change kept pending");
    }

    /* A candidate waits to be sent again (the controller busy) while the
       late answer to 7.5 ms comes in; the stale candidate then goes out and
       is accepted. It must not replace 7.5 ms -- the link goes back. */
    {
        struct hl_search s;
        struct hl_params req;
        struct hl_link link = pc_opened;
        struct hl_search_memory memory = fresh_memory();

        hl_search_start(&s, &limits, &memory, &link);
        hl_search_next(&s, &req); /* secure */
        link = pc_low(link, &req);
        hl_search_answer(&s, &link, false);
        hl_search_next(&s, &req); /* 6-6, silent */
        hl_search_answer(&s, &link, true);
        hl_search_next(&s, &req); /* candidate 9-21, not sent yet */
        const struct hl_params stale = req;

        link = (struct hl_link){6, 30, TIMEOUT}; /* the late answer to 7.5 ms */
        hl_search_link_moved(&s, &link);
        CHECK(s.have_best && s.best_interval == 6, "late answer during a resend lost");
        link = pc_low(link, &stale); /* the stale candidate goes out, accepted */
        hl_search_answer(&s, &link, false);
        CHECK(s.best_interval == 6, "a stale candidate replaced 7.5 ms (best %u)", s.best_interval);
        CHECK(s.restoring, "the link is not put back on 7.5 ms");
    }

    /* A move off the best, then the link back on it by the next answer:
       nothing left to put back. */
    {
        struct hl_search s;
        struct hl_params req;
        struct hl_link link = pc_opened;
        struct hl_search_memory memory = fresh_memory();

        hl_search_start(&s, &limits, &memory, &link);
        hl_search_next(&s, &req); /* secure */
        link = pc_low(link, &req);
        hl_search_answer(&s, &link, false);
        link = (struct hl_link){24, 0, 420}; /* the host moves it away */
        hl_search_link_moved(&s, &link);
        CHECK(s.restoring, "a move off the best not noted");
        hl_search_next(&s, &req); /* restore */
        link = pc_low(link, &req);
        hl_search_answer(&s, &link, false);
        CHECK(!s.restoring, "still restoring with the link on the best");
        hl_search_next(&s, &req); /* 6-6, accepted */
        link = pc_low(link, &req);
        hl_search_answer(&s, &link, false);
        CHECK(!s.restoring && s.best_interval == 6, "restore left set after an accepted answer");
    }

    /* The same request asked again after a late answer, and accepted: no
       pending request is left over. */
    {
        struct hl_search s;
        struct hl_params req;
        struct hl_link link = pc_opened;
        struct hl_search_memory memory = fresh_memory();

        hl_search_start(&s, &limits, &memory, &link);
        hl_search_next(&s, &req); /* secure, silent */
        hl_search_answer(&s, &link, true);
        hl_search_next(&s, &req); /* 6-6 out */
        link = (struct hl_link){12, 30, TIMEOUT}; /* late secure answer */
        hl_search_answer(&s, &link, false);
        CHECK(s.silent_pending && s.silent_kind == HL_ASK_FASTEST, "6-6 not pending");
        hl_search_next(&s, &req); /* 6-6 asked again */
        CHECK(s.asked_kind == HL_ASK_FASTEST, "6-6 not asked again");
        link = pc_low(link, &req);
        hl_search_answer(&s, &link, false);
        CHECK(s.best_interval == 6 && !s.silent_pending, "a pending request left over");
    }

    /* A host that accepts every request and then moves the link back to its
       own (a power manager): the search must end, not put the link back
       forever. */
    {
        struct hl_search s;
        struct hl_params req;
        const struct hl_link own = {24, 0, 420};
        struct hl_link link = own;
        struct hl_search_memory memory = fresh_memory();
        int sends = 0;

        hl_search_start(&s, &limits, &memory, &link);
        while (hl_search_next(&s, &req) && sends < 64) {
            sends++;
            link = pc_low(link, &req);
            hl_search_answer(&s, &link, false);
            link = own; /* ...and back to its own */
            hl_search_link_moved(&s, &link);
        }
        printf("host reverting  %d requests, best %u, host keeps moving %d\n\n", sends,
               s.best_interval, s.host_keeps_moving);
        CHECK(sends < 8, "a host reverting the link keeps the search going (%d sends)", sends);
        CHECK(s.host_keeps_moving && s.phase == HL_PHASE_DONE, "the search did not end");
        CHECK(!s.restoring, "still restoring after giving up");
    }

    /* A search over after a failed restore, and the late answer to 7.5 ms
       comes in after: it stays over, and the restore is no longer failed. */
    {
        struct hl_search s;
        struct hl_params req;
        struct hl_link link = pc_opened;
        struct hl_search_memory memory = fresh_memory();

        hl_search_start(&s, &limits, &memory, &link);
        hl_search_next(&s, &req); /* secure */
        link = pc_low(link, &req);
        hl_search_answer(&s, &link, false);
        hl_search_next(&s, &req); /* 6-6, silent */
        hl_search_answer(&s, &link, true);
        link = (struct hl_link){24, 0, 420}; /* the host moves it */
        hl_search_link_moved(&s, &link);
        hl_search_next(&s, &req); /* restore, refused */
        CHECK(s.asked_kind == HL_ASK_RESTORE, "restore expected");
        hl_search_answer(&s, &link, false);
        CHECK(s.restore_failed && s.phase == HL_PHASE_DONE, "restore not failed");
        link = (struct hl_link){6, 30, TIMEOUT}; /* the late answer to 7.5 ms */
        hl_search_link_moved(&s, &link);
        CHECK(s.best_interval == 6, "late answer after the end lost");
        CHECK(s.phase == HL_PHASE_DONE, "a finished search started again");
        CHECK(!s.restore_failed, "restore still failed with the link on the best");
        CHECK(!hl_search_next(&s, &req), "requests after the end");
    }

    /* The host moving the link now and then, the search getting on in
       between: the moves do not add up, the search runs to its end. */
    {
        struct hl_search s;
        struct hl_params req;
        const struct hl_link own = {24, 0, 420};
        struct hl_link link = own;
        struct hl_search_memory memory = fresh_memory();
        int moves = 0;

        hl_search_start(&s, &limits, &memory, &link);
        while (hl_search_next(&s, &req)) {
            const bool restore = s.asked_kind == HL_ASK_RESTORE;

            link = pc_low(link, &req);
            hl_search_answer(&s, &link, false);
            if (!restore && moves < 4) {
                /* Once after each of the first answers: never twice in a row. */
                moves++;
                link = own;
                hl_search_link_moved(&s, &link);
            }
        }
        printf("host moving now and then  %d moves, best %u latency %u, keeps moving %d\n\n",
               moves, s.best_interval, s.best.latency, s.host_keeps_moving);
        CHECK(moves > HL_SEARCH_HOST_MOVES_TOLERATED, "the host did not get to move enough");
        CHECK(!s.host_keeps_moving, "moves with the search getting on between ended it");
        CHECK(s.best_interval == 6 && s.best.latency > 30, "the search did not run to its end");
    }

    /* Moves that are not the host's (ZMK Studio's request at connect): put
       back, never counted. Then the host's own, twice in a row, ends the
       search -- and the link back on the best clears that. */
    {
        struct hl_search s;
        struct hl_params req;
        const struct hl_link studio = {9, 0, 400};
        const struct hl_link own = {24, 0, 420};
        struct hl_link link = own;
        struct hl_search_memory memory = fresh_memory();

        hl_search_start(&s, &limits, &memory, &link);
        hl_search_next(&s, &req); /* secure */
        link = pc_low(link, &req);
        hl_search_answer(&s, &link, false);
        const struct hl_link on_best = link;

        for (int i = 0; i < 3; i++) {
            link = studio;
            hl_search_link_moved_not_by_host(&s, &link);
            CHECK(s.restoring && s.phase != HL_PHASE_DONE, "ZMK Studio's move %d not put back",
                  i);
            hl_search_next(&s, &req);
            CHECK(s.asked_kind == HL_ASK_RESTORE, "no restore after ZMK Studio's move");
            link = pc_low(link, &req);
            hl_search_answer(&s, &link, false);
        }
        CHECK(s.host_moves_in_a_row == 0 && !s.host_keeps_moving,
              "ZMK Studio's moves counted against the host");

        link = own;
        hl_search_link_moved(&s, &link);
        hl_search_next(&s, &req); /* restore */
        link = pc_low(link, &req);
        hl_search_answer(&s, &link, false);
        link = own;
        hl_search_link_moved(&s, &link);
        CHECK(s.host_keeps_moving && s.phase == HL_PHASE_DONE, "two moves in a row tolerated");
        link = on_best; /* the host puts it back after all */
        hl_search_link_moved(&s, &link);
        CHECK(!s.host_keeps_moving && !s.restoring && s.phase == HL_PHASE_DONE,
              "link back on the best: still marked as kept off it");
    }

    /* An answer to a request out when the search ended (the glue drops such
       requests; should one get through): taken, the search stays over. */
    {
        struct hl_search s;
        struct hl_params req;
        const struct hl_link own = {24, 0, 420};
        struct hl_link link = own;
        struct hl_search_memory memory = fresh_memory();

        hl_search_start(&s, &limits, &memory, &link);
        hl_search_next(&s, &req); /* secure */
        link = pc_low(link, &req);
        hl_search_answer(&s, &link, false);
        hl_search_next(&s, &req); /* 6-6, waiting to go out */
        const struct hl_params waiting = req;

        for (int i = 0; i < HL_SEARCH_HOST_MOVES_TOLERATED; i++) {
            link = own;
            hl_search_link_moved(&s, &link);
            if (s.phase != HL_PHASE_DONE) {
                struct hl_params restore;

                hl_search_next(&s, &restore);
                link = pc_low(link, &restore);
                hl_search_answer(&s, &link, false);
            }
        }
        CHECK(s.phase == HL_PHASE_DONE && s.host_keeps_moving, "the search did not end");
        s.asked = waiting;
        s.asked_kind = HL_ASK_FASTEST;
        link = pc_low(link, &waiting);
        hl_search_answer(&s, &link, false);
        CHECK(s.phase == HL_PHASE_DONE && !hl_search_next(&s, &req),
              "an answer after the end started the search again");
    }

    /* A request waiting to go out, the link moving to ZMK Studio-looking
       parameters every time: dropped, its step given back, the link put
       back, the same request again... The search must still end: past
       HL_SEARCH_MOVES_NOT_BY_HOST_TAKEN such moves count as the host's. */
    {
        struct hl_search s;
        struct hl_params req;
        const struct hl_link studio = {9, 10, 400};
        struct hl_link link = pc_opened;
        struct hl_search_memory memory = fresh_memory();
        int cycles = 0;

        hl_search_start(&s, &limits, &memory, &link);
        hl_search_next(&s, &req); /* secure */
        link = pc_low(link, &req);
        hl_search_answer(&s, &link, false);
        const struct hl_link on_best = link;

        while (hl_search_next(&s, &req) && cycles < 100) {
            if (s.asked_kind == HL_ASK_RESTORE) {
                link = on_best;
                hl_search_answer(&s, &link, false);
                continue;
            }
            cycles++;
            /* Waits to go out; the link moves meanwhile; dropped. */
            link = studio;
            hl_search_link_moved_not_by_host(&s, &link);
            hl_search_request_not_sent(&s);
        }
        printf("ZMK Studio-looking moves under unsent requests  %d cycles, requests %u, "
               "keeps moving %d\n\n",
               cycles, s.requests, s.host_keeps_moving);
        CHECK(cycles == HL_SEARCH_MOVES_NOT_BY_HOST_TAKEN + HL_SEARCH_HOST_MOVES_TOLERATED,
              "the search did not end where expected: %d cycles", cycles);
        CHECK(s.phase == HL_PHASE_DONE && s.host_keeps_moving && s.best_interval == 12,
              "the search did not end on what it found");
    }

    /* The host moved off the best once; the next request went on air, ZMK
       Studio's request landed on top of it, and while it waits to go out
       again the host answers it: the glue judges that as the answer -- not
       a second move of the host's. */
    {
        struct hl_search s;
        struct hl_params req;
        const struct hl_link own = {24, 0, 420};
        const struct hl_link studio = {9, 10, 400};
        struct hl_link link = own;
        struct hl_search_memory memory = fresh_memory();

        hl_search_start(&s, &limits, &memory, &link);
        hl_search_next(&s, &req); /* secure */
        link = pc_low(link, &req);
        hl_search_answer(&s, &link, false);
        link = own;
        hl_search_link_moved(&s, &link);
        hl_search_next(&s, &req); /* restore */
        link = pc_low(link, &req);
        hl_search_answer(&s, &link, false);
        CHECK(s.host_moves_in_a_row == 1, "the host's move not counted");
        hl_search_next(&s, &req); /* 6-6, on air */
        link = studio;
        hl_search_link_moved_not_by_host(&s, &link);
        link = pc_low(link, &req); /* the host's answer to 6-6 */
        hl_search_answer(&s, &link, false);
        CHECK(s.best_interval == 6 && !s.host_keeps_moving && s.phase != HL_PHASE_DONE &&
                  s.host_moves_in_a_row == 0,
              "the answer to the request on air lost: best %u, keeps moving %d",
              s.best_interval, s.host_keeps_moving);
    }

    /* The restore unanswered, then answered late: the link is on the best
       again, nothing failed. */
    {
        struct hl_search s;
        struct hl_params req;
        const struct hl_link own = {24, 0, 420};
        struct hl_link link = own;
        struct hl_search_memory memory = fresh_memory();

        hl_search_start(&s, &limits, &memory, &link);
        hl_search_next(&s, &req); /* secure */
        link = pc_low(link, &req);
        hl_search_answer(&s, &link, false);
        const struct hl_link on_best = link;

        link = own;
        hl_search_link_moved(&s, &link);
        hl_search_next(&s, &req); /* restore: no answer by the deadline */
        hl_search_answer(&s, &link, true);
        CHECK(s.restore_failed && s.phase == HL_PHASE_DONE, "restore not failed");
        link = on_best;
        hl_search_link_moved(&s, &link);
        CHECK(!s.restore_failed && !s.restoring, "the late restore left the search failed");
    }

    /* The host moving the link on its own to the interval of a request it
       left unanswered, at a latency of its own: not that request's answer --
       a move of the host's, put back. */
    {
        struct hl_search s;
        struct hl_params req;
        struct hl_link link = pc_opened;
        struct hl_search_memory memory = fresh_memory();

        hl_search_start(&s, &limits, &memory, &link);
        hl_search_next(&s, &req); /* secure */
        link = pc_low(link, &req);
        hl_search_answer(&s, &link, false);
        hl_search_next(&s, &req); /* 6-6, no answer */
        CHECK(s.asked_kind == HL_ASK_FASTEST, "6-6 expected");
        hl_search_answer(&s, &link, true);
        link = (struct hl_link){6, 0, TIMEOUT}; /* the host, on its own, at latency 0 */
        hl_search_link_moved(&s, &link);
        CHECK(s.best_interval == 12 && s.best.latency == 30 && s.restoring,
              "the host's own move taken as an answer: best %u latency %u", s.best_interval,
              s.best.latency);
        CHECK(s.memory.fastest_refused && s.host_moves_in_a_row == 1,
              "the host's own move not counted");
        link = (struct hl_link){6, 30, TIMEOUT}; /* ...and the real late answer after */
        hl_search_link_moved(&s, &link);
        CHECK(s.best_interval == 6 && !s.restoring && !s.memory.fastest_refused,
              "the exact late answer not taken");
    }

    /* Silence is no answer: the host's moves around a request it left
       unanswered still count in a row. */
    {
        struct hl_search s;
        struct hl_params req;
        const struct hl_link own = {24, 0, 420};
        struct hl_link link = own;
        struct hl_search_memory memory = fresh_memory();

        hl_search_start(&s, &limits, &memory, &link);
        hl_search_next(&s, &req); /* secure */
        link = pc_low(link, &req);
        hl_search_answer(&s, &link, false);
        link = own;
        hl_search_link_moved(&s, &link);
        hl_search_next(&s, &req); /* restore */
        link = pc_low(link, &req);
        hl_search_answer(&s, &link, false);
        hl_search_next(&s, &req); /* 6-6: no answer */
        hl_search_answer(&s, &link, true);
        CHECK(s.host_moves_in_a_row == 1, "silence started the count over");
        link = own;
        hl_search_link_moved(&s, &link);
        CHECK(s.host_keeps_moving && s.phase == HL_PHASE_DONE,
              "two moves around a silent request tolerated");
    }

    /* A request that never went out gives its step back: the request limit,
       the candidate's turn, the latency step. A restore has none. */
    {
        struct hl_search s;
        struct hl_params req, again;
        struct hl_link link = mac_opened;
        struct hl_search_memory memory = fresh_memory();

        hl_search_start(&s, &limits, &memory, &link);
        hl_search_next(&s, &req); /* secure */
        link = mac_strict(link, &req);
        hl_search_answer(&s, &link, false);
        hl_search_next(&s, &req); /* 6-6, refused */
        link = mac_strict(link, &req);
        hl_search_answer(&s, &link, false);
        hl_search_next(&s, &req);
        CHECK(s.asked_kind == HL_ASK_FALLBACK_CANDIDATE, "no candidate");
        const uint8_t requests = s.requests, candidate = s.asked_candidate;

        hl_search_request_not_sent(&s);
        CHECK(s.requests == requests - 1, "the request limit not given back");
        hl_search_next(&s, &again);
        CHECK(s.asked_kind == HL_ASK_FALLBACK_CANDIDATE && s.asked_candidate == candidate &&
                  again.interval_min == req.interval_min && s.requests == requests,
              "the candidate lost its turn (%u, not %u)", s.asked_candidate, candidate);
    }
    {
        struct hl_search s;
        struct hl_params req, again;
        struct hl_link link = pc_opened;
        struct hl_search_memory memory = fresh_memory();

        hl_search_start(&s, &limits, &memory, &link);
        while (hl_search_next(&s, &req) && s.asked_kind != HL_ASK_LATENCY) {
            link = pc_low(link, &req);
            hl_search_answer(&s, &link, false);
        }
        const uint8_t steps = s.latency_steps, requests = s.requests;

        hl_search_request_not_sent(&s);
        CHECK(s.latency_steps == steps - 1 && s.requests == requests - 1,
              "the latency step not given back");
        hl_search_next(&s, &again);
        CHECK(s.asked_kind == HL_ASK_LATENCY && again.latency == req.latency,
              "a different latency asked after giving the step back: %u, not %u", again.latency,
              req.latency);

        link = (struct hl_link){24, 0, 420}; /* off the best: a restore */
        hl_search_link_moved(&s, &link);
        hl_search_next(&s, &req);
        CHECK(s.asked_kind == HL_ASK_RESTORE, "restore expected");
        const uint8_t before = s.requests;

        hl_search_request_not_sent(&s);
        CHECK(s.requests == before, "a restore gave back a step it never took");
    }

    /* Latency 0 is always worse: a host giving it to everything ends on the
       slowest interval at once, one that gives it to the faster ones keeps
       15 ms at its latency. Fixed at 0 by hand, the fastest interval still
       wins. */
    {
        struct hl_search_memory memory = fresh_memory();
        const struct hl_search s = run("latency 0", pc_latency_0, pc_opened, &limits, &memory);

        CHECK(s.have_best && s.best_interval == 12 && s.best.latency == 0 && s.requests == 1,
              "latency 0 everywhere: best %u lat %u after %u requests", s.best_interval,
              s.best.latency, s.requests);
    }
    {
        struct hl_search_memory memory = fresh_memory();
        const struct hl_search s =
            run("fast at 0", pc_fast_latency_0, pc_opened, &limits, &memory);

        CHECK(s.have_best && s.best_interval == 12 && s.best.latency == 30 &&
                  s.requests == 2 && !s.restore_failed,
              "latency 0 on the faster: best %u lat %u after %u requests", s.best_interval,
              s.best.latency, s.requests);
    }
    {
        /* The keyboard asks latency 0 itself (no latency while searching, or
           a ceiling of 0): compared as any latency -- the faster interval
           wins, no going back and forth. */
        struct hl_search_limits zero = limits;
        struct hl_search_memory memory = fresh_memory();

        zero.latency_while_searching_interval = 0;
        const struct hl_search s = run("asks 0 itself", pc_low, pc_opened, &zero, &memory);

        CHECK(s.have_best && s.best_interval == 6 && s.requests <= 3,
              "latency 0 asked by the keyboard: best %u after %u requests", s.best_interval,
              s.requests);
    }
    {
        /* The faster interval at the host's latency 0, not kept: still the
           request a drop meanwhile is about. */
        struct hl_search s;
        struct hl_params req;
        struct hl_link link = pc_opened;
        struct hl_search_memory memory = fresh_memory();

        hl_search_start(&s, &limits, &memory, &link);
        hl_search_next(&s, &req); /* secure, taken at 30 */
        link = pc_fast_latency_0(link, &req);
        hl_search_answer(&s, &link, false);
        const uint8_t taken = s.taken_count;

        hl_search_next(&s, &req); /* 6-6, given at 0 */
        link = pc_fast_latency_0(link, &req);
        hl_search_answer(&s, &link, false);
        CHECK(s.taken_count != taken && s.taken_kind == HL_ASK_FASTEST && s.restoring,
              "the request the link ran at 0 not noted as taken");
        CHECK(s.best_interval == 12 && s.host_forces_latency_0, "best or flag wrong");
    }
    {
        struct hl_search_limits pinned = limits;
        struct hl_search_memory memory = fresh_memory();

        pinned.pinned_latency = 0;
        const struct hl_search s = run("pinned 0", pc_latency_0, pc_opened, &pinned, &memory);

        CHECK(s.have_best && s.best_interval == 6 && s.best.latency == 0,
              "latency fixed at 0 by hand: best %u", s.best_interval);
        printf("\n");
    }

    /* Fastest equal to slowest: the secure request is all there is. */
    {
        struct hl_search_limits one = limits;
        struct hl_search_memory memory = fresh_memory();

        one.fastest_interval = 12;
        const struct hl_search s = run("only 15 ms", pc_low, pc_opened, &one, &memory);

        CHECK(s.have_best && s.best_interval == 12 && s.requests == 1 && s.best.latency == 30,
              "fastest == slowest");
        printf("\n");
    }

    /* A board without a pointer: up to 30 ms, the candidate mask nearly full. */
    {
        struct hl_search_limits wide = limits;
        struct hl_search_memory memory = fresh_memory();

        wide.slowest_interval = 24;
        CHECK(hl_fallback_candidate_count(&wide) == 31, "candidates up to 30 ms: %u",
              hl_fallback_candidate_count(&wide));
        uint16_t min, max;

        CHECK(hl_fallback_candidate(&wide, 30, &min, &max) && min == 22 && max == 22,
              "last candidate %u-%u", min, max);
        run("mac_strict 30", mac_strict, mac_opened, &wide, &memory);
        run("  again", mac_strict, mac_opened, &wide, &memory);
        printf("\n");
    }

    /* A latency fixed by hand and a protected candidate together. */
    {
        struct hl_search_limits pinned = limits;
        struct hl_search_memory memory = fresh_memory();
        const uint8_t index = hl_fallback_candidate_index(&limits, 9, 21);

        pinned.pinned_latency = 10;
        memory.fastest_refused = true;
        memory.protected_fallback_candidate = index;
        run("refuses lat10", refuses_all, mac_opened, &pinned, &memory);
        CHECK(!(memory.fallback_candidates_tried & (1u << index)),
              "protected candidate marked with a pinned latency");
        printf("\n");
    }

    /* The timeout nudge against Zephyr's -EALREADY. */
    {
        const struct hl_params range = {9, 21, 30, TIMEOUT};

        CHECK(hl_timeout_that_reaches_the_host(&range, &(struct hl_link){12, 30, TIMEOUT}) ==
                  TIMEOUT + 1,
              "range holding the link is not nudged");
        CHECK(hl_timeout_that_reaches_the_host(&range, &(struct hl_link){9, 30, TIMEOUT}) ==
                  TIMEOUT,
              "a matching link is nudged");
        CHECK(hl_timeout_that_reaches_the_host(&range, &(struct hl_link){12, 0, TIMEOUT}) ==
                  TIMEOUT,
              "a link Zephyr sends to anyway is nudged");
        CHECK(hl_timeout_that_reaches_the_host(&(struct hl_params){9, 21, 30, 600},
                                               &(struct hl_link){12, 30, 600}) == 599,
              "nudge leaves Apple's 6 s");
    }

    /* The candidate list: ranges from 11.25 ms, then exact 8.75..13.75 ms. */
    CHECK(hl_fallback_candidate_count(&limits) == 8, "candidate count %u",
          hl_fallback_candidate_count(&limits));
    CHECK(hl_fallback_candidate_index(&limits, 7, 7) == 3, "exact 7 index");
    CHECK(hl_fallback_candidate_index(&limits, 6, 6) == HL_NO_FALLBACK_CANDIDATE,
          "the fastest is not a candidate");

    /* The rule helpers. */
    CHECK(hl_latency_for_idle_listen_period(6, 465, TIMEOUT, false) == 61, "lat @7.5");
    CHECK(hl_latency_for_idle_listen_period(9, 465, TIMEOUT, false) == 40, "lat @11.25");
    CHECK(hl_latency_for_idle_listen_period(12, 465, TIMEOUT, false) == 30, "lat @15");
    CHECK(hl_latency_for_idle_listen_period(12, 465, TIMEOUT, true) == 30, "lat @15 apple");
    CHECK(hl_latency_for_idle_listen_period(9, 465, TIMEOUT, true) == 30, "lat @11.25 apple");
    CHECK(hl_latency_allowed(499, 6, TIMEOUT, false) == 265, "spec cap @7.5");
    /* A stored result asked at a shorter timeout than it was found at: its
       latency within what that timeout allows (compose_request). */
    CHECK(hl_latency_allowed(30, 21, 200, true) == 24, "apple cap 11.25-26.25 @2 s: %u",
          hl_latency_allowed(30, 21, 200, true));
    CHECK(hl_latency_allowed(30, 22, 240, true) == 28, "apple cap 12.5-27.5 @2.4 s: %u",
          hl_latency_allowed(30, 22, 240, true));
    CHECK(hl_latency_allowed(30, 21, 410, true) == 30, "apple 11.25-26.25 @4.1 s: %u",
          hl_latency_allowed(30, 21, 410, true));
    CHECK(hl_latency_allowed(61, 6, 410, false) == 61, "7.5 ms latency 61 @4.1 s");

    printf("%s (%d failures)\n", failures ? "FAILED" : "ALL PASSED", failures);
    return failures ? 1 : 0;
}
