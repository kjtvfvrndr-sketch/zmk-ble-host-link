/*
 * SPDX-License-Identifier: MIT
 *
 * The decisions around the search (src/decisions.c) as tables of cases:
 * what happens to a stored result the host refuses, connection after
 * connection; what a finished search changes; which drops count, what they
 * change and which request they are about; when ZMK Studio has the link.
 *
 *   make -C tests        builds and runs it with the search tests
 */

#include <stdio.h>
#include <string.h>

#include "../src/decisions.h"

static int failures;

#define CHECK(cond, ...)                                                                           \
    do {                                                                                           \
        if (!(cond)) {                                                                             \
            printf("    FAIL (line %d): ", __LINE__);                                              \
            printf(__VA_ARGS__);                                                                   \
            printf("\n");                                                                          \
            failures++;                                                                            \
        }                                                                                          \
    } while (0)

static const struct hl_params P6 = {6, 6, 61, 400};
static const struct hl_params P9 = {9, 21, 30, 400};
static const struct hl_params P12 = {12, 12, 30, 400};

static struct hl_learned stored(const struct hl_params *found) {
    return (struct hl_learned){
        .flags = HL_LEARNED_SEARCHED | HL_LEARNED_FOUND,
        .found = *found,
    };
}

/* A finished search with this result (have_best false for NULL). */
static struct hl_search search_with(const struct hl_params *best, uint32_t tried) {
    struct hl_search s;

    memset(&s, 0, sizeof(s));
    s.memory.fallback_candidates_tried = tried;
    if (best) {
        s.have_best = true;
        s.best = *best;
        s.best_interval = best->interval_min;
    }
    return s;
}

static const char *step_name(enum hl_stored_refusal_step step) {
    switch (step) {
    case HL_STORED_REFUSED_ASK_FALLBACK:
        return "fallback";
    case HL_STORED_REFUSED_ASK_SECURE:
        return "secure";
    case HL_STORED_REFUSED_SEARCH_AGAIN:
        return "search";
    case HL_STORED_REFUSED_FALLBACK_REPLACES_IT:
        return "replace";
    case HL_STORED_REFUSED_DROPPED:
        return "drop";
    }
    return "?";
}

/* ------------------------------------------------ stored result refused */

/*
 * A script, one connection after another. Each step: what happened on the
 * connection, and what the module must do next.
 */
enum event {
    REFUSED,          /* refused twice, the first time on a connection */
    REFUSED_SAME,     /* refused twice again on the same connection */
    ACCEPTED,         /* the stored result accepted */
    SEARCH_FOUND_6,   /* the search run again found 7.5 ms */
    SEARCH_FOUND_12,  /* ...found only 15 ms */
    SEARCH_NOTHING,   /* ...found nothing */
};

struct step {
    enum event event;
    int want_step;    /* hl_stored_refusal_step, or -1 for search outcomes */
    int want_outcome; /* hl_search_outcome, or -1 */
    uint8_t want_refusals;
};

struct script {
    const char *name;
    struct hl_learned start;
    struct step steps[10];
    size_t count;
    /* At the end. */
    uint16_t want_found_interval; /* 0: no stored result (asks nothing) */
    bool want_fallback;
};

static void run_script(const struct script *sc) {
    struct hl_learned l = sc->start;

    printf("%-26s", sc->name);
    for (size_t i = 0; i < sc->count; i++) {
        const struct step *st = &sc->steps[i];
        struct hl_search s;

        switch (st->event) {
        case REFUSED:
        case REFUSED_SAME: {
            const enum hl_stored_refusal_step got =
                hl_stored_result_refused(&l, st->event == REFUSED);

            printf(" %s", step_name(got));
            CHECK((int)got == st->want_step, "%s step %zu: %s", sc->name, i, step_name(got));
            break;
        }
        case ACCEPTED:
            hl_stored_result_accepted(&l);
            printf(" ok");
            break;
        case SEARCH_FOUND_6:
        case SEARCH_FOUND_12:
        case SEARCH_NOTHING: {
            s = search_with(st->event == SEARCH_FOUND_6    ? &P6
                            : st->event == SEARCH_FOUND_12 ? &P12
                                                           : NULL,
                            0);
            const enum hl_search_outcome got = hl_search_finished(&l, &s, true, 400);

            printf(" [%d]", (int)got);
            CHECK((int)got == st->want_outcome, "%s step %zu: outcome %d", sc->name, i, got);
            break;
        }
        }
        CHECK(l.refused_connections == st->want_refusals, "%s step %zu: %u refusals", sc->name, i,
              l.refused_connections);
        CHECK(l.refused_connections <= HL_STORED_REFUSALS_THAT_REPLACE_IT,
              "%s: refusal count past the cap", sc->name);
    }
    printf("\n");

    if (sc->want_found_interval == 0) {
        CHECK(hl_learned_asks_nothing(&l), "%s: should ask nothing", sc->name);
    } else {
        CHECK((l.flags & HL_LEARNED_FOUND) && l.found.interval_min == sc->want_found_interval,
              "%s: stored %u, wanted %u", sc->name, l.found.interval_min,
              sc->want_found_interval);
    }
    CHECK(!!(l.flags & HL_LEARNED_HAS_FALLBACK) == sc->want_fallback, "%s: fallback %d",
          sc->name, !!(l.flags & HL_LEARNED_HAS_FALLBACK));
}

#define S_FALLBACK HL_STORED_REFUSED_ASK_FALLBACK
#define S_SECURE HL_STORED_REFUSED_ASK_SECURE
#define S_SEARCH HL_STORED_REFUSED_SEARCH_AGAIN
#define S_REPLACE HL_STORED_REFUSED_FALLBACK_REPLACES_IT
#define S_DROP HL_STORED_REFUSED_DROPPED
#define O_REPLACED HL_SEARCH_OUTCOME_REPLACED_STORED
#define O_FALLBACK HL_SEARCH_OUTCOME_FALLBACK
#define O_NOTHING HL_SEARCH_OUTCOME_NOTHING_CHANGED

static void stored_result_scripts(void) {
    const struct script scripts[] = {
        {"busy host, back to normal",
         stored(&P6),
         {{REFUSED, S_SECURE, -1, 1},
          {REFUSED, S_SEARCH, -1, 2},
          {SEARCH_FOUND_12, -1, O_FALLBACK, 2},
          {REFUSED, S_FALLBACK, -1, 3},
          {REFUSED_SAME, S_FALLBACK, -1, 3},
          {ACCEPTED, -1, -1, 0}},
         6,
         6,
         true},
        {"host changed for good",
         stored(&P6),
         {{REFUSED, S_SECURE, -1, 1},
          {REFUSED, S_SEARCH, -1, 2},
          {SEARCH_FOUND_12, -1, O_FALLBACK, 2},
          {REFUSED, S_FALLBACK, -1, 3},
          {REFUSED, S_FALLBACK, -1, 4},
          {REFUSED, S_REPLACE, -1, 0}},
         6,
         12,
         false},
        {"search again cut short",
         stored(&P6),
         {{REFUSED, S_SECURE, -1, 1},
          {REFUSED, S_SEARCH, -1, 2},
          /* the search is lost (link down, profile switch, Studio) */
          {REFUSED, S_SEARCH, -1, 3},
          {SEARCH_FOUND_6, -1, O_REPLACED, 0}},
         4,
         6,
         false},
        {"searched again, nothing",
         stored(&P6),
         {{REFUSED, S_SECURE, -1, 1},
          {REFUSED, S_SEARCH, -1, 2},
          {SEARCH_NOTHING, -1, O_NOTHING, 2},
          {REFUSED, S_SECURE, -1, 3},
          {REFUSED, S_SECURE, -1, 4},
          {REFUSED, S_DROP, -1, 0}},
         6,
         0,
         false},
        {"searched again, as good",
         stored(&P9),
         {{REFUSED, S_SECURE, -1, 1}, {REFUSED, S_SEARCH, -1, 2}, {SEARCH_FOUND_6, -1, O_REPLACED, 0}},
         3,
         6,
         false},
        {"no searching allowed",
         {.flags = HL_LEARNED_SEARCHED | HL_LEARNED_FOUND | HL_LEARNED_NO_SEARCHING, .found = P6},
         {{REFUSED, S_SECURE, -1, 1},
          {REFUSED, S_SECURE, -1, 2},
          {REFUSED, S_SECURE, -1, 3},
          {REFUSED, S_SECURE, -1, 4},
          {REFUSED, S_DROP, -1, 0}},
         5,
         0,
         false},
        {"old fallback still there",
         {.flags = HL_LEARNED_SEARCHED | HL_LEARNED_FOUND | HL_LEARNED_HAS_FALLBACK,
          .found = P6,
          .fallback = P12},
         {{REFUSED, S_FALLBACK, -1, 1},
          {REFUSED, S_FALLBACK, -1, 2},
          {REFUSED, S_FALLBACK, -1, 3},
          {REFUSED, S_FALLBACK, -1, 4},
          {REFUSED, S_REPLACE, -1, 0}},
         5,
         12,
         false},
        {"one refusal per connection",
         stored(&P6),
         {{REFUSED, S_SECURE, -1, 1},
          {REFUSED_SAME, S_SECURE, -1, 1},
          {REFUSED_SAME, S_SECURE, -1, 1},
          {ACCEPTED, -1, -1, 0}},
         4,
         6,
         false},
    };

    printf("Stored result refused, connection after connection:\n");
    for (size_t i = 0; i < sizeof(scripts) / sizeof(scripts[0]); i++) {
        run_script(&scripts[i]);
    }

    /* An accepted stored result with nothing to reset writes nothing. */
    struct hl_learned l = stored(&P6);

    CHECK(!hl_stored_result_accepted(&l), "accepted with nothing to reset reports a change");
    printf("\n");
}

/* ------------------------------------------------- a finished search */

static void search_outcomes(void) {
    printf("Finished searches:\n");

    /* A first search: its result is stored, everything else reset. */
    struct hl_learned l = {.flags = HL_LEARNED_HAS_FALLBACK, .refused_connections = 3};
    struct hl_search s = search_with(&P9, 0x5);

    s.memory.fastest_refused = true;
    s.best.timeout = 401; /* moved to reach the host */
    CHECK(hl_search_finished(&l, &s, false, 400) == HL_SEARCH_OUTCOME_STORED, "first search");
    CHECK((l.flags & HL_LEARNED_FOUND) && l.found.interval_min == 9 && l.found.timeout == 400,
          "first search: stored 9 at timeout 400");
    CHECK((l.flags & HL_LEARNED_FASTEST_REFUSED) && l.fallback_candidates_tried == 0x5,
          "first search: memory kept");
    CHECK(!(l.flags & HL_LEARNED_HAS_FALLBACK) && l.refused_connections == 0,
          "first search: fallback and count reset");

    /* A first search that found nothing: asks nothing from then on. */
    l = (struct hl_learned){0};
    s = search_with(NULL, 0);
    hl_search_finished(&l, &s, false, 400);
    CHECK(hl_learned_asks_nothing(&l), "nothing found: asks nothing");

    /* Searched again, nothing: a busy host must not spoil the memory. */
    l = stored(&P6);
    l.fallback_candidates_tried = 0x1;
    s = search_with(NULL, 0xFF);
    s.memory.fastest_refused = true;
    CHECK(hl_search_finished(&l, &s, true, 400) == HL_SEARCH_OUTCOME_NOTHING_CHANGED,
          "again, nothing");
    CHECK(l.fallback_candidates_tried == 0x1 && !(l.flags & HL_LEARNED_FASTEST_REFUSED),
          "again, nothing: memory spoiled");
    CHECK((l.flags & HL_LEARNED_FOUND) && l.found.interval_min == 6, "again, nothing: stored lost");

    /* Searched again, slower: what the search learned waits with the
       fallback, and holds only once the fallback replaces the stored one. */
    l = stored(&P6);
    l.fallback_candidates_tried = 0x1;
    s = search_with(&P12, 0xFE);
    s.memory.fastest_refused = true;
    CHECK(hl_search_finished(&l, &s, true, 400) == HL_SEARCH_OUTCOME_FALLBACK, "again, slower");
    CHECK(l.fallback_candidates_tried == 0x1 && !(l.flags & HL_LEARNED_FASTEST_REFUSED),
          "again, slower: memory applied before the fallback replaced anything");
    l.refused_connections = HL_STORED_REFUSALS_THAT_REPLACE_IT - 1;
    CHECK(hl_stored_result_refused(&l, true) == HL_STORED_REFUSED_FALLBACK_REPLACES_IT,
          "fallback did not replace");
    CHECK(l.fallback_candidates_tried == 0xFE && (l.flags & HL_LEARNED_FASTEST_REFUSED),
          "fallback replaced, its search's memory not applied");

    /* no_worse_than */
    CHECK(hl_no_worse_than(&P6, false, &P9, false) && !hl_no_worse_than(&P12, false, &P6, false),
          "no_worse_than intervals");
    {
        const struct hl_params p6_0 = {6, 6, 0, 410}, p12_0 = {12, 12, 0, 410};

        CHECK(!hl_no_worse_than(&p6_0, true, &P12, false) &&
                  hl_no_worse_than(&P12, false, &p6_0, true),
              "the host's latency 0 is worse than any latency above it");
        CHECK(hl_no_worse_than(&p12_0, true, &p6_0, true) &&
                  !hl_no_worse_than(&p6_0, true, &p12_0, true),
              "at the host's latency 0 the slower interval wins");
        CHECK(hl_no_worse_than(&p6_0, false, &p12_0, false) &&
                  !hl_no_worse_than(&p12_0, false, &p6_0, false),
              "latency 0 asked for (by hand): the faster interval wins");
    }
    const struct hl_params p6_30 = {6, 6, 30, 400};

    CHECK(hl_no_worse_than(&P6, false, &p6_30, false) && !hl_no_worse_than(&p6_30, false, &P6, false),
          "no_worse_than latency");
    printf("  done\n\n");
}

/* --------------------------------------------------------------- drops */

/* A search run again weighed against the stored result as it is asked: at a
   shorter timeout an Apple host's 11.25-26.25 ms goes out at latency 24, and
   finding exactly that replaces it. */
static void search_outcomes_at_timeout(void) {
    printf("Search outcomes against the stored result as asked:\n");
    struct hl_learned l = stored(&P9);
    struct hl_search s;
    const struct hl_params p9_at_2s = {9, 21, 24, 200};

    l.flags |= HL_LEARNED_FASTEST_REFUSED;
    s = search_with(&p9_at_2s, 0);
    s.memory.fastest_refused = true;
    CHECK(hl_search_finished(&l, &s, true, 200) == HL_SEARCH_OUTCOME_REPLACED_STORED,
          "the stored result as asked at 2 s found again: not replaced");

    l = stored(&P9);
    l.flags |= HL_LEARNED_FASTEST_REFUSED;
    s = search_with(&p9_at_2s, 0);
    s.memory.fastest_refused = true;
    CHECK(hl_search_finished(&l, &s, true, 410) == HL_SEARCH_OUTCOME_FALLBACK,
          "at 4.1 s latency 24 is slower than the stored 30");
    printf("  done\n\n");
}

/* The host's latency 0 against latency 0 asked for: a search run again. */
static void search_outcomes_latency_0(void) {
    printf("Search outcomes, latency 0:\n");
    const struct hl_params p9_0 = {9, 21, 0, 410}, p6_0 = {6, 6, 0, 410},
                           p12_0 = {12, 12, 0, 410};
    struct hl_learned l;
    struct hl_search s;

    /* Fixed at 0 by hand: the faster interval found again replaces. */
    l = stored(&p9_0);
    s = search_with(&p6_0, 0);
    CHECK(hl_search_finished(&l, &s, true, 410) == HL_SEARCH_OUTCOME_REPLACED_STORED,
          "asked 0: 7.5 ms does not replace 11.25 ms");
    l = stored(&p6_0);
    s = search_with(&p9_0, 0);
    CHECK(hl_search_finished(&l, &s, true, 410) == HL_SEARCH_OUTCOME_FALLBACK,
          "asked 0: 11.25 ms replaces 7.5 ms");

    /* The host's 0: the slower interval, anything with latency above. */
    l = stored(&p6_0);
    l.flags |= HL_LEARNED_FOUND_LATENCY_0_BY_HOST;
    s = search_with(&p12_0, 0);
    s.best_latency_0_by_host = true;
    CHECK(hl_search_finished(&l, &s, true, 410) == HL_SEARCH_OUTCOME_REPLACED_STORED &&
              (l.flags & HL_LEARNED_FOUND_LATENCY_0_BY_HOST),
          "host's 0: 15 ms does not replace 7.5 ms, or the mark is lost");
    l = stored(&P12);
    s = search_with(&p6_0, 0);
    s.best_latency_0_by_host = true;
    CHECK(hl_search_finished(&l, &s, true, 410) == HL_SEARCH_OUTCOME_FALLBACK &&
              (l.flags & HL_LEARNED_FALLBACK_LATENCY_0_BY_HOST),
          "host's 0 replaces 15 ms at latency 30, or the fallback's mark is lost");
    printf("  done\n\n");
}

static void drops(void) {
    printf("Drops:\n");

    /* Reasons */
    CHECK(hl_drop_reason(0x16) == HL_DROP_NOT_THE_HOSTS, "0x16");
    CHECK(hl_drop_reason(0x3B) == HL_DROP_COUNTS_NOW, "0x3B");
    CHECK(hl_drop_reason(0x28) == HL_DROP_COUNTS_IF_HOST_RETURNS, "0x28 waits");
    CHECK(hl_drop_reason(0x08) == HL_DROP_COUNTS_IF_HOST_RETURNS, "0x08");
    CHECK(hl_drop_reason(0x13) == HL_DROP_COUNTS_IF_HOST_RETURNS, "0x13");
    CHECK(hl_drop_counts_on_return(5000, 30000) && !hl_drop_counts_on_return(31000, 30000),
          "host return window");

    /* What a counted drop changes, by request. */
    struct {
        const char *name;
        struct hl_learned start;
        struct hl_dropped_request request;
        unsigned int want_bits;
        uint16_t want_ceiling_plus1;
    } cases[] = {
        {"fastest, searching",
         {0},
         {false, HL_ASK_FASTEST, 0, {6, 6, 30, 400}},
         HL_DROP_SKIPS_FASTEST,
         0},
        {"candidate 2",
         {0},
         {false, HL_ASK_FALLBACK_CANDIDATE, 2, {11, 23, 30, 400}},
         HL_DROP_SKIPS_CANDIDATE,
         0},
        {"latency 61",
         {0},
         {false, HL_ASK_LATENCY, 0, {6, 6, 61, 400}},
         HL_DROP_LOWERS_LATENCY,
         62},
        {"latency above the ceiling",
         {.latency_ceiling_plus1 = 41},
         {false, HL_ASK_LATENCY, 0, {6, 6, 61, 400}},
         0,
         41},
        {"stored result took effect",
         stored(&P6),
         {true, HL_ASK_RESTORE, 0, P6},
         HL_DROP_LOWERS_LATENCY | HL_DROP_CLEARS_STORED,
         62},
        {"last search request = stored",
         stored(&P9),
         {false, HL_ASK_FALLBACK_CANDIDATE, 0, P9},
         HL_DROP_SKIPS_CANDIDATE | HL_DROP_CLEARS_STORED,
         0},
        /* The stored 7.5 ms at latency 61 is no safer than 7.5 ms at 30. */
        {"fastest below stored latency",
         stored(&P6),
         {false, HL_ASK_FASTEST, 0, {6, 6, 30, 410}},
         HL_DROP_SKIPS_FASTEST | HL_DROP_CLEARS_STORED,
         0},
        /* The stored result asked with its latency lowered (shorter
           timeout): still the stored result. */
        {"stored, latency lowered",
         stored(&P9),
         {false, HL_ASK_FALLBACK_CANDIDATE, 0, {9, 21, 24, 200}},
         HL_DROP_SKIPS_CANDIDATE | HL_DROP_CLEARS_STORED,
         0},
        /* An exact request at the stored range's minimum ran the link
           alike. */
        {"exact at the stored range's minimum",
         stored(&P9),
         {false, HL_ASK_FALLBACK_CANDIDATE, 5, {9, 9, 30, 410}},
         HL_DROP_SKIPS_CANDIDATE | HL_DROP_CLEARS_STORED,
         0},
        /* Another interval: says nothing about the stored result. */
        {"other interval",
         stored(&P9),
         {false, HL_ASK_FALLBACK_CANDIDATE, 1, {10, 22, 30, 410}},
         HL_DROP_SKIPS_CANDIDATE,
         0},
        /* More latency than the stored result: says nothing about it. */
        {"latency above stored",
         stored(&P12),
         {false, HL_ASK_LATENCY, 0, {12, 12, 45, 410}},
         HL_DROP_LOWERS_LATENCY,
         46},
        /* The fallback broke the link: it goes, the stored result stays. */
        {"fallback",
         {.flags = HL_LEARNED_SEARCHED | HL_LEARNED_FOUND | HL_LEARNED_HAS_FALLBACK |
                   HL_LEARNED_SEARCHED_AGAIN,
          .found = P6,
          .fallback = P9},
         {false, HL_ASK_FALLBACK_CANDIDATE, 0, P9},
         HL_DROP_SKIPS_CANDIDATE | HL_DROP_CLEARS_FALLBACK,
         0},
    };

    for (size_t i = 0; i < sizeof(cases) / sizeof(cases[0]); i++) {
        struct hl_learned l = cases[i].start;
        uint8_t strikes = 0;
        const unsigned int got = hl_count_drop(&l, &strikes, &cases[i].request);

        printf("  %-30s bits 0x%02x\n", cases[i].name, got);
        CHECK(got == cases[i].want_bits, "%s: bits 0x%02x, wanted 0x%02x", cases[i].name, got,
              cases[i].want_bits);
        CHECK(l.latency_ceiling_plus1 == cases[i].want_ceiling_plus1, "%s: ceiling %u",
              cases[i].name, l.latency_ceiling_plus1);
        if (got & HL_DROP_CLEARS_STORED) {
            CHECK(!(l.flags & (HL_LEARNED_FOUND | HL_LEARNED_SEARCHED)),
                  "%s: stored result not cleared", cases[i].name);
        }
        if (got & HL_DROP_CLEARS_FALLBACK) {
            CHECK(!(l.flags & (HL_LEARNED_HAS_FALLBACK | HL_LEARNED_SEARCHED_AGAIN)) &&
                      (l.flags & HL_LEARNED_FOUND),
                  "%s: fallback not cleared, or the stored result with it", cases[i].name);
        }
    }

    /* Two in a row stop the searching. */
    struct hl_learned l = {0};
    uint8_t strikes = 0;
    const struct hl_dropped_request fastest = {false, HL_ASK_FASTEST, 0, {6, 6, 30, 400}};

    hl_count_drop(&l, &strikes, &fastest);
    CHECK(!(l.flags & HL_LEARNED_NO_SEARCHING), "one drop stops searching");
    CHECK(hl_count_drop(&l, &strikes, &fastest) & HL_DROP_STOPS_SEARCHING, "two drops");
    CHECK(l.flags & HL_LEARNED_NO_SEARCHING, "two drops: flag");

    /* Which request a drop is about. */
    struct {
        bool out, taken;
        int64_t since;
        enum hl_drop_blame want;
    } blame[] = {
        {false, false, 0, HL_DROP_BLAMES_NOTHING},
        {true, false, 0, HL_DROP_BLAMES_REQUEST_OUT},
        {false, true, 1000, HL_DROP_BLAMES_REQUEST_TAKEN},
        {false, true, 7000, HL_DROP_BLAMES_NOTHING},
        /* a late answer moved the link while the next request was out */
        {true, true, 1000, HL_DROP_BLAMES_REQUEST_TAKEN},
        {true, true, 7000, HL_DROP_BLAMES_REQUEST_OUT},
    };

    for (size_t i = 0; i < sizeof(blame) / sizeof(blame[0]); i++) {
        CHECK(hl_drop_blame(blame[i].out, blame[i].taken, blame[i].since, 6000) == blame[i].want,
              "blame case %zu", i);
    }
    printf("  done\n\n");
}

/* --------------------------------------------------------- ZMK Studio */

static void zmk_studio(void) {
    printf("ZMK Studio:\n");
    struct {
        bool params, subscribed, at_connect;
        uint32_t up_ms;
        bool want;
    } cases[] = {
        {true, true, false, 60000, true},  /* opened while connected */
        {true, false, false, 60000, false}, /* the parameters only (a value fixed by hand) */
        {true, true, true, 60000, false},  /* the subscription that came with the bond */
        {true, true, false, 5000, false},  /* in the first seconds of the link */
        {false, true, false, 60000, false}, /* subscribed, but the link is ours */
    };

    for (size_t i = 0; i < sizeof(cases) / sizeof(cases[0]); i++) {
        CHECK(hl_zmk_studio_has_link(cases[i].params, cases[i].subscribed, cases[i].at_connect,
                                     cases[i].up_ms, 15000) == cases[i].want,
              "studio case %zu", i);
    }
    printf("  done\n\n");
}

int main(void) {
    stored_result_scripts();
    search_outcomes();
    search_outcomes_at_timeout();
    search_outcomes_latency_0();
    drops();
    zmk_studio();

    printf("%s (%d failures)\n", failures ? "FAILED" : "ALL PASSED", failures);
    return failures ? 1 : 0;
}
