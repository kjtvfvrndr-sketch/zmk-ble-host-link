/*
 * SPDX-License-Identifier: MIT
 *
 * The decisions around the search (src/decisions.c), as tables of cases, on
 * the build machine:
 *   make -C tests
 */

#include <stdio.h>
#include <string.h>

#include "../src/decisions.h"

static int failures;

#define CHECK(cond, ...)                                                                           \
    do {                                                                                           \
        if (!(cond)) {                                                                             \
            failures++;                                                                            \
            printf("FAIL %s:%d: ", __FILE__, __LINE__);                                            \
            printf(__VA_ARGS__);                                                                   \
            printf("\n");                                                                          \
        }                                                                                          \
    } while (0)

#define R(i, l) ((struct hl_result){(i), (l)})
#define NONE R(0, 0)

static bool same(struct hl_result a, struct hl_result b) { return hl_same_result(&a, &b); }

/* ------------------------------------------------- stored and candidate */

static void test_first_search(void) {
    struct hl_learned l = {0};
    const struct hl_result found = R(6, 61);

    CHECK(!hl_first_search_finished(&l, NULL), "nothing found: no change");
    CHECK(!hl_result_exists(&l.stored), "nothing found: nothing stored");
    CHECK(hl_first_search_finished(&l, &found), "found: changed");
    CHECK(same(l.stored, R(6, 61)), "found: stored");
}

struct research_case {
    const char *name;
    struct hl_result stored, candidate;
    bool found_anything;
    struct hl_result found;
    enum hl_research_outcome outcome;
    struct hl_result stored_after, candidate_after;
};

static const struct research_case research_cases[] = {
    {"nothing found", R(6, 61), R(9, 40), false, NONE, HL_RESEARCH_FOUND_NOTHING, R(6, 61),
     R(9, 40)},
    {"found the stored result", R(6, 61), R(9, 40), true, R(6, 61), HL_RESEARCH_FOUND_STORED,
     R(6, 61), NONE},
    {"found something, no candidate", R(6, 61), NONE, true, R(12, 30),
     HL_RESEARCH_NEW_CANDIDATE, R(6, 61), R(12, 30)},
    {"found the candidate again", R(6, 61), R(12, 30), true, R(12, 30),
     HL_RESEARCH_CANDIDATE_STORED, R(12, 30), NONE},
    {"found another candidate", R(6, 61), R(12, 30), true, R(9, 40), HL_RESEARCH_NEW_CANDIDATE,
     R(6, 61), R(9, 40)},
    {"same interval, other latency", R(6, 61), R(6, 30), true, R(6, 45),
     HL_RESEARCH_NEW_CANDIDATE, R(6, 61), R(6, 45)},
};

static void test_research(void) {
    for (size_t i = 0; i < sizeof(research_cases) / sizeof(research_cases[0]); i++) {
        const struct research_case *c = &research_cases[i];
        struct hl_learned l = {.stored = c->stored, .candidate = c->candidate};
        const enum hl_research_outcome o =
            hl_search_after_stored_refused(&l, c->found_anything ? &c->found : NULL);

        CHECK(o == c->outcome, "%s: outcome %d, expected %d", c->name, o, c->outcome);
        CHECK(same(l.stored, c->stored_after), "%s: stored %u/%u", c->name, l.stored.interval,
              l.stored.latency);
        CHECK(same(l.candidate, c->candidate_after), "%s: candidate %u/%u", c->name,
              l.candidate.interval, l.candidate.latency);
    }

    /* Two connections that find the same: the second makes it stored. */
    struct hl_learned l = {.stored = R(6, 61)};
    const struct hl_result mac = R(12, 30);

    CHECK(hl_search_after_stored_refused(&l, &mac) == HL_RESEARCH_NEW_CANDIDATE, "two: first");
    CHECK(hl_search_after_stored_refused(&l, &mac) == HL_RESEARCH_CANDIDATE_STORED,
          "two: second");
    CHECK(same(l.stored, mac) && !hl_result_exists(&l.candidate), "two: stored now");
}

static void test_stored_accepted(void) {
    struct hl_learned l = {.stored = R(6, 61), .candidate = R(12, 30)};

    CHECK(hl_stored_result_accepted(&l), "accepted: changed");
    CHECK(!hl_result_exists(&l.candidate), "accepted: the candidate goes");
    CHECK(same(l.stored, R(6, 61)), "accepted: stored stays");
    CHECK(!hl_stored_result_accepted(&l), "accepted again: no change");
}

/* ------------------------------------------------------------- drops */

#define STABLE 30000
#define WITHIN 30000

static void test_drop_two_connections(void) {
    struct hl_drop_watch w = {0};
    struct hl_result confirmed = NONE;
    const struct hl_result req = R(6, 61);

    /* connection 1: drops after the request, host straight back */
    CHECK(hl_drop_seen(&w, 0x08, &req, 9000, STABLE, 1000, &confirmed) ==
              HL_DROP_SEEN_WAITS_FOR_HOST,
          "two: waits");
    CHECK(hl_host_returned(&w, 3000, WITHIN, &confirmed) == HL_RETURN_SUSPECTED, "two: once");
    /* connection 2: the same */
    CHECK(hl_drop_seen(&w, 0x08, &req, 9000, STABLE, 20000, &confirmed) ==
              HL_DROP_SEEN_WAITS_FOR_HOST,
          "two: waits again");
    CHECK(hl_host_returned(&w, 22000, WITHIN, &confirmed) == HL_RETURN_CONFIRMED,
          "two: confirmed");
    CHECK(same(confirmed, req), "two: the request");
}

static void test_drop_not_in_a_row(void) {
    struct hl_drop_watch w = {0};
    struct hl_result confirmed = NONE;
    const struct hl_result req = R(6, 61);
    const struct hl_result other = R(7, 30);

    hl_drop_seen(&w, 0x08, &req, 9000, STABLE, 1000, &confirmed);
    CHECK(hl_host_returned(&w, 2000, WITHIN, &confirmed) == HL_RETURN_SUSPECTED, "row: once");
    /* the request held on the next connection */
    hl_request_held(&w, &req);
    hl_drop_seen(&w, 0x08, &req, 9000, STABLE, 50000, &confirmed);
    CHECK(hl_host_returned(&w, 51000, WITHIN, &confirmed) == HL_RETURN_SUSPECTED,
          "row: held between, once again");
    /* another request blamed: starts over */
    hl_drop_seen(&w, 0x08, &other, 9000, STABLE, 90000, &confirmed);
    CHECK(hl_host_returned(&w, 91000, WITHIN, &confirmed) == HL_RETURN_SUSPECTED,
          "row: another request, once");
}

/* A search: the secure request holds before the step that drops. */
static void test_drop_other_request_held(void) {
    struct hl_drop_watch w = {0};
    struct hl_result confirmed = NONE;
    const struct hl_result secure = R(12, 30);
    const struct hl_result step = R(6, 30);

    hl_drop_seen(&w, 0x08, &step, 25000, STABLE, 1000, &confirmed);
    CHECK(hl_host_returned(&w, 2000, WITHIN, &confirmed) == HL_RETURN_SUSPECTED, "other: once");
    hl_request_held(&w, &secure);
    hl_drop_seen(&w, 0x08, &step, 25000, STABLE, 30000, &confirmed);
    CHECK(hl_host_returned(&w, 31000, WITHIN, &confirmed) == HL_RETURN_CONFIRMED,
          "other: another request holding does not break the row");
    CHECK(same(confirmed, step), "other: the step");
}

static void test_drop_late_return(void) {
    struct hl_drop_watch w = {0};
    struct hl_result confirmed = NONE;
    const struct hl_result req = R(6, 61);

    hl_drop_seen(&w, 0x08, &req, 9000, STABLE, 1000, &confirmed);
    CHECK(hl_host_returned(&w, 1000 + WITHIN + 1, WITHIN, &confirmed) == HL_RETURN_LATE,
          "late: slept");
    CHECK(hl_host_returned(&w, 90000, WITHIN, &confirmed) == HL_RETURN_NOTHING_PENDING,
          "late: nothing pending after");
}

static void test_drop_unstable(void) {
    struct hl_drop_watch w = {0};
    struct hl_result confirmed = NONE;
    const struct hl_result req = R(6, 61);

    /* drops by itself, host straight back: unstable */
    CHECK(hl_drop_seen(&w, 0x28, NULL, 9000, STABLE, 1000, &confirmed) ==
              HL_DROP_SEEN_UNPROVOKED,
          "unstable: unprovoked");
    CHECK(hl_host_returned(&w, 2000, WITHIN, &confirmed) == HL_RETURN_UNSTABLE,
          "unstable: marked");
    /* a drop after a request now does not count */
    CHECK(hl_drop_seen(&w, 0x08, &req, 9000, STABLE, 5000, &confirmed) ==
              HL_DROP_SEEN_HOST_UNSTABLE,
          "unstable: not counted");
    CHECK(hl_host_returned(&w, 6000, WITHIN, &confirmed) == HL_RETURN_NOTHING_PENDING,
          "unstable: nothing pending");
    /* a link up for the stable time clears it */
    CHECK(hl_drop_seen(&w, 0x08, &req, STABLE, STABLE, 60000, &confirmed) ==
              HL_DROP_SEEN_WAITS_FOR_HOST,
          "unstable: cleared by a long link");

    /* an unprovoked drop with a late return does not mark it */
    struct hl_drop_watch v = {0};

    hl_drop_seen(&v, 0x08, NULL, 9000, STABLE, 1000, &confirmed);
    CHECK(hl_host_returned(&v, 1000 + WITHIN + 1, WITHIN, &confirmed) == HL_RETURN_LATE,
          "unprovoked late");
    CHECK(!v.unstable, "unprovoked late: stable");
}

static void test_drop_reasons(void) {
    struct hl_drop_watch w = {0};
    struct hl_result confirmed = NONE;
    const struct hl_result req = R(6, 61);

    CHECK(hl_drop_seen(&w, HL_DROP_REASON_LOCAL_HOST, &req, 9000, STABLE, 1000, &confirmed) ==
              HL_DROP_SEEN_IGNORED,
          "0x16 ignored");
    CHECK(!w.pending, "0x16: nothing pending");
    CHECK(hl_drop_seen(&w, HL_DROP_REASON_LOCAL_HOST, NULL, 9000, STABLE, 1000, &confirmed) ==
              HL_DROP_SEEN_IGNORED,
          "0x16 unprovoked ignored too");
    CHECK(hl_drop_seen(&w, HL_DROP_REASON_UNACCEPTABLE_PARAMS, &req, 9000, STABLE, 1000,
                       &confirmed) == HL_DROP_SEEN_SUSPECTED,
          "0x3B once");
    CHECK(hl_drop_seen(&w, HL_DROP_REASON_UNACCEPTABLE_PARAMS, &req, 9000, STABLE, 5000,
                       &confirmed) == HL_DROP_SEEN_CONFIRMED,
          "0x3B twice");
    CHECK(same(confirmed, req), "0x3B: the request");
}

/* Bluetooth switched off on the host (0x13), its adapter powered off (0x15):
   closed on purpose, not lost on air -- the host does not become unstable. */
static void test_drop_closed_on_purpose(void) {
    const uint8_t closed[] = {0x13, 0x15, 0x22, 0xFF};
    const uint8_t on_air[] = {0x08, 0x28, 0x3D, 0x3E};
    struct hl_result confirmed = NONE;

    for (size_t i = 0; i < sizeof(closed); i++) {
        struct hl_drop_watch w = {0};

        CHECK(hl_drop_seen(&w, closed[i], NULL, 9000, STABLE, 1000, &confirmed) ==
                  HL_DROP_SEEN_NOT_LOST_ON_AIR,
              "0x%02x: not lost on air", closed[i]);
        CHECK(hl_host_returned(&w, 2000, WITHIN, &confirmed) == HL_RETURN_NOTHING_PENDING,
              "0x%02x: nothing pending", closed[i]);
        CHECK(!w.unstable, "0x%02x: stays stable", closed[i]);
    }
    for (size_t i = 0; i < sizeof(on_air); i++) {
        struct hl_drop_watch w = {0};

        CHECK(hl_drop_seen(&w, on_air[i], NULL, 9000, STABLE, 1000, &confirmed) ==
                  HL_DROP_SEEN_UNPROVOKED,
              "0x%02x: lost on air", on_air[i]);
        CHECK(hl_host_returned(&w, 2000, WITHIN, &confirmed) == HL_RETURN_UNSTABLE,
              "0x%02x: unstable", on_air[i]);
    }

    /* After a request, closed on purpose still waits for the host as before. */
    struct hl_drop_watch w = {0};
    const struct hl_result req = R(6, 61);

    CHECK(hl_drop_seen(&w, 0x13, &req, 9000, STABLE, 1000, &confirmed) ==
              HL_DROP_SEEN_WAITS_FOR_HOST,
          "0x13 after a request: waits for the host");
}

struct blame_case {
    const char *name;
    struct hl_learned before;
    struct hl_result blamed;
    unsigned int changed;
    uint16_t ceiling_plus1_after, floor_after;
    struct hl_result stored_after, candidate_after;
};

static const struct blame_case blame_cases[] = {
    {"a latency step: ceiling", {.stored = R(6, 61)}, R(6, 61),
     HL_BLAME_LOWERS_LATENCY | HL_BLAME_CLEARS_STORED, 62, 0, NONE, NONE},
    {"a ladder step: floor", {.stored = R(7, 52)}, R(7, 30),
     HL_BLAME_RAISES_FLOOR | HL_BLAME_CLEARS_STORED, 0, 8, NONE, NONE},
    {"the secure request: ceiling", {.stored = R(12, 30)}, R(12, 30),
     HL_BLAME_LOWERS_LATENCY | HL_BLAME_CLEARS_STORED, 31, 0, NONE, NONE},
    {"a faster candidate goes, the stored stays", {.stored = R(9, 40), .candidate = R(6, 61)},
     R(7, 30), HL_BLAME_RAISES_FLOOR | HL_BLAME_CLEARS_CANDIDATE, 0, 8, R(9, 40), NONE},
    {"a lower ceiling only", {.stored = R(9, 20), .latency_ceiling_plus1 = 50}, R(6, 40),
     HL_BLAME_LOWERS_LATENCY, 41, 0, R(9, 20), NONE},
    {"a higher ceiling changes nothing", {.latency_ceiling_plus1 = 41}, R(6, 50), 0, 41, 0, NONE,
     NONE},
    {"a lower floor changes nothing", {.interval_floor = 9}, R(6, 30), 0, 0, 9, NONE, NONE},
};

static void test_blame(void) {
    for (size_t i = 0; i < sizeof(blame_cases) / sizeof(blame_cases[0]); i++) {
        const struct blame_case *c = &blame_cases[i];
        struct hl_learned l = c->before;
        const unsigned int changed = hl_blame_confirmed(&l, &c->blamed, 30, 12);

        CHECK(changed == c->changed, "%s: changed 0x%x, expected 0x%x", c->name, changed,
              c->changed);
        CHECK(l.latency_ceiling_plus1 == c->ceiling_plus1_after, "%s: ceiling+1 %u", c->name,
              l.latency_ceiling_plus1);
        CHECK(l.interval_floor == c->floor_after, "%s: floor %u", c->name, l.interval_floor);
        CHECK(same(l.stored, c->stored_after), "%s: stored %u/%u", c->name, l.stored.interval,
              l.stored.latency);
        CHECK(same(l.candidate, c->candidate_after), "%s: candidate %u/%u", c->name,
              l.candidate.interval, l.candidate.latency);
    }
}

static void test_drop_blame(void) {
    CHECK(hl_drop_blame(false, true, 1000, 6100) == HL_DROP_BLAMES_REQUEST_TAKEN, "taken");
    CHECK(hl_drop_blame(true, true, 7000, 6100) == HL_DROP_BLAMES_REQUEST_OUT, "out");
    CHECK(hl_drop_blame(false, true, 7000, 6100) == HL_DROP_BLAMES_NOTHING, "past window");
    CHECK(hl_drop_blame(false, false, 0, 6100) == HL_DROP_BLAMES_NOTHING, "nothing");
}

static void test_zmk_studio(void) {
    CHECK(hl_zmk_studio_has_link(true, true, false, 20000, 15000), "studio open");
    CHECK(!hl_zmk_studio_has_link(true, true, true, 20000, 15000), "restored at connect");
    CHECK(!hl_zmk_studio_has_link(true, true, false, 5000, 15000), "too early");
    CHECK(!hl_zmk_studio_has_link(false, true, false, 20000, 15000), "not its parameters");
    CHECK(!hl_zmk_studio_has_link(true, false, false, 20000, 15000), "not subscribed");
}

int main(void) {
    test_first_search();
    test_research();
    test_stored_accepted();
    test_drop_two_connections();
    test_drop_not_in_a_row();
    test_drop_other_request_held();
    test_drop_late_return();
    test_drop_unstable();
    test_drop_reasons();
    test_drop_closed_on_purpose();
    test_blame();
    test_drop_blame();
    test_zmk_studio();

    if (failures) {
        printf("decisions_test: %d FAILED\n", failures);
        return 1;
    }
    printf("decisions_test: all passed\n");
    return 0;
}
