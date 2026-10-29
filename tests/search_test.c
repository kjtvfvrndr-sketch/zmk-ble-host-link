/*
 * SPDX-License-Identifier: MIT
 *
 * The search (src/search.c) against simulated hosts, on the build machine:
 *   make -C tests
 *
 * A host is a function: what it gives for a request. The test drives the
 * search as the module does -- a request, the host's link, the verdict by
 * hl_link_matches() -- and checks the requests and the result.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "../src/search.h"

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

/* A host: given what the link runs and a request, what it runs after. */
typedef struct hl_link (*host_fn)(const struct hl_link *link, const struct hl_params *asked);

static struct hl_link give(const struct hl_params *asked) {
    return (struct hl_link){asked->interval, asked->latency, asked->timeout};
}

/* Windows, Android: everything. */
static struct hl_link host_all(const struct hl_link *link, const struct hl_params *asked) {
    (void)link;
    return give(asked);
}

/* macOS on Apple Silicon: exactly 15 ms, latency up to 30; nothing else. */
static struct hl_link host_mac(const struct hl_link *link, const struct hl_params *asked) {
    if (asked->interval == 12 && asked->latency <= 30) {
        return give(asked);
    }
    return *link;
}

/* Nothing at all. */
static struct hl_link host_none(const struct hl_link *link, const struct hl_params *asked) {
    (void)asked;
    return *link;
}

/* From 11.25 ms up, latency up to 40. */
static struct hl_link host_from_9_lat40(const struct hl_link *link, const struct hl_params *asked) {
    if (asked->interval >= 9 && asked->latency <= 40) {
        return give(asked);
    }
    return *link;
}

/* Gives the interval but lowers the latency to 0 (BlueZ may). */
static struct hl_link host_lat0(const struct hl_link *link, const struct hl_params *asked) {
    (void)link;
    return (struct hl_link){asked->interval, 0, asked->timeout};
}

/* Everything, latency up to 45. */
static struct hl_link host_lat45(const struct hl_link *link, const struct hl_params *asked) {
    if (asked->latency <= 45) {
        return give(asked);
    }
    return *link;
}

static const struct hl_search_limits default_limits = {
    .fastest_interval = 6,
    .slowest_interval = 12,
    .latency_while_searching_interval = 30,
    .idle_listen_period_ms = 465,
    .timeout = 410,
    .pinned_latency = HL_LATENCY_NOT_PINNED,
    .interval_floor = HL_NO_INTERVAL_FLOOR,
    .latency_ceiling = HL_NO_LATENCY_CEILING,
};

#define MAX_LOG 32

struct run {
    struct hl_params asked[MAX_LOG];
    enum hl_search_step steps[MAX_LOG];
    int count;
    bool have_result;
    struct hl_params result;
    struct hl_link link;
};

static struct run run_search(host_fn host, const struct hl_search_limits *limits,
                             struct hl_link opened) {
    struct hl_search s;
    struct hl_params req;
    struct run r = {.link = opened};

    hl_search_start(&s, limits);
    while (hl_search_next(&s, &req)) {
        if (r.count < MAX_LOG) {
            r.asked[r.count] = req;
            r.steps[r.count] = s.asked_step;
        }
        r.count++;
        CHECK(r.count < 40, "search does not end");
        if (r.count >= 40) {
            break;
        }
        r.link = host(&r.link, &req);
        hl_search_answer(&s, hl_link_matches(&req, &r.link));
    }
    r.have_result = s.have_result;
    r.result = s.result;
    return r;
}

static void check_result(const char *name, const struct run *r, bool have, uint16_t interval,
                         uint16_t latency) {
    CHECK(r->have_result == have, "%s: have_result %d, expected %d", name, r->have_result, have);
    if (have && r->have_result) {
        CHECK(r->result.interval == interval && r->result.latency == latency,
              "%s: result %u/%u, expected %u/%u", name, r->result.interval, r->result.latency,
              interval, latency);
        CHECK(r->link.interval == interval && r->link.latency == latency,
              "%s: link left at %u/%u, not on the result %u/%u", name, r->link.interval,
              r->link.latency, interval, latency);
    }
}

static void print_run(const char *name, const struct run *r) {
    printf("%-28s", name);
    for (int i = 0; i < r->count && i < MAX_LOG; i++) {
        printf(" %u/%u", r->asked[i].interval, r->asked[i].latency);
    }
    if (r->have_result) {
        printf("  -> %u/%u\n", r->result.interval, r->result.latency);
    } else {
        printf("  -> nothing\n");
    }
}

static const struct hl_link mac_opened = {24, 0, 720};

static void test_windows(void) {
    const struct run r = run_search(host_all, &default_limits, mac_opened);

    print_run("all (Windows, Android)", &r);
    /* secure, then 7.5 ms at once, then the latency target 61 */
    CHECK(r.count == 3, "all: %d requests", r.count);
    CHECK(r.asked[0].interval == 12 && r.asked[0].latency == 30, "all: secure first");
    CHECK(r.steps[0] == HL_STEP_SECURE, "all: step 0 secure");
    CHECK(r.asked[1].interval == 6 && r.asked[1].latency == 30, "all: ladder from 7.5 ms");
    CHECK(r.steps[2] == HL_STEP_LATENCY && r.asked[2].latency == 61, "all: latency 61");
    check_result("all", &r, true, 6, 61);
}

static void test_mac(void) {
    const struct run r = run_search(host_mac, &default_limits, mac_opened);

    print_run("mac (only 15 ms)", &r);
    /* secure, then every ladder step 6..11 refused, latency target 30 = no step */
    CHECK(r.count == 7, "mac: %d requests", r.count);
    for (int i = 1; i < 7; i++) {
        CHECK(r.asked[i].interval == 5 + i && r.asked[i].latency == 30,
              "mac: ladder step %d is %u/%u", i, r.asked[i].interval, r.asked[i].latency);
    }
    check_result("mac", &r, true, 12, 30);
}

static void test_none(void) {
    const struct run r = run_search(host_none, &default_limits, mac_opened);

    print_run("none", &r);
    CHECK(r.count == 7, "none: %d requests", r.count);
    check_result("none", &r, false, 0, 0);
}

static void test_from_9(void) {
    const struct run r = run_search(host_from_9_lat40, &default_limits, mac_opened);

    print_run("from 11.25 ms, latency 40", &r);
    /* 12/30 ok, 6 7 8 refused, 9 ok, latency target at 9: 465/11.25 = 41 -> 40 */
    check_result("from 9", &r, true, 9, 40);
    CHECK(r.count == 6, "from 9: %d requests", r.count);
}

static void test_lat0(void) {
    const struct run r = run_search(host_lat0, &default_limits, mac_opened);

    print_run("lowers latency to 0", &r);
    /* a lower latency is no acceptance: nothing found */
    check_result("lat0", &r, false, 0, 0);
}

static void test_lat45(void) {
    const struct run r = run_search(host_lat45, &default_limits, mac_opened);

    print_run("latency up to 45", &r);
    /* 6/30, target 61 refused, halfway 45 ok, halfway 53 refused, steps used up */
    CHECK(r.count == 5, "lat45: %d requests", r.count);
    check_result("lat45", &r, true, 6, 45);
}

static void test_pinned_latency(void) {
    struct hl_search_limits limits = default_limits;

    limits.pinned_latency = 0;
    const struct run r = run_search(host_all, &limits, mac_opened);

    print_run("all, latency pinned 0", &r);
    /* every request at latency 0, no latency steps */
    CHECK(r.count == 2, "pinned: %d requests", r.count);
    CHECK(r.asked[0].latency == 0 && r.asked[1].latency == 0, "pinned: latency 0");
    check_result("pinned", &r, true, 6, 0);

    limits.pinned_latency = 499;
    const struct run t = run_search(host_all, &limits, mac_opened);

    print_run("all, latency pinned 499", &t);
    /* trimmed by the spec: 15 ms -> 135, 7.5 ms -> 272 */
    CHECK(t.asked[0].latency == 135, "pinned 499 at 15 ms: %u", t.asked[0].latency);
    CHECK(t.asked[1].latency == 272, "pinned 499 at 7.5 ms: %u", t.asked[1].latency);
}

static void test_floor_and_ceiling(void) {
    struct hl_search_limits limits = default_limits;

    limits.interval_floor = 8;
    const struct run r = run_search(host_all, &limits, mac_opened);

    print_run("all, floor 10 ms", &r);
    CHECK(r.asked[1].interval == 8, "floor: ladder starts at %u", r.asked[1].interval);
    check_result("floor", &r, true, 8, 45);

    limits = default_limits;
    limits.latency_ceiling = 30;
    const struct run c = run_search(host_all, &limits, mac_opened);

    print_run("all, latency ceiling 30", &c);
    CHECK(c.asked[0].latency == 29, "ceiling: secure at %u", c.asked[0].latency);
    check_result("ceiling", &c, true, 6, 29);

    limits = default_limits;
    limits.interval_floor = 12;
    const struct run e = run_search(host_all, &limits, mac_opened);

    print_run("all, floor 15 ms", &e);
    CHECK(e.count == 1, "floor at slowest: %d requests", e.count);
    check_result("floor at slowest", &e, true, 12, 30);
}

static void test_late_answer(void) {
    struct hl_search s;
    struct hl_params req;

    hl_search_start(&s, &default_limits);
    CHECK(hl_search_next(&s, &req) && req.interval == 12, "late: secure");
    hl_search_answer(&s, false);
    /* the link moves onto it before the next step */
    CHECK(hl_search_late_answer(&s), "late: taken");
    CHECK(!hl_search_late_answer(&s), "late: only once");
    CHECK(hl_search_next(&s, &req) && req.interval == 6, "late: ladder next");
    hl_search_answer(&s, false);
    /* the next step chosen: a late answer to the one before does not count */
    CHECK(hl_search_next(&s, &req) && req.interval == 7, "late: ladder 7");
    CHECK(!hl_search_late_answer(&s), "late: nothing pending");
    CHECK(s.have_result && s.result.interval == 12, "late: secure is the result");

    /* a ladder step accepted late ends the ladder */
    hl_search_answer(&s, false);
    CHECK(hl_search_late_answer(&s), "late: ladder 7 taken");
    CHECK(hl_search_next(&s, &req) && s.asked_step == HL_STEP_LATENCY,
          "late: latency step after the ladder");
    CHECK(s.result.interval == 7, "late: result 7, got %u", s.result.interval);
}

static void test_latency_helpers(void) {
    CHECK(hl_latency_allowed(100, 24, 410) == 67, "30 ms at 4.1 s: %u",
          hl_latency_allowed(100, 24, 410));
    CHECK(hl_latency_allowed(30, 12, 410) == 30, "15 ms 30");
    CHECK(hl_latency_for_idle_listen_period(12, 465, 410) == 30, "target at 15 ms");
    CHECK(hl_latency_for_idle_listen_period(9, 465, 410) == 40, "target at 11.25 ms");
    CHECK(hl_latency_for_idle_listen_period(6, 465, 410) == 61, "target at 7.5 ms");
    CHECK(hl_latency_under_ceiling(61, HL_NO_LATENCY_CEILING) == 61, "no ceiling");
    CHECK(hl_latency_under_ceiling(61, 40) == 39, "ceiling 40");
    CHECK(hl_latency_under_ceiling(61, 0) == 0, "ceiling 0");

    const struct hl_params asked = {12, 30, 410};
    const struct hl_link same = {12, 30, 600};
    const struct hl_link lower = {12, 0, 410};

    CHECK(hl_link_matches(&asked, &same), "match ignores the timeout");
    CHECK(!hl_link_matches(&asked, &lower), "a lower latency is no match");
}

int main(void) {
    test_latency_helpers();
    test_windows();
    test_mac();
    test_none();
    test_from_9();
    test_lat0();
    test_lat45();
    test_pinned_latency();
    test_floor_and_ceiling();
    test_late_answer();

    if (failures) {
        printf("search_test: %d FAILED\n", failures);
        return 1;
    }
    printf("search_test: all passed\n");
    return 0;
}
