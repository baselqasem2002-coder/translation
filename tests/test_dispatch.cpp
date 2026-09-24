/*
 * test_dispatch.cpp -- tests tc_dispatch.h WITHOUT Pin.
 *
 *   g++ -O2 -pthread -o test_dispatch tests/test_dispatch.cpp && ./test_dispatch
 *
 * Several threads keep calling a routine through its dispatch stub while
 * the main thread switches the slot from "TC1" to "TC2", like the pintool's
 * background thread does. We check that:
 *   - nothing crashes and only the two valid versions are ever executed,
 *   - once a thread has reached TC2 it never goes back to TC1,
 *   - after the switch, every thread ends up in TC2.
 */
#include "../tc_dispatch.h"

#include <pthread.h>
#include <stdio.h>
#include <unistd.h>

// Stand-ins for "routine 0 as translated in TC1" and "... in TC2".
extern "C" __attribute__((noinline)) int routine_in_tc1() { return 1; }
extern "C" __attribute__((noinline)) int routine_in_tc2() { return 2; }

typedef int (*routine_fn)();

static dispatch_area_t g_dispatch;
static volatile bool   g_stop = false;

struct worker_result_t {
    long calls_tc1, calls_tc2, bad_values;
    bool went_back_to_tc1;
};

static void *worker(void *arg)
{
    worker_result_t *r = (worker_result_t *)arg;
    routine_fn call_via_stub = (routine_fn)dispatch_stub_addr(&g_dispatch, 0);
    bool seen_tc2 = false;

    while (!g_stop) {
        int v = call_via_stub();          // jmp [slot0] -> TC1 or TC2
        if (v == 1) {
            r->calls_tc1++;
            if (seen_tc2) r->went_back_to_tc1 = true;
        } else if (v == 2) {
            r->calls_tc2++;
            seen_tc2 = true;
        } else {
            r->bad_values++;
        }
    }
    return NULL;
}

int main()
{
    if (!dispatch_create(&g_dispatch, 3, (uintptr_t)routine_in_tc1)) {
        printf("FAIL: dispatch_create\n");
        return 1;
    }

    const int N = 4;
    pthread_t threads[N];
    worker_result_t results[N] = {};
    for (int i = 0; i < N; i++)
        pthread_create(&threads[i], NULL, worker, &results[i]);

    usleep(200 * 1000);                                    // "profiling" in TC1
    dispatch_set_target(&g_dispatch, 0, (uintptr_t)routine_in_tc2);   // the switch
    usleep(200 * 1000);                                    // running in TC2
    g_stop = true;

    bool ok = true;
    for (int i = 0; i < N; i++) {
        pthread_join(threads[i], NULL);
        worker_result_t &r = results[i];
        printf("thread %d: tc1=%ld tc2=%ld bad=%ld went_back=%d\n",
               i, r.calls_tc1, r.calls_tc2, r.bad_values, (int)r.went_back_to_tc1);
        if (r.calls_tc1 == 0 || r.calls_tc2 == 0 || r.bad_values || r.went_back_to_tc1)
            ok = false;
    }

    // The other slots were never switched.
    if (dispatch_get_target(&g_dispatch, 1) != (uintptr_t)routine_in_tc1)
        ok = false;

    printf(ok ? "PASS\n" : "FAIL\n");
    return ok ? 0 : 1;
}
