/*
 * project.cpp  --  Probe-mode pintool  "project.so"
 *
 *   pin -t obj-intel64/project.so [-prof_time <seconds>] -- ./app <args>
 *
 * ===========================================================================
 * BIG PICTURE
 * ===========================================================================
 *
 *  time 0            the main executable is loaded (ImageLoad callback):
 *                    - build TC1 = translated routines + profiling counters
 *                      (exercise 4 / bprofile-with-gearing.cpp)
 *                    - probe every translated routine so it enters TC1
 *                      through a "dispatch slot"   (see tc_dispatch.h)
 *
 *  app starts        spawn ONE background (Pin internal) thread
 *
 *  0 .. prof_time    the application runs in TC1 and the counters grow.
 *                    The background thread just sleeps.
 *
 *  prof_time         the background thread:
 *                    1. copies the counters (a "snapshot" of the profile)
 *                    2. builds TC2 using that profile
 *                    3. switches every dispatch slot from TC1 to TC2
 *
 *  afterwards        every NEW call of a translated routine runs in TC2.
 *                    Calls that are already running in TC1 finish in TC1
 *                    (TC1 is never freed, so this is safe).
 *
 *
 *       original routine        dispatch stub (fixed)         slot (data)
 *     +------------------+    +-------------------------+    +-----------+
 *     | jmp stub (probe) |--->| jmp qword ptr [slot_i]  |--->| TC1 entry |  before
 *     |  ...             |    +-------------------------+    | TC2 entry |  after
 *     +------------------+                                   +-----------+
 *
 * ===========================================================================
 * FILE LAYOUT
 * ===========================================================================
 *   Part 0 - includes and knobs
 *   Part 1 - global tables (translated routines, BB profile)
 *   Part 2 - the translation engine: build_tc1() / build_tc2()
 *            >>> this is where the code from bprofile-with-gearing.cpp goes <<<
 *   Part 3 - the switch TC1 -> TC2
 *   Part 4 - the background thread (-prof_time timer)
 *   Part 5 - Pin callbacks (ImageLoad, application start)
 *   Part 6 - main()
 *
 * DEMO MODE: until Part 2 is filled in, TC1 and TC2 both fall back to Pin's
 * relocated copy of the ORIGINAL routine. The program then runs unchanged,
 * but the full flow (knob -> probes -> timer thread -> slot switch) still
 * runs, so you can test this skeleton before plugging in the translator.
 */

/* ===========================================================================
 * Part 0 - includes and knobs
 * ===========================================================================*/
#include "pin.H"

extern "C" {
#include "xed-interface.h"   // used by the translation engine (Part 2)
}

#include <algorithm>
#include <iostream>
#include <string>
#include <vector>

#include "tc_dispatch.h"

using std::cerr;
using std::endl;
using std::string;
using std::vector;

KNOB<UINT32> KnobProfTime(KNOB_MODE_WRITEONCE, "pintool", "prof_time", "2",
                          "profiling time in seconds (running in TC1) before "
                          "switching to the optimized TC2");

// Short prefix for all our messages, so they stand out from the program's output.
#define LOG cerr << "[project] "

/* ===========================================================================
 * Part 1 - global tables
 * ===========================================================================
 * Filled in ImageLoad (while the application is NOT running yet).
 * After that the background thread only READS them, and writes tc2_entry.
 */

// One entry per routine we translate.
struct rtn_info_t {
    string  name;
    ADDRINT orig_addr;        // start of the routine in the original binary
    USIZE   size;
    ADDRINT orig_entry_copy;  // returned by RTN_ReplaceProbed: Pin's relocated
                              // copy of the original routine start (DEMO MODE)
    ADDRINT tc1_entry;        // routine entry inside TC1 (0 = not built)
    ADDRINT tc2_entry;        // routine entry inside TC2 (0 = not built)
};

static vector<rtn_info_t> g_rtns;   // index in this vector == dispatch slot index

// One entry per basic block that TC1 counts (exercise 4).
// This is a FIXED array on purpose: the TC1 code has the ADDRESS of
// exec_count/taken_count baked into it, so they must never move
// (a std::vector could move them when it grows).
struct bbl_profile_t {
    ADDRINT orig_addr;        // start of the BB in the original binary
    int     rtn_idx;          // index into g_rtns
    UINT64  exec_count;       // ++ by TC1 each time the BB runs
    UINT64  taken_count;      // ++ by TC1 each time the BB's ending jump is taken
};

static const int     MAX_BBLS = 200000;
static bbl_profile_t g_bbls[MAX_BBLS];
static int           g_num_bbls = 0;

static dispatch_area_t g_dispatch;          // slots + stubs (tc_dispatch.h)
static bool            g_tc1_ready = false; // true once probes point at TC1

// Register a basic block for profiling. Returns its index, or -1 if full.
// Use &g_bbls[idx].exec_count / .taken_count in the code you emit into TC1.
static int add_bbl(ADDRINT orig_addr, int rtn_idx)
{
    if (g_num_bbls >= MAX_BBLS)
        return -1;
    bbl_profile_t &b = g_bbls[g_num_bbls];
    b.orig_addr   = orig_addr;
    b.rtn_idx     = rtn_idx;
    b.exec_count  = 0;
    b.taken_count = 0;
    return g_num_bbls++;
}

/* ===========================================================================
 * Part 2 - the translation engine   (plug in bprofile-with-gearing.cpp)
 * ===========================================================================
 * We do NOT re-implement instruction copying here. bprofile-with-gearing.cpp
 * already does it: decode with XED, fix RIP-relative displacements, fix
 * direct branches/calls, write the code into a TC. Move that code into this
 * part and call it from build_tc1() and build_tc2().
 *
 * THREE RULES that make the switch to TC2 safe:
 *
 *  (R1) Calls from TC1 to another translated routine must go through the
 *       dispatch stub, i.e. target either the ORIGINAL routine address (its
 *       probe jumps to the stub) or dispatch_stub_addr(&g_dispatch, idx).
 *       Do NOT "chain" such a call straight to the callee's TC1 copy,
 *       otherwise the callee keeps running in TC1 after the switch.
 *       (Branches INSIDE the same routine can and should be chained.)
 *
 *  (R2) build_tc2() runs on the background thread, while the application
 *       keeps running. Do not call IMG_* / RTN_* / INS_* / BBL_* Pin
 *       functions there. Everything TC2 needs (decoded instructions, BB
 *       boundaries, sizes) must be saved in your own data structures in
 *       build_tc1(). XED can be used from any thread.
 *       Also allocate TC2's memory in build_tc1(), the same way you
 *       allocate TC1, so it gets the same placement as TC1.
 *
 *  (R3) Never free or overwrite TC1. Threads that were inside TC1 during
 *       the switch keep running there, and their stacks still hold return
 *       addresses that point into TC1.
 *
 * Emitting a counter in TC1 (you have this from exercise 4). It must not
 * change the flags and must not touch the red zone (the 128 bytes below
 * rsp that leaf functions may use):
 *
 *      lea    rsp, [rsp-128]      ; step over the red zone (lea keeps flags)
 *      pushfq                     ; inc changes flags -> save them
 *      push   rax
 *      mov    rax, <&g_bbls[idx].exec_count>    ; 64-bit absolute address
 *      inc    qword ptr [rax]
 *      pop    rax
 *      popfq
 *      lea    rsp, [rsp+128]
 *
 *  (The counters are not atomic. In a multi-threaded program a few
 *   increments may be lost, which is fine for profiling.)
 */

// Called from ImageLoad: Pin API allowed, application not running yet.
// Must set g_rtns[i].tc1_entry for every routine it translates.
static bool build_tc1(IMG img)
{
    // TODO(bprofile): plug in the TC1 generation of bprofile-with-gearing.cpp:
    //   1. for every routine in g_rtns: decode its instructions (save them!)
    //   2. for every BB: idx = add_bbl(bb_addr, rtn_idx) and emit the
    //      counter code above in front of it (+ taken_count for jumps)
    //   3. fix displacements / branch targets (remember rule R1)
    //   4. copy the code into TC1
    //   5. g_rtns[i].tc1_entry = address of routine i inside TC1
    //   6. allocate the (empty) TC2 memory now (rule R2)
    //
    // DEMO MODE: we do nothing, tc1_entry stays 0.
    return true;
}

// Called from the BACKGROUND THREAD after -prof_time seconds.
// 'profile' is a stable copy of g_bbls[0 .. g_num_bbls-1].
// Must set g_rtns[i].tc2_entry for every routine that should move to TC2
// (routines left at 0 simply stay in TC1).
static bool build_tc2(const vector<bbl_profile_t> &profile)
{
    // --- Step 1: use the profile. Here: total executions per routine. ---
    vector<UINT64> rtn_count(g_rtns.size(), 0);
    for (size_t b = 0; b < profile.size(); b++)
        rtn_count[profile[b].rtn_idx] += profile[b].exec_count;

    // --- Step 2: order the routines from hottest to coldest. ---
    vector<int> hot_order(g_rtns.size());
    for (size_t i = 0; i < hot_order.size(); i++)
        hot_order[i] = (int)i;
    std::stable_sort(hot_order.begin(), hot_order.end(),
                     [&rtn_count](int a, int b) { return rtn_count[a] > rtn_count[b]; });

    LOG << "hottest routines (by BB executions in TC1):" << endl;
    for (size_t k = 0; k < hot_order.size() && k < 10; k++) {
        int i = hot_order[k];
        if (rtn_count[i] == 0)
            break;
        LOG << "    " << g_rtns[i].name << " : " << rtn_count[i] << endl;
    }

    // --- Step 3: generate TC2. ---
    // TODO(bprofile): generate TC2 from the saved instructions, WITHOUT
    // counters, and use the profile for optimizations required by
    // project-2026.pdf (e.g. put hot routines / hot BBs first, reorder
    // conditional branches so the common path falls through, inline hot
    // calls, ...). Then set g_rtns[i].tc2_entry.
    // Inside TC2 you may chain calls directly to other TC2 routines.

    // DEMO MODE: "TC2" = the original routine again, so the switch does
    // real slot writes even without a translator. Delete this when TC2 exists.
    for (size_t i = 0; i < g_rtns.size(); i++)
        if (g_rtns[i].tc2_entry == 0)
            g_rtns[i].tc2_entry = g_rtns[i].orig_entry_copy;

    return true;
}

/* ===========================================================================
 * Part 3 - the switch TC1 -> TC2
 * ===========================================================================
 * Called only after TC2 is COMPLETELY written.
 *
 * Why this is safe while application threads are running:
 *   - We do not modify any instruction; we only store a new address into
 *     each slot (one atomic 8-byte store, see tc_dispatch.h).
 *   - A thread entering a routine at the moment of the store reads either
 *     the old or the new address. Both are complete, correct translations.
 *   - A thread already inside TC1 continues in TC1 until that call returns
 *     (rule R3). Its next call to a translated routine goes through a stub
 *     (rule R1) and lands in TC2.
 *   - Some routines already in TC2 while others are still being switched is
 *     fine: all copies use the normal calling convention, so a TC1 routine
 *     can call a TC2 routine and the other way around.
 */
static void switch_to_tc2()
{
    // Full memory barrier: make sure all TC2 bytes are written before any
    // slot points at them (dispatch_set_target also uses release order).
    __sync_synchronize();

    int switched = 0;
    for (size_t i = 0; i < g_rtns.size(); i++) {
        if (g_rtns[i].tc2_entry == 0)
            continue;                       // not in TC2 -> stays in TC1
        dispatch_set_target(&g_dispatch, i, g_rtns[i].tc2_entry);
        switched++;
    }
    LOG << "switched " << switched << " of " << g_rtns.size()
        << " routines to TC2" << endl;
}

/* ===========================================================================
 * Part 4 - the background thread (-prof_time timer)
 * ===========================================================================
 * A Pin "internal thread": created by the tool, not by the application.
 */
static const UINT32 SLEEP_STEP_MS = 100;

static VOID ProfilingTimerThread(VOID *arg)
{
    UINT32 total_ms = KnobProfTime.Value() * 1000;

    // Sleep in small steps rather than one long sleep, so that we stop
    // early if the application is already exiting.
    for (UINT32 slept_ms = 0; slept_ms < total_ms; slept_ms += SLEEP_STEP_MS) {
        if (PIN_IsProcessExiting())
            return;
        PIN_Sleep(std::min(SLEEP_STEP_MS, total_ms - slept_ms));
    }
    if (PIN_IsProcessExiting())
        return;

    LOG << KnobProfTime.Value() << " s of profiling done, building TC2" << endl;

    // 1. Snapshot: TC1 keeps incrementing the counters while we work, so
    //    we take a copy and build TC2 from values that do not change.
    vector<bbl_profile_t> profile(g_bbls, g_bbls + g_num_bbls);

    // 2. Build TC2. If it fails we simply stay in TC1, which still works.
    if (!build_tc2(profile)) {
        LOG << "building TC2 failed - staying in TC1" << endl;
        return;
    }

    // 3. Move execution to TC2.
    switch_to_tc2();

    // Returning ends the internal thread. Its work is done.
}

/* ===========================================================================
 * Part 5 - Pin callbacks
 * ===========================================================================*/

// Choose the routines to translate: every routine of the main executable
// that Pin can safely probe. (You can use bprofile's candidate selection
// instead, e.g. skip very small routines.)
static void find_candidate_rtns(IMG img)
{
    for (SEC sec = IMG_SecHead(img); SEC_Valid(sec); sec = SEC_Next(sec)) {
        if (!SEC_IsExecutable(sec))
            continue;
        for (RTN rtn = SEC_RtnHead(sec); RTN_Valid(rtn); rtn = RTN_Next(rtn)) {
            if (!RTN_IsSafeForProbedReplacement(rtn))
                continue;
            rtn_info_t info;
            info.name            = RTN_Name(rtn);
            info.orig_addr       = RTN_Address(rtn);
            info.size            = RTN_Size(rtn);
            info.orig_entry_copy = 0;
            info.tc1_entry       = 0;
            info.tc2_entry       = 0;
            g_rtns.push_back(info);
        }
    }
}

// Point every candidate routine at its dispatch stub, and every slot at TC1.
// Use this INSTEAD of bprofile's "commit_translated_routines": that one
// probes straight to the TC1 address, which could never be switched later.
static void commit_through_dispatch_slots()
{
    for (size_t i = 0; i < g_rtns.size(); i++) {
        rtn_info_t &r = g_rtns[i];

        // Set the slot BEFORE placing the probe, so the stub never jumps to 0.
        if (r.tc1_entry != 0)
            dispatch_set_target(&g_dispatch, i, r.tc1_entry);

        RTN rtn = RTN_FindByAddress(r.orig_addr);
        AFUNPTR orig_copy =
            RTN_ReplaceProbed(rtn, (AFUNPTR)dispatch_stub_addr(&g_dispatch, i));
        r.orig_entry_copy = (ADDRINT)orig_copy;

        // DEMO MODE (no TC1 for this routine): run the original code.
        // The application has not started yet, so setting it now is fine.
        if (r.tc1_entry == 0)
            dispatch_set_target(&g_dispatch, i, r.orig_entry_copy);
    }
}

static VOID ImageLoad(IMG img, VOID *v)
{
    if (!IMG_IsMainExecutable(img))
        return;

    find_candidate_rtns(img);
    if (g_rtns.empty()) {
        LOG << "no routine can be translated - running natively" << endl;
        return;
    }

    if (!build_tc1(img)) {
        LOG << "building TC1 failed - running natively" << endl;
        return;
    }

    if (!dispatch_create(&g_dispatch, g_rtns.size(), 0)) {
        LOG << "cannot allocate dispatch slots - running natively" << endl;
        return;
    }

    commit_through_dispatch_slots();
    g_tc1_ready = true;
    LOG << g_rtns.size() << " routines now enter through dispatch slots (TC1)" << endl;
}

// Called once, right before the first application instruction runs.
// By then ImageLoad of the main executable has finished and TC1 is live,
// so the -prof_time countdown starts at the right moment.
static VOID ApplicationStart(VOID *v)
{
    if (!g_tc1_ready)
        return;

    PIN_THREAD_UID uid;
    THREADID tid = PIN_SpawnInternalThread(ProfilingTimerThread, NULL, 0, &uid);
    if (tid == INVALID_THREADID)
        LOG << "cannot spawn the profiling thread - staying in TC1" << endl;
    else
        LOG << "profiling for " << KnobProfTime.Value() << " s" << endl;
}

/* ===========================================================================
 * Part 6 - main
 * ===========================================================================*/
static INT32 Usage()
{
    cerr << "project.so: profile in TC1 for -prof_time seconds, then switch to TC2"
         << endl << KNOB_BASE::StringKnobSummary() << endl;
    return -1;
}

int main(int argc, char *argv[])
{
    PIN_InitSymbols();                 // we need routine names/addresses
    if (PIN_Init(argc, argv))
        return Usage();

    IMG_AddInstrumentFunction(ImageLoad, 0);
    PIN_AddApplicationStartFunction(ApplicationStart, 0);

    PIN_StartProgramProbed();          // PROBE mode; never returns
    return 0;
}
