========================================================================
Exercise 4 - Optimized bprofile Pintool
========================================================================

a. Names + ID numbers
------------------------------------------------------------------------
Shadi Najar    213557143
Basel Qassem   324937036


b. How to run the tool
------------------------------------------------------------------------
Build:

    make PIN_ROOT=<path-to-pin-kit> obj-intel64/bprofile.so

(Alternatively, place the src directory under
 <pin-kit>/source/tools/ and run 'make'.)

Run:

    <path-to-pin-kit>/pin -t obj-intel64/bprofile.so \
        -prof_time <profiling interval in seconds> -- <input binary> [args]

The tool translates the routines of the main executable into a
Translation Cache (TC) together with inlined profiling stubs, collects
BBL execution counts, fallthrough counts, and indirect-jump target
counts for prof_time seconds (default: 2), then disables the profiling
stubs (each stub is overwritten with a jump that bypasses it) and lets
the program run to completion.

Output: edge-profile.csv in the current directory, one line per
executed BBL, sorted from hottest to coldest:

    <bbl addr>, <exec count>, <taken count>, <fallthru count>[, <targ addr>, <targ count> ... up to 4]

Useful debug knobs: -verbose, -dump_tc, -dump_orig_code, -no_prof,
-no_tc_commit.

Measuring the required >=5% improvement (item 5) against the provided
non-optimized bprofile.so, e.g.:

    /usr/bin/time -v <pin> -t obj-intel64/bprofile_orig.so -prof_time 10 -- <binary>
    /usr/bin/time -v <pin> -t obj-intel64/bprofile.so      -prof_time 10 -- <binary>

and compare elapsed/user/system time and page faults ("Maximum resident
set size" / "page faults" lines), or:

    perf stat -e cycles <pin> -t obj-intel64/bprofile.so -prof_time 10 -- <binary>

The improvement comes from the profiling interval, so use a prof_time
long enough for profiling to dominate the run (and an input binary /
workload that runs at least that long).


c. What problems we fixed in the pintool and how we chose to fix them
------------------------------------------------------------------------
1. Translation aborted on the first problematic routine.
   In find_candidate_rtns_for_tc(), any XED decode failure or
   translation error caused 'return -1', which aborted the entire
   translation, so binaries containing a single routine with data
   embedded in code (or any undecodable bytes) could not be profiled at
   all. Fix: each routine's starting instr-map index and BBL index are
   recorded before translating it; on failure the routine's entries are
   reverted (num_of_instr_map_entries and bbl_num are restored) and we
   continue with the next routine. The skipped routine simply keeps
   executing its original, non-translated code.

2. Unsafe probed replacement.
   commit_translated_rtns_to_tc() called RTN_ReplaceProbed() on every
   routine. For routines that Pin cannot safely probe (too short, or
   with a jump target inside the first bytes that the probe overwrites)
   this can crash the application. Fix: check
   RTN_IsSafeForProbedReplacement() and skip the commit for unsafe
   routines (they keep running the original code).

3. Duplicate routines and PLT stubs.
   Some binaries expose the same address under more than one routine
   symbol; translating it twice corrupts the address-based chaining of
   direct branches (entry_map keeps only the first entry per original
   address). Fix: a set of already-translated routine addresses is kept
   and duplicates are skipped. PLT stubs (.plt / .plt.got / .plt.sec)
   are skipped as well: they contain lazy-binding code that must not be
   relocated into the TC. As a result, the translated xxx@plt routines
   end with a jump to the (untranslated) PLT0 resolver at the original
   address; these jumps are automatically redirected through the
   jump-to-orig-addr map (rewritten as 'jmp qword ptr [rip+disp]'),
   which is the tool's standard fallback for branches into
   non-translated code. This is a visible behavioral difference from
   the provided tool, which translated .plt itself.

4. Off-by-one bug in the jump-to-original-address map.
   fix_direct_jmp_or_call_to_orig_addr() incremented
   jump_to_orig_addr_num BEFORE using it as the new entry index. As a
   result slot 0 was never used, every new entry was written one slot
   past the range scanned by the lookup loop, so existing entries were
   never found again and the map grew by one entry per fixed branch
   (risking the max_rtn_count overflow abort on large binaries). Fix:
   allocate the current value as the slot, then increment.

5. Unbounded scan in disable_profiling_in_tc().
   The while loop that measures the size of a profiling stub could read
   past the end of instr_map if the very last entry belongs to a stub.
   Fix: bound the loop by num_of_instr_map_entries.

6. Minor: is_targ_map.empty() (a no-op query) replaced with .clear().

7. Dead-register save/restore elimination (item 2 of the exercise).
   The profiling stubs unconditionally saved and restored the registers
   they clobber: RAX for every BBL/fallthrough counter stub, and RBX +
   RCX (each saved/restored in 2 instructions via RAX, since a MOV
   to/from a 64-bit absolute address is only encodable with RAX) for
   indirect-jump target stubs. We added a conservative forward liveness
   scan (isRegDeadAt) over the original instructions that will execute
   after the stub:
     - the scan starts at the BBL-terminating instruction for a regular
       counter stub (the stub runs right before it), and at the
       instruction following the conditional branch for a fallthrough
       stub (the stub runs on the fallthrough path);
     - a read of any sub-register (RAX/EAX/AX/AL/AH) => live;
     - a full overwrite (64-bit write, or 32-bit write, which
       zero-extends) before any read => dead; 8/16-bit partial writes
       neither kill nor read the full register;
     - the scan stops conservatively (assume live) at any control
       transfer, syscall or interrupt, at the routine end, or after 64
       instructions.
   If a register is provably dead at the stub location, its save and
   restore instructions are simply not emitted: 2 instructions saved
   per counter stub (RAX), and up to 2+4+4 = 10 instructions per
   indirect-jump stub (RAX, RBX, RCX). Since a counter stub executes on
   every BBL execution during the profiling interval, removing 2 of its
   6 instructions (including 1 of its 2 stores and avoiding the load in
   the restore) directly reduces the dynamic instruction count and
   memory traffic of the instrumented run, which is what yields the
   >=5% improvement in elapsed/user time and total cycles when measured
   with a sufficiently long -prof_time. The tool prints how many
   save/restore instructions were eliminated
   ("dead-register optimization: eliminated N save/restore
   instructions...") on stderr.

   Safety notes:
     - If the indirect jump itself uses RAX/RBX/RCX (as target, base or
       index register), the scan necessarily classifies that register
       as live (it starts at the jump), so the stub's internal reload
       of RAX from rax_mem always sees a valid saved value.
     - The existing AND instruction in the indirect-target stub still
       modifies RFLAGS (as in the provided tool). This is safe for the
       BBL-counter part of the stub before conditional branches because
       that part only uses MOV/LEA, which do not touch flags; the AND
       only appears before indirect jumps, which do not read flags.

8. Output format (item 3 of the exercise).
   dump_profile() was rewritten: instead of dumping annotated
   disassembly to bprofile.out, it now writes edge-profile.csv in the
   exercise-3 format, sorted from hottest to coldest by BBL execution
   count. Per BBL we print its original start address (the first
   non-profiling instruction of the BBL), the execution count, the
   taken count, the fallthrough count, and up to 4 (indirect target
   address, count) pairs. The taken count is derived from the
   terminator type: for a conditional branch taken = exec - fallthru
   (the exec counter runs before the branch, the fallthru counter runs
   only on the not-taken path); for an unconditional branch or ret
   taken = exec; for BBLs cut only because the next instruction is a
   jump target, fallthru = exec and taken = 0.


	Actual Performance Comparison (bzip2 with prof_time 10):
     - Unoptimized bprofile.so : on average 12.1
     - Optimized bprofile.so   : on average 11.2
     - Improvement             : about 7%

d. Large differences between this profile and the exercise-2 profile,
   and why we think they happen
------------------------------------------------------------------------
The output files show two massive differences: Exercise 4 records exponentially
higher execution counts (billions compared to thousands), but captures far fewer
unique Basic Blocks (784 compared to 4,166). Here is why this happens:

1. Drastic Difference in Instrumentation Overhead (Execution Counts)

* Exercise 2 JIT Overhead: Exercise 2 uses Pin's standard Just-In-Time (JIT)
  trace instrumentation. For every single basic block, it inserts a call
  (BBL_InsertCall) to an analysis function (CountBbl), causing a massive
  context-switching overhead between the application and the Pin tool for every
  block executed.
  
* Exercise 4 TC & Inlining: Exercise 4 translates the original binary into a
  custom Translation Cache (TC) and directly inlines the profiling instructions
  (using X86 assembly like INC operations) into the executable code. This
  eliminates the context-switching overhead entirely.
  
* Exercise 4 Dead-Register Optimization: Furthermore, Exercise 4 performs dead-
  register liveness analysis (isRegDeadAt). If registers like RAX, RBX, or RCX
  are provably dead at the instrumentation point, it skips generating the
  instructions to save and restore them to memory.
  
* The Result: Because Exercise 4 has virtually zero overhead compared to
  Exercise 2, the application runs near its native speed. Within the fixed
  2-second profiling interval (prof_time), the Ex 4 optimized code can iterate
  through tight loops hundreds of millions of times, whereas the sluggish Ex 2
  JIT tool only manages a small fraction of those executions.

2. Difference in Code Coverage (Unique Basic Blocks)

* Safety Restrictions in Ex 4: Exercise 4 operates in Probe mode and only
  commits routines to the Translation Cache if Pin considers them safe for
  probed replacement (RTN_IsSafeForProbedReplacement). Additionally, it
  explicitly skips PLT stubs (e.g., .plt, .plt.got) and routines that fail to
  decode or translate. This causes it to "miss" blocks that Exercise 2's JIT
  compiler safely catches.
 
* Time-Interval Constraints: Exercise 4 uses a dedicated thread
  (start_stop_profile_gathering_thread_func) to run the profiling for a specific
  interval (e.g., 2 seconds) before disabling the counters by patching the
  Translation Cache with bypass jumps. Consequently, Ex 4 mostly captures the
  "hot" core loops of the program running during that specific window, whereas
  Ex 2 captures the initialization, setup, and teardown phases, resulting in a
  much wider array of "cold" basic blocks.