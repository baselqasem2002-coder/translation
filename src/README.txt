========================================================================
Project 2026 - project.so: profiling in TC, optimized TC2
========================================================================

a. Names + ID numbers
------------------------------------------------------------------------
<Name 1>    <ID 1>
<Name 2>    <ID 2>


b. Compilation command
------------------------------------------------------------------------
From this src directory:

    make PIN_ROOT=<path-to-pin-kit> obj-intel64/project.so

(Alternatively, place the src directory under <pin-kit>/source/tools/ and
 run 'make obj-intel64/project.so' there.)


c. How to run the tool
------------------------------------------------------------------------
    <pindir>/pin -t obj-intel64/project.so -prof_time <seconds> -- <binary> [args]

For example:

    <pindir>/pin -t obj-intel64/project.so -prof_time 4 -- ./cc1 200.i -o 200.s
    <pindir>/pin -t obj-intel64/project.so -- ./bzip2 <args>      (prof_time = 2)

On exit the tool prints the elapsed time of the translated code, including
the profiling time:

    Translated code run (including profiling) took: <seconds> seconds

Knobs:
  -prof_time <n>    seconds of profiling in TC before TC2 is built (default 2)
  -devirt_pct <p>   de-virtualization threshold in percent (default 90)
  -no_devirt        do not apply de-virtualization in TC2
  -no_reorder       do not apply code reordering in TC2
  -no_deadreg       do not skip dead RAX save/restore in the profiling stubs
  -no_prof          no profiling stubs in TC (so no profile for TC2)
  -no_tc_commit     build the TC/TC2 but do not redirect execution to them
Debug knobs:
  -dump_orig_code   dump the original code of the main executable
  -dump_tc          dump TC (dump_tc())
  -dump_tc2         dump TC2 (dump_tc())
  -dump_prof        print every profiled indirect jump/call with its targets,
                    and every de-virtualization decision
  -verbose          print every instruction while translating


d. Threshold between frequently and rarely executed instructions
------------------------------------------------------------------------
All counts are the ones collected in TC during the first -prof_time
seconds of the run.

1. De-virtualization (indirect jumps and indirect calls):
   A target of an indirect jump/call is FREQUENT if it took at least 90%
   of the executions of that jump/call:

        hot_target_count * 100 >= site_exec_count * 90

   Only a site with a frequent target is de-virtualized. All its other
   targets are RARE and keep using the original indirect jmp/call.
   (The percentage is set by -devirt_pct.)

2. Code reordering (basic blocks):
   A basic block is FREQUENT (hot) if it was executed at least once during
   profiling, and RARE (cold) if its counter is 0. Cold blocks are moved
   to the end of their routine.


e. What the tool does
------------------------------------------------------------------------
Based on the provided bprofile-with-gearing.cpp and on our exercise 4
(the merged exercise-4 fixes are marked "EX4" in project.cpp).

1. TC with profiling (create_tc, at image load of the main executable):
   every candidate routine of the main executable is translated into TC,
   with inline profiling stubs that count, per basic block, its executions,
   its fall-through executions, and the targets of an indirect jump or an
   indirect call ending it. The original routines are probed to jump to TC.

2. After -prof_time seconds (create_tc2_thread_func, a Pin internal thread):
   the profiling stubs in TC are bypassed, and create_tc2() builds TC2 from
   the TC instructions:
     - Step 0:  choose the indirect jumps/calls to de-virtualize (threshold 1)
     - Step 1:  remove the profiling code (TC2 contains no counters)
     - Step 1b: de-virtualization:
                  call X  ->  cmp X, T / jne miss / call T_tc2 / jmp done /
                              miss: call X / done:
                  jmp X   ->  cmp X, T / je T_tc2 / jmp X
                T is the frequent target and T_tc2 its copy in TC2. The cmp
                checks the real target every time, so a rare target still
                goes through the original jmp/call.
     - Step 1c: code reordering, per routine: entry block, then the hot
                blocks, then the cold blocks (threshold 2). A block whose
                fall-through successor is no longer next gets its cond branch
                reversed (jcc T -> jncc F, when T is now next) or an added
                'jmp F'.
     - Steps 3-7: chaining, layout, fixing displacements, writing TC2, and
                redirecting the head of every TC routine to its TC2 copy.

Not de-virtualized: sites whose target is not translated, or whose target
address does not fit a 32-bit immediate (binary loaded above 2GB). Not
reordered: routines with LOOP/JRCXZ (8-bit displacement, no reversed form).


f. Measurements
------------------------------------------------------------------------
<fill in: binary, arguments, -prof_time, and the elapsed times of>
    provided bprofile-with-gearing.so :  <seconds>
    project.so                        :  <seconds>
    improvement                       :  <percent>
    project.so -no_devirt             :  <seconds>
    project.so -no_reorder            :  <seconds>
