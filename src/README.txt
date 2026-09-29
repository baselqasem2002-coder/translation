========================================================================
Project 2026 - project.so: profile-guided TC2 optimizer (Pin probe mode)
========================================================================

a. Names + ID numbers
------------------------------------------------------------------------
Shadi Najar    213557143
Basel Qassem   324937036


b. Compilation command
------------------------------------------------------------------------
From this 'src' directory (Pin 4.0 kit):

    make PIN_ROOT=<path-to-pin-kit> obj-intel64/project.so

(Or copy the directory to <pin-kit>/source/tools/project and run
 'make obj-intel64/project.so' there.)


c. How to run the tool
------------------------------------------------------------------------
    <pin-kit>/pin -t obj-intel64/project.so [-prof_time <sec>] -- <app> [args]

For example:

    <pin-kit>/pin -t obj-intel64/project.so -prof_time 2 -- ./bzip2 -k -f input-long.txt
    <pin-kit>/pin -t obj-intel64/project.so -prof_time 4 -- ./cc1 200.i -o 200.s

The only output is the elapsed time of the translated code, including
the profiling time:

     Translated code run (including profiling) took: <sec> seconds

Knobs:
    -prof_time <sec>   profiling time in TC before TC2 is built (default 2)
    -no_devirt         disable de-virtualization
    -no_reorder        disable code reordering
    -no_regpromo       disable register promotion
    -no_unroll         disable loop unrolling
    -no_inline         disable leaf function inlining
    -no_constprop      disable constant propagation
    -stats             print what every TC2 optimization did
    -dump_promo        print the promoted stack slots and rewritten instrs
    -dump_prof         write the profile to bprofile.out at exit
    -verbose, -dump_tc, -dump_tc2, -dump_orig_code, -no_tc_commit
                       debug knobs of bprofile-with-gearing


How it works
------------------------------------------------------------------------
1. TC: every routine of the main executable (of at least 120 bytes) is
   translated into TC with profiling code: a counter for every BBL (a
   single 'inc [rip+x]' placed before a CMP when the BBL has one, since
   CMP overwrites the flags; otherwise a flags-free mov/lea/mov stub), a
   fall-through counter for conditional branches, and a target profile
   (4 entries, indexed by target & 3) for every indirect jump AND every
   indirect call.

2. After -prof_time seconds a Pin internal thread turns the profiling
   off, builds TC2 from the TC instructions (without the profiling code)
   and applies the optimizations below, then redirects every routine of
   TC to its TC2 version.

   Race-free switching: the places in TC that are patched while the
   program runs (every routine head and every profiling stub head) are
   'jmp +5; jmp rel32'. The rel32 is written first (it is not executed),
   and then a single byte is changed (jmp +5 -> jmp +0, or directly to
   the end of a short stub). A one byte store is atomic, so a running
   thread never executes a half-written instruction.

3. TC2 optimizations, in this order:

   3.1 De-virtualization (indirect jumps and indirect calls).
       indirect call site:              indirect jump site:
         cmp  <target>, HOT               lea  rsp, [rsp-128]  ; red zone
         jne  MISS                        push rcx
         call HOT       ; direct, TC2     mov  rcx, <target>
         jmp  NEXT                        lea  rcx, [rcx-HOT]
       MISS: (cold BBL)                   jrcxz HIT
         call <target>  ; original        pop rcx / lea rsp,[rsp+128]
                                          jmp  <target>        ; original
                                        HIT:
                                          pop rcx / lea rsp,[rsp+128]
                                          jmp  HOT             ; direct, TC2
       The flags are dead at a call, so the call sequence may change them.
       The jump sequence does not change the flags (lea/mov/push/pop/
       jrcxz), which is much cheaper than pushfq/popfq. A de-virtualized
       jump keeps the hot path inside TC2 (a jump table holds original
       addresses, so without it execution would leave TC2).

   3.2 Code reordering. Inside every routine: the entry BBL first, then
       the hot BBLs in their original order, then the cold BBLs. A BBL
       whose fall-through successor is no longer next gets its
       conditional branch reversed when the branch target is now next
       ('jcc T' -> 'jncc F'), or else an added 'jmp F'. At the end,
       jumps to the next instruction are removed.

   3.3 Loop unrolling. A loop made of a single BBL (ending with a
       conditional branch to its own start) is unrolled twice; the first
       copy exits with the reversed condition.

   3.4 Leaf function inlining. A call to a routine of at most 5
       instructions ending with a plain 'ret', with no branches, calls,
       push/pop or any use of rsp, is replaced by its instructions.

   3.5 Constant propagation. Inside a BBL, 'mov reg1, imm; ...;
       mov reg2, reg1' becomes 'mov reg2, imm' while reg1 is unchanged.

   3.6 Register promotion of stack slots. Code compiled without
       optimization (like bzip2) keeps every local variable in a stack
       slot [rbp+disp]. In a routine without calls/syscalls/indirect
       jumps, that starts with 'push rbp; mov rbp, rsp' and does not
       change rbp, a local slot whose every access is a non-indexed
       [rbp+disp] operand of the same width (1/2/4/8 bytes), with no
       overlapping access and no 'lea' of its address, is kept in a
       register: every 'op ..., [rbp+disp]' is rewritten to
       'op ..., reg' (same instruction and width, so the same flags).
       Every rewritten instruction is decoded again and compared with
       the original; if any access of a slot cannot be rewritten the slot
       stays in memory. Registers, in order:
         - an argument register whose only use is its spill into the
           slot (the slot lives in the register it came in: coalescing),
         - caller-saved GPRs that the routine never uses,
         - callee-saved GPRs (rbx, r12-r15) that the routine never uses:
           saved after 'mov rbp, rsp' into the stack memory of an
           already promoted 8-byte slot and restored before every
           'pop rbp' / 'leave'.
       In bzip2, 10 of the 11 locals of mainGtU (75% of all executed
       instructions) are promoted.


d. Threshold used to distinguish frequently and rarely executed code
------------------------------------------------------------------------
All the counts are the ones collected during the -prof_time seconds in
TC.

* Basic blocks (code reordering): a BBL is FREQUENT (hot) if its
  execution counter is at least 1 during the profiling, and RARE (cold)
  if its counter is 0. Cold BBLs are moved to the end of their routine,
  so the code that really ran is packed together (fewer i-cache lines
  and fewer taken branches). The miss path of a de-virtualized call has
  no counter, so it is cold too. A routine is reordered only when a cold
  BBL lies before a hot one.

* Indirect jump/call targets (de-virtualization): the most frequent
  target of a site is FREQUENT when it was reached at least 1000 times
  (DEVIRT_MIN_COUNT) AND it is at least 80% (DEVIRT_MIN_PERCENT) of all
  the targets profiled at that site. Only then is the site
  de-virtualized; the other targets (rare) go through the original
  indirect instruction on the miss path. The hot target must also be
  translated (in TC2).

* Stack slots (register promotion): the frequency of a slot is the sum
  of the counters of the BBLs of all its accesses. Slots accessed fewer
  than 100 times (PROMO_MIN_ACCESSES) are rare and stay in memory; the
  others get the free registers from the most frequent one down.

* Routines: only routines of at least 120 bytes are translated (small
  routines gain little and cost a probe each).


Files
------------------------------------------------------------------------
project.cpp       the pintool
makefile          standard Pin kit makefile
makefile.rules    builds obj-intel64/project.so
README.txt        this file
