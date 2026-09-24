# translation — `project.so` (Pin, probe mode)

A skeleton for the 2026 project pintool. It profiles the program in **TC1**
for `-prof_time` seconds. Then a background thread builds **TC2** from that
profile and moves execution there while the program is running.

```
pin -t obj-intel64/project.so [-prof_time <seconds>] -- ./app <args>
```

| Knob         | Default | Meaning                                                  |
|--------------|---------|----------------------------------------------------------|
| `-prof_time` | `2`     | Seconds spent profiling in TC1 before switching to TC2   |

## Files

| File                      | What it is                                                         |
|---------------------------|--------------------------------------------------------------------|
| `project.cpp`             | The pintool: knob, tables, TC1/TC2 hooks, timer thread, the switch |
| `tc_dispatch.h`           | Dispatch slots/stubs used for the TC1 → TC2 switch (no Pin needed) |
| `tests/test_dispatch.cpp` | Multi-threaded test of the switch, runs without Pin                |
| `makefile`, `makefile.rules` | Standard Pin kit build files                                    |

## Timeline

```
ImageLoad(main exe)   build TC1 (+ ex4 counters), probe each routine -> its dispatch stub
ApplicationStart      spawn internal thread (PIN_SpawnInternalThread)
0 .. prof_time        app runs in TC1, counters grow; thread sleeps (PIN_Sleep)
prof_time             thread: snapshot counters -> build_tc2(profile) -> switch_to_tc2()
after                 every new call of a translated routine runs in TC2
```

## How the switch works (and why it is safe)

```
original routine        dispatch stub (fixed)          slot (8-byte data)
+------------------+    +-------------------------+    +-----------+
| jmp stub (probe) |--->| jmp qword ptr [slot_i]  |--->| TC1 entry |  before
+------------------+    +-------------------------+    | TC2 entry |  after
```

* Probes (`RTN_ReplaceProbed`) are placed only once, in `ImageLoad`. They
  point at a stub and **not** at TC1, so they never need to change.
* Switching a routine means one atomic 8-byte store into its slot. No
  instruction bytes change while the program runs.
* TC1 is never freed. Threads already inside TC1 finish there, and return
  addresses that point into TC1 stay valid.

## Where `bprofile-with-gearing.cpp` goes

See **Part 2** of `project.cpp`:

* `build_tc1(img)`: TC1 generation from bprofile/ex4. Set
  `g_rtns[i].tc1_entry`, register BBs with `add_bbl()`, and emit counters on
  `g_bbls[idx].exec_count` / `taken_count`.
* `build_tc2(profile)`: TC2 generation without counters, optimized using the
  profile. Set `g_rtns[i].tc2_entry`.
* Use `commit_through_dispatch_slots()` **instead of** bprofile's
  `commit_translated_routines`.

Three rules (explained in the code):

1. **R1:** in TC1, calls to other translated routines must go through the
   original address or the dispatch stub, never straight to TC1.
2. **R2:** `build_tc2` runs on the background thread. It must not call
   `IMG_`/`RTN_`/`INS_` functions, so save what you need in `build_tc1`.
   Allocate TC2's memory there too.
3. **R3:** never free or overwrite TC1.

Until Part 2 is filled in, the tool runs in **demo mode**. TC1 and TC2 both
point at the original code, so the whole flow runs and can be tested.

**Known limitation:** a routine that never returns (for example `main` looping
forever) keeps running in TC1, because the switch happens at routine entry.

## Build and test

```
# the pintool (inside the Pin kit, or with PIN_ROOT)
make PIN_ROOT=/path/to/pin obj-intel64/project.so

# the switch mechanism, no Pin needed
g++ -O2 -pthread -o test_dispatch tests/test_dispatch.cpp && ./test_dispatch
```
