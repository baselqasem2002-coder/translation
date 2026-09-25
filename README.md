# translation — Project 2026: `project.so`

A Pin probe-mode tool. It profiles the main executable in a translation cache
(TC) for `-prof_time` seconds, then builds an optimized second cache (TC2) with
de-virtualization and code reordering, and moves execution to it.

The submission README (names, compilation, how to run, thresholds, design) is
[`src/README.txt`](src/README.txt).

## Layout

| Path | What it is |
|------|------------|
| `src/` | The submission sources: `project.cpp`, `makefile`, `makefile.rules`, `README.txt` |
| `tests/translation/` | Tests that run the tool's code generation natively, without Pin |
| `ex4/` | Our exercise 4 (`bprofile.cpp` and its README), kept for reference |
| `course/` | Course material: `project-2026.pdf`, `bprofile-with-gearing.cpp.txt`, the reverse-cond-jumps example, the code reordering paper |

## Build

```
cd src
make PIN_ROOT=<path-to-pin-kit> obj-intel64/project.so
```

## Tests

The tests compile `src/project.cpp` against a mock `pin.H` and the XED library
of the Pin kit, then execute the generated TC/TC2 code:

```
PIN_ROOT=<path-to-pin-kit> tests/translation/run.sh
```

| Test | Checks |
|------|--------|
| `stub_test` | profiling stubs for indirect jumps/calls record the right targets |
| `pipeline_test` | `create_tc()` → TC → stop profiling → `create_tc2()` → TC2, and TC2 has no profiling code |
| `devirt_test` | de-virtualization: register/memory/`[rip+disp]` forms, the 90% boundary, rare targets |
| `reorder_test` | code reordering: TC2 layout per instruction, reversed branches, added jumps, all paths |
| `ex4_merge_test` | exercise 4 fixes: routine filters, revert on failure, jcc to original code, dead-register optimization |

They test code generation only. They do not replace running the tool under Pin
on the course binaries.

## Submission (`project.zip`)

The project asks for `project.so` plus a `src` directory:

```
cd src && make PIN_ROOT=<path-to-pin-kit> obj-intel64/project.so && cd ..
rm -rf submit && mkdir submit
cp src/obj-intel64/project.so submit/
mkdir submit/src && cp src/project.cpp src/makefile src/makefile.rules src/README.txt submit/src/
(cd submit && zip -r ../project.zip project.so src)
```
