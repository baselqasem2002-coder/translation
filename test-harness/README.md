# Test harness (not part of the submission)

A small stand-in for the part of Pin's probe-mode API that `project.cpp` uses
(`pin.H`, `fakepin.cpp`), built on [XED](https://github.com/intelxed/xed) and the
ELF symbol table. It is loaded with `LD_PRELOAD` into a non-PIE x86-64 program,
runs the tool's `main` and image callback on the main executable, and then lets
the program run, exactly like probe mode does. It was used to test the tool
without a Pin kit; the final numbers must be measured with the real Pin 4.0.

```
XED_KIT=.../xed/kits/xed-install-... ./build.sh ../src/project.cpp project_fake.so
FAKEPIN_ARGS="-prof_time 2 -stats" LD_PRELOAD=./project_fake.so ./bzip2 -c -k input-long.txt > out.bz2
```

`tests/` has two regression programs; they work with the real Pin as well
(compare the output with a native run):

* `dv.c` - hot indirect calls and switch jumps (de-virtualization), including a
  leaf function with a switch and locals in the red zone.
* `rp.c` - `-O0` leaf routines with locals of all widths (register promotion).

```
gcc -O0 -no-pie -fno-pie -o dv_O0 tests/dv.c   # also -O1, -O2
./dv_O0 30000000 > ref.txt
pin -t project.so -prof_time 1 -- ./dv_O0 30000000 > out.txt && cmp ref.txt out.txt
```
