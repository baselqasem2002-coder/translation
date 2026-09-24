#!/bin/sh
# Builds and runs the profiling-stub execution test WITHOUT running Pin.
# It compiles src/project.cpp against a mock pin.H (tests/profiling_stub/mock)
# and the real XED library, then executes the generated stubs.
#
#   PIN_ROOT=<path to pin kit> tests/profiling_stub/run.sh
#   (or XED_KIT=<dir with include/xed and lib/libxed.a> tests/profiling_stub/run.sh)
set -e
cd "$(dirname "$0")"
XED_KIT=${XED_KIT:-$PIN_ROOT/extras/xed-intel64}
if [ ! -f "$XED_KIT/lib/libxed.a" ]; then
  echo "set PIN_ROOT (or XED_KIT) so that \$XED_KIT/lib/libxed.a exists" >&2
  exit 1
fi
g++ -std=c++11 -O1 -no-pie -Wall -Wno-unknown-pragmas \
    -DTARGET_IA32E -DHOST_IA32E -DTARGET_LINUX \
    -Imock -I"$XED_KIT/include/xed" -I"$XED_KIT/include" \
    run_stub_test.cpp "$XED_KIT/lib/libxed.a" -o /tmp/run_stub_test
/tmp/run_stub_test
# Debug: DUMP=1 prints dump_tc() + instr_map per case, VERB=1 enables -verbose.
