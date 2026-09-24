#!/bin/sh
# Builds and runs the translation tests WITHOUT running Pin. They compile
# src/project.cpp against a mock pin.H (tests/translation/mock) and the real
# XED library, and execute the generated code natively.
#
#   PIN_ROOT=<path to pin kit> tests/translation/run.sh
#   (or XED_KIT=<dir with include/xed and lib/libxed.a> tests/translation/run.sh)
#
# stub_test     - profiling stubs for indirect jumps/calls (add_profiling_instrs)
# pipeline_test - create_tc() -> run in TC -> disable profiling -> create_tc2() -> run in TC2
# devirt_test   - de-virtualization of indirect calls/jumps in TC2 (90% threshold)
# Debug: DUMP=1 prints dump_tc() output, VERB=1 (stub_test) enables -verbose.
set -e
cd "$(dirname "$0")"
XED_KIT=${XED_KIT:-$PIN_ROOT/extras/xed-intel64}
if [ ! -f "$XED_KIT/lib/libxed.a" ]; then
  echo "set PIN_ROOT (or XED_KIT) so that \$XED_KIT/lib/libxed.a exists" >&2
  exit 1
fi
OUT=${TMPDIR:-/tmp}
for t in stub_test pipeline_test devirt_test; do
  echo "=== $t"
  g++ -std=c++11 -O1 -no-pie -Wall -Wno-unknown-pragmas \
      -DTARGET_IA32E -DHOST_IA32E -DTARGET_LINUX \
      -Imock -I"$XED_KIT/include/xed" -I"$XED_KIT/include" \
      $t.cpp "$XED_KIT/lib/libxed.a" -o "$OUT/$t"
  "$OUT/$t"
done
