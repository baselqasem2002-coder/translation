#!/bin/bash
# Build a pintool source against the fake Pin runtime (test only).
# usage: XED_KIT=<xed kit dir> ./build.sh <tool.cpp> <out.so>
# XED: git clone https://github.com/intelxed/xed && git clone https://github.com/intelxed/mbuild
#      cd xed && python3 mfile.py install   -> kits/xed-install-*/
set -e
D=$(cd $(dirname $0) && pwd)
: ${XED_KIT:?set XED_KIT to the XED kit directory}
W=$(mktemp --suffix=.cpp)
printf '#include "%s"\nvoid fakepin_exit_hook() { Fini(0, 0); }\n' "$(realpath $1)" > $W
g++ -O2 -g -DTARGET_IA32E -DHOST_IA32E -DTARGET_LINUX -fPIC -shared -w -I$D -I$XED_KIT/include/xed \
    $W $D/fakepin.cpp $XED_KIT/lib/libxed.a -lpthread -o $2
rm -f $W
