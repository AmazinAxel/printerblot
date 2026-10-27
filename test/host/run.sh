#!/bin/sh
# Build and run the host simulation (see sim.cpp). Needs a C++17 compiler:
#   nix shell nixpkgs#gcc -c test/host/run.sh
set -e
here=$(dirname "$0")
src="$here/../../src"
out="${TMPDIR:-/tmp}/blot-sim"
g++ -std=c++17 -O1 -g -Wall -Wno-unused-function \
    -I "$here/stub" -I "$src" \
    "$here/sim.cpp" \
    "$src/stepper.cpp" "$src/planner.cpp" "$src/gcode.cpp" "$src/job.cpp" \
    "$src/settings.cpp" "$src/penservo.cpp" "$src/steptimer.cpp" \
    -o "$out"
exec "$out" "$@"
