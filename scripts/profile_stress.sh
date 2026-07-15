#!/usr/bin/env bash
# profile_stress.sh — Profile demo_stress and print a function breakdown.
#
# Builds demo_stress with optimizations + frame pointers + debug symbols, runs it
# in its headless mode (`demo_stress --headless STEPS` — identical scene/physics,
# no window), and samples it with the macOS `sample(1)` profiler to produce a flat
# self-time function breakdown (where the CPU actually is) plus the full call tree.
#
# Usage:
#   scripts/profile_stress.sh [STEPS]
#     STEPS   number of 16ms ticks to run (default 400)
#
# Requires: cmake, a C++ compiler, raylib (found via CMake), and macOS `sample`
# (ships with the Xcode command-line tools).

set -euo pipefail

STEPS="${1:-400}"

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BUILD_DIR="$REPO_ROOT/build-profile"
BIN="$BUILD_DIR/demo_stress"
SAMPLE_RAW="$BUILD_DIR/sample.txt"

if ! command -v sample >/dev/null 2>&1; then
	echo "error: 'sample' not found — install Xcode command-line tools (xcode-select --install)" >&2
	exit 1
fi

# RelWithDebInfo = -O2 with symbols; add frame pointers so the sampler can walk
# stacks reliably. Configure once, then just rebuild the target on reruns.
echo ">> configuring $BUILD_DIR (RelWithDebInfo, examples on)"
cmake -S "$REPO_ROOT" -B "$BUILD_DIR" \
	-DCMAKE_BUILD_TYPE=RelWithDebInfo \
	-DHOP_BUILD_EXAMPLES=ON \
	-DHOP_BUILD_TESTS=OFF \
	-DCMAKE_CXX_FLAGS_RELWITHDEBINFO="-O2 -g -fno-omit-frame-pointer" \
	>/dev/null

echo ">> building demo_stress"
cmake --build "$BUILD_DIR" --target demo_stress >/dev/null

echo ">> running: demo_stress --headless $STEPS (sampling every 1ms)"
echo "-----------------------------------------------------------------------"
# Launch headless in the background so we can attach the sampler to its PID.
# `sample` runs until the process exits (the large duration is just an upper
# bound) and writes the full report to $SAMPLE_RAW.
"$BIN" --headless "$STEPS" &
PID=$!
sample "$PID" 100000 1 -file "$SAMPLE_RAW" >/dev/null 2>&1 &
SAMPLE_PID=$!
wait "$PID"
wait "$SAMPLE_PID" 2>/dev/null || true

echo "-----------------------------------------------------------------------"
echo
echo "===================  SELF-TIME FUNCTION BREAKDOWN  ===================="
echo "(samples whose stack TOP is this function — i.e. where the CPU actually sat)"
echo
# `sample` emits a flat "Sort by top of stack" table — extract it and demangle.
awk '
	/Sort by top of stack/ { grab=1; next }
	grab && /^$/           { if (seen) exit }
	grab                   { seen=1; print }
' "$SAMPLE_RAW" | c++filt | head -40

echo
echo "Full call tree + binary images written to:"
echo "  $SAMPLE_RAW"
echo "Open it for the hierarchical (inclusive-time) view, or load $BIN in Instruments."
