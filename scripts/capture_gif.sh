#!/bin/bash
# Capture demo_bounce frames and convert to an animated GIF for the README.
# Usage: ./scripts/capture_gif.sh
# Requires: cmake, raylib, ffmpeg

set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
BUILD="$ROOT/build"
FRAMES="$ROOT/build/frames"
OUT="$ROOT/docs/demo_bounce.gif"

# Build
cmake -S "$ROOT" -B "$BUILD" -DHOP_BUILD_EXAMPLES=ON -DHOP_BUILD_TESTS=OFF >/dev/null 2>&1
cmake --build "$BUILD" --target demo_bounce >/dev/null 2>&1

# Capture frames
rm -rf "$FRAMES"
mkdir -p "$FRAMES"
"$BUILD/demo_bounce" --capture "$FRAMES"

# Convert to GIF via ffmpeg (palette-based for quality)
ffmpeg -y -framerate 60 -i "$FRAMES/frame_%04d.png" \
    -vf "fps=30,scale=400:-1:flags=lanczos,split[s0][s1];[s0]palettegen=max_colors=128[p];[s1][p]paletteuse=dither=bayer:bayer_scale=3" \
    "$OUT" 2>/dev/null

rm -rf "$FRAMES"
echo "Created $OUT ($(du -h "$OUT" | cut -f1))"
