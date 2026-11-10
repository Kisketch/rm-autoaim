#!/usr/bin/env bash
# Verify the byte-identical shared-core set against the sibling repository.
# Usage: tools/check_shared_core.sh <path-to-sibling-repo-checkout>
# Exit 0 = identical set matches; 1 = divergence (or bad usage).
# The classified intentional forks are listed in docs/shared_core.md and are
# NOT checked here.
set -u
sibling="${1:-}"
if [ -z "$sibling" ] || [ ! -d "$sibling" ]; then
    echo "usage: $0 <path-to-sibling-repo-checkout>" >&2
    exit 1
fi
root="$(cd "$(dirname "$0")/.." && pwd)"
files="
include/autoaim/pose_buffer.hpp
src/pose_buffer.cpp
include/autoaim/delay_calibration.hpp
src/delay_calibration.cpp
include/autoaim/draw.hpp
src/draw.cpp
include/autoaim/evaluation.hpp
docs/shared_core.md
"
fail=0
for f in $files; do
    if [ ! -f "$sibling/$f" ]; then
        echo "MISSING in sibling: $f"
        fail=1
    elif ! cmp -s "$root/$f" "$sibling/$f"; then
        echo "DIVERGED: $f"
        fail=1
    fi
done
if [ "$fail" -eq 0 ]; then
    echo "shared core identical set: OK"
fi
exit "$fail"
