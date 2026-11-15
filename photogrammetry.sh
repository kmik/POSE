#!/usr/bin/env bash
set -euo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "$root/scripts/java-env.sh"
pose_select_java "$root"
# glibc gives every busy thread its own malloc arena; many matching/stereo threads calling OpenCV
# then keep tens of GB of fragmented native memory outside the Java heap. Four arenas bound that
# (set MALLOC_ARENA_MAX yourself to override). Other C libraries ignore the variable.
# Keep this tool's OpenCV ahead of LAS.utils' legacy opencv-320.jar.
MALLOC_ARENA_MAX="${MALLOC_ARENA_MAX:-4}" exec "$pose_java" --enable-native-access=ALL-UNNAMED "-Xmx${PHOTOGRAMMETRY_HEAP:-8g}" -cp "$root/target/classes:$root/lib/*" photogrammetry "$@"
