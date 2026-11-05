#!/usr/bin/env bash
set -euo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "$root/scripts/java-env.sh"
pose_select_java "$root"
# Keep this tool's OpenCV ahead of LAS.utils' legacy opencv-320.jar.
exec "$pose_java" --enable-native-access=ALL-UNNAMED "-Xmx${PHOTOGRAMMETRY_HEAP:-8g}" -cp "$root/target/classes:$root/lib/*" photogrammetry "$@"
