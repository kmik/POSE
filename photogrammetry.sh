#!/usr/bin/env bash
set -euo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
java_bin="${JAVA_HOME:+$JAVA_HOME/bin/}java"
# Keep this tool's OpenCV ahead of LAS.utils' legacy opencv-320.jar.
exec "$java_bin" --enable-native-access=ALL-UNNAMED "-Xmx${PHOTOGRAMMETRY_HEAP:-8g}" -cp "$root/target/classes:$root/lib/*" photogrammetry "$@"
