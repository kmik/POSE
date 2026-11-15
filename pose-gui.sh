#!/usr/bin/env bash
# POSE desktop window (photogrammetry gui). Needs a display; on a server use pose.sh.
set -euo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
exec bash "$root/photogrammetry.sh" gui "$@"
