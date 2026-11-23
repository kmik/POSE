#!/usr/bin/env bash
# Builds the POSE user manual: option reference from `photogrammetry -h`, HTML, then PDF
# through a headless Chromium browser (Edge or Chrome). Run from anywhere after building POSE.
set -euo pipefail
here="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# The program root: the nearest folder above with the build script (manual/ or docs/manual/).
root="$here"; while [ ! -f "$root/build.sh" ] && [ "$root" != "/" ]; do root="$(dirname "$root")"; done
out="$here/build"
mkdir -p "$out"
sep=":"; cproot="$root"
case "$(uname -s)" in MINGW*|MSYS*|CYGWIN*) sep=";"; cproot="$(cygpath -m "$root")";; esac
java -cp "$cproot/target/classes${sep}$cproot/lib/*" photogrammetry -h > "$out/help.txt" 2>&1 || true
grep -q "^Pipeline steps" "$out/help.txt" || { echo "photogrammetry -h failed; build POSE first" >&2; cat "$out/help.txt" >&2; exit 1; }
# Machine-specific and file-name references do not belong in a manual.
perl -0pi -e 's/at least 1;\n(\s+)this machine: \d+\)\./at least 1)./; s/(see|See) [A-Z_]+\.md/$1 the user manual/g; s/ \(supported variants and conventions in ORIENTATIONS\.md\)//' "$out/help.txt"
awk -f "$here/help2html.awk" "$out/help.txt" > "$out/reference.html"
date_text="$(date +'%-d %B %Y')"
cp -r "$here/images" "$here/figures" "$here/shots" "$out/"
cat "$here"/src/*.html | awk -v ref="$out/reference.html" -v date="$date_text" '
  /@REFERENCE@/ { while ((getline line < ref) > 0) print line; next }
  { gsub(/@DATE@/, date); print }' > "$out/pose-manual.html"
browser=""
for b in "/c/Program Files (x86)/Microsoft/Edge/Application/msedge.exe" "/c/Program Files/Google/Chrome/Application/chrome.exe" \
         "$(command -v chromium 2>/dev/null || true)" "$(command -v google-chrome 2>/dev/null || true)"; do
  [ -n "$b" ] && [ -x "$b" ] && { browser="$b"; break; }
done
[ -n "$browser" ] || { echo "No Edge/Chrome found: open $out/pose-manual.html and print it to PDF." >&2; exit 1; }
html="$out/pose-manual.html"; pdf="$here/POSE_User_Manual.pdf"
case "$sep" in ";") html="$(cygpath -m "$html")"; pdf="$(cygpath -w "$pdf")";; esac
"$browser" --headless=new --disable-gpu --no-pdf-header-footer --generate-pdf-document-outline \
  --print-to-pdf="$pdf" "file:///$html" 2>/dev/null
echo "Manual: $here/POSE_User_Manual.pdf"
