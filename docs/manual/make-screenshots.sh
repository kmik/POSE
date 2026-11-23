#!/usr/bin/env bash
# Regenerates the manual's screenshots and result images (manual/shots) from a demo project.
# Usage: bash manual/make-screenshots.sh IMAGE_FOLDER [DEMO_DIR]
#   IMAGE_FOLDER  a small drone set with GPS (the manual uses 32 forest images)
#   DEMO_DIR      neutral working folder shown in the screenshots (default C:/t/pose-demo)
# Needs a display: the windows open briefly at the top-left of the screen while being captured.
# Build POSE (classes and test classes) first.
set -euo pipefail
here="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# The program root: the nearest folder above with the build script (manual/ or docs/manual/).
root="$here"; while [ ! -f "$root/build.sh" ] && [ "$root" != "/" ]; do root="$(dirname "$root")"; done
images_in="$1"; demo="${2:-C:/t/pose-demo}"
sep=":"; cproot="$root"
case "$(uname -s)" in MINGW*|MSYS*|CYGWIN*) sep=";"; cproot="$(cygpath -m "$root")";; esac
cp_all="$cproot/target/classes${sep}$cproot/target/test-classes${sep}$cproot/lib/*"
mkdir -p "$demo/images" "$demo/gui-settings"
cp "$images_in"/* "$demo/images/"
if [ ! -d "$demo/forest-survey/runs" ]; then
  java --enable-native-access=ALL-UNNAMED -Xmx8g -cp "$cp_all" photogrammetry \
    -i "$demo/images" -o "$demo/forest-survey" --preset drone,forest --quality medium \
    --classify-ground --orthomosaic --tie-point-cleanup auto \
    --point-attributes surface,uncertainty,views --memory-mb 12000 > "$demo/run.log" 2>&1
fi
# Window screenshots; a separate settings folder keeps personal recent projects out of them.
java --enable-native-access=ALL-UNNAMED -Dpose.gui.settings="$demo/gui-settings" -cp "$cp_all" \
  sgm.ManualScreenshots "$demo/forest-survey" "$demo/images" "$here/shots"
# Result images and report pages of the demo run.
run="$(ls -d "$demo"/forest-survey/runs/* | tail -1)/07_results"
for f in dsm dtm orthomosaic image-overlap camera-map ground_support density; do cp "$run/$f.png" "$here/shots/result-$f.png"; done
pages="$demo/report-pages"
java -cp "$cp_all" sgm.PdfReportRegression "$run/processing-report.pdf" "$pages" > /dev/null
cp "$pages/page-01.png" "$here/shots/report-summary.png"
cp "$pages/page-05.png" "$here/shots/report-calibration.png"
cp "$pages/page-08.png" "$here/shots/report-dense.png"
echo "Screenshots in $here/shots; check the report page numbers if the report layout changed."
