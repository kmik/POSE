#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/../.."
if [[ ! -f lib/opencv-4.9.0-0.jar ]]; then
  echo 'Dependencies missing. Run build.sh once before the research tools.' >&2
  exit 2
fi
mode="${1:-small}"
if [[ $# -gt 0 ]]; then shift; fi
runtime=java
compiler=javac
if [[ -n "${JAVA_HOME:-}" ]]; then
  runtime="$JAVA_HOME/bin/java"
  compiler="$JAVA_HOME/bin/javac"
fi
build=target/esgm-research-tool
mkdir -p "$build/classes"
find src/main/java -name '*.java' -print | while IFS= read -r source; do printf '"%s"\n' "$source"; done > "$build/sources.txt"
for name in TestSupport EsgmPrototype EsgmInvestigation AdaptiveStereoInvestigation AdaptiveSearchBenchmark RectifiedResearchReplay; do
  printf '"src/test/java/sgm/%s.java"\n' "$name" >> "$build/sources.txt"
done
"$compiler" --release 25 -encoding UTF-8 -cp 'lib/*' -d "$build/classes" "@$build/sources.txt"
main=sgm.EsgmInvestigation
case_args=()
case "$mode" in
  small) ;;
  large) case_args=(large) ;;
  adaptive) main=sgm.AdaptiveSearchBenchmark ;;
  pair) case_args=(pair) ;;
  replay) main=sgm.RectifiedResearchReplay ;;
  *) echo "Unknown research mode: $mode" >&2; exit 2 ;;
esac
output="${ESGM_RESEARCH_OUTPUT:-target/esgm-research/${mode}-$(date -u +%Y%m%dT%H%M%SZ)}"
"$runtime" --enable-native-access=ALL-UNNAMED "-Xmx${ESGM_RESEARCH_HEAP:-4g}" \
  "-Dsgm.test.gpu=${ESGM_RESEARCH_GPU:-0}" "-Dsgm.research.output=$output" \
  -cp "$build/classes:lib/*" "$main" "${case_args[@]}" "$@"
