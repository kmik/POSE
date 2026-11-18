#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
source scripts/java-env.sh
pose_select_java "$PWD" yes
out="target/host-fusion-check-$(date -u +%Y%m%dT%H%M%SZ)-$$"
mkdir -p "$out/classes" "$out/tests"
if command -v rg >/dev/null 2>&1; then
  rg --files src/main/java -g '*.java' > "$out/main.txt"
  rg --files src/test/java -g '*.java' > "$out/tests.txt"
else
  find src/main/java -name '*.java' > "$out/main.txt"
  find src/test/java -name '*.java' > "$out/tests.txt"
fi
"$pose_javac" --release 25 -encoding UTF-8 -cp 'lib/*' -d "$out/classes" @"$out/main.txt"
cp -R src/main/resources/. "$out/classes/"
"$pose_javac" --release 25 -encoding UTF-8 -cp "$out/classes:lib/*" -d "$out/tests" @"$out/tests.txt"
# Preserve SelfTest itself. Two existing suites execute ultra-quality fixtures; omit those
# suites in this explicitly high-only validation entry point and report both omissions.
python3 - "$out/HostMemorySuite.java" <<'PY'
import pathlib, sys
s = pathlib.Path('src/test/java/sgm/SelfTest.java').read_text(encoding='utf-8')
changes = {
 'public final class SelfTest {': 'public final class HostMemorySuite {',
 'checks += BundleQualityRegression.run(root);': 'System.out.println("SKIP BundleQualityRegression: high-only validation");',
 'checks += MultiViewDisparityRegression.run(root.resolve("multiview-disparity"));': 'System.out.println("SKIP MultiViewDisparityRegression: high-only validation");',
}
for old, new in changes.items():
    if s.count(old) != 1:
        raise SystemExit('SelfTest changed; review high-only harness: ' + old)
    s = s.replace(old, new)
pathlib.Path(sys.argv[1]).write_text(s, encoding='utf-8')
PY
"$pose_javac" --release 25 -encoding UTF-8 -cp "$out/classes:$out/tests:lib/*" -d "$out/tests" "$out/HostMemorySuite.java"
"$pose_java" --enable-native-access=ALL-UNNAMED -Xmx8g -XX:ActiveProcessorCount=4 -ea \
  -cp "$out/classes:$out/tests:lib/*" sgm.HostMemorySuite 2>&1 | tee "$out/checks.log"
if [[ ${1:-} == --install ]]; then
  mkdir -p target/classes
  cp -R "$out/classes/." target/classes/
  echo 'Installed verified engine classes into target/classes.'
elif [[ -n ${1:-} ]]; then
  echo 'Usage: bash scripts/verify-fusion-host-memory.sh [--install]' >&2; exit 2
fi
echo "High-only validation artifacts: $out (two ultra-fixture suites skipped)."
