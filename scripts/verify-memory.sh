#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."

# Isolated compilation: does not replace the installed engine or edit any project.
out="target/memory-validation-$(date -u +%Y%m%dT%H%M%SZ)-$$"
mkdir -p "$out/classes" "$out/tests"
if [[ -n "${POSE_JAVA_HOME:-}" ]]; then
  java_bin="$POSE_JAVA_HOME/bin/java"
  javac_bin="$POSE_JAVA_HOME/bin/javac"
elif [[ -n "${JAVA_HOME:-}" ]]; then
  java_bin="$JAVA_HOME/bin/java"
  javac_bin="$JAVA_HOME/bin/javac"
else
  java_bin=java
  javac_bin=javac
fi

if command -v rg >/dev/null 2>&1; then
  rg --files src/main/java -g '*.java' > "$out/main-sources.txt"
  rg --files src/test/java -g '*.java' > "$out/test-sources.txt"
else
  find src/main/java -name '*.java' -print > "$out/main-sources.txt"
  find src/test/java -name '*.java' -print > "$out/test-sources.txt"
fi
"$java_bin" -version 2>&1 | tee "$out/java-version.log"
"$javac_bin" --release 25 -encoding UTF-8 -cp 'lib/*' -d "$out/classes" @"$out/main-sources.txt"
cp -R src/main/resources/. "$out/classes/"
"$javac_bin" --release 25 -encoding UTF-8 -cp "$out/classes:lib/*" -d "$out/tests" @"$out/test-sources.txt"
classpath="$out/classes:$out/tests:lib/*"
"$java_bin" --enable-native-access=ALL-UNNAMED -Xmx512m -ea -cp "$classpath" sgm.AlignmentMemoryRegression \
  2>&1 | tee "$out/memory-controls.log"
"$java_bin" --enable-native-access=ALL-UNNAMED -Xmx1g -ea -cp "$classpath" sgm.LauncherRegression \
  2>&1 | tee "$out/launcher-controls.log"
"$java_bin" --enable-native-access=ALL-UNNAMED -Xmx1g -ea -cp "$classpath" sgm.BundleSchurRegression \
  2>&1 | tee "$out/bundle-controls.log"
printf 'Memory validation passed. Logs: %s\n' "$out"
