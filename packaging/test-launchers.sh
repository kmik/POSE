#!/usr/bin/env bash
# Packaging regression checks; fake JVMs exercise selection/quoting without running processing.
set -euo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
mkdir -p "$root/target"
testroot="$(mktemp -d "$root/target/portable-launcher-test.XXXXXX")"
bundle="$testroot/relocated bundle"
mkdir -p "$bundle/scripts" "$bundle/runtime/bin" "$testroot/system11/bin" "$testroot/override/bin"
cp "$root/pose.sh" "$root/photogrammetry.sh" "$bundle/"
cp "$root/scripts/java-env.sh" "$bundle/scripts/"
cat > "$bundle/runtime/bin/java" <<'SH'
#!/usr/bin/env bash
if [[ "${1:-}" == -version ]]; then echo 'openjdk version "25.0.1"' >&2; exit 0; fi
printf '%s\n' 'PRIVATE-JAVA' "$@"
SH
cat > "$testroot/system11/bin/java" <<'SH'
#!/usr/bin/env bash
echo 'openjdk version "11.0.22"' >&2
SH
cat > "$testroot/override/bin/java" <<'SH'
#!/usr/bin/env bash
if [[ "${1:-}" == -version ]]; then echo 'openjdk version "25.0.1"' >&2; exit 0; fi
echo EXPLICIT-JAVA
exit 37
SH
chmod +x "$bundle/runtime/bin/java" "$testroot/system11/bin/java" "$testroot/override/bin/java"
result="$(JAVA_HOME="$testroot/system11" POSE_JAVA_HOME= PHOTOGRAMMETRY_HEAP=2g \
  bash "$bundle/pose.sh" -i 'images with spaces' -o 'output with spaces')"
[[ "$result" == PRIVATE-JAVA* ]] || { echo 'Bundled Java did not win over system Java 11' >&2; exit 1; }
grep -Fx -- '-Xmx2g' <<< "$result" >/dev/null
grep -Fx -- 'images with spaces' <<< "$result" >/dev/null
grep -Fx -- 'output with spaces' <<< "$result" >/dev/null
grep -Fx -- "$bundle/target/classes:$bundle/lib/*" <<< "$result" >/dev/null
echo 'PASS: bundled Java wins; relocated path, heap and arguments with spaces preserved'

if POSE_JAVA_HOME="$testroot/override" bash "$bundle/photogrammetry.sh" > "$testroot/override.log"; then
  echo 'Expected explicit JVM exit status 37' >&2; exit 1
else
  status=$?
  [[ "$status" == 37 ]] || { echo "Exit status changed: $status" >&2; exit 1; }
fi
grep -Fx EXPLICIT-JAVA "$testroot/override.log" >/dev/null
echo 'PASS: explicit override wins; launcher propagates JVM exit status'

mkdir -p "$testroot/source/scripts"
cp "$root/scripts/java-env.sh" "$testroot/source/scripts/"
if (source "$root/scripts/java-env.sh"; JAVA_HOME="$testroot/system11" POSE_JAVA_HOME= \
    pose_select_java "$testroot/source") > "$testroot/old.log" 2>&1; then
  echo 'Java 11 was accepted' >&2; exit 1
fi
grep -F 'Java 11 is too old' "$testroot/old.log" >/dev/null
if (source "$root/scripts/java-env.sh"; POSE_JAVA_HOME="$bundle/runtime" \
    pose_select_java "$bundle" yes) > "$testroot/jre.log" 2>&1; then
  echo 'Build accepted a runtime without javac' >&2; exit 1
fi
grep -F 'building requires a JDK' "$testroot/jre.log" >/dev/null
echo 'PASS: old Java and runtime-only build fail with actionable messages'

printf '#!/usr/bin/env bash\necho "javac 25.0.1"\n' > "$bundle/runtime/bin/javac"
chmod +x "$bundle/runtime/bin/javac"
(source "$root/scripts/java-env.sh"; POSE_JAVA_HOME="$bundle/runtime" pose_select_java "$bundle" yes)
echo 'PASS: private JDK accepted for builds'
echo "Launcher fixtures: $testroot"
