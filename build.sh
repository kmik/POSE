#!/usr/bin/env bash
set -euo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
cd "$root"
dep="lib/opencv-4.9.0-0.jar"
mkdir -p lib target/classes target/test-classes
if [[ ! -f "$dep" ]]; then
  curl --fail --location --retry 3 'https://repo.maven.apache.org/maven2/org/openpnp/opencv/4.9.0-0/opencv-4.9.0-0.jar' -o "$dep.part"
  mv -- "$dep.part" "$dep"
fi
for artifact in proj4j proj4j-epsg; do
  jar="lib/$artifact-1.4.3.jar"
  if [[ ! -f "$jar" ]]; then
    curl --fail --location --retry 3 "https://repo.maven.apache.org/maven2/org/locationtech/proj4j/$artifact/1.4.3/$artifact-1.4.3.jar" -o "$jar.part"
    mv -- "$jar.part" "$jar"
  fi
done
dep='lib/*'
javac_bin="${JAVA_HOME:+$JAVA_HOME/bin/}javac"
find src/main/java -name '*.java' -print > target/sources.txt
"$javac_bin" --release 25 -encoding UTF-8 -cp "$dep" -d target/classes @target/sources.txt
find src/test/java -name '*.java' -print > target/tests.txt
"$javac_bin" --release 25 -encoding UTF-8 -cp "target/classes:$dep" -d target/test-classes @target/tests.txt
"${JAVA_HOME:+$JAVA_HOME/bin/}java" --enable-native-access=ALL-UNNAMED -ea -cp "target/classes:target/test-classes:$dep" sgm.SelfTest
