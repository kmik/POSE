#!/usr/bin/env bash
set -euo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
cd "$root"
source "$root/scripts/java-env.sh"
pose_select_java "$root" yes
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
for group in imageio common; do
  if [[ "$group" == imageio ]]; then artifacts=(imageio-tiff imageio-core imageio-metadata); else artifacts=(common-lang common-io common-image); fi
  for artifact in "${artifacts[@]}"; do
    jar="lib/$artifact-3.15.3.jar"
    if [[ ! -f "$jar" ]]; then
      curl --fail --location --retry 3 "https://repo.maven.apache.org/maven2/com/twelvemonkeys/$group/$artifact/3.15.3/$artifact-3.15.3.jar" -o "$jar.part"
      mv -- "$jar.part" "$jar"
    fi
  done
done
dep='lib/*'
for artifact in pdfbox pdfbox-io fontbox; do
  jar="lib/$artifact-3.0.8.jar"
  if [[ ! -f "$jar" ]]; then
    curl --fail --location --retry 3 "https://repo.maven.apache.org/maven2/org/apache/pdfbox/$artifact/3.0.8/$artifact-3.0.8.jar" -o "$jar.part"
    mv -- "$jar.part" "$jar"
  fi
done
if [[ ! -f lib/commons-logging-1.4.0.jar ]]; then
  curl --fail --location --retry 3 'https://repo.maven.apache.org/maven2/commons-logging/commons-logging/1.4.0/commons-logging-1.4.0.jar' -o lib/commons-logging-1.4.0.jar.part
  mv -- lib/commons-logging-1.4.0.jar.part lib/commons-logging-1.4.0.jar
fi
javac_bin="$pose_javac"
find src/main/java -name '*.java' -print > target/sources.txt
"$javac_bin" --release 25 -encoding UTF-8 -cp "$dep" -d target/classes @target/sources.txt
cp -R src/main/resources/. target/classes/
find src/test/java -name '*.java' -print > target/tests.txt
"$javac_bin" --release 25 -encoding UTF-8 -cp "target/classes:$dep" -d target/test-classes @target/tests.txt
"$pose_java" --enable-native-access=ALL-UNNAMED -ea -cp "target/classes:target/test-classes:$dep" sgm.SelfTest
