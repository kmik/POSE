#!/usr/bin/env bash
# Run ONCE per release on a glibc 2.28 Linux x86-64 maintainer machine/VM.
# End users need none of these build tools. No root or container is required.
set -euo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
usage() {
  echo 'Usage: bash packaging/build-linux-release.sh VERSION [--jdk JDK25_DIR] [--jobs N]'
}
[[ ${1:-} != --help ]] || { usage; exit 0; }
version="${1:-}"
[[ "$version" =~ ^[A-Za-z0-9][A-Za-z0-9._-]*$ ]] || { usage >&2; exit 1; }
shift
jdk=""; jobs="${POSE_BUILD_JOBS:-4}"
while (($#)); do
  case "$1" in
    --jdk) [[ $# -ge 2 ]] || { usage >&2; exit 1; }; jdk="$2"; shift 2 ;;
    --jobs) [[ $# -ge 2 ]] || { usage >&2; exit 1; }; jobs="$2"; shift 2 ;;
    *) usage >&2; exit 1 ;;
  esac
done
[[ "$jobs" =~ ^[1-9][0-9]*$ ]] || { echo '--jobs must be positive' >&2; exit 1; }
[[ "$(uname -s)" == Linux && "$(uname -m)" == x86_64 ]] || {
  echo 'Release builder requires Linux x86-64.' >&2; exit 1;
}
for tool in getconf cmake g++ make python3 readelf curl tar sha256sum; do
  command -v "$tool" >/dev/null || { echo "Missing maintainer build tool: $tool" >&2; exit 1; }
done
glibc="$(getconf GNU_LIBC_VERSION)"
[[ "$glibc" == 'glibc 2.28' ]] || {
  echo "Build this compatibility release on glibc 2.28, found $glibc." >&2
  echo 'Use a baseline Linux machine/VM; a build on newer glibc is not a compatibility fix.' >&2
  exit 1
}
output="$root/dist/pose-$version-linux-x64-glibc228"
[[ ! -e "$output" && ! -e "$output.tar.gz" && ! -e "$output.tar.gz.sha256" ]] || {
  echo "Release already exists: $output (choose a new version)." >&2; exit 1;
}
mkdir -p "$root/target" "$root/dist"
work="$(mktemp -d "$root/target/portable-release.XXXXXX")"
echo "Maintainer build directory: $work"
if [[ -z "$jdk" ]]; then
  if [[ -x "$root/runtime/bin/javac" ]]; then jdk="$root/runtime"
  else
    jdk="$work/jdk"
    bash "$root/packaging/fetch-linux-jdk.sh" "$jdk"
  fi
fi
jdk="$(cd -- "$jdk" && pwd)"
source "$root/scripts/java-env.sh"
POSE_JAVA_HOME="$jdk" pose_select_java "$root" yes
curl --fail --location --retry 3 \
  'https://github.com/opencv/opencv/archive/refs/tags/4.9.0.tar.gz' \
  -o "$work/opencv-4.9.0.tar.gz"
sha256sum "$work/opencv-4.9.0.tar.gz" > "$work/opencv-source.sha256"
tar -xzf "$work/opencv-4.9.0.tar.gz" -C "$work"
opencv="$work/opencv-4.9.0"
# The computation modules remain OpenCV 4.9.0. GUI/video/camera capture are not used
# by POSE; disable them to avoid dependencies on desktop packages on headless hosts.
JAVA_HOME="$jdk" PATH="$jdk/bin:$PATH" cmake -S "$opencv" -B "$work/opencv-build" \
  -DCMAKE_BUILD_TYPE=Release \
  -DBUILD_LIST=core,imgproc,imgcodecs,features2d,calib3d,flann,java \
  -DBUILD_JAVA=ON -DBUILD_FAT_JAVA_LIB=ON -DBUILD_SHARED_LIBS=OFF \
  -DCMAKE_POSITION_INDEPENDENT_CODE=ON \
  -DCMAKE_SHARED_LINKER_FLAGS='-static-libstdc++ -static-libgcc' \
  -DCMAKE_MODULE_LINKER_FLAGS='-static-libstdc++ -static-libgcc' \
  -DBUILD_TESTS=OFF -DBUILD_PERF_TESTS=OFF -DBUILD_EXAMPLES=OFF \
  -DBUILD_opencv_apps=OFF -DBUILD_opencv_python2=OFF -DBUILD_opencv_python3=OFF \
  -DWITH_FFMPEG=OFF -DWITH_GSTREAMER=OFF -DWITH_GTK=OFF -DWITH_QT=OFF \
  -DWITH_OPENCL=OFF -DWITH_IPP=OFF -DWITH_ITT=OFF \
  -DWITH_LAPACK=OFF \
  -DWITH_OPENEXR=OFF -DWITH_WEBP=OFF -DWITH_JASPER=OFF \
  -DBUILD_ZLIB=ON -DBUILD_JPEG=ON -DBUILD_PNG=ON -DBUILD_TIFF=ON \
  -DCPU_BASELINE=SSE2 -DCPU_DISPATCH=
JAVA_HOME="$jdk" PATH="$jdk/bin:$PATH" cmake --build "$work/opencv-build" \
  --target opencv_java --parallel "$jobs"
native="$work/opencv-build/lib/libopencv_java490.so"
[[ -f "$native" ]] || { echo 'OpenCV Java target was not produced; inspect the CMake JNI/Python configuration.' >&2; exit 1; }
"$jdk/bin/java" "$root/packaging/PortableRelease.java" check-linux "$jdk" "$native"

# A separate build tree avoids changing the developer's dependency JAR or classes.
stage="$work/source"
mkdir -p "$stage/lib" "$stage/scripts" "$stage/packaging"
cp -R "$root/src" "$stage/src"
cp -R "$root/docs" "$stage/docs"
cp "$root/"*.md "$stage/"
cp "$root/build.sh" "$root/pose.sh" "$root/photogrammetry.sh" \
  "$root/pose.bat" "$root/photogrammetry.bat" "$stage/"
cp "$root/scripts/java-env.sh" "$stage/scripts/"
cp "$root/packaging/PortableRelease.java" "$stage/packaging/"
upstream="$root/lib/opencv-4.9.0-0.jar"
if [[ ! -f "$upstream" ]]; then
  upstream="$work/opencv-4.9.0-0.jar"
  curl --fail --location --retry 3 \
    'https://repo.maven.apache.org/maven2/org/openpnp/opencv/4.9.0-0/opencv-4.9.0-0.jar' -o "$upstream"
fi
"$jdk/bin/java" "$root/packaging/PortableRelease.java" patch-opencv \
  "$upstream" "$native" "$stage/lib/opencv-4.9.0-0.jar"
# Existing regression suite runs unchanged, using the rebuilt native library.
POSE_JAVA_HOME="$jdk" bash "$stage/build.sh" 2>&1 | tee "$work/build-and-tests.log"
"$jdk/bin/java" "$root/packaging/PortableRelease.java" pack \
  --root "$stage" --runtime "$jdk" --platform linux-x64 --version "$version" \
  --native "$native" --opencv-source "$opencv" --output "$output"
mkdir -p "$output/release-validation"
cp "$work/build-and-tests.log" "$work/opencv-source.sha256" "$output/release-validation/"
cp "$work/opencv-build/CMakeCache.txt" "$output/release-validation/"
# Exercise native loading and planning after moving the bundle, without user Java settings.
probe="$work/relocated bundle"
cp -a "$output" "$probe"
chmod +x "$probe/pose.sh" "$probe/photogrammetry.sh"
mkdir -p "$work/smoke-images"
python3 - "$work/smoke-images" <<'PY'
import binascii, os, struct, sys, zlib
def chunk(kind, data):
    return struct.pack('>I', len(data)) + kind + data + struct.pack('>I', binascii.crc32(kind + data) & 0xffffffff)
pixels = b''.join(b'\0' + bytes((x*7+y*11) % 256 for x in range(96)) for y in range(64))
png = (b'\x89PNG\r\n\x1a\n' + chunk(b'IHDR', struct.pack('>IIBBBBB', 96, 64, 8, 0, 0, 0, 0))
       + chunk(b'IDAT', zlib.compress(pixels)) + chunk(b'IEND', b''))
for name in ('left.png', 'right.png'):
    with open(os.path.join(sys.argv[1], name), 'wb') as f:
        f.write(png)
PY
env -u JAVA_HOME -u POSE_JAVA_HOME bash "$probe/pose.sh" \
  -i "$work/smoke-images" -o "$work/smoke-result" --compute cpu \
  --estimate-memory --max-dimension 64 --memory-mb 128 \
  > "$output/release-validation/relocated-smoke.log" 2>&1
# Extra evidence files are added above; regenerate the final manifest in place.
"$jdk/bin/java" "$root/packaging/PortableRelease.java" seal "$output"
chmod +x "$output/pose.sh" "$output/photogrammetry.sh"
tar -czf "$output.tar.gz" -C "$root/dist" "$(basename -- "$output")"
(cd "$root/dist" && sha256sum "$(basename -- "$output.tar.gz")" > "$(basename -- "$output.tar.gz.sha256")")
echo "READY: $output.tar.gz"
