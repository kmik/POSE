#!/usr/bin/env bash
# Maintainer helper: download a private JDK; no package manager, sudo, or global settings.
set -euo pipefail
destination="${1:?Usage: fetch-linux-jdk.sh NEW_DIRECTORY}"
[[ ! -e "$destination" ]] || { echo "Refusing to overwrite $destination" >&2; exit 1; }
[[ "$(uname -s)" == Linux && "$(uname -m)" == x86_64 ]] || {
  echo "This release helper supports Linux x86-64 only." >&2; exit 1;
}
mkdir -p "$(dirname -- "$destination")"
download="$(mktemp -d "$(dirname -- "$destination")/jdk-download.XXXXXX")"
curl --fail --location --retry 3 \
  'https://api.adoptium.net/v3/assets/latest/25/hotspot?architecture=x64&image_type=jdk&os=linux&vendor=eclipse' \
  -o "$download/metadata.json"
mapfile -t package < <(python3 - "$download/metadata.json" <<'PY'
import json, re, sys
with open(sys.argv[1], encoding='utf-8') as f:
    p = json.load(f)[0]['binary']['package']
if not p['link'].startswith('https://github.com/adoptium/temurin25-binaries/releases/download/'):
    raise SystemExit('Unexpected JDK download origin')
if not re.fullmatch('[0-9a-fA-F]{64}', p['checksum']):
    raise SystemExit('Invalid JDK checksum metadata')
print(p['link'])
print(p['checksum'].lower())
PY
)
[[ ${#package[@]} == 2 ]] || { echo "Invalid Adoptium metadata" >&2; exit 1; }
curl --fail --location --retry 3 "${package[0]}" -o "$download/jdk.tar.gz"
printf '%s  %s\n' "${package[1]}" "$download/jdk.tar.gz" | sha256sum --check -
mkdir "$destination"
tar -xzf "$download/jdk.tar.gz" -C "$destination" --strip-components=1
cp "$download/metadata.json" "$destination/pose-download-metadata.json"
"$destination/bin/java" -version
"$destination/bin/javac" -version
