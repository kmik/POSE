#!/usr/bin/env bash
# Sourced by POSE's launch/build scripts. Never export or alter the caller's PATH.
pose_select_java() {
  local root="$1" need_compiler="${2:-no}" selected="" version major
  if [[ -n "${POSE_JAVA_HOME:-}" ]]; then
    selected="$POSE_JAVA_HOME"
  elif [[ -d "$root/runtime" ]]; then
    selected="$root/runtime"
  elif [[ -n "${JAVA_HOME:-}" ]]; then
    selected="$JAVA_HOME"
  fi
  if [[ -n "$selected" ]]; then
    pose_java="$selected/bin/java"
    pose_javac="$selected/bin/javac"
  else
    pose_java="$(command -v java || true)"
    pose_javac="$(command -v javac || true)"
  fi
  if [[ -z "$pose_java" || ! -x "$pose_java" ]]; then
    echo "POSE: Java not found. Extract a portable distribution with runtime/, or set POSE_JAVA_HOME to a JDK 25+ directory." >&2
    return 1
  fi
  if ! version="$("$pose_java" -version 2>&1)"; then
    printf 'POSE: selected Java cannot start: %s\n%s\n' "$pose_java" "$version" >&2
    return 1
  fi
  if [[ "$version" =~ version[[:space:]]+\"([0-9]+) ]]; then
    major="${BASH_REMATCH[1]}"
  else
    printf 'POSE: cannot determine Java version: %s\n' "$version" >&2
    return 1
  fi
  if (( major < 25 )); then
    echo "POSE: Java $major is too old; Java 25+ is required. Use runtime/ or POSE_JAVA_HOME; system Java need not be changed." >&2
    return 1
  fi
  if [[ "$need_compiler" == yes ]]; then
    if [[ -z "$pose_javac" || ! -x "$pose_javac" ]]; then
      echo "POSE: building requires a JDK (including javac). Set POSE_JAVA_HOME to a JDK 25+ directory." >&2
      return 1
    fi
    version="$("$pose_javac" -version 2>&1)" || return 1
    if [[ ! "$version" =~ javac[[:space:]]+([0-9]+) ]] || (( BASH_REMATCH[1] < 25 )); then
      echo "POSE: selected compiler must be javac 25+: $version" >&2
      return 1
    fi
  fi
}
