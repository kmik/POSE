# Portable packaging validation (2026-11-05)

Scope: build/launch scripts, standalone release tools and installation documentation.
No processing source, OpenCL kernel, existing test, application default, or output-format
implementation was edited. All **201 files under src/** matched the SHA-256 baseline
recorded before the changes. Local evidence is under `target/portable-packaging/`.

## Passed locally on Windows x86-64

- The standalone Java release tool compiles/runs with Java 25.
- Bash syntax checks cover both launchers, the Java selector, and Linux release/download
  helpers. Git Bash was used; this is syntax/launcher coverage, not Linux binary validation.
- Launcher fixtures demonstrate that bundled Java beats an existing Java 11 JAVA_HOME;
  explicit POSE_JAVA_HOME overrides it; relocated paths and image/output arguments containing
  spaces are preserved; heap settings and the child exit code propagate; Java 11 and a
  runtime without javac produce build errors.
- A real Windows bundle was assembled with private Temurin 25 and only the 13 pinned JARs.
  Its 787 file checksums verified. From a package path containing spaces, help and a native
  OpenCV-loading/CPU memory-planning smoke test passed with JAVA_HOME deliberately invalid.
  This is a validation fixture, not a published release.
- 13 packaging integrity checks passed: resource replacement preserves the original JAR
  and non-native contents, malformed native files are rejected, existing outputs are not
  overwritten, and verification detects tampering and unlisted files.
- The Linux release builder rejects a non-Linux host before attempting to build a
  compatibility artifact. It cannot label an upstream GLIBC_2.29 library as glibc 2.28.

## Existing processing regression failures (not changed)

`build.bat` compiles the current source successfully and starts its existing SelfTest suite.
The first run hit a Windows native image-write failure at a 260-character diagnostic path.
A rerun using a temporary Q: mapping passed that stage; the mapping was removed afterward.

That rerun then failed in `StorageGpuRegression.storage`, at the comparison reading
`gpuProject.resolve("dense.las")`. The GPU reconstruction itself completed, and its
`latest-files.csv` identifies the canonical cloud under `runs/<id>/07_results/dense.las`.
This test invocation does not request `--compatibility-copies`, which is off by default.
The existing test's root-level filename expectation is therefore stale. No workaround,
test edit, output-default change, or test bypass was added. Later SelfTest checks were not
reached; do not describe the full processing suite as passing.

## Linux release still requires a baseline build

This workspace has Windows Java and Git Bash, but no installed Linux/WSL build environment.
No rebuilt Linux `.so` or validated Linux portable archive has been produced here.

On a Linux x86-64 glibc 2.28 maintainer machine/VM, run:

```bash
bash packaging/build-linux-release.sh VERSION --jobs 4
```

See `PORTABLE_INSTALL.md` for required maintainer tools. The script uses a private JDK,
builds OpenCV once, checks ELF version requirements, runs the existing regression suite
unchanged, performs a relocated-package smoke test and only then creates the end-user
archive. A regression failure prevents creation of the archive. Do not claim numerical
equivalence or glibc 2.28 compatibility until that build/test passes. End users of a completed
release will only extract and run `pose.sh`, without compilers, sudo or containers.
