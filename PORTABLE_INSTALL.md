# Portable POSE releases

Portable packages contain compiled POSE, its exact dependency JAR versions, and a private
Java 25 distribution. They do not install system packages, change shell profiles, update
system Java/glibc, or require sudo or a container. Internet access is unnecessary after
the complete archive has been downloaded and extracted.

## End users: Linux x86-64, glibc 2.28 or later

Download the **release archive**, not GitHub's automatically generated source archive.
The release maintainer must publish it first; these scripts do not upload anything.

```bash
sha256sum -c pose-VERSION-linux-x64-glibc228.tar.gz.sha256
tar -xzf pose-VERSION-linux-x64-glibc228.tar.gz
cd pose-VERSION-linux-x64-glibc228
./pose.sh -h
./pose.sh -i /absolute/path/to/photos -o /absolute/path/to/project --compute cpu
```

Replace VERSION with the published version. Neither compilation nor a system JDK is
required. The package directory can contain spaces and can be moved. Input/output paths
are interpreted relative to the directory from which POSE is invoked, not its installation.

The bundled `runtime/` wins over a user's existing `JAVA_HOME` or PATH Java. An explicit
`POSE_JAVA_HOME=/path/to/jdk25` overrides it for diagnosis or development. This setting is
optional and applies only to the command where it is supplied. A source checkout without
`runtime/` still uses `JAVA_HOME`, then PATH. Java 11 cannot run POSE.

GPU processing still needs a working host GPU driver and vendor OpenCL runtime; those are
machine-specific and are not installed by POSE. Test with `./pose.sh --list-gpus`.
`--compute cpu` needs no OpenCL device. A portable glibc package is not an Alpine/musl or
ARM package, and an old kernel may impose additional JDK limitations.

Windows packages use `pose.bat` with the same private-runtime precedence. They are separate
from Linux packages; Windows Java/OpenCV binaries cannot run on Linux.

## Maintainers: produce the Linux compatibility release once

Use a **Linux x86-64 machine or VM with glibc 2.28**. This can be a build account on an
existing machine. No container or sudo is used by these scripts. Installed build tools
are needed on the maintainer machine only: Bash, CMake 3.13 or later, a C++11-capable g++ with its static
C++/GCC libraries, Make, Python 3, binutils/readelf, curl, tar, and sha256sum. On a cluster,
these may be supplied by environment modules. Keep all compiler and library paths targeted
at the baseline system: merely running on an old OS with a newer sysroot is insufficient.

```bash
# From the latest POSE source directory:
bash packaging/build-linux-release.sh 0.1.0 --jobs 4

# Or reuse an already downloaded Linux JDK 25:
bash packaging/build-linux-release.sh 0.1.0 --jdk "$PWD/runtime" --jobs 4
```

The script:

1. Requires the glibc 2.28 build host, or stops before compiling anything.
2. Uses the supplied JDK or downloads Temurin 25 to a private build directory. The downloaded
   archive is checked against the SHA-256 supplied by the official Adoptium API. Exact
   download metadata is retained with a downloaded runtime.
3. Builds OpenCV **4.9.0** with the modules POSE uses, statically including the codec and C++
   dependencies. GUI/video capture and OpenCV's own OpenCL/IPP acceleration are disabled.
   POSE's OpenCL kernels and GPU backend are untouched. These native build differences can
   produce numerical/performance differences, so regression testing is required.
4. Inspects the native library and every ELF binary in the runtime with `readelf`. A reference
   to GLIBC newer than 2.28, or an external OpenCV codec/C++ dependency, fails the release.
   This gate also catches an unsuitable JDK archive or newer build-tool sysroot.
5. Copies the processing source unchanged into a separate build directory; replaces only
   the Linux native resource inside a copy of the OpenPnP JAR; verifies every other JAR
   entry remains byte-identical; compiles and runs the existing regression suite unchanged.
6. Packages only pinned dependency versions (not stale extra JARs), compiled classes,
   resources, launchers, documentation, the runtime's legal notices, and OpenCV notices.
7. Exercises native loading and memory planning from a relocated package path containing
   spaces, with JAVA_HOME unset, then writes checksums and a `.tar.gz` release archive.

Output: `dist/pose-VERSION-linux-x64-glibc228.tar.gz` and its `.sha256` file. Build directories
under `target/portable-release.*` retain logs for inspection. The source tree's `lib/` and
`target/classes` are not rewritten by the Linux release builder. Existing release directories
are never overwritten: use a different version or manage the old artifacts explicitly.

No numerical equivalence or Linux compatibility should be claimed until the baseline build
and regression suite pass. Test the finished archive on newer supported distributions too.
GPU release validation also needs actual supported devices; the script cannot prove it on a
CPU-only builder. ELF-symbol checks are necessary compatibility checks, not a substitute
for running the application.

The baseline builder deliberately does not offer a switch to skip the regression suite.
If a test fails, no release archive is produced. A successful Windows build does not validate
the rebuilt Linux native library.

## Maintainers: Windows package

Build and run the regression suite with `build.ps1`, then use a Windows x86-64 Java 25 JDK:

```powershell
$jdk = 'C:\path\to\jdk-25'
& "$jdk\bin\java.exe" packaging/PortableRelease.java pack `
  --root . --runtime $jdk --platform windows-x64 --version 0.1.0 `
  --output dist/pose-0.1.0-windows-x64
```

Zip that directory using an archive tool. The runtime is copied in full so its legal notices
are retained. The packaging tool refuses to overwrite an existing output directory.

## Integrity and scope

Each package includes `SHA256SUMS`, `processing-source.sha256`, and
`portable-release.properties`. With a JDK available, verify a package with:

```bash
java packaging/PortableRelease.java verify /path/to/extracted/package
```

The tool is part of the source distribution, not the processing application. Source hashes
record the Java, tests, resources and OpenCL files used; packaging checks that they did not
change while it copied the release. SHA-256 detects accidental changes, not an authenticated
publisher signature. Preserve upstream licenses; the runtime and rebuilt OpenCV remain
third-party components. The project's own license policy is unchanged.

## Packaging regression checks

The packaging-specific checks live outside `src/`:

```bash
bash packaging/test-launchers.sh
javac -d target/portable-tool-tests packaging/PortableRelease.java packaging/PortableReleaseRegression.java
java -cp target/portable-tool-tests PortableReleaseRegression /path/to/a/disposable/test/bundle
```

The last test briefly tampers with a metadata file in the supplied test bundle and restores
it, checking that verification detects the change. Use a disposable validation copy, not an
installed production package. It also checks native resource replacement, preservation of
the original JAR, unchanged Java-binding bytes, checksum coverage, and overwrite refusal.
