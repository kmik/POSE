# Bounded pair matching memory — 18 November 2026

The 3,013-image server run retained 24.7 million `Mat`, 22.0 million `MatOfDMatch`,
and 46.7 million `Finalizer` objects during matching. The native allocator also
reported substantial free space inside its arenas. A Java heap ceiling does not
bound these native headers or force freed allocator pages back to Linux.

## Implementation

- `KnnMatches` stores the two neighbours in flat Java arrays using the same float
  representation as OpenCV's `MatOfDMatch`. Pair verification, focal initialization,
  search memoization and GPU reciprocal matching consume this representation.
  Debug exports create ordinary Java `DMatch` objects only when needed.
- `OpenCvKnn` invokes the same generated JNI search used by the bundled OpenCV 4.9
  Java wrapper. It copies each returned native row and immediately deletes that
  row's header, including cleanup on failure. No Java `Mat` ever owns these raw
  row addresses, so they cannot also be deleted by a finalizer. Java-owned native
  objects are never manually deleted. The pointer table is zeroed before native
  conversion so partial allocation failure leaves safe cleanup entries. Method handles and result layout checks
  isolate the dependency on this pinned JNI binding; no new native library or
  compiler is required. Updating OpenCV requires rerunning these checks.
- Each matching worker reuses its descriptor matcher, reverse-result buffer,
  descriptor-gather buffer and verification matrices. Worker termination releases
  the workspaces. Forward results remain independent until the ordered consumer
  has exported them; the existing bounded look-ahead limits their number.
- On Linux, completed pairs trigger a deadline check for `malloc_trim(0)`, at most
  once per 15 seconds. A slow trim delays the next attempt by at least 100 times
  its duration. There is no forced collection or per-pair native trim.
- The serial execution path also participates in memory admission. Heap sizing,
  feature counts, matcher choice, ratio tests and seeded matching semantics remain
  as configured. The RAM planning ceiling is still not an OS-enforced RSS limit.

## Local comparison

Windows, JDK 25.0.5, OpenCV 4.9.0, one OpenCV compute thread. Three fresh JVMs per
version, alternating execution order, `-Xms256m -Xmx2g`. Each JVM matched 12 pairs
of 40,000 float descriptors of length 128, with half exact correspondences. Both
versions also digested every neighbour and the reverse decisions. The original
compiled classes were archived before rebuilding as
`target/matching-memory-fix/baseline.jar`.

| Measurement | Original median | Revised median |
| --- | ---: | ---: |
| Wall time | 43.022 s | 41.358 s |
| Peak sampled RSS | 506.246 MiB | 167.367 MiB |
| Matching-thread Java allocation | 322.557 MiB | 89.141 MiB |

This is 3.9% less elapsed time, 66.9% less peak sampled RSS, and 72.4% less Java
allocation in this workload. All six result digests were `2e8867e46cf96039`.
Individual wall times varied, so this establishes no material local regression,
not a guaranteed server speedup or a bound on a full project's memory.

Raw results: `target/matching-memory-fix/benchmark-*-*.json`.

Final pre-commit review additionally zero-initialized the native pointer table for
partial-allocation failure cleanup. All engine/test sources then compiled in the
Git checkout, with 56,171 focused checks passing. A final paired 40,000-descriptor
trial took 35.118 s (original) versus 33.522 s (revised), with the same result
digest; sampled peak RSS was 432.875 versus 167.660 MiB. These single paired timings
are saved as `target/matching-memory-fix/precommit-*.json`.

## Correctness

- `MatchingBuffersRegression`: 4,024 checks for exact query/train/image indices,
  distance bits, RNG state, empty and one-neighbour results, buffer reuse, native
  ownership and recovery after a native error. Included in `SelfTest`.
- Existing `ReciprocalMatchingRegression`: 51,981 checks passed, including native
  RNG state and reciprocal decisions for float and binary descriptors.
- Existing `ThroughputRegression gpu`: 6,077 checks passed on the NVIDIA GeForce
  RTX 5070 Laptop GPU, including exact GPU Hamming results versus CPU results.
- Before/after serial and parallel match/align fixtures: all 42 compared match
  CSV/gzip files and saved binary state files were byte-identical. Results are in
  `target/matching-memory-fix/artifact-comparison.json`.
- Integration checks passed: pre-dense/reuse/order (134), governor/checkpoint (32),
  workflow (64), and calibration/GPS (59), including complete reconstruction runs.
  Results are recorded in `target/matching-memory-fix/integration.log`.

The benchmark and functional runs were local Windows runs. Linux's existing
`malloc_trim` binding and the new trim schedule could not be exercised locally
because WSL is not installed; no additional server diagnostics were requested.

## Applying on the server

Copy the updated Java sources, including `KnnMatches.java` and `OpenCvKnn.java`,
and rebuild with the usual `build.sh` workflow and the bundled OpenCV dependency.
An already-running JVM cannot pick up the change. The matching checkpoint format
and settings signature are unchanged; continuation uses its existing descriptor
digest checks. Keep the project directory and checkpoint when restarting a run.

The existing 120.5 million `KeyPoint`/`Point` objects are a separate retained-data
cost; this change addresses temporary matching wrappers, not that fixed cost.
