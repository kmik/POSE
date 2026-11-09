# Calibration and reconstruction throughput revision, 9 November 2026

Scope: the working copy and standalone benchmark tooling. The original
`sgm_java` tree is not changed. Pre-change source and independently compiled classes are retained
under `target/performance-20261109/baseline-src` and `baseline-classes`.

## Implemented changes

### Calibration

Track each parameter's last attempted accepted-model version. A rejected trial restores the exact
lens, poses and points, so a non-focal parameter is retried only after an accepted model change.
`k3` still receives its first trial in the second sweep. Focal retains its nominal-only second
trial: the first sweep selects among three focal starts, which is a different proposal.

Cache the two current-model held-out RMS values until an accepted model changes them. Do not build
the data-only observability matrix for candidates already rejected by validation or prior bounds.
Independent conditional held-out point fits run in parallel, followed by the original serial
observation-by-observation residual sum. This preserves floating-point addition order.

The disjoint training/validation groups, full training population, iteration limits, physical
checks, data-only observability and acceptance thresholds are retained. Unsupported parameters are
still held fixed. Skipped unchanged-model trials appear in the calibration JSON report.

### Dense GPU execution and scheduling

`--gpu-transfer-mode queued` (default) queues banded-stereo uploads in the existing in-order queue.
Native descriptor/image storage and reusable band staging stay alive through the blocking result
read and queue completion, including cleanup. `blocking` retains the comparison path. This batches
uploads; it does not claim that transfer and kernel execution overlap on an in-order queue.

Growing packed volumes release their completed predecessor buffers instead of accumulating them
through retries. Growth headroom includes the complete session's GPU allocation estimate.
Profiles are collected after queue completion.

When a conservative host-memory estimate permits it, concurrent stereo submits one additional
look-ahead task to the fixed worker pool. This lets a finished worker start another pair while an
earlier pair is still awaiting ordered publication. GPU concurrency and per-worker budgets remain
unchanged; output publication remains in original pair order. Low-memory and CPU paths retain the
original submission window. Queued futures discarded by executor shutdown are explicitly cancelled.

### Image-fusion data reuse

A bounded LRU cache retains immutable decoded pair candidates, including triangulation weights,
RGB, source pixel IDs and confidence evidence, across both cameras and fusion passes. A reusable
direct read buffer also replaces one large allocation per pair read.

`--image-fusion-cache-mb auto` (default) uses at most a quarter of the processing-memory allowance,
further limited by conservative heap headroom after compact maps and worst-camera workspace.
An explicit number is an upper limit, also bounded by estimated heap headroom. `0` disables caching.
Entries are evicted before replacement allocation, oversized entries are replayed uncached, and
the candidate stream remains the authoritative fallback. Cache hits, admitted bytes and peak
retained bytes are reported separately from actual pair reads and logical bytes read.

This cache is separate from the existing `--fusion-cache-mb` depth-reader page cache. It does not
change splatting order, depth decisions, independent-camera support or emitted point order.

### Pair selection

The existing opt-in `--dense-pair-selection coverage` now penalizes pairs whose sparse samples fall
below the triangulation-angle threshold. Sparse grid cells require up to `minViews - 1` distinct
pair witnesses where those witnesses are available, in addition to the existing per-camera degree
target and original-component connectivity. Unknown pairs are retained. Reports include useful
angle samples and median triangulation angle.

The default remains `all`: sparse coverage is not proof of dense thin-structure completeness.
Promoting pair reduction to a default requires a separate geometric/coverage comparison.

### GPU feature processing

Binary-descriptor reciprocal matching uploads both descriptor sets once and swaps their device
roles for the reverse search. A device query-index list replaces CPU row gathering and descriptor
re-upload for sparse reverse selections. Brute-force Hamming k=2, stable tie order, ratio tests and
original query IDs are preserved. Both directions of first-use searches are sampled against the
CPU matcher before publication; failure leaves outputs untouched for CPU fallback.

This improves the existing ORB/BRISK/binary-AKAZE GPU path. It does **not** substitute binary
features or exact GPU brute force for SIFT/FLANN. SIFT extraction and FLANN remain CPU based.
GPU SIFT extraction and approximate float-descriptor matching need a separate implementation and
quality study; no native dependency or metric is silently changed here.

### Measurement

Step-by-step telemetry now merges GPU profiles and totals instead of erasing them during later
CPU-only stages. Queued transfer enqueue cost/bytes are distinguished from blocking transfer
times. Kernel durations and overlapping task times must not be summed as GPU wall utilization.

The [matched benchmark launcher](../benchmarks/README_performance.md) applies identical process CPU
affinity before either engine starts, alternates execution order, runs fresh processes, and times
startup through shutdown externally. The comparable workload excludes GPS refinement, ground
classification, disparity previews and PDF/HTML reporting on both sides. Native project formats,
algorithms and memory management still differ.

## Verification

`ThroughputRegression` compares cached/uncached cloud and confidence bytes for owner,
owner-consume and graph-average strategies, including cache eviction. It checks serial/parallel
held-out RMS bit identity, bounded pair look-ahead, telemetry continuation and invalid options.
Its optional `gpu` controls require real device execution and compare Hamming neighbors, distances,
reciprocal IDs/decisions, both transfer modes and growing/reused banded volumes with CPU references.

`AdaptiveCalibrationRegression` retains supported two-image radial recovery, rejects unsupported
focal changes, preserves rejected poses/points and checks that unchanged models use fewer trials.
The existing shared-calibration and TSGM controls also run against this revision.

Benchmark-launcher tests cover input filtering, dry-run isolation, inherited Windows CPU affinity,
restoration after exceptions and recording failed native commands without emitting a success summary.

Completed verification:

- Full regression suite: **114,540 checks passed**, with an isolated compiled runtime.
- Supported two-image calibration: **1,813 checks passed**; shared-calibration controls: **287**.
- TSGM CPU/GPU controls: **273 checks passed**.
- Final production throughput CPU/GPU controls: **5,107 checks passed** on the RTX 5070 Laptop,
  including the additional cache-bound cases and binary-matcher cleanup guards.
- Benchmark tooling: **5 tests passed**, including real Windows child-affinity inheritance and
  Metashape's independent affinity reader. Native CLI memory planning and script dry runs passed.
- Normal `target/classes` and `target/test-classes` are rebuilt. `measured-classes` retains the
  compiled candidate used for the native performance measurements.

## Native 17-image before/after control

Both runs use 12 logical CPUs, 16,000 MiB processing allowance, 8,000 MiB GPU allowance, all 17 city
images, half dimensions, SIFT/FLANN, 40,000 requested features, four sequence neighbors, all 58
verified pairs, image graph-average fusion, moderate filtering, minimum two views, no surface
consolidation, local coordinates, no GPS/ground/report, and full normal/surface attributes.
These settings differ from the original archived GPS/ground-classified run; do not compare its
242 seconds directly to these figures.

| Measurement | Archived baseline | Revised |
|---|---:|---:|
| External process-start through exit | 146.821 s | 126.272 s |
| Initial calibration/bootstrap | 25.625 s | 27.261 s |
| Feature extraction | 6.398 s | 6.480 s |
| Matching | 3.298 s | 3.942 s |
| Geometry / alignment / adjustment | 32.770 s | 25.132 s |
| Dense matching | 42.044 s | 35.885 s |
| Fusion, including final cloud export | 21.344 s | 19.865 s |
| Main calibration trial count | 15 | 9 |
| Main calibration trial time | 23.358 s | 15.858 s |
| Fusion pair reads | 232 | 58 |
| Logical candidate bytes read | 2,132,380,140 | 533,095,035 |
| Tracked OpenCL buffer peak | 3,495.0 MiB | 1,836.7 MiB |
| Sampled process resident peak | 8,405.9 MiB | 9,557.2 MiB |
| Final points | 2,854,359 | 2,854,359 |

**LAS, PLY, binary confidence records and final exterior-orientation CSV are byte-identical.**
SHA-256 evidence and all native commands/results are in
[`target/performance-20261109/verification-summary.json`](target/performance-20261109/verification-summary.json).
The revised cache retained about 518 MiB and recorded 174 hits. This trades bounded host memory
for less reading/decoding and weight recomputation. Growing GPU volumes no longer accumulate old
buffers; tracked allocations are separate from physical device VRAM.

The external elapsed reduction is about **14% in this single before/after observation**. It includes
startup and filesystem/cache variation and is not a statistically established general speedup.
The initial calibration and CPU SIFT/FLANN stages did not get faster in this sample. Reduced pair
reads do not imply a proportional fusion speedup; other fusion work still dominates.

## Matched Metashape runs and initialization cost

With extra Java surface attributes disabled to match the Metashape script's exports, revised Java
took **130.587 seconds externally**. Three fresh Metashape processes under the same `0xfff` CPU mask
took **144.937, 134.293 and 131.676 seconds**, emitting approximately 5.53 million points each.
Java emitted approximately 2.85 million. Equal settings intent and CPU allocation do not establish
equal reconstruction quality or work. The live Metashape process affinity was also checked as 4095.

These Metashape totals contain a reproducible initialization delay: its native log reports the
first CUDA free-memory query taking **85.670, 77.293 and 75.071 seconds**, respectively. Matching
after that delay takes about two seconds. All original timings are retained; no initialization
cost is silently removed from the benchmark.

Subtracting just that explicitly reported delay from the internal Metashape totals leaves
**54.89–56.91 seconds**. Those are arithmetic remainders, not independently measured warm runs.
The previous 16-CPU, in-process Metashape benchmark had a 50.36-second median total. Consequently,
similar fresh-process latency here does **not** establish Metashape-level warm processing speed.
Further gains still need substantial calibration/bootstrap and dense-pipeline work; the SIFT/FLANN
GPU path and general GPU image-fusion rewrite are not implemented by this revision.
