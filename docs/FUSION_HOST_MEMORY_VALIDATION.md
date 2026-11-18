# Host fusion memory validation — 18 November 2026

The implementation remains opt-in. The 3,013-image server workload has **not** been run
here. Its supplied log is evidence of the configuration and interrupted progress, not a
measurement of anonymous versus mapped/file-cache RAM.

## Baseline and real forest fixture

Before editing, the complete source was copied to
`target/host-memory-20261117/baseline-src` and compiled separately into `baseline-classes`.
No Git metadata was present in this checkout. All validation outputs use new directories;
existing user results were not overwritten.

The real fixture is `../example_data/forest/forest_9`: 24 images, `--quality high`, largest
processed image 2640×1978. Both implementations use eight cores, a 4324 MiB JVM heap,
`--memory-mb 2800`, 4000 MiB GPU allowance, 12,000 features, eight matching neighbours,
MVSGM image fusion, optimal neighbours, graph-average ownership, edge-aware depth
outliers, surface attributes, LAS/PLY and binary confidence output. Alignment is recomputed
in each separate project. The hardware is Windows 11 / JDK 25.0.5 / RTX 5070 Laptop GPU.
Exact argument arrays and process-counter measurements are saved beside every case.

The final bounded and constrained runs each produced **16,582,058 points**, with byte-identical
`dense.las`, `dense.ply`, `point-confidence.bin`, and the saved georeferenced frames/tracks
against the legacy baseline. This compares geometry, point counts, colour, surface
attributes, confidence data, candidate identifiers and order. Evidence is in
`target/host-memory-20261117/release-byte-comparison.json`.

Controlled end-to-end measurements (GiB = 2³⁰ bytes):

| Case | Host allowance / cache, MiB | Strip rows | Seconds | Peak RSS, GiB | Peak private, GiB | API read / write, GiB |
| --- | --- | --- | ---: | ---: | ---: | ---: |
| Legacy snapshot | disabled | untiled | 143.55 | 8.162 | 6.364 | 7.259 / 7.659 |
| Bounded | 4600 / 512 | auto: 1978 | 208.00 | 5.648 | 6.604 | 29.158 / 9.416 |
| Constrained | 4200 / 256 | 32 | 321.77 | 4.232 | 5.094 | 217.563 / 10.328 |

The constrained case additionally uses the existing test control
`-Dsgm.fusion.tileRows=32`; other reconstruction options are identical. The bounded path
reduces sampled peak RSS by about 31%; the constrained case reduces it by about 48%.
Runtime increases to about 1.45× / 2.24× baseline. Peak private bytes increase slightly
in the normal bounded case and decrease about 20% in the constrained case. Private bytes
and RSS are distinct quantities and are not summed.

The final bounded cache peaks at 536,666,952 bytes against a 536,870,912-byte cap, with
355,576 loads and 284,717 evictions. The constrained cache peaks at 268,417,152 bytes
against its 268,435,456-byte cap, with 3,443,995 loads and 3,377,232 evictions. Peak page
counts equal their 8,192 / 4,096 metadata limits. Both finish with zero cached bytes and
zero open cache channels. Very small caches are costly even when correctness holds.

Both final cases also produce 16,582,058 points. Final comparisons of LAS, PLY, binary
confidence records and georeferenced binary frames/tracks are saved in
`target/host-memory-20261117/release-byte-comparison.json`; measurement summaries are in
`release-measurement-summary.json` in that directory. Individual `*-args.json`,
`*-measurement.json`, run telemetry and fusion/window reports preserve the exact evidence.
Both timing trials use the final frozen source/classes, including output reservations,
pending-output barriers, reusable row scratch, partial-page metadata limits and interrupted
I/O recovery. Source/class manifests and installed-class hash verification are retained.

The complete high-quality forest also passes a final-build replay from its saved alignment
checkpoint. Run `20261117T234325677Z_1992b3b6` branches from the legacy alignment, inherits
saved options, and completes dense/finish with the same five binary artifacts. This tests
checkpoint continuation, not comparable full-pipeline timing. The first attempt stopped
safely at the disk-space guard; after deleting duplicate task-created validation payloads,
retrying the same failed run without `--branch` succeeded. See
`release-checkpoint-summary.json`, `checkpoint-retry-args.json`, and the comparison file.
Earlier duplicate task-created payloads were removed to reclaim space; their logs, states,
measurements and hash evidence remain. The legacy baseline and final release outputs are retained.

## Synthetic and regression coverage

`FusionHostMemoryRegression` exercises 32, 256 and 3,013 camera files with a fixed
256 KiB page-payload allowance and four-page metadata limit. The final measured peak
payload is 196,624 bytes and four pages at each size. The 3,013-camera test performs
12,806 page loads and 12,802 evictions and verifies every
first/last value after reopening in reverse order. Separate identical-frustum graphs
preserve all N−1 possible neighbours and an N-camera dependency window at every size.
This tests cache scaling beyond the forest fixture; it is not a production reconstruction.

The dedicated final regression passes **11,270 checks** (`host-tests-latest.log`),
and the cloud-output regression passes **642 checks** (`cloud-output-latest.log`).
Additional controls cover concurrent readers during eviction, dirty-page writeback,
late dependencies, active source-reader retirement refusal, retained files, cancellation
and failure barriers, interrupted cleanup, unsafe-allocation refusal, GUI option discovery
and saved option copies. Tiny partial files verify the metadata cap; mixed camera sizes
verify reusable row buffers. Simulated pressure verifies trimming/reopening and serial
row execution. Interrupted file I/O verifies reopening an interrupt-closed channel for
cleanup while preserving the cancellation flag. A planning-only extremely tall raster verifies refusal before
allocating a speckle worker that exceeds its reserved allowance. Ownership strategies and
precision tiers compare ordered cloud and confidence bytes between legacy arrays and
non-dividing 32-row strips. Voxel output, surface consolidation, and tiled LAS spanning
more than 96 output tiles also compare bytes; output-retention guards are exercised.

The high-only verification scripts compile all engine/test sources and run the SelfTest
body with two existing ultra-fixture suites omitted: `BundleQualityRegression` and
`MultiViewDisparityRegression`. These omissions are explicit; the ordinary build scripts
and original SelfTest entry point retain their existing behavior.
The corrected complete high-only harness passes **148,860 checks**
(`regressions-8g.log`). This aggregate precedes the final output-reservation and
interrupted-I/O refinements, which pass the dedicated checks above.

An initial broad run reached an existing calibration fixture whose native PNG artifact
path exceeded Windows MAX_PATH. The Windows verification harness now uses a short,
unique temporary fixture root; no calibration implementation was changed. The initial
failure log is retained as `regressions-v2.stderr` in the validation directory.
The initial 2 GiB aggregate harness later failed the existing 8 GiB GPU planning control
in `HybridSgmRegression`; its heap-dependent admission requires more headroom. The
control and all subsequent suites pass with an 8 GiB heap: **26,155 checks** in
`regressions-remaining-v2.log`. The reusable verification scripts now use 8 GiB. A
separate initial tail JVM failed to load the extracted OpenCV DLL from its default
temporary location; retrying with an explicit short temporary directory succeeded.

## Interpretation and limitations

Peak RSS and private bytes are sampled independently every 200 ms by the measurement
harness; application telemetry also samples heap/RSS. Windows process I/O transfer counts
include cached reads/writes and are **not physical disk traffic**. Mapped-page-fault I/O
and explicit file I/O can be attributed differently, so the I/O columns characterize API
traffic rather than proving an equivalent physical disk amplification factor. Timings
are single local trials under desktop activity, not statistical production benchmarks.

A 3400 MiB admission trial correctly refused a 1736 MiB stereo workspace when only
1568 MiB remained after retained/native/cache reservations. Intermediate 64/128 MiB cache
trials showed severe churn; those interrupted trials are retained as diagnostics and do
not count as completed correctness runs. Very small caches can be much slower. The new
opt-in cache default is 512 MiB (subject to the allowance cap); wider server windows may
benefit from several GiB within their shared allowance.

Application page payloads and planned workspaces are bounded. Process RSS can exceed the
configured application allowance because committed/garbage heap, native allocations,
allocator retention and OS cache are additional measured phenomena. These results do not
establish a strict total-RAM guarantee. No default-path promotion is justified by these
local tests. See [deployment and exact server checkpoint continuation](FUSION_HOST_MEMORY.md).
