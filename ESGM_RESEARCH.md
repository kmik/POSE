# Disk-free SGM investigation

Historical investigation. The production implementation now uses expanded candidate pools with mandatory exact verification of unresolved outcomes; see [SCALING.md](SCALING.md). The candidate-only coverage losses and unchanged-launcher statements below describe the frozen prototypes, not the current production backend.

Status: research prototypes, tested on Windows on 31 October 2026. The photogrammetry command, its defaults and its installed matcher are unchanged. These experiments are not a new `--gpu-sgm` option.

The aggregate spool can be eliminated. The remaining obstacle is accepting enough reliable matches while preserving the meaning of the existing uniqueness test. The adaptive tile experiment also needs a different handling of search margins and ambiguity before it should be used by default.

## What was tested

The standalone tools under `src/test/java/sgm/` are deliberately excluded from `SelfTest` execution and production dispatch:

- `EsgmPrototype`: an OpenCL candidate matcher with continuous whole-image SGM paths. A first sweep finds each path's preferred disparity; a second recomputes the paths and sums costs only at those candidates and their immediate neighbors. It does not allocate a full-image cost/aggregate volume or open an aggregate scratch file.
- `EsgmInvestigation`: an independent CPU recurrence/oracle, GPU comparisons, known-depth controls, independent left/right matching and ordinary-SGM reference comparisons.
- `AdaptiveStereoInvestigation` and `AdaptiveSearchBenchmark`: a frozen research fork of `DenseMatcher`, adding subdivision of uncertain/wide regions, a larger margin for coarse prediction uncertainty, selective widening and a preference for resident GPU tiles. This fork is not maintained as a second production matcher.
- `RectifiedResearchReplay`: reuses production image loading and rectification to reproduce a pair from saved camera orientations. It writes only into a separate research directory.

The candidate design is based on [Hirschmuller, Buder and Ernst, Memory Efficient Semi-Global Matching (2012)](https://isprs-annals.copernicus.org/articles/I-3/371/2012/isprsannals-I-3-371-2012.pdf). It is not an implementation of the paper's optimized three-pass schedule. This prototype recomputes all paths in two complete sweeps: sixteen path computations for eight-direction SGM, rather than the twelve of that schedule. Its candidate records use more space, but the dominant full-image disparity volume is absent.

Current prototype restrictions: grayscale Census, proved-safe unsigned 16-bit aggregate bounds, four/eight paths and at most 4096 hypotheses. It retains the current signed-disparity convention, masks, edge-adapted penalties, path continuation and subpixel formula. Other matching costs, CPU execution, banked candidate buffers and production fallback/report integration remain future work.

## Confidence is the important difference

Ordinary SGM tests the best competing aggregate cost outside the winner's immediate disparity neighbors. Looking only at retained candidates can miss that competitor and make an ambiguous pixel appear unique. The unguarded experiment changed acceptance decisions on both synthetic and real images. Similar overall coverage did not imply equivalent decisions.

The guarded variant adds a sufficient, conservative check:

1. Let `U` contain the retained candidates and their immediate neighbors.
2. During recomputation, each path finds its minimum valid cost outside `U`.
3. Sum those minima to obtain `LB`. For every omitted disparity `d`, `LB <= sum_r L_r(d)`.
4. Accept only if both the known competitors and this lower bound clear the existing uniqueness margin. Reject neighboring minima/ties that cannot certify the selected center and its subpixel result.

This establishes uniqueness against omitted hypotheses without storing their individual aggregate costs. It can reject a match that full SGM would accept, because the minima from different paths may belong to different disparities. It does not improve the physical correctness of a match that ordinary SGM already gets wrong.

On all tested reference cases, every guarded accepted disparity was bit-for-bit identical to the ordinary GPU result, and no extra acceptance was introduced, including after the independent left/right check. The CPU oracle separately checked path minima, candidate-neighbor costs and omitted-disparity bounds with masks, ties, signed ranges, four/eight directions and five-row strip boundaries. These checks passed on the NVIDIA RTX 5070 Laptop GPU and AMD `gfx1103`. This is evidence for the implemented bound and recurrence, not a claim that every SGM output is recovered.

## Large-workload results

Eight paths, 1978 x 2640 pixels, minimum disparity 131, 1037 hypotheses, Census 5 x 5, P1/P2 4/24, 64-row strips; RTX 5070 Laptop GPU, JDK 25, 4 GiB Java heap:

| Case | Left + right matching | Explicit device buffers | Aggregate scratch |
| --- | --- | --- | --- |
| Known planar shift, guarded | 19.99 s | 697.2 MiB | 0 |
| Recreated forest pair, guarded | 18.48 s | 697.2 MiB | 0 |

The existing streamed algorithm's aggregate file alone is 10.09 GiB for this workload. Its full resident plan is about 20.4 GiB. The prototype instead retains compact records per pixel, a strip cost buffer and continuous path carries. Memory still depends on disparity count through strip/path state; it is not independent of all image/disparity dimensions.

The guarded planar control retained 4,280,858 visible samples after left/right validation (99.60% of geometrically visible pixels), with no errors above one pixel and 0.101-pixel RMS disparity error. These are disparity measurements on a synthetic control, not surveyed point-cloud accuracy or a demonstration that real ground thickness is fixed.

The complete guarded planar harness took 28.77 s, including fixture creation, descriptor preparation, selection and reporting. Its measured peak process working set was 1.29 GiB, with sampled private bytes peaking at 2.08 GiB. The forest harness, including PNG reads and three additional reference-strip comparisons, took 34.56 s; peak working set was 0.97 GiB and sampled private bytes 2.23 GiB. Process resources were sampled every 100 ms. Explicit device bytes exclude driver/context overhead; the Java heap limit is not total process RAM. GPU and host allocations are reported separately.

No full-volume ordinary-SGM timing was repeated at this size, so these results do not establish a speed multiplier against the original streamed run. Small images that fit resident SGM remained faster with ordinary SGM.

## Real data and coverage

The saved forest pair was recreated from the original images, final camera CSV and saved calibration. The reproduced R1/R2/P1/P2/Q matrices matched the archived pair exactly; repeat exports produced identical grayscale/mask hashes. Inputs and existing project results were not changed.

The full ordinary aggregate does not fit this GPU. For direct comparisons, three 1978 x 128 strips from the same rasters were processed independently by both methods at the full 1037-disparity range:

| Strip start row | Ordinary LR-valid pixels | Guarded LR-valid pixels | Extra guarded acceptances | Different retained disparities |
| --- | --- | --- | --- | --- |
| 0 | 36,486 | 4,576 | 0 | 0 |
| 1256 | 34,768 | 6,748 | 0 | 0 |
| 2512 | 40,129 | 12,658 | 0 | 0 |

These are comparisons within each strip's shared path context, not proof of full-image equality. There is no ground-truth depth for this pair. The guarded full-image output retained only 134,975 LR-valid pixels, about 2.58% of all raster pixels. That severe coverage loss is a reason not to replace the production fallback yet.

On the separate 450 x 375 rectified photographic pair, ordinary SGM retained 133,902 pixels after LR checking; the guarded experiment retained 129,030. Every retained result agreed exactly. The candidate-only version retained a similar total, 133,811, while changing which observations were accepted.

## Adaptive search results

The adaptive fork preserves all observed coarse disparity extrema, adds a scale-dependent uncertainty margin, splits broad/weak regions down to a 64-pixel core side, and tries local widening before full-range fallback. These measures did not make it a safe default.

512 x 256 controls, 128 disparities, three pyramid levels, 128-pixel initial cores, median disabled and speckle filtering disabled:

| Control | Matcher | Time | Visible valid samples | Errors above 1 pixel |
| --- | --- | --- | --- | --- |
| Plane | Ordinary | 2.37 s | 121,644 | 0 |
| Plane | Existing pyramid | 13.43 s | 125,653 | 0 |
| Plane | Adaptive fork | 22.36 s | 125,645 | 0 |
| Fences and depth layers | Ordinary | 0.93 s | 104,466 | 321 |
| Fences and depth layers | Existing pyramid | 11.90 s | 108,296 | 482 |
| Fences and depth layers | Adaptive fork | 25.89 s | 108,322 | 613 |
| Repetitive texture | Ordinary | 0.92 s | 69 | 69 |
| Repetitive texture | Existing pyramid | 9.75 s | 9,871 | 320 |
| Repetitive texture | Adaptive fork | 38.05 s | 11,069 | 1,937 |

The fence fixture includes three-pixel foreground strips. None of these configurations reliably reconstructed all of them. Coverage alone would have hidden increased wrong-depth acceptance. Repetitive texture is deliberately ambiguous; ordinary SGM also makes mistakes, so preserving its decisions is not a claim of truth.

Two structural issues explain why simply subdividing more is insufficient:

- Independent tiles change the path context and which competing depths participate. A narrow range can hide a valid competitor; a coarse-scale prediction can miss a thin object completely. Boundary pressure cannot detect every omitted structure or confident wrong match.
- The current tile function adds a horizontal margin proportional to the disparity count on both sides of a shared raster. Splitting makes this repeated halo work worse. The adaptive plane experiment evaluated about 49.3 million pixel/hypothesis combinations including halos, retries and the coarse solve, versus 16.8 million for one full-resolution ordinary solve; the repetitive-texture experiment evaluated about 143.4 million. Each stereo solve still has both matching directions and all SGM paths. Per-invocation OpenCL setup and CPU filters also contribute; their separate runtime shares were not measured here.

## Recommended next implementation

1. Keep resident ordinary SGM when it fits. Develop the compact candidate backend as an explicit experimental option before changing automatic fallback selection.
2. Reduce the guard's coverage loss with a larger/adaptive candidate pool and selective exact verification of unresolved competitors. Benchmark memory and completeness together; do not remove the omitted-disparity check to recover density.
3. Optimize the recomputation schedule only after these confidence decisions are stable. Add the production CPU fallback, all supported costs/storage widths, memory planning, cancellation/cleanup and progress reporting.
4. Separate reference-image processing cores from secondary-image search windows before adding more adaptive subdivisions. Preserve coordinate offsets and independent right-to-left checks; deleting the existing margin alone would be incorrect.
5. Use reliable geometry and coarse depths as proposals. Add broad-search checks that can discover omitted fine structures and ambiguity; widen selectively rather than silently clipping the search. Compare against continuous-path controls and include tile seams explicitly.
6. Replay full stereo post-filtering, triangulation, depth fusion and consolidation before accepting a production replacement. Compare output membership, coordinate changes, plane thickness and fence completeness. This investigation stops at raw selection/LR checks for eSGM and complete per-tile stereo filtering for the adaptive experiment; it does not validate final LAS/PLY clouds.

## Reproduce

Use a 64-bit JDK 25, the normal downloaded `lib/` dependencies and an OpenCL GPU runtime. Start from the module/repository root. The runners compile into a separate `target/esgm-research-tool/classes` directory and do not install or overwrite normal launcher classes.

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File research/esgm/run.ps1 -Mode small
powershell -NoProfile -ExecutionPolicy Bypass -File research/esgm/run.ps1 -Mode large
powershell -NoProfile -ExecutionPolicy Bypass -File research/esgm/run.ps1 -Mode adaptive
# Select the second GPU for the small numerical controls:
powershell -NoProfile -ExecutionPolicy Bypass -File research/esgm/run.ps1 -Mode small -Gpu 1
```

```bash
bash research/esgm/run.sh small
bash research/esgm/run.sh large
bash research/esgm/run.sh adaptive
ESGM_RESEARCH_GPU=1 bash research/esgm/run.sh small
```

PowerShell also supports `-Output DIRECTORY`; Bash uses `ESGM_RESEARCH_OUTPUT`. Both honor `ESGM_RESEARCH_HEAP` (default `4g`). Outputs default to a new timestamped research directory. The Windows runner was executed; the Bash runner passed syntax checking, but Linux runtime was not tested.

For an existing run, `replay` takes `RUN_DIRECTORY IMAGE_DIRECTORY REFERENCE_FILENAME SECONDARY_FILENAME`; it currently reproduces the tested `high` quality setting. Then `pair` takes `EXPORTED_RASTER_DIRECTORY MIN_DISPARITY DISPARITY_COUNT`, runs the complete pair and compares three full-width 128-row strips. This helper assumes a height of at least 128. `small` can optionally take two already-rectified grayscale image paths. In PowerShell pass these through `-ResearchArgs @('value1','value2',...)` from a PowerShell session; in Bash append positional arguments after the mode. Supply all four grayscale/mask PNGs when using `pair`.

Machine-readable measurements are in [research/esgm/results-2026-10-31.json](research/esgm/results-2026-10-31.json). Local console/resource logs and recreated image rasters are under `target/esgm-research/` and are excluded from Git. The private forest photographs and generated project clouds are not included.
