# Reconstruction, memory and exchange upgrade

The subsequent [surface-fusion update](SURFACE_FUSION.md) enables reprojection refinement and normal-aware consolidation by default. It supersedes the earlier weighted-refinement/output-order behavior described here.

This implements the nine-item TODO authorized after the initial workflow/GPU work. All changes stay in `sgm_java`. Source photographs and existing completed runs are preserved. `--help` lists every CLI argument, alias, default and example; [ORIENTATIONS.md](ORIENTATIONS.md) documents camera interchange.

## RAM and GPU SGM

An explicit `--memory-mb N` raises the command-line Java heap when necessary to `ceil(N/0.65)+16` MiB. This works through either launcher or direct Java entry point, preserving arguments/classpath and any larger existing heap. The parent waits for the child and returns its status. Embedded `Main.run` does not restart its host JVM; its budget remains capped at 65% of heap. Native OpenCV buffers, depth evidence, object overhead and VRAM are additional allocations. Effective allowances are printed in each run log.

`--gpu-memory-mb 0` still defaults to 70% of total device VRAM. An explicit budget can use up to 95%, leaving 5% for the driver/display. This is an estimate, not a reservation of currently free VRAM.

`--gpu-sgm auto` first uses **banked resident volumes**, with up to eight allocations per cost/aggregate volume. Each bank respects the device's individual allocation maximum; `--gpu-bank-mb` defaults to 1024. This removes the previous single-buffer ceiling. An explicit `resident` selection requires full-volume GPU storage.

When a resident volume will not fit, auto can use **exact streamed SGM**. `--gpu-sgm streamed` forces it. Cost data are recomputed per strip; aggregate costs are paged to disk. Full disparity state and global minimum cross strip boundaries in every vertical/diagonal direction; horizontal paths cover full rows. `--gpu-strip-rows` defaults to 64 and is reduced to fit memory. CPU/GPU selection, border rejection, subpixel estimates and confidence checks remain consistent.

Streaming bounds GPU storage but needs approximately **4 × width × height × disparity-count bytes of temporary aggregate disk** for one directional match, plus page/strip working buffers. The file is removed when the match completes, including normal exception cleanup. Resident SGM needs no volume disk; CPU `--tiled` still uses roughly 6 bytes per hypothesis for disk cost/aggregate storage. Disk capacity is checked before allocation. These are computation working files, separate from standard/debug diagnostic storage.

`--pyramid-levels auto` selects up to six coarse-to-fine levels, aiming for about 96 coarse hypotheses. Local intervals include all validated coarse depths plus a margin, require spatial coverage, and restore the full range on weak prediction or boundary pressure. Vertical halos are sized for image filters instead of the horizontal disparity span. Per-tile predictions, intervals and fallback counts are retained beside disparity-range reports. **Pyramid tiles are an approximation with independent local paths**; use one level with GPU streaming or CPU `--tiled` when continuous full-image paths are required. No resolution downgrade occurs silently.

## Matching costs

| `--matching-cost` | Cost | Main tradeoff |
|---|---|---|
| `census` (default) | Census descriptor Hamming distance | Robust to monotonic brightness changes; may lock subpixel disparities toward integers |
| `ad-census` | Census plus clipped absolute grayscale difference | Adds intensity evidence; sensitive to exposure differences |
| `census-gradient` | Census plus clipped x/y gradient difference | Adds local edge evidence with less dependence on a brightness offset |
| `zncc` | Zero-mean normalized window correlation | Handles brightness offset/gain; more arithmetic and weak on untextured windows |

Hybrid weights use `--census-weight` (default 0.5), `--ad-truncation` (30) and `--gradient-truncation` (60). Costs share the descriptor range `0..window²-1`; P1/P2 are expressed in 5×5 units and scaled by `(window²-1)/24`. The subsequent [thin-structure update](THIN_STRUCTURES.md) changes the default bases from 8/32 to 4/24 and makes P2 edge-adaptive. CPU and GPU use the same integer hybrid rounding and cost units. GPU ZNCC requires FP64 support; auto mode can fall back if the selected kernel is unsupported. Sparse feature descriptors/matchers are controlled separately.

A controlled 384×192 plane with true disparity 13.37, 32 hypotheses, identical filters, one discarded warm-up and three CPU trials produced:

| Cost | RMS disparity error (px) | Median CPU matching/filter time (s) |
|---|---:|---:|
| census | 0.279 | 0.172 |
| ad-census | 0.285 | 0.187 |
| census-gradient | 0.277 | 0.182 |
| zncc | 0.302 | 0.619 |

All retained 54,936 pixels in the comparison region. This fixture does not rank performance/accuracy for real datasets. Integer cost quantization can limit correlation-cost parabolic subpixel accuracy; the separate photometric refinement uses continuous correlation. Benchmark source is `src/test/java/sgm/MatchingCostBenchmark.java`, with results in `target/matching-cost-benchmark.json`.

## Flat/thick surfaces and below-surface points

Two correctness issues were identified:

1. **Resize coordinate alignment:** principal points and inherited alignment keypoints used direct multiplication, inconsistent with OpenCV's pixel-center resize mapping. Both now use `(coordinate+0.5)*scale-0.5`. This also keeps centered intrinsics centered across resolution presets. Cached results from the former implementation are invalidated.
2. **Fusion support after refinement:** the final point could retain a supporting camera whose measured pixel no longer met the reprojection tolerance after refinement. CPU and GPU now recheck the actual measured camera, in addition to the source pair. An independent three-camera fixture reproduces the old false support and verifies its rejection. `--legacy-fusion-reprojection` exists only for diagnostic comparison.

Optional `--subpixel-refinement zncc` locally refines stable disparities before left/right validation. It neither fills holes nor smooths across detected depth jumps. On a continuous synthetic textured plane at disparity 13.37, RMS disparity error decreased from about **0.279 to 0.033 pixels**, with unchanged valid coverage in the test region. It remains **off by default**: real scenes can contain depth transitions, repeated texture and image mismatch, and the extra CPU work is measurable.

Thickness also depends on triangulation angle, calibration, pose accuracy, depth agreement tolerances and retained samples from multiple pairs. A relative agreement allowance of 0.005 at 100 m permits a 0.5 m depth difference; it does not promise a zero-thickness surface. Existing filtering presets and explicit `--fusion-relative`, `--fusion-pixels`, `--min-angle`, `--min-views` and uncertainty controls remain available. There is no arbitrary ground-height clipping, flattening, geoid correction or invented ground classification.

`--analyze-run RUN_DIRECTORY` provides read-only sampled PLY diagnostics: local robust plane fits, residual quantiles, thickness (p90-p10), cell offsets, CSV samples and SVG maps. Use `--analysis-region minX,minY,maxX,maxY` to isolate a verified planar patch, and set cell size/tolerance explicitly for comparable runs. `--analysis-confidence` optionally scans the confidence gzip to attribute samples to source pairs/support/uncertainty. A low point may be real terrain, a lower surface or vegetation; the report calls it a candidate to inspect.

The initial city inspection sampled 298,576 of 37 million points. Its median local spread was about 0.37 m, but mixed terrain/roofs/vegetation prevent interpreting that as surveyed ground error. A three-photo replay with fixed source calibration and poses was retained under `target/city-upgrade-replay`. With the original resize mapping, adding subpixel refinement changed aggregate local spread only from 0.362 to 0.355 m; this is a modest proxy change, not proof of a general accuracy improvement. Corrected-resize replays and planar controls are recorded separately. A fresh reconstruction is needed to benefit from corrected alignment/calibration; old completed clouds are not rewritten.

An approximately planar city patch, selected from diagnostics rather than a surveyed ground mask, also illustrates the limit: median local spread was 0.148 m with the former mapping, 0.156 m with optional subpixel refinement, and 0.143 m with corrected resize coordinates. Median offsets from separately fitted global planes did not uniformly improve. These runs deliberately kept the old calibration/poses fixed; they do not establish a universal waviness fix. Reports and pair attribution are under `target/city-patch-*`. Refit the complete camera network with the corrected mapping before assessing the full project, and compare a known planar region or surveyed control when accuracy matters.

## Fusion utilization

Fusion now records device-event kernel time, blocking read/write time/bytes, CPU guard/audit time, GPU preparation and per-candidate output/metadata/voxel work. Read timings include time waiting for queued kernels and must not be added to kernel times as disjoint costs. Output timing excludes final LAS/PLY publication. See `06_fusion/backend.json` and `06_fusion/output_profile.json`.

GPU measurements use component-major arrays for coalesced access, and geometric weights avoid trigonometric round trips. CPU numerical guards and audits remain active. One background validation worker prepares the next batch while ordered output is written, retaining stable output order and bounded prefetch. `--no-fusion-pipeline`, `--fusion-layout reference` and `--fusion-weight-math reference` provide comparisons. `--output-compression 1..9` trades confidence-file size against CPU compression; default remains 6, with a larger gzip buffer.

A fixed three-photo city replay spent about 0.007 s in device kernels, 0.206 s in GPU batch handling, 0.022 s in guards/audits and 0.541 s in ordered output work for 137,903 candidates. This illustrates why GPU activity can be below 5% even when GPU fusion works: CPU handling/output dominates this case. Larger view graphs have different kernel costs. No universal utilization percentage or speedup multiplier is promised.

With corrected resize coordinates, reference layout/math with synchronous output versus the new defaults retained the same 55,843 accepted candidates, support counts and order. LAS bytes were identical; maximum unquantized PLY coordinate difference was 1.05e-9 m, relative-sigma difference 4.34e-18, refinement-shift difference 4.41e-10 m. The comparison is in `target/city-fusion-equivalence.json`; timing differences on this small replay are not a reliable general speedup estimate.

## Pair overlap

`--min-forward-overlap`, `--min-side-overlap`, and `--min-pair-overlap` accept percentages 0..100, disabled by default. They measure common projected footprint extents along/across flight and common polygon area, divided by the smaller extent/area. Diagonal pairs must pass every enabled threshold. All edges, including sequential fallbacks, are checked; dense pairs are checked again after registration.

Footprints use `--overlap-ground-z` or a constrained sparse scene-plane approximation. `--flight-heading` is degrees clockwise from world +Y toward +X; otherwise a coherent capture direction is inferred. Nonflight scenes can use shared area. Unknown registration, horizon crossings, uncertain plane/direction or strong relief are explicitly reported; `--overlap-unknown keep` is the default, with `reject` available for known-geometry workflows. Estimates and acceptance reasons are in `03_matches/overlap_*.json`. Strict thresholds can disconnect the graph; they do not add registration evidence.

## Examples

```powershell
# From sgm_java; larger explicit RAM/VRAM allowances and automatic exact streaming
.\photogrammetry.bat -i photos -o output/project --memory-mb 6000 --gpu-memory-mb 7000 --gpu-sgm auto --quality high
# Approximate hierarchical local searches, without reducing final raster resolution
.\photogrammetry.bat -i photos -o output/hierarchical --pyramid-levels auto --matching-cost census-gradient
# Known aerial geometry: directional overlap and explicit plane in supplied pose units
.\photogrammetry.bat -i photos -o output/known --calibration camera.properties --poses poses.csv --overlap-ground-z 100 --flight-heading 25 --min-forward-overlap 60 --min-side-overlap 30
# Diagnose a completed cloud; no reconstruction or filtering is performed
.\photogrammetry.bat --analyze-run output/project/runs/RUN_ID --analysis-cell-size 4 --analysis-plane-tolerance 0.05
```

Linux uses the same arguments, e.g. `bash photogrammetry.sh -i photos -o output/project --memory-mb 6000 --gpu-sgm auto`. Windows execution is validated locally; Linux runtime execution is not available in this Windows session. The Java algorithms and launchers retain the portable OpenCV/Proj4J/OpenCL runtime design.
