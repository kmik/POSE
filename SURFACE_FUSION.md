# Reprojection refinement and normal-aware surface consolidation

Current GPU scheduling, metric exports and storage details: [PERFORMANCE_AND_ATTRIBUTES.md](PERFORMANCE_AND_ATTRIBUTES.md).

Both stages are enabled by default. They target random surface thickness without assuming that ground, roofs or walls are horizontal. SGM penalties and image-resolution defaults are unchanged by this update.

## Per-candidate reprojection refinement

`--depth-refinement reprojection` replaces the previous robust weighted-depth average. After the existing depth-mode/support checks, it fits inverse depth along the source camera ray by minimizing a Huber reprojection objective. The source pixel stays fixed, the original stereo partner remains an observation, and each additional measured camera contributes at most once. Six bounded Gauss-Newton iterations with backtracking prevent objective-increasing steps. Source reprojection, depth modes, distinct-camera support and uncertainty are checked afterward.

Evidence rasters store subpixel offsets as well as depths. A neighboring lookup is not necessarily the same surface point: treating its raw image coordinates as an exact correspondence introduces a lateral-offset/parallax error. The new fit first links that depth to the common base ray using a locally constant base-camera-Z patch. This is a local approximation, not a full photometric or surface-warped optimization; sloped/discontinuous surfaces remain an important limitation. A numerical regression isolates this offset error.

The implementation exists in CPU Java and FP64 OpenCL, for both fusion memory layouts. CPU fallback, numerical guards and continuous CPU/GPU auditing remain active. Audit coordinate tolerance uses the scale of the entire world frame on every axis because world-to-camera rotations mix large eastings/northings into height. Local-coordinate tolerance remains 20 nm; at Finnish projected coordinates the roundoff allowance is roughly 15 micrometres, below LAS quantization. Acceptance/support counts must still agree exactly.

The reported uncertainty remains conditional on the supplied/estimated cameras. The reprojection fit cannot reduce the earlier conservative geometric uncertainty, and camera calibration, pose errors and shared measurement correlations are not a surveyed covariance model.

## Surface consolidation

Per-candidate validation alone can export several slightly different positions for the same surface. The added stage:

1. Estimates normals from the source depth map, trying image radii 3, 6 and 12 processed pixels when a smaller window is too noisy. Centered covariance, robust residual trimming, planarity, viewing-angle and foreground-center checks reject unreliable estimates.
2. Streams accepted observations into compressed spatial partitions. Halo observations support fitting across partition boundaries.
3. Finds neighboring samples with compatible **oriented** normals. Opposite sides of a thin object are kept separate. Resolved gaps in normal-direction depth split parallel layers before filtering.
4. Balances contributions by producing stereo pair and uses an uncertainty-weighted median along the normal. Exact even-weight ties use the midpoint to avoid choosing the front/back systematically.
5. Bounds movement and reruns the multi-view checks at the proposed position **without another depth refinement**. A failed proposal keeps the original validated point.
6. Combines nearby redundant samples from different pairs only when their projected surface positions also pass the depth evidence checks. Representative support is re-evaluated; merged sample counts never inflate camera counts.

Normals that cannot be estimated safely cause passthrough, not deletion. Extremely crowded or scale-degenerate partitions that cannot fit the working set also pass through unchanged and are counted explicitly. Unresolved close layers, motion, weak texture and incorrect cameras cannot always be distinguished from noise. This is not terrain classification, a DTM generator, ground clipping or arbitrary plane flattening.

Representative selection is deterministic within spatial partitions. Changing the working-set budget can change partition ownership and representative IDs/density near boundaries. Halo fitting limits boundary artifacts, but identical output across different partition budgets is not promised. Prefetched and synchronous processing with the same settings produce identical results in the regression fixtures.

## Controls

| Argument | Default | Meaning |
|---|---|---|
| `--depth-refinement reprojection\|weighted` | `reprojection` | New pixel-space fit or earlier depth-average estimator |
| `--no-depth-refinement` | off | Disable this per-candidate fit; surface consolidation remains independent |
| `--surface-consolidation normal\|none` | `normal` | Enable/disable spatial consolidation |
| `--surface-radius N` | 2 | Tangential neighborhood radius in local candidate spacings (`pixel footprint * --step`) |
| `--surface-spacing N` | 1 | Representative spacing in local candidate spacings; larger values combine more observations |
| `--surface-max-shift N` | 2 | Maximum normal movement in processed-pixel footprints, independent of `--step` |
| `--surface-normal-angle N` | 30 degrees | Maximum difference between oriented normals |
| `--surface-memory-mb N` | 128 MiB | Working-set target, also capped at a fraction of `--memory-mb`; minimum 8 MiB |

The existing filtering, minimum views, reprojection, relative-depth and uncertainty thresholds remain active. Larger radius/spacing can suppress more variation but reduce detail. `--step 1` still provides the best opportunity to retain narrow structures. The new consolidation stage runs on CPU; GPU SGM and per-candidate GPU fusion/refinement remain available.

To compare against the earlier fusion behavior, add:

```text
--depth-refinement weighted --surface-consolidation none
```

To assess the reprojection fit independently, use `--surface-consolidation none`. To disable both kinds of movement, use `--no-depth-refinement --surface-consolidation none`.

## Project outputs and storage

- `06_fusion/fusion.json` and `backend.json`: refinement method, support/uncertainty policy and GPU timing/resources.
- `06_fusion/surface_consolidation.json`: input/output/merged/moved counts, unreliable-normal and memory passthrough counts, rejected movements, parameters, compressed input and peak partition bytes. Its elapsed time spans staging through consolidation, including fusion waits during staging.
- `06_fusion/output_profile.json`: output staging and normal-preprocessing timings. Consolidation and final cloud publication have separate scopes.
- `07_results/point-confidence.csv.gz`: final published coordinates and representative candidate IDs; existing columns retain their positions. Appended columns are normal XYZ, surface shift in final output units and merged-observation count. A normal is a unit vector in the world coordinate frame; unavailable normals are NaN.
- Debug storage additionally writes `06_fusion/surface_decisions.csv.gz`, mapping input candidates to representative IDs. Original `decisions.csv.gz`/`refinements.csv.gz` describe validation **before** spatial consolidation and remain ordered one row per candidate.

Final LAS/PLY and confidence rows describe the same positions and order. The new stage can reorder points and reduce point count. RGB comes from the retained representative. Optional voxel thinning happens afterward; surface output counts are before that extra thinning.

Only accepted candidates enter the surface spool. Records are now 109 bytes without valid metrics or 146 bytes with valid metrics (planner upper bound 148); temporary partitions use gzip level 1. Splitting temporarily holds a parent and its children, so extra disk is required even with standard storage. The planner reports a conservative uncompressed input estimate and the run report records actual compressed peak usage. Files are removed after success and handled failures, including debug runs. Disabling consolidation avoids this additional spool. No automatic image-quality reduction is performed.

## Validation and measured limits

`SurfaceFusionRegression` covers independent analytical camera projections, the lateral lookup-offset failure, weak baselines, noisy sloping planes, rotation into a wall at large projected coordinates, a one-pixel foreground strip, parallel layers, opposing thin-surface sides, balanced medians, bounded-memory passthrough, cleanup failures, final confidence/cloud agreement and prefetched/synchronous equivalence. The earlier frozen audit tests explicitly select weighted refinement and no spatial consolidation so their old output contract remains independently checked.

Controlled plane tests show a substantial reduction in random scatter while preserving the narrow foreground strip. Numerical results and the current test counts are recorded in `VALIDATION.md`; synthetic improvements are not promises of real-project accuracy.

A fixed-camera three-image city replay was also compared at `--step 4`, maximum dimension 1600, moderate filtering. In an unsurveyed 16 m by 16 m patch, median local p90-p10 spread changed from approximately **0.197 m** with weighted/no consolidation to **0.209 m** with linked reprojection alone and **0.162 m** after consolidation. This is a mixed result for the depth estimator alone and an improvement for the combined pipeline. On the same 2,180 representative candidate IDs across all three variants, fixed-cell/fixed-plane spread was **0.196 / 0.218 / 0.164 m**, reducing sampling-population confounding. These remain surface proxies, not surveyed accuracy. The entire replay retained 42,345 / 42,599 / 35,121 points respectively. Final consolidation refined 14,811 positions, combined 7,478 redundant observations, and preserved 10,805 samples with unreliable normals. Compressed partition peak usage was 6.60 MiB for this small replay.

All 42,599 pre-consolidation accepted candidate IDs, support counts, order and RGB also agree between CPU, coalesced GPU and reference-layout GPU refinement. Maximum raw coordinate difference is 8.21e-8 m; LAS files can differ when that roundoff falls on opposite sides of a 0.1 mm quantization boundary. Prefetched versus synchronous CPU reconstruction is byte-identical in the controlled end-to-end fixture.

The approach draws on Mathias Rothermel, *Development of a SGM-Based Multi-View Reconstruction Framework for Aerial Imagery* (2016), especially sections 4.3 and 6.2-6.4. This is an independent implementation adapted to the program's evidence representation, not an exact implementation or numerical reproduction of SURE.
