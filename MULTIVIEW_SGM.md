# Multi-view plane-sweep SGM (`--dense-method mvsgm`), started 10 November 2026

Goal: one depth map per camera from several neighbouring views at once, instead of pairwise
rectified SGM followed by fusion. Opt-in; `--dense-method sgm` stays byte-identical.

## References

- Identity reference: a run of the previous code on city2 (LAS points `b04c3faf…`, PLY
  `da7bb423…`, confidence `61c2bd0f…`).
- Pair-SGM baseline with the reference city2 settings (10 GB heap): wall 90 s, stereo 29.5 s,
  fusion 20.7 s, 3,720,817 points. C2C to metashape.las: raw median 0.668 m (0.14% within
  10 cm); ICP median 0.207 m, 20.0% within 10 cm; sim median 0.205 m, 22.0% within 10 cm.
  Flagpole: points up to 3.3 m of 14.6 m. Play structure: 3,272 leg-zone and 165 roof points.
- Per-camera merged maps of that run (`-Dsgm.dumpMergedMaps=true`, new diagnostic that only
  writes files): `06_fusion/merged_maps/camera_N.depth` (`DepthMapFile`: size, camera, float z).

## Method as implemented (MultiViewSgm.java)

Per reference camera c:

1. **Neighbours**: up to 6 cameras sharing sparse points with c, scored by the sum over shared
   points of a view-selection weight of the triangulation angle (peak 10 deg, sigma 4 below,
   12 above, nothing under 1 deg; at least 12 shared points; under 5% of the best score dropped).
   Camera 6 of city2: 4, 5, 8, 7, 3, 9 (median angles 6-15 deg).
2. **Labels**: fronto-parallel planes at inverse depth q. Range from the sparse points c sees
   (1%/99% quantiles widened to 3+-view points, margin max(16 labels, 2% of q_max), as
   DisparityRange.fromEvidence does for pairs). Step: one label moves a pixel by at most
   `labelPixels` (1.0) in the neighbour with the largest motion. Camera 6: 114 labels,
   72.4-100 m.
3. **Cost**: census of c once. Each neighbour is resampled **once** (bilinear) into a virtual
   camera at its own centre with c's rotation and intrinsics, and its census is computed there.
   A plane q then maps reference pixel p to `(p + q K t') / (1 + q t'z)` (t' = R_c (C_c - C_j)) in
   that image, a pure epipolar shift: the census is looked up per label (bilinear interpolation
   of the four Hamming costs), never recomputed. Deviation from the plan text: the neighbour's
   census is computed on this rotation-aligned resample, not on its own image, because a census is
   not rotation-invariant and city2's flight lines are flown in opposite directions (180 deg yaw).
4. **Best-k**: per pixel and label, the sum of the 3 smallest neighbour costs (fixed point,
   Hamming x 16); a missing neighbour counts as half the census bits.

## Stage 1: one-camera prototype (WTA, no SGM) - done, measured

Run: `-Dsgm.mvsgm.prototype=6` on a dense branch of the baseline; writes
`05_stereo/mvsgm/camera_6.{depth,margin,cost,views,speckle}` and stops the dense step. Time on the
CPU: 1.9 s preparation + 3-6 s cost/WTA per camera (12 threads).

Comparison (per camera, image space): Metashape points moved into our frame by
the C2C ICP transform and z-buffered into the camera; flat patches = 16x16-pixel tiles whose
visible Metashape points fit a plane with RMS < 2 cm and slope < 10 deg (camera 6: 2,518 tiles).
Per tile: own-plane RMS, robust sigma (1.4826 MAD after a 3-sigma refit) and the share of pixels
more than 25 cm from the robust plane. "Equal coverage" keeps the most unique mvsgm pixels
(margin = (2nd best - best)/2nd best) until coverage equals the merged map's.

Camera 6 (A = today's merged per-camera map; B = mvsgm WTA):

| map | coverage | flat RMS median / p90 | robust sigma median / p90 | >25 cm |
|---|---:|---:|---:|---:|
| A merged map (pair SGM + LR + fusion pass 1) | 77.4% | 9.54 / 18.17 cm | 7.77 / 15.19 cm | 4.62% |
| B raw WTA, per-pixel cost | 98.0% | 32.4 / 35.7 cm | 37.6 / 43.1 cm | 50.4% |
| B raw WTA, speckle-filtered | 57.3% | 16.3 / 22.4 cm | 13.9 / 21.7 cm | 12.7% |
| B WTA on 3x3 box, speckle-filtered, equal coverage | 77.4% | 10.7 / 19.1 cm | 8.05 / 17.2 cm | 5.73% |
| B WTA on 5x5 box, all | 98.6% | 8.43 / 33.5 cm | 6.34 / 38.2 cm | 13.0% |
| **B WTA on 5x5 box, speckle-filtered** | **93.2%** | **7.24 / 16.8 cm** | **5.78 / 14.4 cm** | **4.12%** |

Camera 12 (other flight line, 137 labels):

| map | coverage | flat RMS median / p90 | robust sigma median / p90 | >25 cm |
|---|---:|---:|---:|---:|
| A merged map | 72.0% | 7.36 / 13.1 cm | 6.34 / 11.4 cm | 2.05% |
| B raw WTA, speckle-filtered | 55.6% | 15.3 / 19.6 cm | 14.1 / 19.9 cm | 10.6% |
| B 5x5 box, speckle-filtered | 90.2% | 6.72 / 14.1 cm | 5.69 / 13.0 cm | 2.92% |
| B 5x5 box, speckle-filtered, equal coverage | 72.0% | 5.93 / 11.0 cm | 5.16 / 10.3 cm | 1.28% |

Other variants (camera 6, raw WTA): nearest-pixel lookup, 7x7 census and half-pixel labels all
leave the raw WTA at ~37 cm robust sigma; the raw per-pixel census cost (one 5x5 census per
neighbour) is too weak without spatial support, whatever the details. The distance to the
Metashape plane (median |d|, alignment-limited) is 14.3 cm for the 5x5 variant against 17.8 cm
for the merged map.

Edges and thin objects (crops `pole.png`, `shelter.png`): the merged map has almost nothing of
the flagpole; the 5x5-box map shows a continuous streak along the pole. The play structure's roof
is half missing in the merged map (shaded side) and complete in the mvsgm map; tree crowns and
the roof edge are slightly fattened by the 5x5 box (expected for box aggregation; SGM's
edge-adaptive P2 should do better).

### Reading for the go/no-go

- Literally as specified (per-pixel cost, WTA, nothing else) the multi-view map is much noisier
  than today's map: NO-GO by the letter.
- With any spatial support, which SGM provides in stage 2, the same cost is clearly better: with
  only a 5x5 box (no SGM, no LR check, no cross-view check) it beats today's filtered, LR-checked
  merged map on noise at equal coverage on both cameras (robust sigma -26% / -19%, outliers
  similar or lower), and covers 90-93% of the image against 72-77%.
- Recommendation: GO for stage 2 (SGM over the label volume).

Default output check after the stage-1 code (diagnostic hooks behind system properties only):
The identity check: dense.las/dense.ply/point-confidence.bin SAME.

Go/no-go: continued on the stage-1 recommendation (10 November).

## Stage 2: SGM over depth labels, all cameras - done, measured

Code: `MultiViewSgm` (plan, integer cost, CPU reference SGM, selection), `GpuMultiViewSgm` (new).

- Cost: `MultiViewSgm.cost`, one integer definition for CPU and GPU. Per label and neighbour a
  float table holds the plane mapping (canvas x = x a + bx, y = y a + by; computed in double,
  rounded to float), so the kernel has no divisions; both evaluate the same float products and
  sums (`FP_CONTRACT OFF`), bilinear Hamming interpolation, rounding half up, mean of the best 3
  (missing neighbour = half the census bits). Units: Hamming x 8 (at most 384 < 512 = invalid).
- Aggregation: the packed banked volume and the **unchanged `bpaths` kernel** of
  GpuBandedStereo (continuous 8 paths, edge-adaptive P2 on the reference gray, restart after
  untextured pixels). Penalties: --p1/--p2 (5x5-census units) scaled to the census window, times
  8: P1 = 32, P2 = 192 for the 5x5 census. New `mvwinners` kernel (best, value, second,
  before, after; flags for band edges and range edges).
- CPU reference/fallback: `MultiViewSgm.cpuSummary` (the BandedSgm.cpuSummary recurrence on the
  precomputed costs). Self-check: the first GPU camera in a process also solves a 96-row band on
  the CPU; every summary integer must agree, otherwise the GPU path is disabled for the process.
  city2: "mvsgm GPU: 1125x96-row check (20,932,860 cells) identical to the CPU reference".
  `-Dsgm.cpuMvsgm=true` or `--compute cpu` forces the CPU.
- Memory: the session reserves `GpuMultiViewSgm.deviceBytes` (packed volume x (2 + 2 bytes) +
  inputs incl. neighbour canvases) from a `GpuMemoryPool` installed for the mvsgm phase. No
  per-camera state survives the camera.
- Selection: range-edge winners rejected, uniqueness as pair SGM (`--uniqueness`), sub-label fit
  (`--subpixel-fit`), edge-preserving median and speckle removal on the label map
  (`--speckle-size`, 1 label, `--speckle-connectivity`). Depth = 1/q.
- Run: `-Dsgm.mvsgm.maps=all|LIST` on a dense branch writes `05_stereo/mvsgm/camera_N.depth` and
  `.margin` and `mvsgm.json`, then stops (the option and fusion come in stage 4).

Timing (city2, all 17 cameras, full label range 111-195 labels, 210-440 M cells per camera):
dense step 27 s, of which per camera 0.6-0.9 s preparation (CPU: neighbour resampling and census)
and **0.27-0.35 s GPU SGM** (camera 0 2.9 s incl. the CPU self-check). Baseline pair SGM stereo
stage: 29.5 s. GPU memory: about 1.0-1.9 GB per camera (stage 3 reduces it).

Per-camera maps against today's merged maps (flat RMS = own-plane RMS on
Metashape-flat 16x16 tiles; same tool as stage 1):

| camera | map | coverage | flat RMS median / p90 | robust sigma | >25 cm |
|---|---|---:|---:|---:|---:|
| 0 | merged / mvsgm | 58.0 / 83.0% | 12.9 / 2.34 cm, 23.3 / 4.40 | 9.14 / 2.05 cm | 8.14 / 0.93% |
| 6 | merged / mvsgm | 77.4 / 94.5% | 9.54 / 2.92 cm, 18.2 / 6.78 | 7.77 / 2.50 cm | 4.62 / 1.28% |
| 12 | merged / mvsgm | 72.0 / 89.7% | 7.36 / 2.15 cm, 13.1 / 4.38 | 6.34 / 1.96 cm | 2.05 / 0.75% |
| 16 | merged / mvsgm | 65.6 / 83.7% | 12.9 / 2.25 cm, 21.9 / 4.16 | 9.64 / 1.97 cm | 7.25 / 1.20% |

Where both are valid, the maps agree to a median 0.10-0.12% of depth (about 10 cm). Crops
(camera 6): the flagpole is a continuous streak, the play structure's roof is complete with clean
edges (the merged map misses its shaded half).

Union cloud of the 17 maps (no fusion or consistency check, 34.5 M
points with duplicates; measurement only):

| | SGM baseline (fused) | mvsgm union (stage 2) |
|---|---:|---:|
| C2C raw median | 0.668 m | 0.651 m |
| C2C ICP median / within 10 cm | 0.207 m / 20.0% | 0.222 m / 18.5% |
| C2C sim median / within 10 cm | 0.205 m / 22.0% | 0.233 m / 19.1% |
| flagpole top | 3.3 m | **13.6 m** (true 14.6 m) |

The absolute C2C is dominated by the georeferencing difference (quick calibration; ICP leaves a
~20 cm median for both); the union also keeps every camera's edge and occlusion errors, which
stage 4's consistency check removes. Default output after stage-2 code: see stage 3 check.

## Stage 3: coarse-to-fine banding - done, measured

`MultiViewSgm.solve`: pyramid of `--tsgm-levels` (3) levels below full resolution (stops when a
side is under 48 px or fewer than 8 labels remain). A coarse level (factor f) uses the 2x2-box
downsampled reference and neighbour canvases (`TsgmSearch.downsample`; census recomputed once
per level, not per label), every f-th label, and a level table derived from the full-resolution
geometry (level pixel x_l = full (x_l + .5) f - .5; `MultiViewSgm.table(labels, views, f, n)`).

- Coarsest level: all labels. Finer levels: bands from the parent's unique, non-range-edge
  winners with the unchanged `TsgmSearch.derive` (3x3 parent min/max doubled, margin
  `--tsgm-margin` 3, doubled at depth edges, 9/33/129 fallback windows, then the full range; GPU
  derive via GpuTsgm, self-checked), widened within 4 pixels of every sparse point the camera sees
  to include its label +-(margin+1).
- Full resolution: winners on a band edge (not a range edge) widen their band and a one-pixel
  halo once (as BandedSgm.widen) and the image is solved again; still on an edge = rejected.
- Each level is one GPU session (reserved from the pool, released before the next level).
- Self-checks: the first full-band and the first narrow-band GPU solve in a process are each
  compared with the CPU reference on a 96-row crop of the actual bands. city2: "140x96-row
  full-band check (326,400 cells) identical", "281x96-row narrow-band check (506,937 cells)
  identical", "GPU TSGM band planning: 281x500 level identical to the CPU derive".
- The next camera's plan (neighbour resampling, census, pyramid inputs) is prepared on a second
  thread while the current camera is solved (at most two plans resident); results unchanged.
- `-Dsgm.mvsgm.levels=0` gives the stage-2 full-range solve.

City2, all 17 cameras (camera 0 shown for the per-level numbers):

| | stage 2 (full range) | stage 3 (banded) |
|---|---:|---:|
| full-resolution packed cells, camera 0 | 433 M | 42.2 M (mean band 19.0 labels) |
| packed cells, cameras 1-3 | 373 / 341 / 248 M | 35.5 / 31.9 / 28.0 M |
| peak GPU memory per camera (session reservation) | ~1.8 GB | 0.35-0.41 GB |
| GPU SGM per camera (all levels, retry) | 0.27-0.35 s | 0.64-0.83 s |
| dense step, 17 cameras (maps only) | 27 s | 30.5 s; **21 s** with preparation overlapped |
| valid pixels, camera 0 / 6 | 1.87 M / 2.13 M | 1.90 M / 2.14 M |

Camera 0 levels: f=8 140x250, 25 labels (full) 0.84 M cells; f=4 49 labels, mean band 14.9; f=2
98 labels, mean band 15.7; f=1 195 labels, mean band 19.0, 0 full-range fallbacks, 0.52 M
band-edge retry pixels.

Quality is unchanged or slightly better (coverage +0.8 to +2.2 points, same noise):

| camera | coverage stage 2 / 3 | flat RMS median stage 2 / 3 | robust sigma | >25 cm |
|---|---:|---:|---:|---:|
| 0 | 83.0 / 84.3% | 2.34 / 2.34 cm | 2.05 / 2.05 cm | 0.93 / 0.95% |
| 6 | 94.5 / 95.3% | 2.92 / 2.91 cm | 2.50 / 2.50 cm | 1.28 / 1.25% |
| 12 | 89.7 / 91.9% | 2.15 / 2.16 cm | 1.96 / 1.96 cm | 0.75 / 0.69% |
| 16 | 83.7 / 85.2% | 2.25 / 2.25 cm | 1.97 / 1.98 cm | 1.20 / 1.20% |

For comparison, today's pair path: 58 pairs, each a banded session of about 45 M cells at full
resolution, 3-4 pairs in flight, stereo stage 29.5 s.

## Stage 4: consistency check, fusion integration, full city2 run - done, measured

### Option and command

`--dense-method mvsgm` (opt-in; requires `--fusion-mode image`). New options (package-private,
so default run settings and reports are unchanged; recorded in `05_stereo/mvsgm/mvsgm.json`):

| option | default | meaning |
|---|---:|---|
| `--mvsgm-neighbors N` | 6 | neighbour cameras per reference camera (1..8) |
| `--mvsgm-best-k K` | 3 | mean of the K lowest neighbour costs per pixel and label |
| `--mvsgm-census-window W` | 5 | census window (3, 5, 7) |
| `--mvsgm-levels N` | -1 | coarse-to-fine levels (-1 = `--tsgm-levels`; 0 = full range) |
| `--mvsgm-consistent-views N` | 1 | neighbour maps that must agree (0 = no check) |
| `--mvsgm-consistency-labels T` | 1 | agreement tolerance in the neighbour's label steps |

A pair-SGM command works unchanged with `--dense-method mvsgm` in place of `--dense-method sgm`
(only the method changes): `--two-view-confidence-threshold` is ignored with a note (pair SGM only),
`--pairwise-search`, `--similarity-window`, `--census-edge-mask`, `--disparity-edge-erosion` and
`--stereo-pipeline` have no effect, `--depth-outlier-filter edge-aware` keeps its fusion-side
camera checks. `--speckle-size`, `--uniqueness`, `--subpixel-fit`, `--p1/--p2`, `--median-mode`,
`--sgm-edge-scale`, `--tsgm-margin` and `--step` apply to the depth maps.

### Consistency check and fusion (MultiViewSgm.run)

- Phase 1: every camera's depth map (stages 2-3) is written to `05_stereo/mvsgm` (depth, margin,
  sharpness, photometric; 16 bytes per pixel per camera on disk), deleted after the run unless
  `--diagnostics` or `-Dsgm.mvsgm.keepMaps=true`.
- Phase 2 (instead of the left/right check): a depth is kept if at least
  `--mvsgm-consistent-views` of the camera's neighbours' maps hold, at the rounded reprojection,
  a depth within `--mvsgm-consistency-labels` of that neighbour's inverse-depth label steps (one
  label = about one pixel of image motion: the analogue of the 1-pixel LR check). Neighbour maps
  are read through an LRU cache of neighbours+2 maps; memory per camera is bounded.
- Survivors (every `--step`-th pixel) are written as candidates of one fusion pair per camera,
  (camera, best neighbour), marked single-view (`DepthFusion.markSingleView`): image fusion splats
  it into the camera only (`ImageFusion` constructor; no effect without such pairs), so pass 1
  merges one sample per pixel; the neighbour supplies the triangulation-angle weight; the
  cross-check, edge-aware camera checks, graph averaging and emission are unchanged. Evidence:
  confidence = PointConfidence.score(photometric, margin, sharpness, lr), margin = (second - best)
  / second aggregated cost, sharpness = rise to the lower neighbouring label / (directions x P1),
  photometric = 1 - aggregated cost / (directions x half the census bits), LR error = best
  agreement in labels.

City2: 32,274,747 of 35,046,423 depths (92.1%) consistent with at least one neighbour map
(per camera 86.9-95.8%).

### Full city2 run, reference settings (10 GB heap, back to back)

| | pair SGM (baseline) | mvsgm |
|---|---:|---:|
| run | ab6ee3f7 / a55ff791 | 87ae93f3 |
| wall, whole pipeline | 90 s / 103 s | 88 s |
| 05 stereo | 29.5 / 33.8 s | 30.1 s (depth maps 21.9 s, consistency + candidates 8.0 s) |
| 06 fusion | 20.7 / 27.5 s | 22.1 s |
| peak sampled GPU memory (whole process) | 2,562 / 1,617 MiB (3-4 pairs in flight) | 774 MiB |
| GPU reservation per camera / pair | ~0.3-0.5 GB per pair | 0.35-0.41 GB per camera |
| points | 3,720,817 | 5,522,864 |
| Metashape-flat 1 m cells: own-plane RMS median / p90 | 3.38 / 10.5 cm | **0.91 / 1.77 cm** |
| same cells: robust sigma median / p90 | 3.08 / 7.93 cm | **0.90 / 1.75 cm** |
| same cells: median offset to the Metashape plane | 6.7 cm | 4.7 cm |
| C2C raw median / within 10 cm | 0.668 m / 0.14% | 0.642 m / 0.26% |
| C2C, baseline's ICP transform for both: median / within 10 cm | 0.207 m / 20.0% | 0.259 m / 14.2% |
| C2C ICP fitted per cloud: median / within 10 cm | 0.207 m / 20.0% | 0.301 m / 14.8% |
| C2C sim per cloud: median / within 10 cm | 0.205 m / 22.0% | 0.275 m / 16.6% |
| points > 1 m from any Metashape point (fixed transform) | 13.0% | 16.0% |
| points in 1 m cells without any Metashape point | 0.72% | 3.04% |
| flagpole (side view) | 3 m stub (top 3.3 m) | **complete to ~14 m (top 13.5 m)** |
| play structure (side view) | roof partly missing, scattered points below | roof complete, flat and thin; legs not resolved |

The flat-cell metric (1 m cells where Metashape fits a plane with
RMS < 2 cm and slope < 10 deg, 5,293 cells) is alignment-independent for the noise columns. The
two clouds agree with each other closely (mvsgm vs baseline in the same frame: median 6.0 cm,
signed dz median -1.9 cm, 74.6% within 10 cm), so the poorer whole-cloud C2C is not a bias: it
comes from where the extra 1.8 M points are (image-border regions the maps now reach, tree
interiors and ground under crowns, which Metashape does not cover; 3% of mvsgm points lie in
cells with no Metashape point at all) and from the per-cloud ICP converging differently. The thin-object
shelter counters use hard-coded heights that do not match this georeferencing (it reports 0 roof
points for mvsgm although the side view shows the complete roof at the same height as the
baseline's), so the side views are the comparison for the play structure.

(baseline top row, mvsgm bottom row).

### Default output

The identity check after all stage-4 code: dense.las SAME (points; header date), dense.ply SAME,
point-confidence.bin SAME.

### Tests

Full SelfTest from C:\t\s2: 118,545 checks, all passed (fixtures deleted). `MultiViewSgmRegression` (new, in SelfTest, 32 checks): synthetic two-layer textured scene, four
neighbours (one rotated 180 deg); CPU depth within 0.5% on >= 90% of non-edge pixels; GPU summary
= CPU summary for full and three random narrow band sets; banded solve within one label of the
full-range solve on >= 98%; level-table mapping; option validation and copy.

### Known limits / next steps

- Points outside Metashape's coverage (borders, under canopy) raise C2C distances; a stricter
  consistency setting (`--mvsgm-consistent-views 2`) or a minimum-views rule for border pixels
  would trade coverage for that.
- Pole and edges are a little fattened (about 2-3 pixels): SGM with census 5x5; a smaller census
  window or a final edge-aware check could tighten them.
- Consistency + candidate writing (8 s) is serial file reads and per-value DataOutputStream
  writes; a bulk ByteBuffer writer would roughly halve it.
- `--compute cpu` / no GPU: the CPU reference is exact but slow (packed volumes ~40 M cells per
  camera, 8 paths).

## Supplied poses, clear errors and --step (10 November 2026, later)

A run on a 4-image large-format block (31,520 x 13,440 frames, poses and calibration from
an imported `.prj` orientation) failed with "No dense candidates": with fixed supplied poses POSE
skipped features and matching, so there were no tie points, and mvsgm takes both each camera's
neighbours and its depth range from tie points. Every camera was skipped as "no neighbour".

Changes (pair SGM and the default output unchanged; identity check SAME afterwards):

1. **Tie points with fixed supplied poses** (`Pipeline.fixedPoseTiePoints`): with `--poses` /
   an imported orientation, `--extrinsics-mode fixed` and `--dense-method mvsgm`, the match step
   still extracts and matches features, and the align step triangulates the tracks with the poses
   and calibration exactly as supplied (no bundle adjustment; reprojection limit 2 x
   `--reprojection`). Fewer than 12 triangulated tracks fail with a clear message.
2. **Clear mvsgm errors**: the dense step fails at once when the project has fewer than 12
   triangulated tie points (message says to rerun from the match step with `--branch`, or use
   `--dense-method sgm`); a camera without neighbours or depth range is skipped with a WARNING
   line and its reason in `05_stereo/mvsgm/mvsgm.json`; if no camera gets a map, the step fails
   naming the reason instead of fusion's generic "No dense candidates".
3. **Bug fix, `--step` > 1** (found while testing; the reference settings use `--step 1`): mvsgm wrote only
   every step-th pixel as candidates, and fusion's 4-connected speckle filter then removed all of
   them (0 points). mvsgm now passes every consistent pixel to fusion whatever `--step` is (a note
   says so), and fusion's automatic agreement radius uses 1 for mvsgm, as at `--step 1`.

Tests on `very_large` at `--quality low` (frames processed at 3,940 x 1,680; the reference command
otherwise):
- single call: 19,513 tie points, all triangulated with the supplied poses; 4 cameras, 3
  neighbours each; 21.8 M consistent depths; 7,550,380 points; whole run 69 s.
- steps `inputs` (sgm) -> `calibrate match align georeference` -> `dense --dense-method mvsgm`:
  fails with the tie-point message; `match ... dense finish --branch --dense-method mvsgm`: 7,109
  tie points (default feature settings), 7,089,833 points with the default `--step 2`.

Not done yet: mvsgm at `--quality high` on these frames (15,760 x 6,720, 106 Mpx) needs a tiled
mode; each neighbour canvas would exceed the GPU's single-allocation limit and the CPU fallback
would be very slow. Use `--dense-method sgm` (windowed stereo) for such frames until then.

## Next: large frames (100+ Mpx per image) - plan, not started

Target: the 4-image large-format block (31,520 x 13,440 frames; 15,760 x 6,720 = 106 Mpx per
image at `--quality high`). Today mvsgm runs it at `--quality low` only.

What has to change (in this order):

1. **Check pair SGM first**: run the block at high with `--dense-method sgm`. Windowed stereo
   handles the pairs, but image fusion's per-camera working set (merge, splats, candidates; about
   100-200 bytes per pixel) is 10-20 GB per 106 Mpx camera, so fusion probably fails there too.
2. **Fusion in tiles within a camera** (needed by both methods for such frames): pass 1 and pass 2
   per camera tile with an overlap that covers every neighbourhood operation (5x5 spike window,
   7x7 stride-4 normals = +-12 px, agreement radius); per-tile marks. The speckle filter's connected
   components are not local: label components per tile and join them across tile borders
   (union-find on border pixels) so the result stays identical to the untiled filter.
3. **mvsgm in tiles**: per reference tile (about 2,048 x 2,048 plus a 64-px overlap, as
   BandedSgm strips), resample each neighbour only over the region the tile can reach (the canvas
   rule in `MultiViewSgm.warp`, applied to the tile's corners), solve the bands, select, and write
   the tile core into the camera's map files by row ranges. Median filtering is local (overlap
   suffices); speckle removal as in 2. Phase-2 consistency reads neighbour maps through
   memory-mapped files instead of whole arrays.
4. Test on the large-format block at high, then at ultra; identity checks for untiled sizes.

Rough size: 2 and 3 are each about one working session.

## Depth previews (11 November 2026)

With `--export-disparity-images` or `--diagnostics`, mvsgm writes per image
`05_stereo/mvsgm/image_<id>_<name>/`: `depth.*` (the SGM depth map after selection and filters)
and `consistent_depth.*` (the depths that pass the multi-view consistency check, i.e. what goes to
fusion). Each is `.pfm` (depth in scene units, NaN = none), `.png` (linear grayscale over that
image's label depth range, near dark, far bright, 0 = none) and `.json`, written with the same
helper as the pair path's `disparity.*`. Without either option nothing extra is written.
