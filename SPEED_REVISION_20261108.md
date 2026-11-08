# Speed revision 2026-11-08: city2 18.7 → 5.0 minutes, identical output

## Result

city2 (17 DJI images, `--quality high`, 40,000 ORB features, the full city2 dense, ground
and fusion options; RTX 5070 Laptop, 12 cores):

| Stage | Before | After |
|---|---|---|
| 00 inputs / 01 calibration (overlapping) | 267 s | 29 s |
| 03 matches | 181 s | 2.7 s |
| 04 geometry (bundle, calibration, GPS adjustment) | 347 s | 134 s |
| 05 stereo | 215 s | 61 s |
| 06 fusion | 58 s | 35 s |
| 07 results (ground classification, report) | 40 s | 30 s |
| **Wall time** | **1,123 s** | **302 s** |

`04_geometry/final/exterior.csv`, `07_results/point-confidence.bin` and the LAS point records are
byte-identical to the original run (SHA-256). Only the LAS header creation date differs.

No change alters a computed value. Every parallel step either works on independent items or keeps
the original order of every floating-point addition (see "Bundle adjustment"). An earlier attempt
that summed in parallel chunks changed rounding. The rolling-shutter / focal / flying-height
direction is so flat that this moved the final solution measurably (readout 0.090 → 0.106 s,
flat-surface RMS vs Metashape 3.77 → 3.98 cm). It was replaced by the order-preserving version.

## Changes

### Exact GPU feature matching (`GpuHamming`, `--gpu-feature-matching on|off`, default on)

ORB descriptors (binary) were matched by OpenCV's CPU brute-force Hamming k=2 search, about 1.4 s
per 33k × 33k search. Focal initialization (115 pairs), distortion refinement and stage 03 (115
pairs again) each ran it in both directions. An OpenCL kernel now does the same exhaustive search:
one work-item per query, train descriptors streamed through local memory in index order, and a
neighbour replaced only when strictly closer. That is OpenCV's tie order. It takes 30 ms per
search instead of 1.4 s; all 33,130 rows of a real city2 pair were identical (neighbour indices,
distances, order). The first search of every run is also checked against the CPU matcher on 512
queries; a mismatch or any GPU error switches the run to the CPU matcher. One dedicated OpenCL
session (`GpuSgm.Compute(…, dedicated)`) serves all matching threads, so thread-local sessions
closed elsewhere cannot invalidate it. Ignored with `--compute cpu`.

### Bundle adjustment (`BundleAdjuster`), bit-identical

Profile before: all on the main thread; `solve` 21 %, `subtractRows` 19 %, `StrictMath.hypot`
19 %, linearization about 15 %, `product` 7 %.

- *One linearization per LM iteration.* Jacobians and residuals do not depend on the damping, and
  a rejected trial restores the exact parameters, so the undamped system is built once.
  `Linear.damped(lambda)` adds the damping and inverts the point blocks per trial; this is the same
  arithmetic as before.
- *Preconditioner only when needed.* The block-Jacobi preconditioner (including an 8×8×3×3 lens
  term per point) was built before the direct Cholesky solve, which nearly always succeeds. It is
  now built only for the PCG fallback.
- *Parallel without changing summation order.* Per-track Jacobians and per-observation robust
  costs (including `hypot`) are computed in parallel. Shared sums are then filled per
  *destination*: each camera (its 6 rows) and each lens parameter row is owned by one task, which
  visits its contributions in the original track/observation/point order. Every matrix element
  therefore receives the same additions in the same order as the serial loop. This applies to the
  normal equations, the Schur matrix (`reducedMatrix`), the reduced right-hand side and the
  matrix-vector product. Point updates are independent and run in parallel.

Bundle and calibration regressions pass (Schur 925, quality 66, shared calibration 287, GPS 28,
geo-calibration 59, self-calibration 27 checks). GPS replay of the city2 geometry: 64.5 s with
per-row tasks, 42.2 s with per-camera + per-lens-row tasks (identical results).

### Stereo

- `--stereo-pairs-in-flight` now defaults to `auto` (up to 4 pairs, one per 4 cores; 3 here). It
  already guaranteed serial-identical output; verified on city2 (point records identical),
  stage 05 174 → 82 s.
- Edge-aware depth outlier filter: pixel decisions read only the unfiltered map, so rows are
  decided in parallel and counted/applied serially (identical). Stage 05 82 → 61 s.

### Fusion

`SurfacePatchGrid` caches immutable per-cell surface models in a direct-mapped array capped at
2,048 slots. A 1125×2000 camera has about 8,750 cells, so models were rebuilt continuously (28 % of
fusion samples). The cache now covers the whole grid when the memory budget allows (about 4 KB per
cell; 9,400 cells per camera with 16 GB here). Models are deterministic, so results are unchanged.
Stage 06 46 → 35 s.

## Where the time goes now (302 s)

- Geometry 134 s: two GPS-weighted adjustments (focal starts 2012 and 2121 px, about 65 + 48 s),
  staged lens calibration fits (about 65 s). The work is now in the parallel lens-row Schur
  elimination and accumulation. Further gains need fewer floating-point operations (for example
  skipping inactive lens parameters), which would end bit-identity.
- Stereo 61 s: banded tsgm SGM runs on the CPU for 77 of 92 pairs, plus disparity range and
  triangulation per pair.
- Fusion 35 s, ground classification 24 s, report 10 s.

## Verification

- Full city2 runs after each step, compared by SHA-256 with the original run.
- Bundle/calibration suites as above; `GcpRegression` 38 checks; full `SelfTest`: all groups pass. Run from a
  short working directory it passes every group up to `LauncherRegression`, which needs `src/`
  relative to the working directory; that group and the 10 after it pass from the repo root
  (1,196 checks).
