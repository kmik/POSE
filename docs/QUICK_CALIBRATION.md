# `--quick-calibration` (opt-in alternative calibration path), 9 November 2026

## What it does

| Step | default | `--quick-calibration` |
|---|---|---|
| calibrate | focal bootstrap: features on 12 images, 58 uncalibrated pairs, shared-focal estimate, then guarded focal/distortion refinement (~25 s on city2) | focal from EXIF/XMP (as `--calibration-pipeline joint`), no bootstrap (~1 s) |
| align | parameter-wise shared-lens trials (f with starts 1, 0.9, 1.1; k1, k2, cx, cy, p1, p2, k3) with two held-out track groups | unchanged |
| georeference | GPS-weighted adjustment; a second focal start at the EXIF/XMP prior when the first fit ends more than 2 sigma from it | first start only; `gps_bundle.json` notes when the second start was skipped |
| all converging bundles | stop when two successive iterations each gain < 1e-6 (relative), at most 100 | < 1e-4 (added later the same evening, see below) |

Without a consistent EXIF 35 mm-equivalent focal the calibrate step falls back to the bootstrap,
as `joint` does. Give the option with the first step (`inputs`, or the single call); later steps
reuse it from the saved arguments. It is part of the calibration and geometry cache keys.

The solved cameras differ from the default, so the cloud differs. Everything else is unchanged.

## Measurements

Same machine and settings as the reference city2 run (`--quality high`, `--features 40000`, ...),
whole pipeline in one call, current code for both columns.

| | city2 default | city2 quick | forest_tiny default | forest_tiny quick |
|---|---:|---:|---:|---:|
| calibration stage | 24.9 s | 0.6 s | 20.5 s | 0.2 s |
| geometry stage | 37.4 s | 48.0 s | 28.6 s | 12.1 s |
| whole run (wall) | 192 s | 165 s | 90 s | 52 s |
| final focal (source px) | 2362.4 | 2247.9 | 2387.0 | 2382.4 |
| rolling-shutter readout | 0.046 s | 0.075 s | | |
| GPS RMS after adjustment | 0.970 m | 0.920 m | 0.158 m | 0.143 m |
| reprojection RMS | 0.354 px | 0.354 px | 0.216 px | 0.207 px |

On city2 the alignment calibration starts from the EXIF focal (2121 px), accepts k1 and k2 and
keeps that focal; the GPS-weighted adjustment then moves it to 2248 px. The default path ends at
2362 px. Focal, flying height and rolling-shutter readout trade off along a flat valley here, so
both are fits of similar residual. Geometry takes longer in quick mode because the alignment
calibration accepts more parameters and runs its second sweep.

Comparison with the Metashape clouds (`example_data/forest/<set>/metashape.las`), 300k samples,
nearest-neighbour distance from our cloud to Metashape's (scratch tool C2C, see below):

| | city2 default | city2 quick | forest_tiny default | forest_tiny quick |
|---|---:|---:|---:|---:|
| raw: median distance | 0.160 m | 0.501 m | 0.496 m | 0.515 m |
| raw: within 10 cm | 32.0% | 0.9% | 3.7% | 3.4% |
| after rigid ICP: median | 0.162 m | 0.086 m | | |
| after rigid ICP: within 10 cm | 28.5% | 57.1% | | |
| after similarity ICP: median | 0.147 m | 0.087 m | 0.237 m | 0.143 m |
| after similarity ICP: within 10 cm | 31.6% | 56.2% | 17.8% | 35.6% |
| fitted rotation / scale to Metashape | 0.10 deg / 1.0054 | 0.20 deg / 1.0011 | 2.3 deg / 1.0046 | 1.5 deg / 1.0088 |

Reading:

- **Shape**: after a rigid or similarity alignment the quick clouds agree with Metashape's more
  closely than the default ones on both data sets (city2 median 8.7 vs 14.7 cm; forest_tiny 14.3
  vs 23.7 cm). Metashape to ours (completeness direction, rigid ICP) on city2: median 9.7 cm
  (quick) vs 18.6 cm (default). The city2 scale difference to Metashape is 0.11% (quick) vs
  0.54% (default).
- **Absolute placement**: on city2 the quick cloud sits further from Metashape's before
  alignment (median 50 cm vs 16 cm; a 0.2 deg tilt and a translation). Both reconstructions are
  georeferenced from consumer GPS only (fit RMS about 1 m), so neither is ground truth for
  absolute position; ground control points would be needed to say which is closer.
- Metashape is a reference, not truth. Different solvers, so a closer shape match is evidence,
  not proof, of better geometry.

Conclusion: no sign of degraded relative geometry on these two data sets; absolute position
differs within GPS accuracy. Kept opt-in; worth checking on a data set with GCPs before making it
a default.

## Bundle convergence tolerance (added later, same evening)

In a step-by-step quick run (187 s)
the first alignment bundle hit its 100-iteration cap; 80 of its iterations improved the objective
by less than 0.01%. Quick mode now stops a converging bundle once two successive iterations each
gain less than 1e-4 (relative) instead of 1e-6. The default is unchanged.

| city2, single call | quick (1e-6) | quick (1e-4) |
|---|---:|---:|
| first alignment bundle | 101 iterations (cap) | 22 iterations |
| geometry stage | 48.0 s | 33.0 s |
| whole run | 165 s | 158 s |
| final focal | 2247.9 px | 2243.2 px |
| reprojection / GPS RMS | 0.354 px / 0.920 m | 0.353 px / 0.920 m |
| vs Metashape after similarity ICP: median, within 10 cm | 8.74 cm, 56.2% | 8.76 cm, 56.1% |
| Metashape -> ours after rigid ICP: median | 9.67 cm | 9.63 cm |
| vs Metashape raw: median | 50.1 cm | 50.2 cm |

forest_tiny: 52 -> 50 s (geometry 12.1 -> 10.8 s), similarity-aligned median 14.29 -> 14.30 cm,
same focal (2382.4 px). The looser tolerance changes the clouds by far less than the difference
between the calibration paths.

In that step run, the other large items were dense matching (96 s, 3 stereo pairs in
flight because the launcher got a 6.2-6.6 GB heap from the free RAM) and about 15-20 s of
per-step JVM starts and state reloads; a single call avoids the latter, and
`--stereo-pairs-in-flight 4` the former when about 8 GB of RAM is free.

## Verification

- Default path unchanged: GPS replay `gps_bundle.json` SHA identical to before.
- Quick mode is deterministic and step-split runs equal the single call: `inputs` with
  `--quick-calibration` then `calibrate`, `match`, `align`, `georeference` as separate processes
  gave the same `04_geometry/final/exterior.csv` (SHA `577b8456…`) as the single call.
- Self tests: the launcher regression checks that the help lists the new option.

Comparison tool: `C2C.java` (single-file scratch tool, not in this repository: `java C2C.java A.las B.las [samples]
[raw|icp|sim]`).
