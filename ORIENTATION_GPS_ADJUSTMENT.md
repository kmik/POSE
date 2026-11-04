# GPS-weighted orientation adjustment (G1–G5)

All of this runs by default after the GPS similarity fit (`Georeference.align` -> `GpsAdjustment.run`).
Report: `04_geometry/georeference/gps_bundle.json`. Without usable GPS nothing changes.

## Update: convergence-based iterations (later on 3 Nov)

Without `--bundle-iterations`, every bundle fit now iterates until two successive iterations each
gain less than 1e-6 (relative), up to 100 (`BundleAdjuster.AUTO_LIMIT`). The plain forest_10
bundle converges in 15–17 iterations, where the old default stopped at 8. At convergence the
f, k1, k2 lens domed the block again (height RMS 1.16 m with the 0.5 m height prior), so the
defaults changed to the variant that stayed flat and fitted best:

| Converged variant (forest_10 replay) | GPS RMS | Height RMS | Reprojection | f | Time |
| --- | --- | --- | --- | --- | --- |
| f,k1,k2, height σ 0.5 m | 1.583 m | 1.16 m | 0.515 px | 4701 | 191 s |
| f,k1,k2, height σ 0.2 m | 1.340 m | 0.46 m | 0.518 px | 4913 | 320 s |
| all 8 lens terms, height σ 0.5 m | 1.529 m | 1.17 m | 0.514 px | 4600 | 248 s |
| **all 8 lens terms, height σ 0.2 m (new default)** | **1.183 m** | **0.46 m** | **0.517 px** | **4776** | **176 s** |

Defaults now: `--gps-intrinsics f,k1,k2,k3,cx,cy,p1,p2`, `--gps-height-sigma auto` = 0.2 m with
DJI barometric heights. The tables below were measured with a fixed 10-iteration budget and
f,k1,k2.

## Model

One bundle adjustment in the projected frame (Levenberg–Marquardt, Schur), with a shared block that
holds, besides the lens:

| Index | Parameter | Option (default) | Prior |
| --- | --- | --- | --- |
| 0–7 | f, k1, k2 (cx, cy, p1, p2, k3 optional) | `--gps-intrinsics f,k1,k2` | focal: `--gps-focal-prior auto`, `--gps-focal-sigma 0.03` (log) |
| 8 | rolling-shutter readout (s, signed) | `--rolling-shutter estimate` | 0 ± 0.1 s |
| 9 | GPS time offset dt (s) | `--gps-time-offset estimate` | 0 ± 1 s |
| 10–12 | GPS lever arm L (camera x right, y down, z viewing; m) | `--gps-lever-arm estimate` | 0 ± 0.5 m |

- **Camera priors:** |C + R^T L + V dt − GPS| with sigma `--gps-sigma` (2 m) horizontally and
  `--gps-height-sigma` vertically (auto: 0.5 m when DJI XMP barometric relative altitudes exist,
  else the horizontal sigma). Without the tight height prior, radial distortion domed the
  forest_10 block (height RMS 0.5 -> 1.6 m).
- **Rolling shutter:** observed row y is exposed at s = readout·(y/h − 1/2); the camera is then at
  C + V s. Translation only. Pose Jacobians are unchanged, because the shifted camera-frame point
  moves with pose updates exactly like the plain one.
- **Velocities V:** DJI XMP `FlightXSpeed/YSpeed/ZSpeed` (north/east/down, m/s), rotated into the
  projected grid and accepted only if they point along the camera track (median cosine > 0.8;
  forest_10: 0.9999). Fallback: camera-centre differences over capture times (whole-second EXIF
  times smoothed by a local line fit), then velocities already attached to the cameras.
- **Focal prior (auto):** XMP `CalibratedFocalLength` (source pixels) if it agrees within 10% with
  the resolution-independent EXIF 35 mm value, else the 35 mm value.
- **Solve:** `--gps-adjustment-rounds` (3) rounds of: exact similarity re-fit of the block to the
  GPS-implied centres (the seven directions images cannot see, along which the bundle otherwise
  crawls), a linear antenna fit as the start value, then the bundle (20 iterations in round 1,
  `--bundle-iterations` after). If the first fit ends more than 2σ from the focal prior, a second
  start from the prior (structure re-expressed at the new focal) runs and the lower objective wins.
- **Guard:** reverts to the poses-only adjustment if the result is unphysical, reprojection RMS
  grows more than 15%, or GPS RMS grows more than 1.5× + 0.5 m. Any refitted lens is published as
  the final calibration (camera.txt, constraints.json, dense inputs).

## forest_10 (replay of run 2af2c8b5's exported sparse geometry)

`GpsAdjustmentReplay RUN IMAGES OUT [g1|time|rs|lens|full]`, 10 bundle iterations:

| Variant | GPS RMS (m) | Reprojection (px) | f (px) | dt (s) | Readout (ms) | Median sparse z (m) | Time |
| --- | --- | --- | --- | --- | --- | --- | --- |
| similarity only (before) | 1.684 | 0.535 | 5606 | – | – | 78.5 | – |
| g1: poses only | 1.861 | 0.535 | 5606 | – | – | 78.9 | 6 s |
| time: + dt, lever arm | 1.523 | 0.535 | 5606 | −0.18 | – | 79.0 | 17 s |
| rs: + readout only | 1.752 | 0.522 | 5606 | – | 40.5 | 78.2 | 35 s |
| **full (defaults)** | **1.270** | **0.517** | **4743** | −0.19 | **44.3** | **89.9** | 83 s |

- **Focal length:** the bootstrap value 5606 px was about 20% too long. DJI's XMP says 4545 px
  (15 mm / 3.3 µm). The refit gives 4743 px with k1 −0.037, k2 0.007. Starting from 5606 px and
  from 4545 px ends at 5163 vs 4743 px, only 123 objective units apart: the images hardly separate
  focal length from flying height, so the prior decides. GPS altitudes fix the cameras, not the
  ground: a 15% focal change moved the median sparse height by +11 m. The DJI takeoff ground is
  92.3 m (AbsoluteAltitude 192.2 − RelativeAltitude 99.9); before the refit the median sparse
  point lay 14 m below it.
- **GPS timing:** the mean along-track residual (camera ahead of its GPS fix) was +1.0 m before the
  fit and about 0 after. dt and the along-track lever arm are nearly interchangeable at constant
  speed; the vertical lever arm trades against the block height. Only the combined correction is
  meaningful.
- **Remaining systematic:** an affine fit of camera centres to GPS shows the block about 2% wider
  across track than GPS (E 0.978, N 1.007; 4–5% with the old lens). Not explained yet (G6).

## Synthetic control

`GpsAdjustmentRegression` (in SelfTest): two opposite nadir lines at 5 and 8 m/s, 0–15 m relief,
0.3 px image noise, 3 cm GPS noise, true dt −0.2 s, L (0.1, −0.25, 0.3) m, readout 40 ms, start
focal 6% wrong. Recovered: readout 41 ms, focal 999 px (prior 1000), cross-track lever arm, and the
identifiable along-track combination 13·dt − 2·Ly. The individual dt / Ly / Lz are not
identifiable in that geometry and are not tested. Off and fixed modes leave parameters exactly as
given.

## Limits

- Dense matching uses the mid-exposure pinhole poses (rolling shutter not applied in
  rectification; about ±0.1 m at 4.4 m/s; G2b).
- Rolling shutter is translational; fast yaw in turns is not modelled.
- Lens refit and antenna model need one shared calibration (skipped for mixed cameras or a fixed
  supplied calibration; the report says why).
- Cost: about 80 s extra in stage 04 on the 89-image forest_10 at 1600 px alignment (two focal
  starts). `--gps-intrinsics none` or `--gps-bundle off` restores the earlier speed.

## Single flight line (4 Nov)

GPS centres on one line (second principal spread < 3% of the first) cannot fix the block's roll
about the line, so the similarity fit used to fail ("requires non-collinear camera positions")
and the output stayed in the local frame, which also stopped `--classify-ground`. Now
`Georeference.lineFit` takes scale, line direction and translation from GPS and turns the roll
so the mean camera viewing direction points straight down; outliers beyond `--gps-threshold`
are dropped iteratively. `transform.json` records `"method": "single_line_with_downward_view"`.
Tilt across the line is assumed, not measured. forest_4 (32 images, one line): GPS fit RMS
1.59 m (30/32 inliers), GPS-weighted adjustment 1.40 m; trunks vertical in cross-sections.
