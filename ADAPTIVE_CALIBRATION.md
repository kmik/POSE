# Parameter-wise camera calibration

Bootstrap and alignment now share `SharedCalibration`'s adaptive model selection. The runtime no
longer requires four photographs or three-view tracks before considering lens refinement.

## Acceptance policy

1. Select deterministic spatially balanced, disjoint 80/10/10 track folds. Two-view tracks qualify.
   Require 80 training tracks and 20 tracks in each validation fold, with a registered stereo baseline.
2. Add one requested parameter to the currently accepted model. Try focal length, k1, k2, principal
   point coordinates and tangential terms; revisit rejected terms before trying k3. There are at most
   two sweeps, with three initial focal starts. Explicit parameter subsets remain authoritative.
3. Jointly fit training points, movable poses and active lens parameters. Automatic fitting and
   supplied `prior` mode use priors; supplied `initial` mode retains its unpenalized fitting semantics. Log-focal and
   distortion prior sigmas use `--focal-sigma` and `--radial-sigma`; principal-point priors use 2% of
   source width/height. Changes must remain within four prior sigmas and pass the full-image
   nonfolding/physical checks.
4. Assess information without LM damping or lens priors. Eliminate point positions, then marginalize
   camera freedoms using a normalized eigendecomposition and pseudoinverse. This projects out the
   scale gauge without treating numerical regularization as information. Unobservable active lens
   combinations fail. Marginal standard deviations must pass the existing parameter-effect bounds;
   radial terms additionally require P90 radius coverage of at least 55% of the half-image diagonal.
5. Refit only the withheld points under the candidate cameras/lens. Both validation folds must improve
   by 0.5% and 0.002 processed pixels. For each fold, the reduction in squared residuals must also exceed
   `noise_sigma^2 * log(observation_count)`, where sigma is at least 0.35 processed pixels. This penalizes
   extra parameters that merely explain negligible residuals.
6. Accept the correction or restore the preceding lens, poses and structure exactly. On acceptance,
   refit all structure, remap feature coordinates through the original distorted pixel frame, and
   finish the fixed-intrinsics pose/point bundle when poses are movable.

The validation folds select the model; they are not independent accuracy checkpoints. Information is
local and conditional on parameters retained as fixed. Priors, parameter bounds and low reprojection
error cannot guarantee physical calibration or absolute scene accuracy from a weak stereo pair.

## Constraints and diagnostics

Supplied fixed intrinsics, `--no-self-calibration`, rectified input and fixed poses keep their existing
meaning. Fixed poses can support intrinsic refinement without moving the poses. Heterogeneous camera
groups retain their calibration. The dense observability workspace is bounded to 768 reduced variables
and a fraction of the processing memory budget; larger assessments retain the preceding model.

`01_calibration/refinement/result.json` reports bootstrap decisions.
`04_geometry/bundle/shared_calibration.json` reports alignment decisions. Each trial records the added
parameter, active parameter set, both validation errors, complexity test, data-only covariance and
correlations, information threshold, candidate calibration and acceptance/rejection reason. Detailed
residual maps remain available with `--diagnostics`. The reference interior and saved pipeline state
carry accepted calibration into dense matching. The cache engine fingerprint includes the new
observability implementation.

## Validation

`AdaptiveCalibrationRegression` exercises two-view radial recovery, rejection of pure-translation
focal degeneracy even with an extremely strong focal prior, disjoint folds, exact rollback of poses and
structure, anchor/baseline preservation, fixed-calibration controls and the complexity penalty.
`SharedCalibrationRegression` covers known and movable poses, full Brown calibration, selected
parameter subsets, priors, feature-coordinate invariance and propagation to dense processing.

The city comparison measures curvature on the same long-roof XY region after a seven-parameter
similarity registration to the provided Metashape cloud. No nonrigid registration or roof flattening
is used. Quantities are in the Metashape file's local units, not verified metres. Analysis scripts and
numeric results are in `../tmp/cloud_comparison/`.

On Windows, deeply nested full-suite image fixtures can exceed native OpenCV's path limit. Run
`sgm.SelfTest` with `-Dsgm.test.root=ABSOLUTE_SHORT_WORKSPACE_DIRECTORY` to shorten artifact paths.

The complete suite passed on Windows 11 with JDK 25 and OpenCV 4.9.0: **113,600 checks**,
including the new adaptive calibration cases. The final log is `target/adaptive-selftest-final2.log`.

## City pair result, 2026-11-08

The original run's matching/fusion settings were supplied explicitly because current source defaults
differ from that run's archived defaults. No filtering or flattening settings were changed to obtain
the improvement. A control with `--no-self-calibration` reproduces the original LAS XYZ coordinates
and RGB values exactly.

| Measurement | Original / control | Adaptive | Metashape reference |
|---|---:|---:|---:|
| Exported points | 220,943 | 269,299 | 2,553,620 |
| Sparse reprojection RMS, processed pixels | 0.361774 | 0.201194 | Not available |
| Roof bow over a 7.3-unit span | 0.188460 | 0.024552 | 0.013202 |
| Roof RMS after a robust quadratic fit | 0.020533 | 0.017563 | 0.015935 |

Bootstrap accepts k1 and k2; alignment subsequently accepts a further k1 correction. Focal length,
principal point, tangential coefficients and k3 retain their initialized values. Final source-raster
calibration has f=2121.446249 pixels, k1=-0.0954401904 and k2=0.0596105608.

The final validated output is
`output/city2-adaptive-matched/runs/20261108T172658133Z_4dcfa879/07_results/dense.las`.
The control is `output/city2-adaptive-control/runs/20261108T172750160Z_cd59193e`.
The plot and numerical measurements are `../tmp/cloud_comparison/adaptive_roof_comparison.png` and
`../tmp/cloud_comparison/adaptive_validation.json`. This is a shape comparison against another
reconstruction, not a survey accuracy measurement.

An additional supplied-fixed-pose two-image alignment check accepts k1/k2 while retaining the supplied
rotations and translations to numerical precision; see
`output/city2-fixedpose-calibration-check/runs/20261108T173033997Z_1104b84d`.
