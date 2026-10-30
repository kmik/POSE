# Kermit reconstruction investigation

## Observed failure

The eleven 640×480 photographs in `example_data/kermit` contain a stationary scene with a Kermit doll, chair, books and surrounding objects. The investigation used `--neighbors 10 --ratio 0.8 --max-dimension 800`, moderate filtering and no GPU.

Automatic initialization estimated a shared focal of 592.634675 pixels. The guarded distortion fit was rejected for insufficient radial coverage, so these runs retained the initialized pinhole model. This is an image-derived estimate, not a measured calibration. Nine images registered; `kermit004.jpg` and `kermit006.jpg` did not. Simply increasing bundle refinement or trying several viable seed pairs did not recover those two images.

One concrete dense-stage problem was excessive rectification magnification. For pair 0000/0001, OpenCV's border-free `alpha=0` rectification changed a native focal of 592.635 pixels to 5289.877 pixels while retaining a 640×480 output raster: approximately 8.93× magnification. This retains a narrow crop, magnifies alignment errors and discards much of the useful scene. Some other convergent pairs exhibited similarly extreme crops. These are camera/rectification effects, not evidence that the SGM path recurrence itself is wrong.

## Changes

- Seed ranking includes triangulation angle and trials of registration support across up to eight seed candidates. Trials are deterministic and exported.
- PnP numerical failures exclude that registration candidate; finite pose, positive depth and refined reprojection checks precede acceptance.
- Sparse cleanup preserves camera support and graph connectivity, repeats bundle adjustment, then retries registration.
- Unregistered images are reported and excluded from subsequent dense processing.
- Default `--rectification-alpha auto` retains `alpha=0` for ordinary pairs, but recomputes with `alpha=1` when rectified/native focal magnification exceeds 1.5×. Invalid image borders remain masked. Explicit `0` and `1` settings are available for comparison.

## End-to-end results

| Local output | Rectification | Registered | Points |
| --- | --- | --- | --- |
| `output/kermit-investigation` | Original border-free crop | 9/11 | 2,450 |
| `output/kermit-multiview-seed` | Border-free crop, seed trials and cleanup | 9/11 | 2,034 |
| `output/kermit-field-of-view` | Automatic field-of-view safeguard | 9/11 | 18,978 |

The last run took approximately 337 seconds, produced 56,785 candidates, retained 18,978 points after fusion and exported 793 sparse points. Its orthographic cloud projections show coherent book/chair/scene structure with substantially increased coverage. The median distance to a sparse track is 0.03924 local units and the p95 is 0.12269; sparse tracks are reconstruction evidence, not independent ground truth. End-to-end runs can differ in refinement and registration details, so their point-count differences alone do not isolate the rectification change.

Reproduce the corrected configuration from the repository root:

```powershell
.\sgm_java\photogrammetry.bat -i example_data/kermit -o sgm_java/output/kermit-test `
  --neighbors 10 --ratio 0.8 --max-dimension 800 --cores 2
```

For the recorded comparison, the final run supplied the earlier automatically estimated calibration explicitly via `--calibration sgm_java/output/kermit-investigation/camera.txt`. This keeps that estimated model fixed. Omitting it reruns automatic calibration and can select a different initialization or accepted model as implementation changes.

Local evidence: `target/kermit-field-of-view.log`, `target/kermit-field-of-view-diagnostics/cloud-projections.png`, `statistics.txt`, and `target/kermit-field-of-view-verification.json`. The diagnostics utility's printed disparity-boundary summary assumes 128 hypotheses unless a count is supplied; do not interpret it as a valid boundary-rejection measure across these automatically varying pair ranges. Actual per-pair ranges and rejected-pixel masks are in the project.

All **3,745** cataloged artifacts passed independent file-size/SHA-256 verification, and all **939** JSON files parsed successfully.

## Controlled rectification comparison

A second comparison copies only the nine registered photographs to `target/kermit-fixed-geometry/images`, supplies the same exported `cameras.csv` and `camera.txt`, and uses all sequential pairs (`--neighbors 10`). Both runs fix supplied poses/calibration, so there is no registration or bundle adjustment difference. The sole CLI setting changed is `--rectification-alpha auto` versus `0`; automatically selected disparity ranges can consequently change with the rectification.

| Output | Alpha | Points | Time | Pair 0000/0001 valid disparity pixels |
| --- | --- | --- | --- | --- |
| `output/kermit-controlled-crop` | `0` | 2,034 | 258 s | 0 |
| `output/kermit-controlled-auto` | `auto` | 17,445 | 342 s | 26,841 |

The two camera CSV files have identical SHA-256 `ec5c7c3aaa59af8d3787e5fde4286d9fb63bd752332ff011bf9e14f2ff07971c`. Applied calibration properties are identical; their comment timestamps differ. This isolates a substantial coverage improvement from the rectification policy under the same supplied geometry. The automatically preserved field also increases stereo work, so it is not a speed improvement. The cloud remains incomplete, and these counts do not establish point accuracy.

```powershell
.\sgm_java\photogrammetry.bat -i sgm_java/target/kermit-fixed-geometry/images `
  -o sgm_java/output/kermit-controlled-auto `
  --calibration sgm_java/output/kermit-field-of-view/camera.txt `
  --poses sgm_java/output/kermit-field-of-view/cameras.csv `
  --neighbors 10 --rectification-alpha auto --cores 1
```

Use a different output directory and `--rectification-alpha 0` for the crop comparison. Diagnostics and projections are under `target/kermit-controlled-auto-diagnostics` and `target/kermit-controlled-crop-diagnostics`.

## Remaining limits

This fixes a demonstrated loss of stereo coverage. It does not establish surveyed accuracy, guarantee complete doll/surface coverage, or solve the two remaining registrations. Low texture, occlusion, convergence, depth geometry and the unmeasured camera model remain relevant. Compare exported calibration decisions, registration diagnostics and per-pair disparities before loosening filtering. A denser cloud alone is not proof of better geometry.
