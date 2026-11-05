# Noise investigation and repair — 2026-10-29

The user-reported noise was real. The first Java implementation could export a successfully reconstructed cloud while retaining severe stereo outliers. The original validation emphasized execution, point counts, and uncomplicated synthetic geometry; it did not exercise inconsistent multi-view evidence. That was insufficient.

The original output at `output/example-test/` has been preserved. The final corrected reconstruction is in **`output/example-test-fixed/`**. Source changes are confined to `sgm_java`.

## Observed failure

I read the user's LAS, sparse tracks, camera poses, and all three floating-point disparity maps, inspected the source photographs and disparity previews, and generated point-cloud projections. The input is the three photographs directly in the C++ repository's `example_data` directory.

| Measurement | Original output | Final corrected output |
|---|---:|---:|
| Exported points | 101,114 | 62,236 |
| Seed-frame Z minimum | 5.511 | 9.311 |
| Seed-frame Z median | 13.357 | 13.556 |
| Seed-frame Z p99 | 47.583 | 15.992 |
| Seed-frame Z maximum | **2,046.414** | **16.651** |
| Maximum nearest-sparse-track distance | 2,402.280 | 2.831 |
| Sparse-track Z range | 9.684–16.570 | unchanged |
| Sparse reprojection RMS | computed from the same poses/tracks | 0.465 pixels |

These coordinates are in arbitrary seed-camera units, not metres or elevation. Z points away from the seed camera. Sparse tracks are a useful geometric reference but are **not surveyed ground truth**, and sparse coverage is uneven. The reduction in point count is not a claim that every discarded point was noise: conservative corroboration also removes valid points visible in only two views.

The sparse reprojection p95 values are 1.056, 0.709, and 1.104 pixels for the three images. The camera CSV in the original run and in the first corrected run is byte-identical (SHA-256 checked). This isolates the major failure to dense matching/fusion rather than a changed camera solution. LAS binary round-trip checks also pass; the extreme values were real reconstructed coordinates, not an export scaling error. The physical camera calibration has not been independently surveyed.

The [before/after comparison](output/noise-debug/before-after.png) uses identical plotting windows in both columns and reports points outside each window. Full-range numeric statistics are in [the original diagnostics](output/noise-debug/before/statistics.txt) and [the final diagnostics](output/example-test-fixed/diagnostics/statistics.txt); the comparison does not silently hide outliers by deleting them from the measurements.

## Root causes and fixes

### 1. Fusion counted its own evidence

Previously, each stereo candidate was projected into a global depth buffer for both source cameras. The later consistency check looked up those buffers and accepted support from two cameras. An isolated wrong point could therefore write its own depth twice and pass the check. This was a logical validation bug, not simply an unfortunate threshold.

`DepthFusion` now keeps depth evidence **separate for each stereo pair**. The producing pair is excluded from corroboration. Another pair must agree in relative depth and in reprojection back into both source cameras. Support counts distinct camera IDs; rerunning the same two-camera pair does not create a third observation.

The default is three-camera support when at least three input images exist. With two input images, the program retains validated stereo and explicitly cannot claim independent third-view verification. Separate pair maps are populated from every accepted pixel; `--step` only samples export candidates, so exported sampling no longer creates avoidable holes in the evidence maps.

### 2. Very small disparities produced unstable enormous depths

Depth varies inversely with disparity. A false match at a fraction of a pixel can produce a point hundreds or thousands of units away. The old positive-depth and `maxDepth=10000` checks permitted this.

The corrected code rejects search-endpoint optima and triangulation angles below one degree by default. The angle threshold is geometric and independent of global coordinate scale. The original `--max-depth` default was not tightened to the sparse cloud's range, and no sparse-depth envelope or percentile clipping was used to obtain the corrected output.

### 3. Median filtering invalidated an earlier check

Left-right validation originally ran before median filtering. The median could change the disparity and therefore change the corresponding right-image pixel, but that new match was exported without another reciprocity check.

Reciprocity now runs again after the median. The default LR tolerance is 0.75 pixels. A regression explicitly constructs a center pixel that passes before median filtering and fails after its disparity changes.

### 4. Stereo acceptance lacked sufficient confidence checks

The original texture range and 3% uniqueness check allowed noisy islands and visually dissimilar patches.

The pipeline now applies zero-mean normalized cross correlation at the subpixel correspondence (7x7 window, default minimum 0.55) and removes small connected disparity components (default minimum 50 pixels, neighbor disparity difference at most 1). Flat photometric patches are not treated as confident correlations. Grayscale and RGB-distance census modes use the corresponding grayscale or per-channel color photometric checks.

The output remains allowed to have holes; no invented depths are filled into occluded or rejected areas. Correlation and speckle checks alone are not a substitute for independent geometric support.

### 5. Remapping validity was incompletely represented

The first implementation rectified a fresh all-valid image rather than propagating the image's actual undistortion validity. It checked only a matched center pixel, allowing a census/correlation neighborhood to include padded border pixels.

`ImageFrame` now creates a validity mask from the actual undistortion mapping. Both masks follow the same rectification mapping as the photographs, are thresholded to exclude partially supported samples, and are eroded to cover the full matching/photometric neighborhood plus a one-pixel bilinear sampling halo. True black scene content remains valid; black remapping padding does not.

### 6. The suggested search range was too narrow for the widest baseline

The earlier example used 96 disparity hypotheses. On pair 0–2, the original output had 13,599 accepted pixels with disparity at least 94. The corrected 128-disparity run finds valid disparities up to approximately 112. The narrow range could not cover the nearest canopy on that pair.

The final example uses **128**. It has no accepted values in the last two hypotheses of that search range. Search endpoints are rejected, and `stereo-quality.csv` distinguishes upper-boundary optima when the whole search interval was available. The program emits a range warning when those counts are substantial; ordinary invalid borders are not, by themselves, treated as proof of clipping.

## Controlled checks

### Fusion ablation on the photographs

During debugging, before adding the final extra bilinear-mask pixel, I held the camera CSV, calibration, resolution, 96-disparity search, and stereo confidence settings fixed, and changed only the minimum view support:

| Configuration | Points | Maximum Z |
|---|---:|---:|
| Original implementation | 101,114 | 2,046.414 |
| Corrected stereo checks, `--min-views 2` | 80,209 | **68.123** |
| Same stereo checks, independent `--min-views 3` | 56,074 | **16.651** |

The [two-view ablation diagnostics](output/noise-debug/ablation-two-view/diagnostics/statistics.txt) demonstrate why simply adding a stereo filter was insufficient. The final wider-range reconstruction recovers more valid canopy coverage while retaining the independent-view rule. No point count alone was used as a quality criterion.

### Known-depth false-surface regression

`AdversarialSceneTest` generates three cameras viewing a textured plane at Z=6. A second texture is injected into a patch in only the first two images, with a consistent disparity corresponding to a false plane at Z=3. This false surface is coherent in its producing stereo pair but contradicted by the third view.

The saved original compiled classes were run on exactly this fixture, not a rewritten approximation of the old filter. The final implementation was then run on the same deterministically generated input:

| Test | Original implementation | Final implementation |
|---|---:|---:|
| Clean control: valid plane points | 53,572 | 50,924 |
| Corrupted fixture: valid plane points | 44,343 | 41,070 |
| Corrupted fixture: points outside the true-plane tolerance | 1,885 | **0** |
| Corrupted fixture: points on the false Z=3 surface | **1,668** | **0** |

The final implementation retains approximately **92.6%** of the old run's valid plane points on the corrupted fixture, and **95.1%** on the clean control. This measures both noise rejection and valid-surface retention. It does not prove the same retention percentage on real forest imagery.

The regression fails if the phantom surface survives, if more than 1% of output points are outside the known plane tolerance, or if less than half of the clean-control valid surface survives. The measured outcome is substantially stronger than those bounds.

### Automated verification

`build.bat` passes **102 numerical and integration checks** on Windows x64 with JDK 25, compiling Java 25 bytecode. The added checks cover self-support, duplicate camera IDs, contradictory independent depth, legitimate three-view support, two-image behavior, triangulation angle, NCC, flat patches, post-median LR, speckles with surface preservation, search endpoints, unrelated textures, masks, and the end-to-end adversarial scene.

The previous calibration, signed/multiword/color census, metric stereo depths, vertical/reversed stereo, full SIFT/PnP reconstruction, bundle optimization, LAS encoding, and CLI tests continue to pass. Linux CI remains supplied but has not been executed in this Windows workspace.

### Secondary photographs

The `example_data/forest/temp` set also reconstructs successfully with the corrected implementation. Its 128-disparity run correctly emitted a clipping warning on its widest-baseline pair. Repeating that run with **192** hypotheses removed the warning and produced **82,404** points, with Z spanning **10.999–13.829** and sparse reprojection RMS **0.312 pixels**. The largest accepted disparity is approximately 148, inside the search range.

Results and statistics are in `output/forest-test-fixed/`. Calibration suitability for this secondary set is not independently established; this is an execution and internal-consistency check, not a surveyed accuracy result.

## Reproduce the repaired user example

From the original C++ repository root, in PowerShell:

```powershell
.\sgm_java\build.bat
.\sgm_java\photogrammetry.bat `
  -i .\example_data `
  -o .\sgm_java\output\example-test-fixed `
  --calibration .\calibration\resize_25\calib.result `
  --max-dimension 800 --disparities 128 --step 3 --cores 2 `
  --bundle-iterations 10 --overwrite
```

`--overwrite` here replaces only the newly generated corrected run's filenames. The original `example-test` directory is separate.

Open [the corrected LAS](output/example-test-fixed/dense.las) or [PLY](output/example-test-fixed/dense.ply). Inspect `report.txt`, `geometry-quality.csv`, `stereo-quality.csv`, `fusion-quality.txt`, and `run-settings.properties` beside them for the effective settings and rejection counts.

The final run had 83,889 sampled candidates, of which independent fusion rejected 21,653, leaving 62,236 exported points. Parallax rejection is counted before export sampling and therefore has a different denominator. Stereo-quality counters likewise refer to full disparity pixels, not sampled cloud points.

## Remaining practical limits

- The repaired cloud is materially cleaner, but no surveyed forest ground truth is available. Consistency tests do not establish absolute geometric accuracy or prove that every remaining point is correct.
- Three-view corroboration intentionally reduces coverage on occluded surfaces and small/moving branches. Lowering `--min-views` weakens that safeguard; it is not a universal way to recover trustworthy detail.
- Two photographs alone cannot disambiguate every mutually consistent stereo mistake. The stereo confidence checks still apply, but independent third-view evidence is unavailable.
- A fixed search range must cover each baseline and resolution. The new warning and diagnostics make an inadequate range visible rather than silently accepting its endpoint.
- This remains a functional Java reimplementation with the algorithm differences described in [PORTING.md](PORTING.md), not an established numerical replica of the original C++ program.

The OpenCV [SGM source](https://github.com/opencv/opencv/blob/4.9.0/modules/calib3d/src/stereosgbm.cpp) was consulted as a reference for confidence/speckle validation concepts. The implementation here continues to use the project's own Java census matcher and the independently tested fusion code.

## Near-ground "salt and pepper" points — 2026-11-05

**Symptom.** On smooth ground (city dataset, `--fusion-mode image`), isolated points float 0.2–1.5 m
above the surface, almost never below it. With `--ground-aware-fusion` the ground itself becomes
smooth, so the floaters stand out even more. (A first look suggested GS2 terraced the ground; that
was an artefact of plotting heights against the cell-wise DTM. Against a fitted line the GS2 sheet is
smooth.)

**Measured** (raw city cloud at `--quality high`, 89 m flying height, points over observed
ground): 7% of all points lie 0.25–1.5 m above the terrain, and 40–60% of those have at most one
neighbour within 10 cm (ground sheet: 2–10%). They have fewer supporting views (5.6 vs 9.8),
lower matching confidence and higher depth sigma than the ground.

**Cause.** Image fusion's visibility test rejects a point only when another camera's reliable
surface lies more than 3 x `--fusion-relative` x depth behind it: 1.33 m at 89 m. A depth that is
too near by less than that (an ambiguous pixel keeps the nearest pair depth; a mismatched patch) is
"confirmed" by the ground itself within the relative agreement tolerance (0.44 m) and passes. That
gap is exactly the 0.2–1.5 m band, and it is one-sided: too-far depths are hidden behind the
surface and fail the other checks.

**Fixes** (image fusion, both default on, both off gives byte-identical output to before):

- `--fusion-free-space on|strict|off`: a free-space vote at an image-scale margin. A camera *sees
  through* a point when its reliable surface lies more than three pixel footprints (about 0.3 m
  here) behind it; the point is rejected when at least two cameras see through it and more see
  through it than confirm it. `strict` also requires confirmations within three footprints.
- `--fusion-spike-filter on|off`: single-pixel depth spikes in each camera's merged map (further
  from the 5 x 5 median than max(4 x robust spread, 2 footprints), not supported by two neighbours
  at their own depth, so thin poles and branches stay).

**Results** (same city cloud, points by height above the DTM; isolated = at most one neighbour
within 15 cm):

| Height band | raw | on (default) | strict |
| --- | ---: | ---: | ---: |
| 0.25–0.5 m | 253,965 (16% isolated) | 216,341 (13%) | 195,174 (11%) |
| 0.5–1 m | 143,779 (24%) | 113,268 (18%) | 104,774 (16%) |
| 1–1.5 m | 46,693 (17%) | 42,126 (14%) | 40,715 (13%) |
| ground ±0.1 m | 2,710,485 | 2,735,717 | 2,722,365 |
| 6–12 m (roofs, crowns) | 875,737 | 867,675 | 847,990 |
| above 12 m | 285,426 | 277,041 | 267,613 |

The default removes about a fifth of the near-ground floaters and leaves roofs and crowns within
1–3%. `strict` removes more floaters but also 3–6% of roof and crown points, which are not
isolated (their isolated share stays at 5%), so they are mostly real edge points. Rejected
alternatives: requiring two pairs for the nearer cluster of an ambiguous pixel (shifted roof and
crown points between height bands, did not reduce floaters); removing small depth islands of low
support (islands have the same support as their surroundings); an isolation filter on the final
cloud (removed sparse real ground).

**Not fixed.** Floaters within about 0.3 m of the ground are inside the depth noise of a single
pair at this baseline-to-height ratio; tightening the relative agreement (`--fusion-relative`,
default 0.005) would remove them but changes the whole cloud. The isolation diagnostics:
`NoiseProbe`, `NoiseZoom`, `HeightBands` (scratch tools, not in the repository).
