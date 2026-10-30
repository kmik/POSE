# SGM defaults and thin structures

The revised defaults are **P1=4, P2=24** in 5×5 Census cost units, with image-edge-adaptive P2. The previous defaults were fixed 8/32. This change also addresses two filtering steps that could erase a fence even after stereo matching recovered it.

## What caused the problem in controlled tests

- Constant smoothness penalties favored the broad background over narrow foreground posts. Reducing penalties alone recovered more foreground disparities, but many were subsequently lost.
- The ordinary 3×3 median could replace a one-pixel foreground line with its majority background neighborhood. Subsequent validation could then retain background depth or reject the changed match.
- Four-connected speckle grouping treated a one-pixel diagonal wire as separate components, even when it formed a continuous visible line.

The tests render foreground posts/rails and an occluding background from known ray-plane intersections. They reproduce these failures without camera-estimation uncertainty. They do not establish the cause of every real fence failure; unresolved texture, occlusion, calibration, resolution and sampling can also matter.

## Current controls

| Argument | Default | Behavior |
|---|---:|---|
| `--p1` | 4 | Penalty for a one-hypothesis change |
| `--p2` | 24 | Base penalty for larger disparity changes |
| `--sgm-edge-scale` | 32 | Intensity scale for reducing P2 across image edges; 0 disables adaptation |
| `--median-mode` | edge-preserving | Ordinary median on smooth support; restrict support when it would jump layers |
| `--median-max-change` | 0.5 px | Median-shift trigger and disparity tolerance for restricted support |
| `--median-edge-threshold` | 24 | Maximum grayscale difference in restricted median support; 0 disables the intensity gate |
| `--speckle-connectivity` | 8 | Include diagonal neighbors; 4 restores axial-only connectivity |

P1/P2 still scale by `(census_window²-1)/24`. Every path computes:

```text
effective_P2 = max(scaled_P1 + 1,
                   floor(scaled_P2 * edge_scale /
                         (edge_scale + abs(I_current - I_previous))))
```

For a 5×5 window, P1=4/P2=24 gives an effective P2 of 24 across identical intensities and 5 across a 100-level grayscale transition. Higher `--sgm-edge-scale` retains more smoothing over texture; lower values weaken it more quickly at edges. This is computed identically for CPU, resident GPU and streamed GPU, using the correct image and predecessor across strip boundaries.

The edge-preserving median first computes the ordinary valid-neighbor median. If its movement from the center is less than 0.5 pixels, it retains that denoising result. Otherwise it takes a median restricted to disparities within 0.5 pixels of the center and grayscale values within 24 levels. This prevents a thin foreground center from borrowing an unrelated surrounding depth. It never fills an invalid center. `--ws_mFilter 1` disables median processing entirely.

Eight-connected speckle grouping keeps connected diagonal structures. The existing component-size threshold and neighboring-disparity agreement still apply, including mild/moderate/aggressive filtering presets. It does not join gaps or synthesize points. Left/right reciprocity, ZNCC thresholds, masks, search-boundary rejection, triangulation and multi-view fusion checks remain active.

## Validation

The fence comparisons aggregate three deterministic textures and post widths of 1, 2, 3 and 5 processed pixels, with horizontal and diagonal rails. Only mutually visible background rays are scored. Each case retains stage-by-stage counts before/after left-right checks, median, correlation and speckle removal.

| Disparity geometry | Correct fence pixels: previous → revised | Incorrect fence/background pixels: previous → revised | Valid background retained relative to previous |
|---|---:|---:|---:|
| Foreground 24, background 8 | 8,613 → 17,591 | 6,192 → 1,378 | 99.91% |
| Foreground 40, background 39 | 16,049 → 19,070 | 9,451 → 5,919 | 99.73% |

The one-pixel jump tests use a 0.35-pixel depth-label tolerance; larger-jump tests use 0.6 pixels. These are controlled disparity results, not a promised real-project accuracy improvement. Some one-pixel structures still have insufficient reliable support.

Additional flat/sloping textured-plane controls use sensor noise sigma 0, 2 and 4 grayscale levels. RMS disparity errors remain below 0.22 pixels in those revised runs, with over 99% of previous valid coverage. Some noisy sloping cases show slightly higher errors, and flat error spread can increase while its systematic bias decreases. The detail/smoothness tradeoff remains real.

A fixed-camera city replay also completed. It retained 42,345 points versus 55,843 with the former defaults. An approximately planar, unsurveyed patch changed median local p90-p10 spread from 0.143 to 0.197 m while its median offset from a separately fitted plane decreased. Different retained point populations and existing camera errors prevent treating this as surveyed accuracy. These defaults prioritize recoverable discontinuities; they are not a universal cure for surface roughness. The user's specific fence still needs a known location for direct assessment.

Tests: `src/test/java/sgm/FenceRegression.java` and `RegularizationSurfaceRegression.java`. Comparison JSON and representative source/truth/old/new disparity previews are retained in the fence test directory. In the previews, background is blue, foreground red, intermediate disparities purple, and invalid pixels black. Surface controls are written to `target/fence-surface-controls-p24-e32.json`.

## Running and tuning

Existing commands use the revised defaults automatically after rebuilding. Explicit `--p1` and `--p2` continue to override the bases. For a comparison using the previous regularization/filtering behavior:

```text
--p1 8 --p2 32 --sgm-edge-scale 0 --median-mode classic --speckle-connectivity 4
```

For fences, also inspect **processed resolution** and **candidate sampling**. `--quality high` or `ultra` can retain more source pixels; `--step 1` exports every accepted candidate instead of sampling a grid with default step 2, which can miss a narrow structure. These are independent controls and increase work/output. A fence blurred below the source/processed pixel scale cannot be recovered reliably by reducing penalties.

Example, from `sgm_java`:

```powershell
.\photogrammetry.bat -i photos -o output/fence-project --quality high --step 1
```

Linux: `bash photogrammetry.sh` with the same arguments. For less smoothing, try smaller P1/P2 or edge scale; for calmer surfaces, increase them. Assess both the fence and a smooth region when tuning. New settings and implementation changes invalidate incompatible stereo caches.

References: [DLR SGM algorithm notes](https://www.dlr.de/de/rm/forschung/expertise/archiv/vision/reconstruction/stereo-algorithms), [OpenCV's penalty definitions](https://docs.opencv.org/4.x/d2/d85/classcv_1_1StereoSGBM.html). OpenCV's sample penalty formula uses a different matching cost and is not a calibrated default for this program's Census costs.
