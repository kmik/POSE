# Automatic disparity intervals for each pair

Without `--disparities` or `--min-disparity`, the dense pipeline now estimates **both ends** of each rectified pair's search. It uses verified alignment keypoints, sparse structure and rectified preview matches, checks spatial coverage, pads the interval, and validates it with a small whole-domain search. No image-resolution or filtering setting is changed.

## Controls

| Arguments | Behavior |
| --- | --- |
| Neither option | Automatically estimate a nonnegative lower bound and an upper bound for each pair; expand either endpoint when necessary. |
| `--min-disparity 0` | Pin the lower bound to zero and estimate/expand only the upper bound. Useful for a broad reference comparison. |
| `--min-disparity N` | Pin the lower bound to N, including signed values. Estimate/expand the upper bound. |
| `--disparities N` | Use exactly N hypotheses, beginning at zero unless a minimum is supplied. Bypass automatic estimation/validation/expansion. |
| Both arguments | Use the exact interval `minimum .. minimum + count - 1`. |

Existing commands omitting both options automatically use the new behavior on their next run. The current running JVM retains its already loaded implementation. An explicit minimum is never silently overridden; endpoint disparities themselves remain subject to the existing endpoint-rejection rule.

## Evidence and confidence

Verified track observations are transformed from processed undistorted camera coordinates through each pair's rectification matrices. Vertical stereo uses transposed matching coordinates. When alignment and dense sizes differ, keypoint IDs and coordinates are carried forward with the actual per-axis scale; descriptors are not duplicated. Shared sparse points are projected into the same matching frame. Invalid footprints, inconsistent epipolar rows and excessive point reprojection errors are excluded.

Reciprocal matches on the bounded 640-pixel preview remain a fallback/additional source. A narrow interval normally requires at least twelve samples, four occupied cells in a 4×3 coverage grid, 20% vertical coverage, and 20% horizontal coverage **relative to feasible overlap**. The latter accommodates high-parallax pairs whose shared strip is narrow. These checks are confidence heuristics, not an accuracy certificate.

The central 1st–99th percentile envelope rejects isolated mismatches. Spatially supported tails and sparse points supported by three or more registered views preserve credible minority foreground/background depths. Padding is the largest of sixteen matching pixels, four times the reprojection tolerance, three scaled preview pixels, and 2% of the largest observed disparity magnitude. Reprojection uncertainty is scaled from alignment pixels to the actual dense resolution before computing the margin. Both backends support arbitrary hypothesis counts; the estimator does not round a fitting interval up to an unnecessary multiple of sixteen.

Weak or clustered feature evidence initially keeps a broader zero-based search. A well-distributed, full-resolution-validated dense fallback may subsequently establish both bounds. Otherwise the broader fallback remains. The supported interval is limited to 4,096 hypotheses; a larger inferred span fails explicitly rather than silently dropping part of the estimate.

## Whole-domain guard

A grayscale SGM probe covers the allowed disparity domain at a maximum long side of 128 pixels, reduced further if needed to fit its bounded RAM estimate. It is not exported as a full diagnostic volume. Potential depths outside the keypoint interval are checked on the original processed images using ZNCC, a distinct correlation peak (gap 0.03 excluding immediate neighbours), valid masks, and reciprocal agreement within one pixel. The correlation threshold is at least 0.5 or the selected stronger threshold. Weak-feature fallback cases sample the whole coarse depth distribution rather than only its exterior.

Supported confirmations can widen the interval. At least 24 confirmed samples plus adequate combined spatial coverage are needed for a weak-feature case to acquire an automatic lower bound from this fallback. The small auxiliary probe can use CPU for tiny workloads even when the main dense pass is required to use a GPU.

This reduces the risk of excluding surfaces absent from the keypoints. It does not prove that every thin, occluded or weakly textured surface is represented. Pin a broad minimum or exact range when comparing uncertain results. Successful narrowing also need not produce bit-identical results to a broader SGM search because the available path hypotheses change.

## Adaptive expansion and resource limits

CPU and GPU matching count both lower and upper endpoint selections where the **complete interval is searchable**. Counts at masked/image boundaries do not masquerade as range truncation. Sustained pressure means more than 2% of those searchable pixels, with minimum hit counts to suppress isolated noise. Pyramid matching also carries the coarse global pressure signal forward.

Automatic matching expands the pressured side by approximately half the current count, with a minimum sixteen-hypothesis increment. Persistent pressure can trigger a whole-domain fallback; at most four retries follow the initial attempt. A pinned minimum remains fixed. Initial intervals exceeding resource limits fail clearly. Later expansion blocked by a resource/retry limit is reported in the pair diagnostics and warning; intervals are not clipped merely to force GPU use. In auto compute mode a genuinely wide interval may still require CPU processing.

## Project artifacts and caches

Every profile retains `05_stereo/pair_.../disparity_range.json`. It records:

- Initial and effective minimum, maximum and hypothesis count.
- Source sample counts, quantiles, credible envelope, padding and spatial confidence.
- Coarse-validation size, valid pixels and full-resolution confirmation settings/counts.
- Each attempted interval, lower/upper hits, searchable denominator, pressure fractions and stop reason.
- CPU/GPU memory estimates and the requested compute mode.

Debug storage additionally writes `range_samples.csv.gz` with each used disparity, rectified left coordinates, evidence source and three-view anchor flag. Standard storage retains only its final disparity raster, as before. Stereo caches include both bounds and the added counters; implementation fingerprints invalidate older incompatible entries. Existing project history is preserved.

## Validation and Sheffield Park replays

The complete Windows/JDK 25 suite passes **514 checks**, including 58 new range checks (`target/pair-range-final-validation.log`, fixtures `target/self test 2532011139733187058`). New coverage includes minority layers/rare three-view anchors, spatially clustered and insufficient data, narrow-overlap confidence, signed/pinned/explicit ranges, lower/upper/both-side expansion, border-pressure suppression, CPU/GPU boundary parity, independent OpenCV rectification, alignment-to-dense coordinate and uncertainty scaling, missing-background recovery by the whole-domain guard, known metric depth/coverage, resumed positive-minimum stereo and reuse of real verified tracks at a different dense resolution. An additional AMD run passes the 57 checks preceding the final uncertainty-scaling assertion (`target/pair-range-amd-validation.log`); the existing 26 GPU numerical comparisons also pass on NVIDIA (`target/pair-range-gpu-parity.log`).

Two image-pair replays reuse the original applied calibration and supplied camera poses, with zero pose-adjustment iterations, medium resolution, mild filtering and step two. Comparisons are to the previous broad disparity maps, not surveyed truth:

| Pair | Earlier search | New search | New backend |
| --- | --- | --- | --- |
| 6–9 | 0–607 (608 hypotheses) | 465–513 (49 hypotheses) | GPU |
| 6–10 | 0–749 (750 hypotheses) | 0–708 (709 hypotheses) | GPU |

Pair 6–9 uses 183 verified keypoint samples, 184 sparse-geometry samples and 73 preview samples. The coarse guard adds one isolated external suggestion, which does not overcome the spatial-support requirement. New matching retains 223,602 of the prior 224,416 valid pixels (99.64%), adds 1,363 and loses 814; the common-pixel median difference is zero and p99 is 0.05771 disparity pixels. Hypothesis count is reduced by approximately 92%.

Pair 6–10 has only two usable preview matches in this two-image replay, so its well-distributed, full-resolution-validated fallback remains conservative and retains a low-disparity tail. Removing unused upper padding and unnecessary count rounding fits the device's approximately 709-hypothesis allocation limit. Matching completes in 1.37 seconds on the GPU. It retains 39,146 of 39,167 previous valid pixels (99.95%), adds 32 and loses 21; all common pixels agree within one disparity pixel and 39,143 agree within 0.1 pixels.

Logs and comparisons: `target/park-range-bounded-6-9.log`, `target/park-range-bounded-6-10.log`, and `target/pair-range-park-6-9-comparison.json` / `target/pair-range-park-6-10-comparison.json`. Both replays pass all 62 artifact size/SHA-256 checks and 22 JSON parses per run (`target/pair-range-replay-catalog-verification.json`). The full block's multi-view evidence can differ from these isolated replays. No claim is made that every pair will narrow or remain on GPU.
