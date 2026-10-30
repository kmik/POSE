# Workflow controls: TODO items 1–11

The workflow now supports optional OpenCL GPU dense matching. Current defaults are standard storage, opt-in stage caching and automatic GPU selection; see [STORAGE_GPU.md](STORAGE_GPU.md). Exhaustive audit artifacts described below require `--storage debug`. Run `photogrammetry.bat -h` on Windows or `bash photogrammetry.sh -h` on Linux for every argument. Examples below start in the repository root; replace the launcher and paths for Linux.

## 1–2. Calibration and pose constraints

Supplied inputs are **fixed by default**, independently:

| Control | Values | Meaning |
| --- | --- | --- |
| `--intrinsics-mode` | `fixed`, `prior`, `initial` | Preserve supplied calibration, refine it with uncertainty penalties, or use it as an unpenalized initialization |
| `--extrinsics-mode` | `fixed`, `prior`, `initial` | Preserve supplied poses, refine them with uncertainty penalties, or use them as an unpenalized initialization |
| `--refine-intrinsics` | A comma-separated subset of `f,k1,k2` | Default `f,k1,k2`; focal scaling preserves the supplied fx/fy ratio |

Principal point, skew convention, tangential distortion and k3 are not additional free parameters. Supplied distortion terms are preserved. Intrinsic fitting happens before final image undistortion; the final undistorted-image bundle solve keeps the accepted intrinsics fixed. Selected lens parameters are shared across the camera collection. Automatic calibration also respects the selected subset.

Priors are Gaussian standard deviations, rather than confidence percentages or hard bounds:

| Argument | Default | Units |
| --- | --- | --- |
| `--focal-sigma` | `0.05` | Relative focal scale, approximately 5% |
| `--radial-sigma` | `0.05` | Normalized k1/k2 coefficients |
| `--position-sigma` | `10` | Supplied pose-coordinate units |
| `--rotation-sigma` | `10` | Degrees |

Smaller values give stronger penalties. For metre-based, accurate camera positions, `--position-sigma 0.05` represents a five-centimetre assumption. These defaults are deliberately weak and cannot infer the precision of an arbitrary pose file. Pose penalties use camera centers and rotation-log differences, rather than applying positional accuracy directly to world-to-camera translation coefficients. Pose priors anchor the absolute frame; image-only/unpenalized geometry uses a first-pose and baseline gauge.

```powershell
.\sgm_java\photogrammetry.bat -i D:/photos -o D:/project `
  --calibration D:/camera.txt --intrinsics-mode prior --refine-intrinsics f,k1,k2 `
  --focal-sigma 0.02 --radial-sigma 0.03 `
  --poses D:/poses.csv --extrinsics-mode prior --position-sigma 0.05 --rotation-sigma 1
```

Refinement still requires observable geometry, sufficient matches and the existing held-out/physical lens acceptance checks. A requested fit can be rejected, retaining the initialization; read the calibration report. Calibration uses a bounded bootstrap (up to twelve images and 1,800 tracks), whereas the final pose/point bundle uses the full registered collection. Supplied pose refinement requires at least eight triangulated tracks. Every input image must have a row in a supplied pose file.

Inspect `01_calibration/constraints.json`, calibration proposals/acceptance reports, and `04_geometry/pose_constraints.json` for applied settings and pose changes. Original input files remain archived with provenance.

## 3. Separate alignment and dense resolution

`--quality` and `--max-dimension` control dense resolution. `--alignment-quality same|custom|ultra|high|medium|low` defaults to `same`. `--alignment-max-dimension N` supplies an independent cap; with `same`, specifying this cap selects custom alignment sizing. An explicit alignment preset uses its own size rather than inheriting an explicit dense cap. Neither stage upsamples.

```powershell
.\sgm_java\photogrammetry.bat -i D:/photos -o D:/project `
  --alignment-quality high --alignment-max-dimension 2400 --quality low --estimate-memory
```

Remove `--estimate-memory` to reconstruct. Keypoint coordinates, matches and sparse geometry refer to the alignment rasters in `00_inputs/`. If dense dimensions differ, `00_inputs/dense/` contains the separately scaled rasters/intrinsics. Intrinsics scale using actual decoded dimensions. Planning records both sizes, full-volume and disk-tiled SGM estimates, retained depth evidence and approximate native allocations. The plan is an estimate, not a total process memory guarantee.

## 4. Pair selection

`--pair-selection sequential|overlap|gps|all` defaults to the existing sequential filename-neighbor behavior. `--neighbors` still supplies fallback capture-order edges. `overlap` adds each image's strongest reciprocal descriptor-preview candidates; `gps` adds nearby EXIF GPS images and falls back to descriptor previews when coordinates are missing. `--pair-candidates 6` controls the additional shortlist. `all` considers every pair. Preselection is a heuristic: proximity does not establish viewing overlap, and descriptor preview scoring itself evaluates many pairs. Essential-matrix verification still decides usable matches.

Inspect `03_matches/pair_selection.json` for scores, selected edges and methods. Matching all pairs can be expensive for large collections.

## 5. Resume and selective reruns

Use the same project directory and `--resume` to reuse compatible calibration, features, verified matches, local geometry and stereo results. Every invocation creates a separate auditable run. Without `--resume`, stages recompute; `--cache` enables saving them for future reuse. Default runs do not write a stage cache.

`--rerun-from calibration|features|matches|geometry|stereo|fusion` implies resume and forces that stage and later stages to recompute. For example:

```powershell
.\sgm_java\photogrammetry.bat -i D:/photos -o D:/project --resume
.\sgm_java\photogrammetry.bat -i D:/photos -o D:/project --rerun-from fusion --step 1
```

Repeat the relevant input/settings arguments: resume does not implicitly load an old command line. Input content, applicable settings and implementation bytecode participate in cache keys. Changing image content or detector settings invalidates affected results; changing dense resolution can preserve alignment work when alignment sizing is explicitly independent. Fusion is regenerated from reusable stereo results. GPS alignment is reapplied after local geometry reuse.

Cache entries under project `cache/v3/<storage>/` use checksummed payloads and an atomically published generation pointer. Interrupted or checksum-invalid entries are ignored and recomputed. Cache hits/misses are logged. Superseded completed generations of the same key are pruned after publishing their replacement; distinct keys remain reusable. Caches can be removed while the project is idle to reclaim space; saved run artifacts remain separate. Older cache versions are preserved but not reused by this implementation.

## 6. Sparse cleanup and dense confidence

`--cleanup-passes 1` enables observation/track cleanup followed by repeat bundle adjustment; `0` disables it. Controls: `--track-min-views 2`, `--track-max-error 4` alignment pixels and `--track-min-angle 0.1` degrees. Cleanup preserves at least twelve supporting tracks per camera and refuses removals that increase camera-graph component count. This can deliberately retain a questionable observation that is needed for connectivity. All decisions are recorded in `04_geometry/cleanup/decisions.csv`, with geometry snapshots. Registration is retried after refinement.

`07_results/point-confidence.csv.gz` has one row per exported cloud point, with a zero-based point index, candidate/source identifiers, final XYZ/RGB, supporting views, independent pairs, relative depth uncertainty and refinement shift. It supplements the detailed fusion decision artifacts. These values are geometric diagnostics, not calibrated probabilities or surveyed accuracy. Dense filtering intensity remains independently configurable.

## 7. Partial registration

Failed image-only registrations no longer abort a viable reconstruction. Unregistered cameras are excluded from dense matching, pose exports and the registered-view requirement. The final console/report and `04_geometry/registration.json` list excluded filenames. At least two registered cameras and a productive stereo pair are still necessary. Invalid supplied files and unrelated processing errors still fail explicitly.

## 8. Kermit investigation

Seed selection now considers triangulation angle and trials of multi-image registration, rather than ranking only pair match count. Numerical PnP failures remain local to the candidate image. Registration is retried after adjustment.

The Kermit investigation also exposed severe border-free rectification magnification: one native focal of 592.635 pixels became 5289.877 pixels in a 640×480 rectified raster. Default `--rectification-alpha auto` switches to full-field rectification when the focal magnification exceeds 1.5×; invalid borders remain masked. `--rectification-alpha 0` reproduces border-free cropping and `1` always preserves the full field. This is a coverage safeguard, not a general solution for poor calibration or weak baselines. See [KERMIT_DEBUG.md](KERMIT_DEBUG.md) for measurements and remaining limitations.

## 9. Detectors and descriptor matching

`--matching-algorithm sift|orb|akaze|kaze|brisk` defaults to SIFT. It applies consistently to calibration, alignment, disparity previews and exports. SIFT/KAZE use floating-point L2 descriptors; ORB/AKAZE/BRISK use binary Hamming descriptors. `--descriptor-matcher auto|bruteforce|flann` defaults to exact brute force. FLANN uses a KD-tree for floating descriptors; binary descriptors retain compatible exact Hamming matching.

Detector controls: `--sift-contrast 0.04`, `--sift-edge-threshold 10`, `--orb-fast-threshold 20`, `--brisk-threshold 30`, `--akaze-threshold 0.001`, `--kaze-threshold 0.001`. `--features`, reciprocal `--ratio` and geometric verification remain applicable. A larger ratio admits less distinctive matches; changing detector or thresholds can change registration and calibration evidence.

## 10. Bounded-memory disk tiling

`--tiled` puts the cost and aggregation volumes in paged disk storage. It preserves complete global SGM path state across storage tiles, including diagonal paths. This is exact storage tiling, not independently matching image crops. `--tile-size 512` controls storage sizing and pyramid tile cores. The full disparity volume no longer needs to fit a Java array or heap, although image/mask/census/output/row buffers still do. Temporary storage is approximately six bytes per pixel/disparity hypothesis, in addition to output and cache files. Disk I/O can make it much slower than in-memory SGM.

Project depth evidence is disk-backed by default, independently of `--tiled`. All pair maps share a bounded resident page pool and share at most 32 cached open file handles. `--memory-depth` opts back into heap storage. Evidence PFMs are streamed rather than materialized as full extra arrays. These controls do not bound all native allocations, tracks or candidate objects; they are not a guarantee that arbitrarily large images fit RAM.

## 11. Coarse-to-fine dense matching

`--pyramid-levels 1` preserves ordinary SGM. Values 2–6 start with a coarse global search and refine overlapping image tiles using upscaled predictions. `--pyramid-radius 16` sets the local disparity margin in current-level pixels. `--tile-overlap 32` is a minimum halo; actual context grows to cover the search and filter footprint. Weak predictions or search-boundary pressure trigger a full-range fallback for that tile. Automatic global disparity expansion remains available when no explicit `--disparities` is given.

```powershell
.\sgm_java\photogrammetry.bat -i D:/photos -o D:/project `
  --alignment-quality high --quality high --pair-selection overlap `
  --tiled --tile-size 512 --tile-overlap 32 --pyramid-levels 3 --pyramid-radius 16
```

Pyramid refinement changes the search and local SGM context, so it is approximate and need not equal single-level global SGM. It can miss thin structures or incorrect coarse predictions. Level rasters and fallback decisions are retained with stereo artifacts. Disk tiling and pyramids are independently selectable; compare results on representative images before committing a large collection.
