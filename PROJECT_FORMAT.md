# Photogrammetry project format

Latest additions: `04_geometry/orientation_model.json` archives source and processed cameras for post-processing export; optional `08_orientations/` contains format-specific exports and convention sidecars. `05_stereo/*/disparity_range.json` includes the GPU memory plan and streamed disk estimate, with `pyramid_*.json` local-search/fallback reports when used. `03_matches/overlap_*.json` records geometric thresholds, estimates and acceptance decisions. Fusion adds kernel/transfer/CPU timings in `06_fusion/backend.json` and ordered-output timing in `06_fusion/output_profile.json`. See [RECONSTRUCTION_UPGRADE.md](RECONSTRUCTION_UPGRADE.md) and [ORIENTATIONS.md](ORIENTATIONS.md).

`-o` names a project, rather than a disposable flat output folder. The project retains the actual data produced at each stage, together with enough conventions and provenance to inspect how a result was obtained.

**Current policy:** standard storage is the default; exhaustive exports listed in this format reference require `--storage debug`. Minimal storage omits diagnostic images/features/disparities. Caching is opt-in, under `cache/v3/<storage>/`, and standard/minimal remove working depth files after successful fusion. See [STORAGE_GPU.md](STORAGE_GPU.md) for the exact retained sets, GPU execution and disk measurements. Data from previous runs/cache versions is preserved.

Automatic pair ranges now estimate both endpoints. `disparity_range.json` includes source/coverage confidence, coarse/full-resolution guard validation, lower/upper searchable-boundary counters, initial/final intervals and retries. Debug mode adds `range_samples.csv.gz`. Explicit minimum/count behavior and coordinate conventions are detailed in [PAIR_RANGES.md](PAIR_RANGES.md).

Fusion now retains `06_fusion/backend.json` in every storage profile. It records GPU/CPU candidate counts, numerical CPU rechecks, CPU audit sampling, cache budget and active GPU device/evidence/batch/preparation details. GPU candidate counts include numerically rechecked candidates; the separate CPU-guard count distinguishes them. See [GPU_FUSION.md](GPU_FUSION.md) for execution policy and validation.

## Use

From the original repository root in PowerShell:

```powershell
.\sgm_java\photogrammetry.bat `
  -i .\example_data -o .\sgm_java\output\photogrammetry-project `
  --calibration .\calibration\resize_25\calib.result `
  --max-dimension 800 --disparities 128 --step 3 --cores 2 `
  --bundle-iterations 10 --copy-inputs
```

On Linux, use `bash sgm_java/photogrammetry.sh` with equivalent paths and arguments. The data formats and directory layout are the same.

Reusing the same project path creates a **new run**. It does not edit or erase a previous run. `--resume` reuses compatible, checksummed stage results from the project-local cache, and `--rerun-from` forces selected stages onward. See [WORKFLOW_UPGRADE.md](WORKFLOW_UPGRADE.md) for invalidation rules. `--overwrite` is needed only when initially adopting an existing nonempty directory that is not already an mSGM project. Ordinary files outside the generated run directories are not removed. Existing reserved root filenames that will be replaced, such as an older README or flat result files, are first preserved under the adopting run's `00_inputs/adopted_root_files/`, with an inventory. Original photographs are never edited.

## Layout and lifecycle

```text
photogrammetry-project/
  README.md                   entry point and latest-run links
  project.json                latest run, latest successful run, history
  project.properties          project format marker and run pointers
  latest-files.csv            provenance of root compatibility copies
  .project.lock               operating-system lock used during a run
  dense.las, dense.ply, ...    compatibility copies, when produced
  runs/
    <UTC timestamp>_<id>/
      README.md               stage guide, conventions, working quick links
      run.json                mode, original arguments, status and stage states
      stages.csv              completed/skipped/failed/pending stage inventory
      artifacts.csv           relative path, file size, SHA-256
      00_inputs/
      01_calibration/
      02_features/
      03_matches/
      04_geometry/
      05_stereo/
      06_fusion/
      07_results/
      logs/
```

Completed run directories are not subsequently modified by the program. A failed run remains available with its actual partial artifacts, console output, exception trace, events, and stage status. Root aliases are refreshed after processing succeeds. They are published individually rather than as a transactional result set; canonical per-run artifacts remain the authoritative record. A project lock prevents concurrent writers to the same project.

`artifacts.csv` inventories numerical data, previews, source copies, logs, and documentation with byte sizes and SHA-256 hashes. The catalog and `run.json` are excluded from their own catalog to avoid self-reference and lifecycle updates. The root project metadata is separate from the per-run catalog.

The project root contains convenience copies of the legacy `dense.las`, `dense.ply`, `cameras.csv`, `sparse.csv`, report/quality files, and final pair disparities. **Canonical data lives inside a run.** When different modes produce different file sets, old root aliases may remain. `latest-files.csv` records exactly which run produced each managed alias. Preexisting unrelated or legacy files not listed there are not implicitly part of the latest run.

## 00_inputs: photographs and provenance

`resolution_plan.json` is written from image headers before decoding/reconstruction. It records the quality preset, downsample factor, explicit/legacy size cap, each source and planned processed size, per-axis scales, SGM budget and feasible disparity limit, memory scenarios, approximate native image/feature/rectification storage and potential retained depth evidence. Estimates exclude additional optimizer/SIFT/codec/object overhead; automatic disparity ranges remain unknown at this stage. `--estimate-memory` creates only this plan and logs, marks subsequent stages skipped and emits no cloud. An oversized scenario may be inspected in planning mode; actual reconstruction enforces its minimum/explicit SGM requirements.

`images.csv` records the image ID, original filename/path, byte size, SHA-256, original raster dimensions, available JPEG/TIFF GPS coordinates, and optional archived-original path. GPS altitude is in metres with unspecified vertical datum; missing altitude is not replaced with zero. Image IDs follow the lexicographically sorted input filenames and are stable **within this run**, not across arbitrary changes to an input collection.

Each `image_NNNN/` contains:

- `undistorted_color.png` and `undistorted_gray.png`: exact lossless processed images used by the pipeline.
- `valid_mask.png`: actual valid support after undistortion. Black scene content is distinguished from invalid remapping padding.
- `undistort_source_x.pfm`, `undistort_source_y.pfm`: inverse sampling maps from the processed undistorted raster into the resized distorted raster. These are omitted when inputs are explicitly already rectified and no undistortion is applied.
- `image.json`: original/processed sizes, effective K, pixel-coordinate conventions, and whether distortion was applied.

`--copy-inputs` adds `originals/image_NNNN.<original-extension>`, copied byte for byte, preserving the original full resolution and embedded EXIF. Without this option the original paths and hashes are recorded, while processed images are still archived. This avoids automatically duplicating an entire full-resolution flight.

## 01_calibration: interior orientations and calibration evidence

- `supplied_calibration.txt`: the exact supplied calibration file, when present.
- `reference_camera.properties` and `reference_interior.json`: source/reference resolution, fx/fy/cx/cy, zero skew, and distortion in **k1, k2, p1, p2, k3** order.
- `processed_interior.csv`: K for each processed image. Its distortion coefficients are zero because these exported images are already undistorted. Source distortion remains in the reference files.

Reference and processed calibration must not be interchanged. Focal length and principal point values are pixels at the corresponding resolution. Command-line millimetre focal/sensor inputs are retained in the effective settings when used.

When calibration/focal-sensor inputs are omitted, `self_calibration.json` records the initial focal method and final applied focal/distortion coefficients. `exif_intrinsics.json` preserves EXIF camera/focal metadata. `self_calibration_pairs.json` records all attempted uncalibrated pairs and acceptance reasons. Each sufficiently matched `pair_NNNN_NNNN/` includes reciprocal match coordinates normalized about the original raster center by the image diagonal, with fundamental RANSAC flags. Estimable pairs also have a fundamental matrix report and the complete sampled focal-search curve. The reusable `camera.txt` is published at the project root on success. Insufficient initial scene constraints use EXIF 35mm-equivalent focal when available or fail explicitly.

Without a calibration file, guarded joint `f,k1,k2` refinement follows either this automatic initialization or supplied focal/sensor inputs. `refinement/result.json` records the candidate coefficients, acceptance reason, solver/gauge, iteration history, image-radius coverage, marginal information conditioning, approximate standard deviations and held-out robust reprojection RMS. A rejected candidate is **not applied**: consult the reference camera file for the actual model. `refinement/observations.csv` retains raw processed-distorted observations, held-out flags, initial/refined reprojections and 3D points. `bootstrap_exterior.json` retains both pose sets, lens vectors `[log(f),k1,k2]`, image scales and fixed principal points. These belong to the calibration bootstrap, before the final image undistortion and final SfM. Centered principal point, square pixels, zero skew and zero `p1,p2,k3` remain assumptions. See README for acceptance thresholds and bounded calibration sampling.

For a `--calib` run, `views/image_NNNN/` includes the detected chessboard corners, corner overlays, object-space board coordinates, and detection status, including unsuccessful detections. `reprojection.csv` records every detected corner's observed and fitted coordinates and residual. `calibration.json` contains calibration RMS and the fitted **board-to-camera** exterior poses for the successful views. These are calibration-board poses, not a shared aerial world coordinate system. Board coordinates use the units specified by `--square-size`.

## 02_features: keypoints and descriptors

Each image has:

- `keypoints.csv`: complete zero-based keypoint IDs, processed-undistorted x/y, size, orientation in degrees, response, raw OpenCV octave/class ID, and mapped original-raster u/v.
- `descriptors.f32`: the selected algorithm's complete descriptor matrix. Row i belongs to keypoint i. Native type/metric are recorded in metadata; binary bytes are represented exactly as floats in this export container.
- `keypoints.png`: all detected keypoint centers overlaid on the processed color image.
- `features.json`: detector, count, coordinate frame, and binary-format description.

Descriptor format is **8 ASCII bytes `SGMDESC1`, big-endian int32 row count, big-endian int32 column count, then row-major big-endian float32**. SIFT has 128 columns for nonempty results. This is deliberately independent of host endianness. Empty extractions have an empty matrix rather than invented descriptors.

The primary x/y coordinates are in processed undistorted pixels. Original-raster u/v are obtained by applying the reference distortion model and undoing resize with the pixel-center convention. Keypoint size and angle remain those measured in the processed raster.

## 03_matches: complete decisions and track construction

Every attempted neighbor pair gets `pair_AAAA_BBBB/`, including rejected pairs:

- `matches.csv.gz`: every executed forward nearest-neighbor decision, best/second feature IDs and L2 distances, ratio, ratio/reciprocity flags, essential-RANSAC and cheirality flags, whether the match actually entered track construction, and rejection stage.
- `pair.json`: pair acceptance status, initial normalized essential matrix, fundamental matrix for processed undistorted pixels, relative R and unit-baseline t when estimated, and preview conventions.
- `ratio_matches.png`, `geometric_inliers.png`, `rejected_matches.png`: deterministic previews showing at most 300 evenly sampled matches. **Numerical exports are not subsampled.**

`pairs.csv` summarizes all attempted pairs, including insufficient-descriptor and insufficient-inlier cases. Initial pair poses are local two-view estimates; they are not the globally refined metric exterior orientations. Use final geometry and stereo metadata for later stages.

`track_observations.csv` records each retained track's image/keypoint memberships. `all_track_components.csv.gz` also retains unmatched singleton components and conflicting components rejected because they contained more than one keypoint in an image. `track_build.json` summarizes those decisions. IDs can be joined directly to keypoints and match decisions.

## 04_geometry: exterior orientations and sparse reconstruction

Snapshots are retained for the seed reconstruction, incremental camera-registration steps, pre-bundle geometry, every bundle iteration including iteration zero, post-bundle geometry before track rejection, and final geometry. Depending on the run, optional GPS alignment adds a documented similarity transform and control residuals.

When joint self-calibration is accepted, final SfM initializes a covered seed pair from that fit's relative exterior orientation, then rebuilds structure using final undistorted-image observations. The seed event records `orientation_source` as `accepted_self_calibration` or `pair_essential_matrix`. A supplied seed-pair choice remains authoritative.

Each snapshot contains:

- `interior.csv` and `exterior.csv`: processed calibration; registration/fixed flags; row-major R, t, camera centers, and Rodrigues rotation vectors.
- `orientations.json`: world-to-camera and camera-to-world 4x4 matrices (`world_to_camera_4x4`, `camera_to_world_4x4`) and full K[R|t] pixel projection matrices. JSON keys are distinct even in case-insensitive Windows readers; the t translation vector and homogeneous matrix have separate unambiguous names.
- `tracks.csv`: all retained feature-track IDs, triangulation status, XYZ when available, and observation count.
- `observations.csv`: observed/predicted pixel coordinates, signed residuals, residual norms, and positive-depth status.
- `sparse.ply`: colored triangulated points.
- `camera_centers.ply` and `camera_frustums.ply`: camera display geometry. Frustum size is a documented display scale, not a reconstructed object.

The convention is:

```text
X_camera = R * X_world + t
C_world  = -transpose(R) * t
camera axes: x right, y down, z forward
```

Rodrigues components use radians. No ambiguous omega/phi/kappa or yaw/pitch/roll convention is imposed. Unregistered cameras in an intermediate snapshot are explicitly flagged; their placeholder transforms must not be treated as solved poses.

`pnp/attempt_NNNN_image_NNNN/` records the actual 3D-to-2D correspondences, track/keypoint IDs, RANSAC inlier flags, estimated pose, and success status for each attempted camera registration. `bundle/solver.json` records the joint Schur LM solver, first-pose/seed-baseline gauge, fixed intrinsics, variable camera count and linear tolerance. `bundle/iterations.csv` retains the robust objective history for committed steps, including initial iteration zero; `bundle/linear_solver.csv` includes every trial's damping, PCG iteration count, relative linear residual, full objective and acceptance. Rejected trials do not change the geometry snapshots. Camera `fixed` is true only for the first seed after image-only refinement; the second seed's distance is constrained while its pose can improve. `track_filter.csv` records post-refinement retention and rejection reasons. `geometry-quality.csv` gives per-camera residual summaries.

GPS alignment is automatic unless `--no-gps` is supplied. `georeference/gps_inventory.json` records every image's coordinates, datum, altitude and suitability for 3D control. At least three non-collinear registered camera centers with complete WGS84 GPS are needed. A robust majority fit uses `--gps-threshold` metres. `georeference/transform.json` explicitly describes `X_projected = s * R * X_reconstruction + t`, the chosen horizontal EPSG/PROJ definition, inlier count, RMS and height limitations; `camera_control.csv` retains source, projected target and fitted centers, residuals and inlier flags. The final snapshot and subsequent stereo/fusion XYZ use that projected frame; earlier snapshots retain the original reconstruction frame. Failed/disabled automatic alignment records its reason and leaves the result unreferenced; `--gps` makes this failure fatal. The default CRS is EPSG:3067, x=easting, y=northing, metres. Z is retained EXIF altitude with unspecified vertical datum, without a geoid/N2000 conversion.

## 05_stereo: rectification, disparity stages and confidence

Each actual matching pair has its own folder. The reference/secondary order can differ from the original candidate order when the program swaps cameras to obtain positive disparity; IDs and filenames are recorded explicitly.

- `rectification.json`: processed K1/K2, relative R/t, R1/R2, P1/P2, Q, already-rectified status, and matching-raster coordinate transform.
- `left/right_source_x/y.pfm`: inverse maps from matching pixels into the processed undistorted source images.
- `left/right_color.png`, `left/right_gray.png`, and validity masks: exact matching rasters and eroded support masks.
- `epipolar_rows.png`: side-by-side rectified images with fixed row guides.
- `disparity/00_left_selected`, `01_right_selected`, `02_left_right_validated`, `03_median`, `04_photometric`, `05_final_stereo`: each available stage as numerical PFM plus PNG preview and preview-range metadata.
- `disparity/zncc` and `left_right_error`: floating-point confidence evidence with previews.
- `disparity/rejection_codes.png` and `rejection_preview.png`: machine-readable codes and a color overview.
- `left_camera_depth.pfm`, `triangulation_angle_degrees.pfm`, and `triangulation_valid_mask.png`: geometric validity before candidate sampling/fusion.
- `stereo-quality.csv`: per-pair rejection counters, updated as pairs finish so partial runs retain completed work.
- `disparity_range.json` in each pair directory: explicit/automatic mode, feature/sparse-geometry estimates and sample counts, effective minimum/count, actual matching width/height, SGM byte estimate, memory limit, stop reason and per-attempt endpoint/valid counts. Omitted `--disparities` enables estimation and bounded expansion. `disparity_attempt_NN/` preserves each automatic attempt; `disparity/` copies the final attempt. Preview scaling uses the actual pair range, rather than a global default. Explicit counts remain fixed.

Disparity is `x_left_matching - x_right_matching`, in pixels at matching resolution. Vertical epipolar geometry is transposed for horizontal matching. Metadata describes how matching coordinates map to original rectified x/y before applying Q. Map arrays follow the matching raster while their **values** remain source-image coordinates.

Stereo rejection codes are: 0 valid; 1 mask/texture/border/no valid hypothesis; 2 search endpoint; 3 uniqueness; 4 initial LR; 5 post-median LR; 6 photometric; 7 speckle. These describe the stereo stage, not final cloud acceptance. NaN disparities stay invalid, and holes are not invented. Pair depth/disparity estimates can still fail parallax or independent fusion afterwards.

## 06_fusion: provenance of accepted and rejected points

Filtering intensity is selected by `--filtering mild|moderate|aggressive`, default moderate. The resolved base preset is recorded in `logs/run-settings.properties` as `filtering`; `filteringOverrides` lists explicit threshold flags, sorted and comma-separated. Individual numeric properties contain the effective values after overrides. The console and report print the same summary. In `fusion.json`, `filtering_preset` and `filtering_explicit_overrides` identify the profile/provenance, while numeric fusion tolerances, `minimum_correlation` and `minimum_speckle_component_pixels` record the settings actually applied. Explicit thresholds override a preset in either CLI order; required views and image resolution are independent of intensity.

`candidates.csv.gz` contains every sampled candidate before fusion, with a sequential candidate ID, fusion-pair ID, matching x/y, disparity, world XYZ, RGB, triangulation angle, and correlation score. `--step` defines this sampling; dense evidence maps still use every valid pixel.

`decisions.csv.gz` has one row per candidate, using the same ID and pixel coordinates, with supporting distinct-view count and a decision: `accepted`, `independent_support_rejected`, `depth_consensus_rejected`, `depth_uncertainty_rejected`, `refinement_reprojection_rejected`, `refinement_support_rejected`, or `voxel_duplicate`. Accepted row count equals final point count. This permits tracing a final candidate back to its exact images, pose, disparity and confidence evidence.

`fusion.json` records consensus, depth/reprojection tolerances, minimum views, uncertainty assumptions/limit, refinement setting and evidence layout. `refinements.csv.gz` has one row per candidate with original and final proposed XYZ, adjustment length, independent-pair count, actual measured-view count, estimated relative stereo sigma, and fusion-geometry decision. It precedes optional voxel rejection. For an accepted output, use its refined XYZ (then apply `--scale`) rather than the raw `candidates.csv.gz` position. The raw candidate remains unchanged for audit and rectification reprojection checks. Two-image/no-refinement cases retain the original positions.

`pair_evidence/source_NNNN/` stores each producing pair's separate camera-Z maps in the original **processed-undistorted** image rasters, plus camera IDs and a link to the stereo artifacts. Per-image `_offset_x.pfm`/`_offset_y.pfm` files retain the subpixel displacement from each map pixel's integer center; use them only where camera-Z is finite. In-memory offsets are packed signed 16-bit values, reconstructed by dividing by 65534; this adds four bytes per pixel to each camera's four-byte depth array. The producing pair participates in the visibility/majority-mode check, but is excluded from independent corroboration. A camera is counted only if its own measurement passed, not merely because the other endpoint of its pair agreed.

Each pair also has supporting-view PFM/preview and candidate-status maps. Status values are 0 unsampled/no candidate, 1 accepted, 2 independent-support rejection, 3 voxel duplicate, 4 per-ray depth-consensus rejection, 5 uncertainty rejection, and 6 refined-point reprojection/support rejection. A separate color preview and local legend are included. Unsampled pixels are NaN in the support raster, rather than falsely reporting zero support. `fusion-quality.txt` summarizes rejection and accepted-refinement counts. No global terrain plane or lowest-height clipping is applied.

## 07_results and logs

Final standard RGB LAS 1.2 format 2 and binary PLY are written under `07_results`, together with `report.txt` and `coordinates.json`. Numerical geometry, poses, sparse points, and candidate XYZ use **pre-output-multiplier units**. Only final LAS/PLY coordinates apply `--scale`; successful GPS alignment requires `--scale 1` so all final geometrical exports share projected coordinates. GPS-referenced LAS includes a `LASF_Projection` VLR, record 34735, with GeoTIFF keys for projected model, EPSG and metre units. PLY includes a horizontal CRS comment; `coordinates.json` carries the CRS and explicit unspecified vertical datum. Unanchored image-only units remain arbitrary and receive no CRS tag. Reading LAS point records must use the header's point-data offset rather than assuming 227 bytes.

`logs/console.log` captures application stdout/stderr; `events.jsonl` records stage and pair completion events. `request.json` retains the exact argument array and runtime information. `run-settings.properties` records effective options, including defaults and processed image intrinsics. Failed runs include `error.txt` with the exception trace.

## Storage, scope, and verification

This is a full audit capture by default. Lossless image copies, intermediate rasters, per-iteration snapshots, and complete candidate/match tables consume more disk and I/O than the former flat output. Gzip is used for large row tables; PNG previews are distinct from lossless numerical PFM data. Full multi-gigabyte SGM cost volumes are temporary in `--tiled` mode and removed on completion. Private native-solver workspaces are not persisted. Resume uses the separate checksummed project `cache/v2/`, rather than interpreting arbitrary edited audit exports as restart state.

Stages genuinely bypassed by the selected mode are marked `skipped`. Fixed supplied poses and rectified stereo bypass feature/SfM estimation; requested supplied-pose refinement extracts features and performs adjustment. Calibration-only runs export calibration-board evidence rather than fabricated aerial features or depths.

## Workflow additions

Alignment and dense rasters can differ. Primary keypoints/matches and sparse observations refer to alignment images; `00_inputs/dense/` holds the actual dense rasters/intrinsics when sizes change. `01_calibration/constraints.json` and `04_geometry/pose_constraints.json` record constraint settings and pose adjustments. `03_matches/pair_selection.json` records pair-preselection evidence. `04_geometry/registration.json` records failed images; exported pose rows include registered cameras only. `04_geometry/seed_trials.json` and `cleanup/decisions.csv` retain seed trials and conservative cleanup decisions.

Descriptor exports retain the SGMDESC1 floating-point container. Metadata identifies native descriptor type and distance metric. Binary descriptor bytes are represented exactly as float values; cast back to unsigned bytes before Hamming matching. Do not apply L2 to those exported binary descriptors.

`06_fusion/storage/` contains internal paged camera-Z/packed-offset files when disk evidence is enabled (the default). Portable pair-evidence PFMs remain the interchange format; internal `.tiles` files have no independent public restart contract. Persistent evidence maps share a bounded global resident page pool. Temporary SGM cost/aggregate files are distinct from retained depth evidence.

`07_results/point-confidence.csv.gz` contains exactly one row per final LAS/PLY point in output order, indexed from zero. It includes final scaled XYZ, 8-bit RGB, candidate ID, producing fusion-pair ID, supporting views, independent pair count, estimated relative stereo sigma and scaled refinement shift. Unlike pre-output candidate/refinement tables, these coordinates include `--scale`. Confidence diagnostics are not accuracy certificates.

The cache has versioned, content-addressed stage keys, immutable generations, SHA-256 payload manifests and an atomic active-generation pointer. Original input content, applicable options and implementation fingerprints determine compatibility. A new resumed run restores reusable artifacts and regenerates its own catalog; previous runs remain unchanged. See [WORKFLOW_UPGRADE.md](WORKFLOW_UPGRADE.md) for commands, cache lifecycle and limitations.

The automated suite validates catalog hashes, descriptor layouts and IDs, complete match decisions, track memberships, exterior centers, candidate-to-source reprojection through exported maps, per-pixel rejection masks, candidate/fusion foreign keys, output counts and scaling, history preservation, failure isolation, input archives, and calibration-only output. The real example's final LAS and camera CSV were also compared byte-for-byte against the repaired pre-project implementation: the capture did not change the reconstruction.

The populated local example is output/photogrammetry-project/README.md (generated locally at `output/photogrammetry-project/README.md`; omitted from the source package). On 2026-10-29, the Windows/JDK 25 build passed **105 checks**. Its real-photo run exported **377 indexed artifacts (198,404,490 bytes, approximately 189 MiB)** and the same **62,236 points** as the repaired baseline. Independent Windows JSON-reader validation parsed all **76 JSON files and 20 JSONL events** without errors, and an independent filesystem pass found **zero artifact size/hash mismatches**. The final LAS and camera CSV were byte-identical to the repaired baseline. Verification record (generated locally at `output/photogrammetry-project-verification.json`; omitted from the source package).

Linux launch/build support remains included; Linux execution of this export workflow has not been performed in this Windows workspace.


## Surface attribute extensions

Current LAS/PLY attributes and the appended confidence columns are documented in [PERFORMANCE_AND_ATTRIBUTES.md](PERFORMANCE_AND_ATTRIBUTES.md). `--point-attributes surface` is the default; `none` restores compact XYZ/RGB cloud records. Source-depth normals and metrics travel with final representatives; they are not a refit to the final cloud.

## Processing report and resource telemetry

New runs archive all effective public processing options (including null/unset values) and initial settings, monotonic task timings and sampled resources in `logs/processing.json`. `logs/request.json` retains supplied arguments; `logs/command-help.txt` captures the processing version's option descriptions. `run.json` stage records include start timestamps and elapsed seconds.

`07_results/processing-report.html` is a self-contained printable report; `processing-report.json` preserves the report data, resolved camera/pair settings and provenance. Projected runs also produce bounded Float32 `dsm.tif`, `dsm.tfw`, `dsm.png` and `dsm.json`, with explicit nodata, CRS and height-reference notes. Report generation does not change cloud geometry. `--export-report RUN_DIRECTORY` regenerates these products without reconstruction; older missing measurements are labeled unavailable. See [PROCESSING_REPORT.md](PROCESSING_REPORT.md).
