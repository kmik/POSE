# Scope and algorithm correspondence

This port replaces the main application's **functional pipeline** with maintainable Java components. It does not mechanically translate all experimental, commented-out, unreachable, or third-party C++ source. The C++ main contains unreachable demo code after its return; that code is not part of the runnable application.

Optional OpenCL GPU kernels now accelerate integer census-Hamming costs and global SGM aggregation through Java 25 FFM, without a CUDA toolkit or extra JNI dependency. CPU selection/filtering/fusion remain shared; real NVIDIA/AMD parity checks and complete Kermit clouds match CPU output exactly. Standard storage and opt-in caching replace the earlier exhaustive default capture. See [STORAGE_GPU.md](STORAGE_GPU.md).

| C++ area | Java counterpart | Status / differences |
|---|---|---|
| `main.cpp`, Boost program options | `photogrammetry.java`, `sgm.Main`, `sgm.Options` | Executable CLI; hardcoded Linux paths removed; strict validation; calibration option works |
| `Block::ReadImages`, `Image`, EasyEXIF | `ImageFrame`, `Calibration`, `Exif` | Portable loading/undistortion and bounded JPEG/classic TIFF GPS and focal metadata parsing; no GDAL dependency |
| `Block::MakePairs`, `overlap` | `FeatureTracks.build`, `PairSelection` | Filename-neighbor fallback, descriptor-overlap/GPS shortlists or all pairs, followed by geometric verification; different from the legacy aerial-footprint overlap heuristic |
| `Block::matchPairs` | `FeatureTracks.match` | Selectable SIFT/ORB/AKAZE/KAZE/BRISK, compatible descriptor metrics, reciprocal ratio-test matches, essential-matrix RANSAC and cheirality validation |
| `Block::align`, orientation/PnP helpers | `Geometry.reconstruct` | Seed trials, shared tracks, incremental PnP registration, cleanup/retry; partial registration continues with excluded-image reporting |
| `5point`, `Rpoly`, polynomial and triangulation helpers | OpenCV essential solver, `Geometry.triangulate` | Native five-point estimation through Java bindings; normalized DLT triangulation rather than literal translations of numerical helper classes |
| `Block::bundle`, Ceres | `BundleAdjuster`, `SelfCalibration` | Final undistorted-image geometry uses joint robust Schur LM with analytic Jacobians and a matrix-free block-Jacobi PCG camera solve. Accepted intrinsics and the first seed pose stay fixed; a baseline norm fixes scale while the second seed pose moves. All final registered cameras/triangulated tracks participate. The preceding bounded self-calibration uses joint f,k1,k2/pose/point Schur LM. No Ceres numerical-parity claim |
| `Block::rectifyPairs`, pair transforms | `DenseReconstruction.rectify` | Calibrated rectification, baseline orientation, vertical-pair transposition, world-space reprojected points |
| `runSGM`, image census transforms | `CensusSgm` | Java 4/8-direction normalized SGM recurrence, grayscale or legacy RGB-distance Hamming census (3..11 windows), subpixel refinement, endpoint rejection, uniqueness/texture/LR checks, median with repeated LR validation, ZNCC, and speckle filtering |
| C++ adaptive disparity search / experimental CCT and alternative correlation costs | Canonical full-range census SGM | Deliberate algorithm change; those alternate cost modes and per-pixel search schedules have not been replicated |
| `Array3D*`, queues, pthread/OpenMP | Primitive arrays or `PagedVolume` | Checked dimensions/budget, rolling global path rows, optional disk paging, separate approximate coarse-to-fine tiles; aggregation sequential, OpenCV operations use configurable threads |
| Dense triangulation and `mergeDisparities`, consistency filters | `DenseReconstruction`, `DepthFusion`, `CloudWriter` | Shared-world triangulation, dominant per-ray depth modes, actual per-camera independent support, subpixel evidence, uncertainty-weighted depth refinement and optional voxel deduplication; different filtering from the original custom binary workflow |
| Original GPS/world-coordinate handling | `Georeference`, `Crs` | Automatic robust projected GPS similarity, selectable metre-based EPSG (default 3067), LAS GeoTIFF CRS metadata; no vertical datum/geoid correction |
| Missing intrinsic calibration | `AutomaticCalibration`, `SelfCalibration` | Focal initialization from matches/EXIF, followed by guarded joint shared f,k1,k2/pose/structure LM; held-out validation and marginal uncertainty; centered principal point and zero tangential/k3 terms |
| Dense search-range selection | `DisparityRange` | Per-pair rectified feature estimate and bounded endpoint-driven retries when --disparities is omitted; explicit ranges preserved |
| `calibration` and chessboard branch | `Calibration.calibrate` | Working board detection, subpixel corners, OpenCV calibration, portable output |
| Cloud text/binary output | `CloudWriter` | Standard RGB LAS 1.2 and binary PLY; legacy temporary-file formats are not reproduced |
| `bundler/`, `bundleOLD/`, `bundler_sfm-master/`, `sba/` | Not included | Separate vendored projects outside the main Code::Blocks compilation units; no Java translations of their executables |

## Accuracy and parity work still needed

The workflow additions in [WORKFLOW_UPGRADE.md](WORKFLOW_UPGRADE.md) provide selected shared-intrinsic constraints, absolute pose priors, independent alignment sizing, overlap/GPS pair selection, verified stage caches, sparse cleanup, partial registration, five feature algorithms, disk-paged global SGM and approximate coarse-to-fine refinement. Global disk tiling preserves path continuity; pyramid image tiles deliberately use local contexts and are not numerically identical to global matching. GPU dense matching is now available; see STORAGE_GPU.md. The final bundle gauge described above applies to image-only/unpenalized geometry; supplied pose priors instead anchor all adjustable poses to their reference frame.

The test suite validates known geometry and full synthetic reconstruction, not agreement with the original C++ outputs. The original executable is built for Linux and cannot be run in this Windows environment without a compatible Linux runtime and dependencies.

Before treating this as a production replacement for the old aerial pipeline:

1. Run both implementations on representative original flights with the same calibrated inputs and capture intermediate poses/disparities/clouds.
2. Compare recovered baselines, reprojection error, valid disparity coverage, point-to-surface distances, GPS alignment, run time, and peak memory. There is no current claim of equivalent accuracy or performance.
3. Decide which legacy experimental census/CCT, adaptive search, interpolation, overlap selection, and fusion behavior is required. Port those modes explicitly if their behavior is part of the desired specification.
4. For large aerial blocks or joint intrinsic optimization, replace/extend the current Java optimizer with a sparse bundle solver. The current optimizer is suitable for initial functional integration and small/moderate blocks but is not a scalable Ceres replacement.
5. Run the supplied Linux CI and install the module in a full LAS.utils checkout. Its launch conventions were inspected, but that external codebase has not been modified or used to run this module in this workspace.

These are explicit compatibility/validation limits; unimplemented legacy modes are not silently exposed as working CLI options.
