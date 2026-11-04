# Local validation

## Fusion, consolidation and processing reports (2026-10-31)

Complete Windows suite: **2,139 checks**, including **198** throughput/report checks. AMD parity: fusion **60**, surface **58**, follow-up **54** checks. The fixed-camera city replay yields a byte-identical final PLY with **35,089 points**. Exact consolidation output and merge membership are preserved for **6,503,432 samples**. Independent Pillow decoding verifies Float32 DSM values, EPSG:3067, pixel-is-area transforms and NaN nodata. Every public processing option is checked against archived effective parameters; sampled RAM and tracked OpenCL allocations are verified separately.

Measurements, rejected experiments, temporary-disk tradeoffs and limits: [FOLLOWUP_PERFORMANCE.md](FOLLOWUP_PERFORMANCE.md), `research/followup/results-2026-10-31.json`. Reports from older runs do not invent missing historical memory peaks or settings.


## POSE follow-up: performance and attributes (2026-10-30)

Verified production release: `target/collinea-final` (134 production class files, 55 test class files), copied byte-for-byte into the normal launcher directories. Previous compiled directories are retained; `target/collinea-installed.json` records their locations. The IDE can rebuild normal output directories: a later local rebuild was observed without changes to Java source, and its installed launcher was also replayed successfully. `target/collinea-validation.json` records release and installed hashes at the final check.

- Full numerical/integration suite: **1,590 checks passed**, OpenCV 4.9.0, Windows 11. Evidence: `target/collinea-suite-verified.log`.
- Surface controls on both NVIDIA RTX 5070 Laptop and AMD gfx1103: **58 checks per device**. Evidence: `target/collinea-surface-nv.log`, `target/collinea-surface-amd.log`. Plane/wall noise controls, thin foreground, opposing layers, failure cleanup and ordered output remain covered.
- New attribute/aggregate regressions: **54 checks**, including independent covariance controls, scaled LAS/PLY scalar round trips, NaN/status metadata, unsigned bulk paging, four costs, 11x11 descriptors, signed intervals and exact CPU/int32 versus GPU/uint16/int32 comparisons. AMD evidence: `target/collinea-attributes-stereo-amd.log`; NVIDIA checks also run in the full suite. Eight-path/one-row/693-disparity cases are additionally covered by the existing stereo regressions.
- NVIDIA resident stress: **900x1600 pixels, 693 disparities**, 1,178,680 valid control pixels, 100% within the independent known-shift tolerance. Planned device buffers: **3867.2 MiB**, versus 5770.6 MiB with 32-bit aggregates; bound 4288, no cost/aggregate clipping. Evidence: `target/collinea-large-gpu.log`. The observed 2.674-second run is a hardware stress observation, not a general throughput promise.
- Real fixed-camera, three-image city replay through the installed Windows launcher: **35,121 final points**, identical candidate IDs, XYZ, RGB, normals, camera/pair support, order and merge counts to the previous surface build. Maximum relative-sigma difference: 2.61e-18. **24,316** points have valid surface attributes, and every LAS attribute was checked against its corresponding confidence row. Evidence: `target/collinea-city-comparison.json`, `target/collinea-city-installed.log`. No surveyed accuracy claim follows from this comparison.
- Windows launcher zero-argument help equals `-h`; all parser arguments are checked for help coverage by the suite. The standalone source directory also compiles independently.

### Consolidation resource replay

The benchmark stages **6,503,432 accepted synthetic samples** with known source-depth evidence, 16 cameras, eight pairs and 256 spatial partitions. Both versions use a 4 GiB Java heap and a 128 MiB surface working-set budget. Original and updated output hashes include ordered representative IDs/XYZ/support/counts and merged candidate-to-representative membership, and are identical: `78eed122b547f0f57eff4b689ef7ac1ca3b38dfbefb7cc57a69f23695fefe2f7`.

| Instrumented observation | Previous CPU consolidation | Updated auto consolidation |
| --- | ---: | ---: |
| Consolidation finish time | 276.8 s | 228.0 s |
| Process peak working set | 535.1 MiB | 748.8 MiB |
| Sampled peak private memory | 673.9 MiB | 1093.8 MiB |
| Device-wide observed GPU memory peak | 0 MiB | 234 MiB |
| Output representatives | 893,665 | 893,665 |
| Samples combined | 5,609,767 | 5,609,767 |

This pair of observed runs reduced consolidation time by approximately 17.6%. Earlier development trials ranged from 247 to 277 seconds for the previous implementation and 228 to 275 seconds for evolving new implementations; the first unconditional GPU-neighborhood prototype was slower. Auto mode measures backend cost and can keep neighborhoods on CPU while retaining batched GPU depth validation. Weighted medians/layer selection and greedy representative choices remain CPU work. Current query/median/transfer/I/O breakdowns are recorded in `06_fusion/surface_consolidation.json`.

Evidence: `target/collinea-benchmark-comparison.json`, `target/collinea-measured-{old,new}/benchmark.json`, and the corresponding `*-resources.json` files. Windows process working-set peaks are observed separately from once-per-second private-memory samples. GPU readings are whole-device values, not isolated per-process allocations. The first observer did not retain the updated JVM's exit code; its completed benchmark JSON, final completion marker and empty JVM stderr were verified without inventing an exit status. The baseline observer retained exit code zero.

These are single synthetic consolidation-only replays, not an end-to-end photogrammetry benchmark or a reproduction of the user's original 237-partition run, whose temporary evidence was already cleaned up. Normal estimation, images, SGM and final LAS/PLY writing are outside this benchmark. The supplied normals have no flatness metrics; unavailable metric records are compact. Real runs with valid attributes have larger surface records and outputs. Timings vary with geometry, support, disk/cache behavior and backend selection; no universal multiplier is claimed.

Streamed SGM now performs eight rather than nine cost-generation passes and seven rather than eight aggregate-volume reads/writes for eight paths. Together with eligible 16-bit storage, logical aggregate host/device payload is 28 rather than 64 bytes per pixel-hypothesis per matching direction (56.25% lower); the aggregate spool is half-sized. These are deterministic payload/layout counts, not physical disk-traffic or wall-clock measurements; paging/driver overhead remains additional.

Linux runtime remains unverified locally; cross-platform CI is configured for repository pushes. Cross-platform source/scripts and CI configuration are included. The source package excludes generated runs, jars, toolchains, internal work notes and credentials. Repository preference: **POSE**, **private**, **no project license assigned**.

## Reprojection refinement and normal-aware consolidation

The full Windows/JDK 25 run completed **1,530 checks** (`target/surface-final-suite.log`). The final focused suite then passed **58 checks on each NVIDIA and AMD GPU** (`target/surface-final-focused.log`, `target/surface-final-amd.log`), including the last voxel/debug-output compatibility checks. Installed CLI/help/heap checks pass **168 assertions**. All **129 production class files** and 52 test class files match the release build by SHA-256; earlier installed builds remain under `target/pre-surface-*`.

- Defaults are `--depth-refinement reprojection --surface-consolidation normal`. Both CPU and FP64 GPU implement the linked inverse-depth fit. Spatial normal extraction/consolidation runs on CPU.
- Controlled sloping-plane RMS falls **8.63 -> 1.74 mm**, including after rotating the plane into a wall at large projected coordinates. A one-pixel foreground strip retains **222/222** validated observations; close parallel layers and opposing thin-surface sides survive without blending.
- Prefetched and synchronous full reconstructions produce identical accepted representatives, support, coordinates, confidence and LAS bytes. Failure, cleanup, budget-passthrough, even-weight median and legacy voxel-decision checks pass.
- The fixed-camera city replay retains **35,121** representatives from 42,599 accepted samples, combining 7,478 observations and refining 14,811 positions. In the unsurveyed ROI, median local p90-p10 spread changes **0.197 -> 0.162 m** from the old weighted/no-consolidation baseline. A fixed-plane comparison on the same 2,180 candidate IDs gives **0.196 -> 0.164 m**. Reprojection alone is slightly worse on this patch; these are surface proxies, not ground-truth accuracy.
- All **42,599** pre-consolidation candidate IDs, support, order and RGB agree between CPU and both GPU layouts. Maximum raw coordinate difference is **8.21e-8 m**; LAS quantization can turn that into a 0.1 mm bin difference, so cross-backend LAS byte identity is not claimed.
- The final city run verifies **64 artifact sizes/SHA-256 hashes** and **29 JSON files**. Compressed partition peak usage is **6.60 MiB** for this replay and all surface work files are removed. The report/planner expose additional temporary-disk requirements for larger projects.

See [SURFACE_FUSION.md](SURFACE_FUSION.md) and `target/surface-validation.json` for controls, limits, build hashes and evidence paths. Windows and both local GPU vendors were exercised; Linux runtime execution remains untested here.

## Thin structures and revised SGM defaults

The current build passes **1,468 checks** (`target/fence-release-suite.log`). Defaults are P1=4/P2=24, edge scale 32, a guarded edge-preserving median, and eight-connected speckle grouping. The installed 115 production classes match the tested build byte-for-byte; the preceding build is retained under `target/pre-fence-*`. Installed CLI/help/heap checks pass 161 assertions.

`FenceRegression` passes **28 checks on NVIDIA and AMD**, covering known ray-plane fence depths, one-pixel disparity jumps, width/texture variations, median behavior, diagonal connectivity, invalid controls, CPU/resident/streamed GPU parity and explicit restoration of classic settings. The broad four-cost/4-or-8-path stereo parity suite passes **81 checks** on both vendors. Streamed edge adaptation uses the actual predecessor across strip boundaries.

Known fence disparity recovery improves from 8,613 to 17,591 pixels for a large depth jump, and from 16,049 to 19,070 for a one-pixel jump, aggregating twelve scenes per geometry. Combined incorrect foreground/background labels decrease in both groups, with over 99.7% of previous valid background coverage. The six clean/noisy flat/sloping-plane controls add **12 checks**; they quantify the detail/smoothness tradeoff rather than asserting universal improvement. Existing noise, false-layer, signed-disparity, disk/RAM parity, registration and end-to-end checks continue to pass.

Representative source, truth and old/new disparity PNGs plus per-stage JSON are in `target/self test 2519382224554540128`. AMD-focused evidence is in `target/fence-release-amd.log` and `target/fence-release-amd-fence.log`. Plane controls are in `target/fence-surface-controls-p24-e32.json`; installation metadata is in `target/fence-validation.json`. A fixed-geometry city replay completes, but neither that scene's fence location nor surveyed accuracy was established. See [THIN_STRUCTURES.md](THIN_STRUCTURES.md) for results, limitations and restoration/tuning commands. Linux execution remains unverified in this Windows session.

## Reconstruction upgrade: 2026-10-30

The full suite passes **1,423 checks** on Windows 11 / JDK 25 / OpenCV 4.9.0 (`target/upgrade-suite-release.log`). All 115 installed production classes match the tested build byte-for-byte; earlier builds are preserved under `target/previous-*`. Installed launchers pass identical no-argument/-h help and a 64 MiB to 213 MiB heap relaunch for a 128 MiB processing budget. The help audit covers every parsed flag and alias.

- CPU, resident GPU and exact streamed GPU comparisons pass **81 checks on each NVIDIA and AMD device**, covering four costs, four/eight paths, masks, signed ranges, strip boundaries and 693 hypotheses. No streamed working volumes remain after completion.
- A separate NVIDIA **900×1600×693** banked resident case uses 5,770.6 MiB estimated device storage and recovers 1,178,680 valid comparison-region pixels, all within 0.6 pixels of the independent known shift. Matching/filtering takes 3.624 s in this run; this is not whole-project timing.
- GPU fusion passes both vendors' differential fixtures; AMD's focused suite passes **60 checks**, including disk evidence, numerical guards and safe failure replay. A separate three-camera fixture reproduces the old invalid support after refinement and verifies its rejection.
- A real fixed-geometry city comparison retains identical accepted candidate IDs, support, order and LAS bytes under reference versus optimized fusion. Maximum raw PLY difference is **1.05e-9 m** across 55,843 points (`target/city-fusion-equivalence.json`).
- Orientation tests pass **584 checks** across eleven format names, independent convention fixtures and round trips, heterogeneous cameras, strict losses, legacy-run recovery and stable import cache identity. Additional basename-relative COLMAP/MicMac checks pass 45/23 assertions. A completed 17-camera city run exports eight formats without photographs; a COLMAP-imported three-photo reconstruction also completes.
- Analytical overlap tests pass **16 checks**, including directional/shared-area values, unknown policies, fallback-edge rejection and large projected offsets.
- Surface tests verify resize rays, robust sloped planes, read-only analysis and fractional disparity. Real city patches show mixed improvements; optional subpixel refinement remains off by default. No surveyed ground-accuracy claim is made.
- The corrected city catalog verifies **62 sizes/SHA-256 hashes** and **26 JSON files** (`target/city-upgrade-catalog-verification.json`).

See [RECONSTRUCTION_UPGRADE.md](RECONSTRUCTION_UPGRADE.md) for measurements/tradeoffs and [ORIENTATIONS.md](ORIENTATIONS.md) for supported interchange variants. Windows and both local GPU vendors were exercised. Linux runtime execution was unavailable here; proprietary receiving applications were not all exercised against the exports.

## FP64 GPU fusion

The complete Windows/JDK 25 suite passes **574 checks** (`target/fusion-full-validation.log`, fixtures `target/self test 15147632684457993228`), including 60 new fusion regressions. The same 60 checks pass on AMD (`target/fusion-amd-final-validation.log`, fixtures `target/fusion gpu 16503096868779646469`). Differential fixtures compare every decision, supporting-view/independent-pair count and coordinate against both the frozen original CPU algorithm and indexed CPU implementation; indexed CPU confidence/coordinates remain bitwise identical. Checks additionally cover numerical guards, genuine/conflicting layers, weak parallax, large coordinates, disabled refinement, disk-backed evidence, invalid candidates, four allocation banks, independent backend selection, insufficient-memory policies and automatic replay after a defined OpenCL buffer error.

A fixed-calibration/pose Sheffield Park pair replay exports exactly **56,072 points** on both CPU and GPU fusion, with identical coordinates, RGB/order, support counts and LAS bytes; maximum relative-sigma difference is 1.31e-18. Both projects pass all 63 artifact size/SHA-256 checks and 23 JSON parses per run (`target/fusion-real-pair-verification.json`). This verifies consistency rather than surveyed accuracy.

The initial controlled 32-camera/204-pair/66,303-candidate benchmark measures 27.867 s original CPU versus 3.920 s GPU batch work on NVIDIA (about 7.1×), and 33.099 s versus 23.425 s on AMD (about 1.4×). These include transfers, guards and auditing, but exclude preparation and final output. [GPU_FUSION.md](GPU_FUSION.md) documents policy, limits, commands, exact data and benchmark caveats. Linux runtime execution was not performed in this Windows workspace.

## Pair-specific lower and upper disparity bounds

The complete suite now passes **514 checks**, including 58 new range regressions (`target/pair-range-final-validation.log`, fixtures `target/self test 2532011139733187058`). Verified keypoints/sparse geometry drive both bounds with coverage checks and margins, a small whole-domain/full-resolution validation guard, and lower/upper adaptive retries. Tests cover minority depths, missing background layers, actual CPU/GPU endpoint counters, independent rectification and coordinate/uncertainty scaling, pinned/explicit settings, known depth and coverage, positive-minimum resume and verified-track reuse at a different dense resolution. Public no-argument/help variants succeed and all 108 parser options are documented.

The Sheffield Park 6–9 replay narrows 608 hypotheses to **49** (465–513) using verified keypoints and sparse geometry, with 99.64% of prior valid pixels retained. The 6–10 weak-feature replay conservatively retains a low tail but fits **709** hypotheses rather than 750, keeping the main pass on GPU; 99.95% of prior valid pixels remain. These are consistency comparisons to the earlier broad maps, not independent ground truth. [PAIR_RANGES.md](PAIR_RANGES.md) records methods, exact counts, timings and limitations.

## Depth-recording I/O repair

The complete build now passes **456 checks**, including 17 new depth-cache/resource regressions (`target/depth-io-full-validation.log`). The apparent Sheffield Park stall was repeated depth-page file I/O after matching, caused by an unnecessary per-map cache cap. Maps now use the shared bounded capacity and a bounded 32-handle LRU. A replay of the actual 608,258-candidate first pair records depth in 0.52 seconds; its disparity is identical to the running job's original disparity, and repaired disk/RAM stores produce identical 607,040-point two-image clouds. Backend/phase and throttled triangulation/fusion progress are now logged. See [DEPTH_IO_DEBUG.md](DEPTH_IO_DEBUG.md) for evidence, timings and candidate-count interpretation.

## Standard storage and OpenCL GPU update

The complete Windows/JDK 25 build now passes **439 checks** (`target/storage-gpu-final-validation.log`, fixtures `target/self test 11423759314254895284`). This includes the previous 381 checks under an explicitly selected CPU/debug/cache contract, plus **58** storage/GPU checks. The new checks exercise standard/minimal/debug output equivalence, default absence of caches, optional cache creation and same-key generation pruning, successful working-depth cleanup, retained debug evidence, final confidence, material storage reduction, required-device/budget failures and automatic CPU fallback. GPU comparisons cover exact disparities/rejection counts with four/eight directions, single/multiword and color-distance census, masks, signed and non-power-of-two disparity ranges, ranges larger than a work-group, coarse-to-fine tile seams/fallbacks and an independent known shift. An entire GPU standard-storage pipeline exports the same LAS bytes as all three CPU storage profiles.

The real full eleven-photo Kermit reconstruction completes in **62.07 seconds**, retains **112.75 MiB** versus the earlier full project's **2.21 GiB**, and exports the same **18,978 points** with byte-identical LAS and camera CSV. Nine of eleven images register; the same two failures are reported. All **490** cataloged artifacts pass size/SHA-256 checks and all **238** JSON files parse. No cache or working `.tiles` remains. A separate fixed-nine-camera comparison retains **49.89 MiB** versus **1.93 GiB**, with the same **17,445-point** LAS. These are retained project file sizes, not peak-working-disk measurements. Records: `target/kermit-full-storage-gpu-comparison.json`, `target/kermit-full-standard-catalog-verification.json` and `target/kermit-storage-gpu-verification.json`.

Real hardware parity passes on both the NVIDIA RTX 5070 Laptop GPU and the AMD `gfx1103` GPU. A warmed 640×384 / 128-disparity / eight-path benchmark measures approximately **24.8×** acceleration on NVIDIA and **6.4×** on AMD; it includes transfers and CPU post-filtering but excludes first-time compilation. Whole-pipeline runtime also depends on calibration, matching, fusion and output. See [STORAGE_GPU.md](STORAGE_GPU.md) for commands, measurements, storage policy and limitations. Linux loading/support is implemented but was not executed in this Windows workspace. Hardware tests skip when no OpenCL GPU is available; CPU storage tests remain active.

Help checks confirm successful no-argument/`-h`/`--help`/`-help` output and coverage of every parsed flag, including the new storage/cache/GPU options.

The final AMD hardware rerun passes all **26 GPU comparisons/checks**, including the expanded disparity ranges and pyramid seams (`target/amd-gpu-final-parity.log`, fixtures `target/storage gpu 7391235489068423271`). Its warmed benchmark measures 0.427 s GPU versus 4.252 s CPU (~10×); earlier concurrent-load measurements were ~6.4×. A public invocation omitting both storage and compute flags also selects NVIDIA automatically, exports 5,280 points and leaves no stage cache or working `.tiles` (`target/default-auto-storage-smoke.log`).

## Workflow items 1–11

The Windows/JDK 25 build passes **381 checks**, including 64 workflow regressions (`target/workflow-release-validation.log`, fixtures `target/self test 2619464703791035543`). New checks exercise bitwise equality of RAM and disk-paged SGM for four/eight paths and single/multiword census; signed disparities; coarse-to-fine recovery of an independently constructed eight-pixel shift across tile seams; page-cache eviction/readback and bounded shared depth-cache release; selected focal refinement with fixed radial/tangential terms and a strong focal prior; all five native detectors and compatible match metrics; independent alignment/dense dimensions; overlap shortlist behavior and GPS fallback; absolute supplied-pose priors; exact resumed LAS and restored stereo artifacts; confidence row/point correspondence; fusion-only reruns; corrupt-cache rejection; SIFT/ORB feature/match/geometry round trips; and continuation with three valid images plus one blank image. The latter exports 41,529 points and explicitly reports the failed fourth image.

The real Kermit run completes with 9/11 cameras and 18,978 points after correcting excessive rectification magnification. See [KERMIT_DEBUG.md](KERMIT_DEBUG.md) for commands, evidence and accuracy limits. [WORKFLOW_UPGRADE.md](WORKFLOW_UPGRADE.md) documents defaults, new controls, cache compatibility, disk requirements and approximation limits. GPU is deferred. Windows execution is tested; Linux launch/build support remains supplied but has not been executed in this workspace.

A controlled Kermit comparison fixes the same nine camera poses and calibration: `--rectification-alpha auto` exports 17,445 points versus 2,034 with `0`; pair 0000/0001 has 26,841 versus zero valid disparity pixels. Pose CSV hashes and calibration properties are identical. The corrected end-to-end project independently passes 3,745 artifact size/SHA-256 checks and 939 JSON parses. No surveyed Kermit ground truth is available.

The final help audit confirms identical successful output for no arguments, `-h`, `--help` and `-help`, and finds every parser option in the help text. A public planning-only smoke run verifies 320-pixel alignment versus 80-pixel dense sizing on Kermit with disk tiling, its resident/disk estimates and the generated project guide (`target/final-guide-smoke.log`).

After the final cache implementation-fingerprint coverage and generated-guide edits, all production sources compile again and the **64 workflow checks pass again** (`target/workflow-final-cache-validation.log`, fixtures `target/workflow 17395442424234480906`). These repeat checks are part of the 381-check suite, not 64 additional distinct tests.

## Earlier bundle and quality validation

The final bundle optimizer and resolution planning were upgraded on 2026-10-29. The complete Windows/JDK 25 build passes **317 checks** (`target/bundle-quality-full-validation.log`), including 66 new checks. The final optimizer jointly solves all poses and points with robust Schur LM, analytic Jacobians and matrix-free block-Jacobi PCG. A 16-camera / 300-point independently constructed scene, with a nonidentity anchored pose and a perturbed second seed, recovers every camera center and point RMS within 0.001 scene units. The first pose remains bitwise unchanged, the baseline norm is preserved within 1e-10, intrinsics stay fixed, and every committed iteration lowers the full objective. A minority of 25/-17-pixel observation corruptions still leaves mean clean-observation error below 0.2 pixels. This tests beyond the calibration bootstrap's 12-camera limit.

Resolution checks cover all four named factors, preserved default/custom behavior, no upsampling, rounding, order-independent explicit caps, filtering invariance, options copying, decoded image sizes and intrinsic scaling. Header-only planning records over-budget scenarios without allocating volumes; reconstruction rejects an impossible explicit search before decoding. Public planning-only and half-resolution stereo workflows pass, with stage/provenance checks and actual per-pair memory estimates. Half-resolution stereo exports 43,792 points from the controlled shifted-image fixture. GPU computation remains deferred; Linux launch/build support remains supplied but was not executed in this Windows workspace.

The seven supplied DJI photographs also completed through automatic calibration, joint final adjustment, automatic disparity ranges, EPSG:3067 GPS alignment and dense fusion:

```powershell
.\sgm_java\photogrammetry.bat -i example_data/5x_test -o sgm_java/output/global-bundle-dji5x --quality high --max-dimension 800 --neighbors 3 --cores 2
```

The explicit cap produces 800 × 599 images. The plan reports approximately 205/381/733/1436 MiB of SGM storage at 64/128/256/512 disparities, plus potential retained depth evidence of 110 MiB and other allocations. The final robust bundle objective drops from 4310.513 to 967.452 over eight accepted steps, and post-filter sparse reprojection RMS is 0.4014 pixels. All seven GPS controls are inliers, with camera-center RMS 0.575 metres. The run exports 278,087 points from 630,991 candidates in approximately 196 seconds. Its catalog independently passes **1,732 artifact size/SHA-256 checks and 450 JSON parses**.

In this run the guarded lens refinement is **rejected**: held-out robust RMS changes only from 0.5642 to 0.5603 pixels, below the existing acceptance thresholds. Consequently the applied focal remains the EXIF initialization, 4574.613 source pixels, with zero distortion. The better pose/point bootstrap can change calibration evidence; the proposed coefficients are preserved in the project but are not applied. This differs from the earlier accepted DJI model and is not a measured calibration assertion.

Compared with the earlier `output/layer-fusion-dji5x` run on 272 common bright/neutral one-metre patches, median local plane-residual thickness changes from 0.1425 to 0.1087 metres and the patch p95 from 0.3289 to 0.2531 metres (`target/global-bundle-automatic-comparison/metrics.json`). These are scene/material proxies, not surveyed ground accuracy. Applied intrinsics, poses, disparity intervals, accepted points and GPS alignment differ, so this end-to-end comparison does not isolate the causal effect of bundle adjustment.

A second DJI run reuses the earlier **estimated** calibration unchanged to avoid the lens-selection difference:

```powershell
.\sgm_java\photogrammetry.bat -i example_data/5x_test -o sgm_java/output/global-bundle-dji5x-measured-model --calibration sgm_java/output/layer-fusion-dji5x/camera.txt --quality high --max-dimension 800 --neighbors 3 --cores 2
```

Despite the historical directory suffix `measured-model`, this file is the prior automatically estimated model, not an independently measured calibration. The run finishes in approximately 140 seconds with 264,885 points, robust objective 5604.916 to 959.709, sparse reprojection RMS 0.3953 pixels and GPS fit RMS 0.560 metres. Across 639 common bright/neutral patches, median thickness changes from 0.1402 to 0.1361 metres and p95 from 0.3501 to 0.3465 metres. The larger exported count therefore comes with similar local thickness on this proxy, rather than establishing a universal accuracy improvement. Camera initialization/poses, disparity ranges, visibility, GPS fit and point membership still change; this is a fixed-intrinsics comparison, not a fixed-geometry ablation (`target/global-bundle-fixed-intrinsics-comparison/metrics.json`).

Filtering-intensity presets (`--filtering mild|moderate|aggressive`) were previously added with moderate preserving existing defaults. That Windows/JDK 25 build passed **251 checks**, including 47 preset-specific checks: all five profile values, resolution/matching/view-count invariance, copy propagation, order-independent explicit overrides, repeated/case-insensitive presets, invalid arguments, actual depth-consensus acceptance differences, complete reconstructions for all three profiles, a custom override run, and saved preset/effective-threshold provenance. Explicit moderate and omitted filtering produce byte-identical LAS output on the controlled stereo fixture. Resolution presets have since been added independently as described above.

Layered-surface filtering and multi-pair depth refinement were then added. The full Windows/JDK 25 suite passed **203 checks**, followed by an additional final-coordinate provenance assertion verified in a focused project-geometry run (**204 unique checks**). These include competing depth bands, true stacked surfaces, per-camera endpoint support, depth uncertainty, subpixel evidence and final-coordinate provenance. The seven-image DJI dataset was run with both frozen old geometry and the latest complete pipeline. On 368 common bright/neutral one-metre patches, the fusion-only median local thickness decreased from 0.803 to 0.199 metres. The latest complete run retains 115,035 points versus 132,402 previously; counts alone are not a quality metric. Its catalog independently passed 1,745 artifact size/SHA-256 checks and 450 Windows JSON parses. See [LAYER_DEBUG.md](LAYER_DEBUG.md) for definitions, coverage changes, limitations and reproduction commands.

Guarded shared `f,k1,k2` self-calibration and automatic per-pair disparity ranges were subsequently implemented. The full Windows/JDK 25 suite passes **191 checks**. The 27 refinement/range checks cover the joint fit, pose/structure marginalization, held-out error, planar degeneracy rejection, retaining a correct zero-distortion model, coefficient recovery through SIFT/bootstrap, wide stereo search, fixed user overrides, and full reconstruction without calibration/focal/disparity arguments. The final cloud is checked against independent source geometry, including preservation of the accepted seed orientation.

Known-value results:

- Synthetic observations: true `f=800, k1=-0.16, k2=0.065`; estimated `800.061, -0.159851, 0.064801`. Held-out robust RMS: 5.410 to 0.095 pixels.
- Planar parallel-camera scene: RMS improved from 1.030 to 0.098 pixels, but the intrinsic information eigenvalue ratio was only `2.94e-10`; the candidate was correctly rejected as unobservable.
- Six rendered distorted photographs: true `f=420, k1=-0.14, k2=0.07`; estimated `420.064, -0.140384, 0.070229` at maximum dimension 640. Held-out robust RMS: 1.125 to 0.093 pixels.
- Full default-argument reconstruction of JPEG versions at maximum dimension 480: initial focal estimate 503.767 pixels refined to `419.999, -0.139933, 0.069675`. Held-out RMS: 0.479 to 0.093 pixels. Final seed orientation is carried from the accepted fit; 6,178 fused points were exported. After similarity alignment to the independently specified camera centers, 99.98% lie within 0.25 source-scene units of the known planes; median surface error 0.0105 units, 95th percentile 0.0617 units. This verifies final geometry in addition to fitted coefficients/reprojection error.
- An actual 180-pixel rectified shift, beyond the previous default 128: automatic search selected 224 hypotheses and reconstructed the known metric plane with mean depth error below 0.005 units.
- A complete six-image run using a focal/sensor seed and no disparity argument also accepts the radial model. Its preserved project history is `output/self-calibration-project/`; this is synthetic validation, not a surveyed accuracy claim. See that project's latest successful run for final clouds and diagnostics.

Automatic calibration preserves supplied calibration files, keeps the principal point centered, and fits only the two radial terms. Rejected refinement retains initialization. Per-pair range estimates supplement preview matches with visible sparse geometry, preserve each retry's rasters and stop at explicit resource limits. Linux build/launch support remains supplied but was not executed in this Windows environment.

Automatic focal calibration and projected GPS georeferencing were added on 2026-10-29. The complete suite passed **162 checks** after those changes; a subsequent targeted run adds two CRS rejection checks, for **164 unique checks** (105 previous checks plus 59 calibration/GPS checks). Windows/JDK 25 was used. Linux launch/build support is updated, with execution still awaiting the supplied CI matrix.

The added coverage includes:

- Exact focal recovery from a known fundamental matrix, followed by SIFT-based recovery of a 420-pixel focal length as 419.9 pixels from rendered non-planar, rotated views without EXIF.
- Degenerate image geometry failing explicitly without metadata, and EXIF 35mm-equivalent fallback being identified in the calibration report.
- JPEG/classic TIFF EXIF, both TIFF byte orders, hemisphere and negative-altitude references, missing altitude, malformed metadata, and focal tags.
- EPSG:3067 central-meridian controls and independently specified northing, EPSG:3857 analytic coordinates, and invalid/geographic/geocentric/foot CRS rejection.
- Robust GPS scale/rotation/translation with gross outliers, collinearity rejection, partial GPS, forced GPS failure, `--no-gps`, and preventing arbitrary `--scale` on referenced coordinates.
- Independent binary decoding of LAS GeoTIFF VLR keys, point-data offset, EPSG, units and actual projected coordinates; PLY and JSON CRS metadata.
- Full synthetic reconstruction with supplied local poses and automatic GPS: projected camera centers agree with known controls within 2 mm; 2,981 dense points were exported in EPSG:3067.
- Full image-only reconstruction without calibration/poses, with GPS and `--epsg 32635`: focal estimate 420.03 pixels for a true 420 pixels, all four cameras registered, GPS fit RMS 0.168 m, and 1,094 independently corroborated dense points. These are synthetic results, not surveyed absolute accuracy. One wide-baseline pair reaches the 128-disparity limit; its warning and rejection statistics are retained.

The supplied `example_data` photographs also complete without calibration input. Weak focal geometry uses EXIF (577.81 pixels at 750x1000 source resolution); their GPS controls are too close to a line for stable 3D alignment, so the cloud correctly remains unreferenced. Lens distortion is assumed zero in automatic focal mode; point counts from this run do not establish the quality of a full lens calibration. The project is under `output/automatic-calibration-check/`.

Project export was subsequently added and validated on Windows/JDK 25 with **105 passing numerical, integration, and project-artifact checks**. The populated example contains 377 indexed artifacts; independent checks parsed every JSON/event record, verified all catalog sizes/hashes, and confirmed byte-identical LAS and camera outputs against the repaired baseline. See [PROJECT_FORMAT.md](PROJECT_FORMAT.md) for the layout and verification details.

**Updated after a user-reported noise defect:** successful execution and point counts in the original runs below did not establish cloud quality. See [NOISE_DEBUG.md](NOISE_DEBUG.md) for the reproduced bug, corrected output, quality measurements, and new regression tests. The original point counts below describe the initial implementation and are retained as historical execution records.

Environment: Windows 11 x64, Temurin JDK 25, Java 25 compilation target, OpenCV 4.9.0. Tests were run on 2026-10-29. The JDK archive was checked against the SHA-256 checksum in Adoptium's release metadata.

## Numerical and synthetic integration suite

Run `build.bat` or `bash build.sh` from this directory. It checks:

- A known six-pixel stereo shift, including signed disparity ranges.
- A multiword 11x11 census descriptor and four-direction aggregation.
- The original RGB-distance descriptor on chromatic texture with a constant grayscale input, plus flat-color rejection; known-pose fusion also runs in color-distance mode.
- Ambiguous flat texture rejection and SGM allocation limits.
- DLT triangulation against an independently specified 3D point.
- A similarity fit against a known rotation, translation, and scale.
- WGS84 equatorial ECEF coordinates.
- LAS header, record length, point/return counts, coordinate round trips, and 16-bit RGB bytes.
- Camera calibration against rendered perspective chessboards with known focal length. Observed calibration RMS: approximately 0.0905 pixels.
- Rectified stereo against three independently defined depth planes (4, 6, and 10 units), with 0.25-unit baseline; over 90% of the reconstructed depths must lie within 0.25 units of a plane.
- Known-pose three-camera reconstruction, vertical stereo, and automatic reversed-baseline handling.
- Full SIFT, RANSAC essential geometry, seed initialization, third-camera PnP, bundle refinement, rectification, Java SGM, and LAS export.
- Bundle objective reduction from perturbed 3D points and a perturbed third camera. An observed run reduced the robust objective from approximately 293.87 to 0.00063 in 12 iterations.
- CLI help, invalid inputs, unknown options, and filesystem paths containing spaces.

Fixtures and outputs remain under `target/self test ...` for inspection. Assertions are explicit, so tests cannot silently pass when JVM assertions are disabled.

## Original repository photographs

These commands are run from the **original C++ repository root** after building `sgm_java`. Both completed using the images and legacy calibration files already present in that repository.

```bat
sgm_java\photogrammetry.bat -i example_data -o sgm_java/target/original-example --calibration calibration/resize_25/calib.result --max-dimension 800 --disparities 96 --step 3 --cores 2 --bundle-iterations 10
```

Observed: all three cameras registered; 1,925 sparse tracks; 101,114 exported RGB points; approximately 13.6 seconds. The seed was `2.jpg / 3.jpg`, with baseline 1 in arbitrary units.

```bat
sgm_java\photogrammetry.bat -i example_data/forest/temp -o sgm_java/target/forest-example --calibration calibration/resize_50/calib.result --max-dimension 800 --disparities 96 --step 3 --cores 2 --bundle-iterations 10
```

Observed: all three cameras registered; 88,244 exported RGB points; approximately 10.9 seconds. The seed was `img1.JPG / img2.JPG`, with baseline 1 in arbitrary units.

Use a new output directory or add `--overwrite` when repeating these commands. The shell equivalent on Linux is `bash sgm_java/photogrammetry.sh ...`.

## What has not been validated

- The original C++ executable has not been run for a side-by-side comparison. These real-image runs establish execution and output generation, not agreement with C++ or surveyed ground truth.
- The calibration files were taken from the repository; their physical suitability for each photograph set was not independently established.
- Linux execution and the supplied CI matrix have not been run in this Windows environment.
- No external LAS.utils checkout has been modified or used to execute this module. Source/launcher conventions were inspected remotely. The generated files follow the standard LAS 1.2 format and are covered by binary decoding checks.
- Large-flight performance, GPS accuracy, and native OpenCV behavior on other machines require further validation.

See [PORTING.md](PORTING.md) for deliberate algorithm differences and remaining legacy-mode parity work.

## Large-image production update: 31 October 2026

The complete Windows suite passed **1,927 checks** with OpenCV 4.9.0. Exact resident/RAM/disk/recompute parity additionally passed 258 checks on each NVIDIA and AMD device; production compact matching passed 36 checks on each. Image/cache regressions passed 37 checks. A 6000 x 4000 tiled TIFF and BigTIFF were each decoded both at native size and 1600 x 1067 under a 64 MiB Java heap (native processed matrices are additional). A virtual BigTIFF fixture also tests physical pixel offsets beyond 4 GiB without creating a large gap file.

A real rectified forest pair at 1978 x 2640 / 1037 hypotheses passed complete filtered-disparity and rejection-counter parity between exact RAM streaming and compact matching with mandatory exact verification. Both retained 128,905 depth pixels; canonical float-bit SHA-256 `af9cbbdd4d5f2d301d3e8d2d98d3f3825ddf44692a9a75f1d8f93b5df80ed74c`. RAM stereo/filtering took 54.93 s; compact took 104.22 s. See [SCALING.md](SCALING.md) and [machine-readable evidence](research/scaling/results-2026-10-31.json) for memory measurements and limits.

These tests do not establish native-scale Vexcel reconstruction, surveyed accuracy or reduced surface noise. The Linux missing-OpenCL discovery path is corrected; its current CI result is recorded separately in repository checks.
