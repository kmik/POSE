# GPU depth fusion

The subsequent [surface-fusion update](SURFACE_FUSION.md) adds reprojection-based depth refinement on CPU/GPU and a CPU normal-aware consolidation stage. That stage can move, reorder and combine points; the exact weighted-estimator/order comparisons below describe the earlier baseline.

For the current coalesced layout, direct geometric weights, asynchronous validation/output, detailed profiling and corrected measured-camera reprojection check, see [RECONSTRUCTION_UPGRADE.md](RECONSTRUCTION_UPGRADE.md). Measurements and limits below describe the original GPU-fusion baseline. Explicit VRAM budgets can now exceed 70% up to a 95% cap.

Fusion now supports OpenCL GPU processing. Default `--compute auto` also selects GPU fusion for large workloads when double precision and the memory budget are available. The Windows and Linux launchers need the existing JDK 25 and vendor OpenCL runtime.

## Controls

| Option | Fusion behavior |
| --- | --- |
| `--compute auto` | Default: try GPU when candidate count × pair count is at least 1,000,000. Fall back on unsupported hardware, insufficient memory, or runtime/audit failure. |
| `--compute cpu` | CPU stereo and CPU fusion. |
| `--compute gpu` | Require GPU stereo and GPU fusion. |
| `--fusion-compute inherit` | Default: follow `--compute`. |
| `--fusion-compute cpu` | Force CPU fusion independently of stereo. |
| `--fusion-compute auto` | Allow fusion fallback independently of stereo's policy. |
| `--fusion-compute gpu` | Require GPU fusion independently of stereo. |
| `--gpu-device N` | Select the same OpenCL device index used by stereo. |
| `--gpu-memory-mb N` | Bound estimated GPU allocations for either stage; zero uses 70% of device VRAM. |

An ordinary automatic run on Windows:

```powershell
.\sgm_java\photogrammetry.bat -i example_data/sheffield_park -o sgm_java/output/park --quality medium --neighbors 10 --ratio 0.8
```

Use `--compute gpu --fusion-compute auto` when stereo must use GPU but fusion may fall back. Use `--compute auto --fusion-compute cpu` for CPU fusion with automatic stereo. Linux uses `bash sgm_java/photogrammetry.sh` with the same arguments. Existing processes retain loaded code; changes apply on the next invocation.

## Computation and filtering

GPU work includes camera projection, nearest depth lookup over the same 3×3 neighbourhood, per-camera majority modes, distinct-camera/independent-pair support, inverse-variance depth refinement, reprojection verification and uncertainty rejection. Original thresholds, majority fractions, median choices, pair ordering, weight clipping and supporting-view rules are preserved. No ground plane is assumed.

Geometry uses FP64 without fast-math or fused-expression contraction. Evidence retains float32 camera-Z and exact packed signed subpixel offsets. Lookup runs in map-major batches for locality. Each refinement work item owns one candidate and its bounded scratch space. Candidate output order is unchanged. Cloud writing, optional voxel deduplication and confidence compression remain on CPU.

CPU fusion now indexes pairs by camera, projects each candidate once per camera for initial lookup, and precomputes camera centres/inverse rotations. The operation order for measurements and refinement is preserved. A frozen original algorithm exists only in test sources for differential checks.

## Numerical checks and fallback

Candidates near projection/lookup ties, depth/reprojection thresholds, competing mode ties, weighted-median boundaries, uncertainty limits, or non-finite geometry receive CPU rechecks. Every GPU batch also audits up to eight evenly spaced candidates against CPU decisions, support counts, coordinates, uncertainty and refinement shift before publishing that batch.

A detected disagreement or OpenCL runtime failure replays the entire unpublished batch on CPU in automatic mode; subsequent batches use CPU. Required GPU mode reports the failure. Numerical CPU rechecks remain active in required mode. Sparse auditing is not a proof of identical results for every possible input. Small floating-point differences remain possible; CPU mode supplies a reference comparison.

## Memory and progress

Depth evidence is uploaded once and stays resident. Up to eight allocation banks each hold at most 1 GiB, also respecting the device's individual allocation limit. Candidate capacity starts at 4,096 and shrinks to fit estimated scratch/allocation budgets, with a minimum of 32. Host staging is 16 MiB; full evidence is not duplicated in a Java array.

Complete evidence must fit the selected GPU budget. This version does not stream spatial subsets for projects exceeding VRAM; automatic mode uses CPU instead. Runtime allocation failure also triggers fallback. Driver program memory and other processes affect actual available memory.

Logs show the device, camera/pair counts, evidence size, batch capacity, progress during long uploads, preparation time and fusion backend. Existing candidate/retained/rejected progress remains.

Completed runs retain `06_fusion/backend.json`: GPU/CPU candidate counts, numerical CPU rechecks, audit sampling, CPU depth-cache budget and active GPU resource/guard details. GPU counts include candidates later replayed by numerical guards; the separate guard count identifies them. Storage profiles and working-depth cleanup remain in effect.

## Measurements and validation

The controlled benchmark has **32 cameras, 204 pairs, 66,303 candidates and 199.2 MiB of evidence**. Every result is compared to the frozen original and indexed CPU algorithms. Batch times include transfers, numerical CPU rechecks and auditing; they exclude one-time GPU preparation and final project output.

| Device | Original CPU | Indexed CPU | GPU batches | Largest coordinate difference |
| --- | ---: | ---: | ---: | ---: |
| NVIDIA RTX 5070 Laptop GPU | 27.867 s | 26.924 s | 3.920 s | 1.24×10⁻¹⁴ scene units |
| AMD gfx1103 | 33.099 s | 31.210 s | 23.425 s | 7.11×10⁻¹⁵ scene units |

Initial batch work is approximately 7.1× and 1.4× faster than the original CPU algorithm, respectively. These hardware/workload-specific measurements do not predict complete reconstruction acceleration. Both retain 66,300 points with identical decisions and view/pair counts; 7,549 candidates receive conservative CPU rechecks.

Initial differential fixtures also cover dominant/stacked/conflicting surfaces, negligible parallax, disabled refinement, disk evidence, off-image/behind-camera/non-finite candidates and large map coordinates. The largest NVIDIA coordinate difference across these fixtures is 5.82×10⁻¹¹ map-coordinate units. Indexed CPU coordinates, confidence and decisions remain bitwise identical to the frozen algorithm.

Initial logs: `target/fusion-benchmark-validation.log` / `target/fusion-amd-validation.log`; fixtures `target/fusion gpu 3978742719557671675` / `target/fusion gpu 13995425687542942098`. Each fixture saves `comparison.json`. Final regressions add allocation-bank boundaries, backend selection, insufficient-memory policies and recovery from a defined OpenCL buffer error. Windows hardware is tested; Linux runtime execution is not available here.

The final complete suite passes **574 checks**, including 60 fusion checks (`target/fusion-full-validation.log`, fixtures `target/self test 15147632684457993228`). AMD independently passes all 60 fusion checks (`target/fusion-amd-final-validation.log`, fixtures `target/fusion gpu 16503096868779646469`). These compare 120,924 candidate results across the eight fixtures, including non-finite/off-image cases and a final partial batch; additional checks exercise memory/failure policies and allocation banks.

A real Sheffield Park 6–9 pair replay fixes calibration, poses, resolution, stereo backend and filtering while changing only the fusion backend. Both export 56,072 points with identical coordinates, RGB/order, supporting views, independent-pair counts and LAS bytes. Relative-sigma differences stay below 1.31×10⁻¹⁸; refinement shifts agree exactly. Both project catalogs pass all 63 size/SHA-256 checks and 23 JSON parses. See `target/fusion-real-pair-verification.json` and `target/fusion-park-pair-cpu.log` / `target/fusion-park-pair-gpu.log`. This small two-image comparison validates outputs; default automatic fusion chooses CPU for such a small workload because preparation can outweigh acceleration.
