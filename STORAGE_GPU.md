# Storage profiles and GPU matching

The current [large-image update](SCALING.md) supersedes the disk-streaming defaults below. Auto now uses RAM-backed exact streaming, then compact proposals with exact verification or bounded recomputation. Aggregate disk storage requires an explicit request. Earlier measurements and limits below are historical.

[RECONSTRUCTION_UPGRADE.md](RECONSTRUCTION_UPGRADE.md) supersedes the original single-buffer SGM limits below. Current SGM banks resident allocations and can stream exact continuous paths when needed; explicit VRAM budgets can reach 95% of device memory. Storage-profile defaults are unchanged. Streamed aggregate working files still require temporary disk space, documented in the upgrade guide.

The defaults are now **standard storage**, **no stage cache**, and **automatic GPU selection**. Existing commands keep the same image resolution and filtering settings. The update does not remove data from earlier runs or earlier `cache/v2` directories.

GPU support now also covers depth fusion (lookup, consensus, support/refinement and uncertainty checks) on FP64-capable devices. It includes CPU numerical guards, automatic fallback and an independent `--fusion-compute` override. See [GPU_FUSION.md](GPU_FUSION.md) for resource limits, commands and measured comparisons. Earlier stereo-only measurements below remain historical measurements of that stage.

Automatic disparity estimation now bounds both ends of each pair's search using verified keypoints and sparse geometry, with a coarse safety check and adaptive expansion. This can substantially reduce the allocation that previously forced some pairs onto CPU. See [PAIR_RANGES.md](PAIR_RANGES.md) for controls and measured examples; genuinely wide intervals may still require CPU processing.

## Run it

From the repository root on Windows:

```powershell
.\sgm_java\photogrammetry.bat --list-gpus
.\sgm_java\photogrammetry.bat -i example_data/kermit -o sgm_java/output/my-kermit --neighbors 10 --ratio 0.8
```

On Linux:

```bash
bash sgm_java/photogrammetry.sh --list-gpus
bash sgm_java/photogrammetry.sh -i example_data/kermit -o sgm_java/output/my-kermit --neighbors 10 --ratio 0.8
```

`-h` lists all arguments. GPU use requires a 64-bit JDK 25 and a working vendor OpenCL runtime. The launcher already enables Java native access. No CUDA toolkit, CUDA-enabled OpenCV rebuild, additional Java jar or locally compiled JNI library is required. CPU mode continues to work without OpenCL installed.

## Storage policy

| Argument | Retained data |
| --- | --- |
| `--storage standard` (default) | Input provenance and processed color images; calibration and final orientations; keypoint coordinates/overlays and accepted match tables/previews; pair geometry/ranges; one final numerical disparity and preview per pair; final clouds, point confidence, quality summaries and logs |
| `--storage minimal` | Input/calibration/pose provenance, final geometry/clouds/confidence, summaries and logs; omits image, keypoint/match and disparity image exports |
| `--storage debug` | The complete earlier audit capture, including descriptors, rejected match decisions, map rasters, intermediate stereo attempts, per-candidate decisions/refinements, depth-evidence PFMs, working depth maps and intermediate geometry snapshots |

Storage selection does not change matching, filtering or reconstruction resolution. Standard/minimal skip producing the omitted artifacts; they do not first write the full debug capture and delete it afterward. Ordinary stage summaries and numerical final geometry remain available. Optional `--copy-inputs` still archives original photographs independently of the selected profile.

After successful fusion, standard/minimal close and delete their working depth/offset `.tiles` files. Candidate binary scratch and CPU SGM cost/aggregate files are cleaned as before. Debug intentionally retains depth maps for investigation. Failed runs retain working depth evidence for diagnosis; successful cleanup never removes files from a previous run. Final point confidence remains a separate compressed table in `07_results/point-confidence.csv.gz` in every profile.

Depth maps share their global page budget without a separate per-map cap. A bounded cache of 32 open file handles avoids repeatedly reopening files for each small page; all handles close when their maps close. Long triangulation and fusion stages report progress. See [DEPTH_IO_DEBUG.md](DEPTH_IO_DEBUG.md) for the Sheffield Park I/O repair and exact disk/RAM comparisons.

With `--cache`, the reusable feature/match/stereo payloads take extra disk space even in minimal storage. With `--tiled` and CPU execution, peak scratch storage can still be approximately **6 × width × height × disparities bytes** for cost/aggregation, independently of the retained-output profile. Fusion also needs per-pair depth evidence until corroboration finishes. GPU execution keeps its cost/aggregation volume in GPU memory and creates no disk cost volume. These distinctions matter when assessing peak working disk versus the final project size.

## Optional caching

Caching is off by default. `--cache` enables writing reusable results, and `--resume` or `--rerun-from STAGE` enables both writing and compatible reuse. `--no-cache` explicitly disables caching and conflicts with resume/selective rerun requests.

```powershell
.\sgm_java\photogrammetry.bat -i D:/photos -o D:/project --cache
.\sgm_java\photogrammetry.bat -i D:/photos -o D:/project --resume
.\sgm_java\photogrammetry.bat -i D:/photos -o D:/project --rerun-from fusion --step 1
```

Repeat relevant input and processing options; resume does not implicitly load the previous command line. New caches live under `cache/v3/<storage>/`, so a minimal/standard cache is not incorrectly used to satisfy an exhaustive debug export. Cache content/settings/implementation checks remain enforced. Once a replacement generation is completely written and atomically published, superseded completed generations **of that same key** are removed. Distinct input/settings keys are retained for reuse; there is no automatic total-cache quota.

Old audit runs, old cache versions and old compatibility exports are preserved. To reclaim their existing space, archive unwanted old projects or remove their caches while those projects are idle. Removing a cache prevents reuse of that cache but does not remove already saved clouds or run artifacts. New runs in an existing large project do not automatically shrink its history.

## GPU controls

| Argument | Behavior |
| --- | --- |
| `--compute auto` (default) | Uses the selected OpenCL GPU for worthwhile volumes (at least one million pixel/disparity cells) that fit device and host limits; otherwise CPU. Native allocation/runtime failures fall back to CPU, subject to the normal CPU resource guards. |
| `--compute gpu` | Requires GPU matching; unavailable devices or insufficient GPU resources produce an explicit error instead of silently switching to CPU. |
| `--compute cpu` | Uses the CPU implementation and does not probe OpenCL during matching/planning. |
| `--list-gpus` | Lists GPU indices, names, driver versions, reported global memory and maximum allocation; exits without requiring input or creating a project. OpenCL CPU devices are excluded. |
| `--gpu-device N` | Selects the zero-based GPU index (default 0). |
| `--gpu-memory-mb N` | Caps the estimated GPU working set. Default 0 uses 70% of reported device global memory; maximum single-allocation and local-memory limits also apply. |

Device memory is not reserved by the estimate. Other applications and driver limits can still make allocation fail. Integrated GPUs may report shared system memory. The backend logs the selected GPU; fallback messages explain unavailable devices or native execution failures. `00_inputs/resolution_plan.json` records requested compute mode, device index, GPU volume estimates and feasible disparity counts, alongside CPU RAM/disk estimates. Per-pair range metadata retains the requested compute mode and GPU working-set estimate.

For large images, use the existing coarse-to-fine mode to keep local refinement volumes manageable:

```powershell
.\sgm_java\photogrammetry.bat -i D:/photos -o D:/project `
  --compute auto --pyramid-levels 3 --tile-size 512 --tile-overlap 32 --tiled
```

`--tiled` supplies a disk-paged CPU fallback; fitting GPU volumes use VRAM even when this flag is present. Coarse-to-fine matching remains approximate because it changes local search/context; the storage profile itself does not introduce that approximation. No option silently lowers image resolution.

## What executes on the GPU

Java 25's Foreign Function and Memory API calls the installed OpenCL 1.2 runtime. Embedded OpenCL kernels construct the multiword census-Hamming cost volume, aggregate all four/eight global SGM directions, and extract the integer winner/neighbour costs. Independent scanlines run in parallel; threads within a scanline share local path state and parallelize disparities. Direction dispatches use an in-order queue. This preserves path continuity and normalized integer recurrence rather than independently resetting SGM at arbitrary image boundaries.

Raw census descriptor extraction and texture/mask handling stay on CPU. Winner rejection, uniqueness and subpixel arithmetic also use the existing Java double/float semantics on CPU, avoiding reliance on GPU double-precision support. Left/right validation, median, ZNCC, speckles, triangulation, calibration, feature matching, bundle adjustment and fusion remain CPU work. CPU/GPU matching is interchangeable, including pyramid tiles, signed ranges, masks, grayscale/color-distance census and multiword windows.

The OpenCL context/program is reused within a run and released afterward; per-pass buffers are released after completion. No new Maven dependency is introduced. Technical references: [Java 25 native linker](https://docs.oracle.com/en/java/javase/25/docs/api/java.base/java/lang/foreign/Linker.html) and [Khronos OpenCL API specification](https://registry.khronos.org/OpenCL/specs/unified/html/OpenCL_API.html).

## Measured results on this Windows machine

The controlled Kermit comparison uses the same nine registered images, calibration, supplied camera poses, automatic disparity ranges, full-field rectification policy and one OpenCV worker:

| Project | Total retained file bytes | Points | Elapsed |
| --- | --- | --- | --- |
| Earlier CPU/debug/cache `output/kermit-controlled-auto` | 2,069,216,921 (1.93 GiB) | 17,445 | 342 s |
| Standard/GPU/no-cache `output/kermit-standard-gpu` | 52,316,630 (49.89 MiB) | 17,445 | 59 s |

That is **97.47% less retained data**, with identical LAS SHA-256 `a248131761634d983821ed42ea33f83ae45874229f0f42b8ebebc14afb1c3388`. All 271 cataloged artifacts pass size/SHA-256 verification. No stage cache or `.tiles` file remains. The speed comparison includes both GPU acceleration and reduced export I/O; it does not isolate either change. Final retained sizes are not measurements of peak scratch use.

A full eleven-image Kermit run also repeats feature matching and registration, using the same supplied estimated calibration and `--neighbors 10 --ratio 0.8 --max-dimension 800 --compute gpu --cores 1`. It exports **18,978 points**, retains **112.75 MiB** versus the earlier full project's **2.21 GiB** (95.02% reduction), and completes in **62 seconds**. The final LAS and camera CSV are byte-identical to `output/kermit-field-of-view`. Nine cameras register; `kermit004.jpg` and `kermit006.jpg` remain excluded and reported. Evidence: `target/kermit-standard-full-gpu.log` and `target/kermit-full-storage-gpu-comparison.json`.

A warmed 640×384, 128-disparity, eight-path benchmark measures **0.193 s GPU versus 4.796 s CPU** on the NVIDIA RTX 5070 Laptop GPU (~24.8×). The AMD `gfx1103` GPU also passes parity tests and measures **0.851 s versus 5.468 s** (~6.4×). Timings include census, transfers, matching and CPU post-filtering, exclude first context/kernel compilation, and vary with load. They are not whole-pipeline speed guarantees.

Windows execution is verified on both available GPU vendors. Linux loading uses `libOpenCL.so.1` and the same OpenCL/JDK APIs; Linux runtime execution has not been performed in this workspace. CPU-only build/test environments skip hardware comparisons when no OpenCL GPU is available and still exercise the storage profiles.
