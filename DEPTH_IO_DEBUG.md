# Dense depth-recording stall

The Sheffield Park run `output/example-test-park/runs/20261030T081222044Z_66eabb8b` appeared to stop after `Disparity search pair_0001_0000`, with an idle GPU. A read-only live thread dump showed the main thread active in `FileChannel.open` through `PagedVolume.page`, `DepthRaster.put`, `DepthFusion.add` and `DenseReconstruction.pair`. Stereo matching had already finished; CPU triangulation/depth recording was repeatedly reopening paged depth files. The process later completed that pair with 608,258 candidates and proceeded to the next pair, confirming progress rather than a deadlock.

The shared depth cache had a 64 MiB budget, but each individual depth/offset channel also had a 1 MiB cap. Oblique rectification writes a sheared traversal of the original camera raster; that traversal repeatedly evicted/reloaded pages despite available shared capacity. Increasing `--memory-mb` did not remove the fixed per-map cap.

## Repair

- Shared maps now compete against the global byte budget without an extra per-map cap. Standalone non-pooled SGM volumes keep their existing local limit.
- Persistent depth files share an LRU cache of at most 32 open handles. Evicted handles close immediately; closing a volume removes its handle and cached pages. This avoids a Windows file open/close for every page while keeping handle use bounded across large collections.
- The byte-for-byte float depth/packed-offset representation and all geometry/filter thresholds remain unchanged.
- Logs identify the stereo backend, report matching/filtering completion, announce triangulation, show row progress during long triangulation, and show candidate/retained/rejected progress during fusion. Progress is throttled to approximately five-second intervals.

The existing process keeps its already loaded classes. Restart the same command after rebuilding to use this repair; the launchers in this workspace have already been rebuilt. Earlier run artifacts remain separate. No running user process was stopped by the diagnostic work.

## Verification

All **456** numerical/integration checks pass (`target/depth-io-full-validation.log`, fixtures `target/self test 5841269279713372004`), including 17 new depth-I/O checks. They verify exact sheared-raster depth/offset readback, use of available global capacity, zero premature paging when the working set fits, forced page/handle eviction, bounded RAM and handles, nonzero defaults, clean-page write suppression, resource release and Windows file deletion without deleting unrelated files.

The same 16,000-ray sheared workload on two 1000×750 maps takes **6.530 s before** and **0.020 s after**, with checksum `8921052513856` in both cases. Including final flush/cleanup, the measurements are 6.546 s and 0.052 s. Repaired resident pages use 7.859 MiB of the configured 16 MiB pool; the old cap limited residency to 4 MiB and caused repeated I/O. These timings isolate a cache-thrashing workload and are not whole-program speed guarantees.

An actual first-pair replay copies only `DJI_0056.JPG`/`DJI_0057.JPG` and their applied calibration/final poses, preserving medium quality (1000×750), mild filtering, step 1 and 80 disparities:

| Measurement | Disk-backed repaired store | RAM-backed reference |
| --- | --- | --- |
| Matching/filtering | 1.39 s | 1.41 s |
| Triangulation/depth recording | 0.52 s | 0.31 s |
| Raw candidates | 608,258 | 608,258 |
| Retained two-image points | 607,040 | 607,040 |
| Total replay | 11.80 s | 11.32 s |

The repaired disparity PFM matches the original running job's PFM exactly, SHA-256 `28b9db3bae329ea30b32402924a25f445ca3b2c67df2b6817bb00f2af4f9c1ae`. The two replay LAS files are identical, SHA-256 `a0d5fadc3b4c1d09c51f8e654127b1e92a914b9d14eda20f6666904d5801b68c`. This is a two-image I/O/control comparison; final filtering in the full 32-image project uses additional views. Logs: `target/park-depth-fixed-disk.log` and `target/park-depth-fixed-memory.log`.

## Candidate counts

A 1000×750 processed image has 750,000 pixels. `--step 1` samples every valid stereo pixel, so 608,258 raw candidates corresponds to approximately 81% coverage for that pair. Candidates precede cross-pair fusion and optional voxel deduplication; different pairs can sample the same physical surface. `--step 2` samples about one quarter of as many candidates while still computing full-resolution disparities/depth evidence. It changes export density, not the matching resolution. The cache repair does not require changing step, quality or filtering.
