# Image fusion memory: bounded per camera, spill to disk when RAM is short (10 November 2026)

Problem: `--quality ultra` with `--dense-method mvsgm` on city2 ran out of heap in image fusion
(`OutOfMemoryError` in `ImageFusion.crossCheck`, 6.4 GB heap). Two causes:

1. Every camera's filtered depth map (and weights, marks, surface-patch cache) stayed on the heap
   for the whole fusion: 4-17 bytes per pixel per camera, growing with the number of images
   (3,000 images at high: about 54 GB; at ultra about 216 GB).
2. The heap check counted the maps and one camera's candidates but not pass 2's extras: the
   emitted-point objects of a whole camera (several hundred bytes each, a few million per camera at
   ultra, up to three cameras alive with the writer queue), the normal cache and the filter
   temporaries. City2 ultra passed the check and then ran out.

## What changed (ImageFusion.java; results unchanged in every mode)

- **Memory plan** at the start of fusion, printed and stored in `06_fusion/image_fusion.json`
  (`memory`): heap budget (75% of the heap, as before), all cameras' map bytes, one camera's work
  (its pairs' candidates, splats, merge, filter temporaries, normal cache, emissions of three row
  bands), cameras in parallel in pass 1, candidate-cache size, normal cache on/off.
  - Plenty of heap: every map stays resident, no temporary files, and the spare heap goes to
    parallel pass-1 cameras and the candidate cache (as before; now with the extras counted).
  - Not enough: maps are **spilled** to `06_fusion/map-spill` and only the camera being fused plus
    the cameras it cross-checks (those that see at least 1% of its surface) are resident; pass 1
    runs one camera at a time; the candidate cache gets the rest. Spill files go through the
    operating system's file cache, so free RAM still speeds up re-reads. They are deleted at the
    end of fusion.
  - Too little even for that: fails at once with the heap it needs (e.g. "needs at least about
    4994 MiB heap even with depth maps on disk (largest camera's work 2518 MiB ...)"), instead of
    running out of memory later.
- **Resident set**: per-camera arrays are unchanged in form (`compact`, `compactWeight`,
  `localBest`, `used`, `emitted`, `surfaceContext`); a camera is either resident or in its spill
  files (depth/weights written once; marks rewritten at every eviction). `hasMap` replaces the
  `compact[c] != null` existence test. Surface-patch grids are rebuilt on reload (their models are
  deterministic; the cache size never changed results).
- **Pass 2 in row bands of 262,144 pixels**: the cross-check, ownership marks and output hand-off
  run band by band; at most two bands wait for the writer. Within a camera the cross-check reads no
  mark it writes (it reads its own `used` marks and earlier cameras' `emitted` marks; it writes
  other cameras' `used` marks and its own `emitted` marks), so bands give exactly the whole-camera
  result. The per-camera point cache for normals is computed once per camera as before.
- `-Dsgm.fusion.mapBudgetMb=N` forces the spilled mode with N MiB of resident maps (tests).

What still grows with the number of images in fusion: about 100 KB of overlap samples per camera,
small per-camera counters and lists. The resident set in spilled mode is one camera plus its
overlapping cameras: city2 is a worst case (every camera overlaps 16 others), a large block
typically has 10-40 overlapping cameras per image.

## Results (city2, RTX 5070 laptop, about 9 GB of RAM free)

| run | heap | plan | fusion | result |
|---|---:|---|---:|---|
| mvsgm, ultra, before | 6.4 GB | (old check passed) | failed after 35-40 s | OutOfMemoryError |
| mvsgm, ultra | 7.2 GB | all resident, 1 camera in pass 1 | 70.9 s | 22,554,925 points |
| mvsgm, ultra | 4.0 GB | spilled (755 MiB resident; 8 loads, 549 MiB written/read) | 75.8 s | **byte-identical LAS** |
| pair SGM, ultra | 4.0 GB | refused at once: needs about 5.0 GB | 0.7 s | clear message |
| pair SGM, ultra | 6.4 GB | spilled (746 MiB resident; 9 loads, 617 MiB) | 73.2 s | 11,002,137 points (failed before this change) |
| pair SGM, high (bench) | 8 GB | all resident, 2 cameras in pass 1 | 24.6 s | identical to the reference |
| pair SGM, high, forced spill (100 MiB) | 8 GB | spilled, 15 loads | 25.1 s | identical to the reference |

Live heap of the 7.2 GB ultra run after full GCs: 3.2-3.5 GB (GC pauses 1.1 s in fusion), so
the full heap seen in the GC log is garbage between collections, not pressure.

Where fusion time goes at ultra (mvsgm, 71 s): cross-check 35.4 s (CPU; the GPU port of
`ImageFusion.crossCheck` is the open "piece 5" of GPU_PORT_20261110.md), 2D speckle filter
10.1 s (single-threaded per camera; parallel only when several pass-1 cameras fit), output
staging 12.1 s (writer thread, overlapped), candidate reads 10.2 s (12.2 GB: each camera's
candidates are read in pass 1 and pass 2; the LRU candidate cache got 0 hits because the two
passes scan the cameras in the same order).

The spilled mode cost about 7% here (75.8 vs 70.9 s). More heap helps mainly through parallel
pass-1 cameras (merge and speckle filter) and the candidate cache.

## Verification

- Default output (identity check, everything resident) and forced spill: dense.las (points),
  dense.ply, point-confidence.bin SAME as the reference.
- Ultra mvsgm with 7.2 GB (resident) and 4 GB (spilled) heaps: `cmp` of the 1.3 GB LAS files,
  identical.
- `ImageFusionAgreementRegression` (forest_tiny, standalone) extended with forced-spill reruns of
  graph-average (used marks) and owner-consume (used + emitted marks): byte-identical PLY,
  evictions recorded, spill files removed; 40 checks pass.
- Full SelfTest from C:\t\s2: 118,545 checks, all passed (fixtures deleted).

## Other stages (checked, not changed)

- Dense stage depth evidence (`DepthFusion` per pair): `PagedVolume` keeps a bounded set of pages
  resident with explicit file I/O; bounded.
- Candidate stream and cloud output: files on disk (`candidates-*.bin`, `cloud-*.bin` spool).
  mvsgm at ultra writes a 6 GB candidate file (111 M candidates).
- mvsgm phase 2 reads neighbour maps through a small LRU (neighbours + 2); bounded.
- Alignment: see GPU_PORT_20261110.md (TrackIndex); not re-checked here.
- Found, not changed: the dense-stage evidence tiles (`06_fusion/storage/*.tiles`) are deleted
  only when fusion completes, so a failed or refused fusion leaves them (pair SGM at ultra:
  7.9 GB per run). mvsgm writes no such tiles.

## Possible next steps

- Candidate cache policy for the two sequential passes (keep the first cameras instead of LRU) so
  pass 2 hits it; for mvsgm, hand the depth maps to fusion directly instead of the candidate file.
- Parallel speckle filtering inside one camera (or overlap it with the next camera's splats) for
  the one-camera-at-a-time plan.
- Read neighbours' maps through memory-mapped files in the cross-check, so even the resident set
  does not need heap (larger refactor of the hot loop).

## Fusion speed-up plan (agreed 10 November 2026; not started)

Measured on city2, mvsgm, `--quality ultra`, fusion 71 s: cross-check 35.4 s, output staging
12.1 s (writer thread, partly overlapped), pass-1 speckle filter 10.1 s, candidate reads 10.2 s
(12.2 GB, 0 cache hits), merge 1.8 s. Gains below are estimates until measured.

1. **Profile the cross-check (JFR) and optimise it on the CPU, results identical.** Suspects:
   per-pixel allocations (backProject/Math3.mv arrays, `BitSet.valueOf(views.clone())`, Result
   objects), SurfacePatchGrid.predict and the camera vote per neighbour. Expected 1.5-2x on 35 s.
2. **mvsgm: hand the depth maps to fusion directly** instead of the candidate file (6 GB at ultra,
   about 8 s to write in the stereo stage, read twice in fusion, about 10 s), and skip fusion's
   pass-1 speckle/spike filters for mvsgm (its maps are already speckle-filtered). Changes mvsgm
   output slightly (measure); pair SGM untouched. Expected 20-25 s off an ultra mvsgm run.
3. **Parallel speckle filter, results identical**: connected components in row strips, joined at
   strip borders (component sizes do not depend on the split). Most of the 10 s when one camera
   runs at a time; helps pair SGM too. *Done: see "Parallel speckle filter" below.*
4. **Candidate-cache policy for the two sequential passes** (keep the first cameras instead of
   LRU, which always evicts the camera needed next). Identical results; mostly fewer disk reads on
   large blocks.
5. **Cross-check on the GPU** ("piece 5" of GPU_PORT_20261110.md), only if it is still dominant
   after 1. FP64 and hypot decisions need the float-with-margin pre-test used for GpuDepthOutlier.

Order: profile first (about 10 minutes), then 1-4, then decide on 5.

## Follow-up (11 November 2026): adaptive row bands, evidence tiles on failure

- **Row-band size is now part of the plan.** The first plan reserved emissions for three full
  262,144-pixel bands (about 500 MB, assuming every pixel emits) and therefore refused a 1.5 GB
  heap on city2 at high quality that the earlier code had handled. Bands now get up to half of what
  is left (full bands when there is room) and shrink to 16,384 pixels when the heap is tight; the
  refusal threshold uses the smallest band. Bands never change results.
  city2 high, 1.5 GB heap (`--memory-mb 900`): spilled maps (95 MiB resident, 13 loads), bands of
  39,583 pixels, fusion 37.5 s (24-25 s with a 6 GB heap), output identical to the reference.
  1.0 GB heap: refused at once ("needs at least about 1194 MiB"); the earlier code refused too.
- **Dense evidence tiles are deleted when fusion fails or is refused** (they were kept only on
  success; nothing reads them afterwards). City2 high refusal at 1.0 GB: earlier code left 1.7 GB in
  `06_fusion/storage`, now nothing. `--diagnostics --storage debug` still keeps them. A cleanup
  problem prints a warning and never hides the original failure.

## Fusion speed, first round (11 November 2026)

Profile (JFR, mvsgm on city2 at high quality, fusion 18.7 s; samples within fusion):
cross-check's own code 29% (lines reading the neighbour camera's 3x3 depth window for the camera
vote dominate: memory reads into other cameras' maps), surface-patch model building 11% (all from
`SurfacePatchGrid.predict`: the per-camera cache held about 5,400 of 8,900 cells, so models were
rebuilt), per-point normal sampling about 18% (`surfaceSample`: two plane fits, a residual sort,
`StrictMath.hypot` inside `SurfaceGeometry.plane`), spike filter's 5x5 median sort about 7%,
per-camera merge 6%.

Done (results identical):
- **Medians by selection** instead of a full sort in the spike filter and in the normal
  sampling's residual median (`ImageFusion.select`, `maxBelow`; the same order statistics; checked
  against `Arrays.sort` on 900,000 random cases with duplicates and zeros).
- **Whole-grid surface-patch caches when the all-resident plan has room** (one camera's work kept
  in reserve); otherwise the previous `SurfacePatchGrid.cacheCells` size. Models are
  deterministic, so the cache size never changes results.

Measured back to back (mvsgm, city2 high, 3.8 GB heap because little RAM was free): fusion
21.4 -> 19.8 s (-7%), LAS byte-identical; pair SGM identity check SAME.

Still open from the plan above: parallel speckle filter (matters when only one camera fits in
pass 1, e.g. ultra: 10 s), mvsgm map hand-off to fusion (skips the candidate file), candidate cache
policy, and the cross-check itself (memory-bound neighbour reads and the normal fits; the GPU port
remains the large step).

Update (11 November 2026): the mvsgm map hand-off (item 2 of the speed plan, without skipping
the pass-1 filters, so the output stays byte-identical) and fusion in row strips for cameras whose
work does not fit the heap are done; see STREAMED_PROCESSING.md.

## Parallel speckle filter (11 November 2026)

Item 3 of the speed-up plan. The pass-1 speckle filter (4-connected components of a camera's
merged depth map, components below `--speckle-size` removed) ran on one thread per camera. The
untiled version held two `int[w*h]` arrays (8 bytes per pixel); the strip version labelled every
strip twice with 21 bytes per strip pixel.

### What changed

Both paths (heap arrays and the memory-mapped maps of fusion in strips) now call one filter,
`SpeckleFilter`:

1. The map is cut into row strips of its own (independent of the fusion strips). Strips are
   labelled in parallel on the fusion pool, each in a per-thread buffer, by breadth-first search.
   The buffer has a border of zeros, so the search tests no image bounds. A component that touches
   neither the first nor the last row of its strip is complete: it is removed at once when too
   small. The others get a node with their size, and the strip keeps the node number of each pixel
   of its first and last row.
2. Border components are joined across each strip border with union-find where the two border
   pixels connect (two rows of depths per border; sequential, a few milliseconds), sizes summed at
   the root.
3. Strips that hold part of a too-small joined component find its pixels by flooding from their
   border pixels of that component, in parallel, and remove them. No strip is labelled twice: the
   second step costs only the removed pixels.

The connection rule is the former one, expression for expression: float `|z_q - z_p|` against
`max(fusionRelative, 2 / min(fx, fy)) * min(z_p, z_q)` in double. It is symmetric, so components
are the connected components of a graph and do not depend on strips, threads or search order.

Threads: `--cores`, divided by the number of cameras running pass 1 at once (they share the pool);
on the strip path as many as the plan's speckle budget allows.

### Memory

Per thread 9 bytes per strip pixel (depths 4, queue 4, marks 1); per strip two border rows of
node numbers (8 bytes per image column) and the union-find nodes (at most two per column, 12
bytes each, plus 8 while they are collected: 40 bytes per column as the worst case, every border
pixel a component of its own). The strip height minimises the sum,
`rows = sqrt(40 h / (9 threads))` within 16..1024, so the bound grows with `w * sqrt(h * threads)`
instead of `w * h`:

| Image | Former untiled (8 B/px) | Former strips (21 B per strip pixel) | Now (bound, 12 threads) |
| --- | --- | --- | --- |
| city2 high, 2,000 x 1,125 | 17 MiB | - | 7 MiB |
| large format at high, 15,760 x 6,720 (106 Mpx) | 808 MiB | 268 MiB (strips of 849 rows) | 168 MiB (1.6 B/px) |

The memory plan counts this bound instead of the former "9 bytes per pixel of filter temporaries"
(now 1 byte per pixel for the spike marks plus the speckle bound), so "largest camera's work"
drops a little and `passOneCameras`, the spill decision and the bands are computed from the real
working set. On the strip path the filter runs when no fusion strip is in work and may use the
strips' share of the heap; `SpeckleFilter.threadsFor` lowers the thread count until the bound
fits (one thread at least: about `38 w sqrt(h)` bytes, 47 MiB for the large format). The
plan line now ends with `speckle filter N MiB on up to T thread(s)`.

### Results are identical

For `--speckle-size` up to 65,536 the removed pixels are exactly those of the former filters on
both paths. Above 65,536 the former untiled filter removed only the first 65,536 members of a
too-small component (its member buffer) while the former strip filter removed all; now both paths
remove whole components. This is the only intended difference, and it needs an explicit
`--speckle-size` above 65,536. (NaN depths, which fusion does not produce, connect to every valid
neighbour as in the former untiled filter.)

### Timings

Same machine, old and new classes alternating. "Filter time" is the telemetry entry
`Image fusion: pass 1 2D speckle filter`, which also covers the spike filter and the depth sanity
check; the new entry `Image fusion: pass 1 speckle components (within the filter time)` is the
speckle filter alone (for the old classes measured with a temporary timer). Times are summed over
cameras, so with several cameras in pass 1 at once they include waiting for the shared pool.

| Case | Speckle components | Filter time | Fusion |
| --- | --- | --- | --- |
| Large format, 4 x 106 Mpx, high, 6.5 GB heap, strips of 849 rows | about 11 s -> **1.2 s** | 30.3 -> 20.3 s | 222 -> 248 s (see below) |
| city2 mvsgm high, forced strips of 64 rows, 6 GB | 1.05 -> 0.16-0.18 s | 4.5 -> 3.7-3.8 s | 24.6 -> 22.1-22.4 s |
| city2 mvsgm high, 1.5 GB heap (one camera at a time) | 0.81 -> 0.18 s | 3.9 -> 3.3 s | 23.3 -> 20.2 s |
| city2 pair SGM high, 1.5 GB heap, spilled maps | 0.67 -> 0.19 s | 3.2 -> 2.8 s | 29.3 -> 28.5 s |
| city2 pair SGM high, 1.5 GB heap, forced strips of 64 rows | 1.02 -> 0.17 s | 4.2 -> 3.3 s | 174 -> 164 s |
| city2 pair SGM high, 6 GB (3 cameras at once), 8-10 runs each | 0.76-0.85 -> 0.50-1.17 s | 3.7-4.9 -> 3.8-4.8 s | 18.1-20.1 -> 18.0-22.8 s |
| city2 mvsgm high, 6 GB (6 cameras at once), 2-4 runs each | 1.0-1.5 -> 1.4-1.9 s | 7.6-9.3 -> 7.9-9.6 s | 13.9-19.1 -> 15.0-17.8 s |

- One camera at a time (large frames, small heaps, strips): the speckle filter is 4-8 times
  faster on 12 threads.
- Several cameras at once (city2 with a 6 GB heap): the cores were already busy with the other
  cameras, so nothing is gained; the differences are within the run-to-run noise.
- The filter time is mostly **not** the speckle filter: on city2 the components were 0.7-1.0 s of
  3-5 s, on the large format about 11 of 30 s (by difference; the old strip filter was not timed
  alone there). The rest is the spike filter (5x5 medians, already row-parallel) and, in strips,
  writing the map. The remaining 19-20 s on the large format are the next candidate in pass 1.
- Large format: the whole fusion took 248 s in this run against 222 s earlier the same day. The
  steps that grew (cross-check 100 -> 115 s, output staging 54 -> 59 s, re-merge 19 -> 25 s) do not
  touch the filter; the two runs were hours apart on a laptop, not back to back. Live heap after
  collections stayed at 3.4 GB as before.

### Checks

- Pair SGM identity check on city2 (default output): dense.las SAME (points), dense.ply SAME,
  point-confidence.bin SAME in every run with the new classes (12 runs), also with a 1.5 GB heap
  (spilled maps, 101 MiB resident), with forced strips at 1.5 GB, with a forced spill of 100 MiB
  and with `--streamed-processing on`. No out-of-memory error.
- mvsgm on city2 at high: dense.las `cmp`-identical to a reference made with the former classes,
  untiled (4 runs), forced into strips of 64 rows (2 runs), with a 1.5 GB heap and with strips at
  a 1.2 GB heap.
- Large format (4 frames of 106 Mpx, high, 6.5 GB heap, end to end into a new project): the
  6.98 GB dense.las is `cmp`-identical to the run made with the former classes; 120,346,379 points,
  speckle filter removed 216,358 pixels in both. Outputs deleted afterwards.
- `SpeckleFilterRegression` (new, in SelfTest; 3,147 checks): 40 random maps (noise, patches with
  holes, vertical lines, combs and spirals crossing every strip border, blobs of sizes around the
  speckle size on and off strip borders, depth ramps at the connection limit, NaN depths, images of
  1-4 columns or 1-3 rows) with speckle sizes 1 to 3,000. The parallel filter with strip heights
  1, 2, 3, 5, 7, 16, 33, 64 and whole image on 1, 2, 3 and 8 threads, without a pool, on a memory
  segment and with planned rows and threads gives the map (bit for bit) and removed count of the
  former untiled filter; the former strip filter, kept in the test as a second reference, agrees
  too. Speckle sizes above 65,536 behave as documented above. Memory-plan helpers checked.
- Full SelfTest (from a short working folder): 121,749 checks, all passed. Standalone on
  forest_tiny: `LargeFormatRegression` 53 checks and `ImageFusionAgreementRegression` 48 checks
  (with their forced-strip and forced-spill reruns), passed. Fixtures deleted.

## Spike filter on the GPU (12 November 2026)

The pass-1 spike filter (5 x 5 median and MAD decision per pixel of a camera's merged depth map)
was the largest part of the telemetry label `Image fusion: pass 1 2D speckle filter` after the
speckle filter became parallel. It now runs in an OpenCL kernel (`GpuSpikeFilter`); the Java code
(`ImageFusion.spikeRow`, the former loop body unchanged) stays the reference and the fallback.

### Identity

The kernel repeats `spikeRow` operation for operation: the same float operations (the median of
an even count, the deviations) and the same double operations (footprint, spread, limits), FP64
with `#pragma OPENCL FP_CONTRACT OFF`, and only `+ - * /` and comparisons, which are correctly
rounded on both sides. The medians are the same order statistics (an insertion sort of at most 24
values instead of the selection). NaN and negative depths take the same branches. The marks are
therefore equal bit for bit. The first map of a process is compared with the CPU code on every
eighth row (printed as `GPU spike filter: ... identical to the CPU code`); a difference or a GPU
error prints one warning and switches the GPU path off for the rest of the process.

### Device memory

4 bytes per uploaded pixel (the rows of a band plus two halo rows on each side) and 1 byte per
decided pixel. Before anything is allocated the size is checked against the GPU budget
(`--gpu-memory-mb`, default 70% of the device memory) and the largest single allocation of the
device. A map that does not fit is sent in bands of rows (also never more than 16.8 million
pixels per band, which bounds the host copy at 16 MiB per upload slice); when not even 256 rows
fit, the CPU code runs and one line says so. The bytes of a band are reserved from the shared GPU
pool when one is installed. Cameras that reach the filter at the same time take turns, so one
band is on the device at a time. Fusion in strips calls the filter per strip with its halo rows,
as before. Kernels run in launches of at most 4 million pixels (milliseconds each). The device
context is opened at the first map and closed at the end of pass 1.

Switching it off: `--compute cpu` or `-Dsgm.cpuSpikeFilter=true`. Maps below 250,000 pixels always
use the CPU.

### Measured (city2, `--quality high`, 17 cameras of 2000 x 1125, two cameras at once in pass 1)

New telemetry label `Image fusion: pass 1 spike filter (within the filter time)`: the sum over
cameras of the time in the filter (including waiting for the device).

| | spike filter, 17 maps | one map alone |
|---|---:|---:|
| CPU (`-Dsgm.cpuSpikeFilter=true`), 12 threads | 2.2 s | 65-75 ms |
| GPU | 0.5-0.6 s | 13-18 ms |

Of the GPU time about 0.1 s is opening the device context and the first-map check; the kernels
take 55 ms for all 17 maps. The first run after a driver or kernel-source change compiles the
kernel once (about 2.5 s; the driver caches it). On this small set the fusion step's wall time
changes by less than its run-to-run spread (19.6 s before, 18.5-20.5 s after); the gain is meant
for large frames, where the spike filter is most of the pass-1 filter time (not re-measured there).

`dense.las` and `point-confidence.bin` are byte-identical to the run with the classes before the
change; `spike_rejected` 123,715 in both.

### Verification

- `GpuSpikeFilterRegression` (in SelfTest): GPU marks against the CPU code on every pixel of six
  synthetic maps (smooth to rough surfaces, a depth edge, isolated spikes, thin lines, holes,
  sparse regions, NaN and negative depths, row ranges as fusion in strips uses them, maps sent in
  several bands under a 2 MiB budget); band sizes against budget and allocation limits; a 1 MiB
  budget, `--compute cpu` and `-Dsgm.cpuSpikeFilter=true` give the CPU path.


## Cross-check: CPU overhead looked for, none found (12 November 2026)

Question: does the pass-2 cross-check lose time to per-access `MemorySegment` overhead in its
inner loop, and is a GPU port worth it afterwards?

Measured on city2 (`--quality high`, 17 cameras), JFR sample proportions and the telemetry label
`Image fusion: cross-check, ownership, normals`:

- With resident maps the neighbour loop reads the arrays directly (`at()`); segment alignment
  checks are about 1% of all samples of the dense step and none of them lie under the
  cross-check. An earlier note that put them at 14% of the phase does not hold for this code.
- Forced strips (`-Dsgm.fusion.tileRows=256`, every neighbour map read through its mapped file):
  13.6 s, against 13.9 s with resident maps in the same session. Reading through the segment
  costs nothing measurable.
- Samples under the cross-check: 63% in the loop body itself, 8% `SurfacePatchGrid.build`, 7%
  patch predictions, 11% normals (`surfaceSample`, `SurfaceGeometry.plane`), 2% `hypot`. Within
  the loop body about 40% sits on the 3 x 3 neighbourhood scan of the neighbour's map and 13% on
  the neighbour's patch prediction.
- Scanning the 3 x 3 neighbourhood as floats (exact) gave no gain (15.2 s against 13.9 s, within
  the spread of this session: the same classes gave 11.4 s and 13.9 s an hour apart); removed
  again. The time on those lines is most likely waiting for memory, not arithmetic.

`crossCheck` is unchanged from the baseline. Output of all four runs byte-identical.

GPU port, estimate only (not written): the kernel could take the projection into each neighbour,
the 3 x 3 statistics, the vote and the agreement search, at most about half of the phase (city2
about 13 s to 7 s, large frames about 100 s to 50 s if the proportions hold). The patch
predictions (lazy grids, `hypot`), ownership marks and normals stay on the CPU, so the kernel
would have to hand back per pixel and neighbour results (tens of bytes each, several hundred MiB
per camera on city2), and neighbour maps larger than the device budget need bands per neighbour.
It is the largest kernel so far with every fusion option as a branch.
