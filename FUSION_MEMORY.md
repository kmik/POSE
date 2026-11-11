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
   runs at a time; helps pair SGM too.
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
