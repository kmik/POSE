# Streamed processing for hundreds of large-format images (design, 11 November 2026)

Goal: process hundreds of large-format aerial frames (for example 31,520 x 13,440 pixels; 106 Mpx
per image at `--quality high`) on a machine whose RAM cannot hold the normal working set (an 8 GB
laptop), with memory and temporary disk that do not grow with the number of images. A machine with
enough RAM (for example a 270 GB server) keeps the normal path and its output unchanged.

Status (11 November 2026): mvsgm tiles, the map hand-off, fusion in strips and the option are done;
the 4-image large-format block runs end to end at high on a 6.5 GB heap (sections at the end).
Original status line: design only. Step 1 measurement results are at the end. Implementation starts in the next
session, in the order below.

## Option

`--streamed-processing off|auto|on` (default `auto`):
- `off`: never.
- `auto`: only when the memory plan shows that the normal path cannot run in the available heap
  (the same plan style as image fusion's: per-image work plus all images' maps). With enough RAM
  nothing changes and the default output stays byte-identical.
- `on`: always (tests, and small machines known to need it).
The run report states whether streaming was used and why.

Streamed output is not required to be identical to the normal path (overlap is predicted
geometrically, see below), but it must be deterministic and give the same points with any heap
size once streaming is chosen.

## Pipeline (mvsgm)

1. **Order**: images in id order (flight order for normal aerial blocks). For each image, its
   mvsgm neighbours (tie-point score, as now) and its fusion overlap set are known up front from
   geometry: camera frusta intersected with the sparse points' depth range (no pass-1 samples, which
   need every map first).
2. **Window**: image c can be fused once every image in its overlap set has a depth map. Depth maps
   are computed ahead in id order until that holds; after c is fused, maps no longer needed by any
   later image are deleted. With strips flown in order the window is about two strips of images,
   whatever the block size.
3. **No candidate file**: mvsgm hands its consistent depth (plus margin, sharpness, photometric
   and LR-agreement maps) to fusion directly as the camera's merged map (pass 1 is one sample per
   pixel for mvsgm). This removes the about 55 bytes per pixel candidate stream (5.8 GB per 106 Mpx
   image).
4. **Tiles within an image** (106 Mpx does not fit a laptop at all otherwise):
   - mvsgm: reference tiles of about 2,048 x 2,048 plus a 64-pixel overlap (BandedSgm strips use the
     same idea); each neighbour is resampled only over the region the tile can reach; the tile core
     is written into the image's map files by row ranges.
   - fusion: cross-check, ownership and emission per tile with an overlap covering every
     neighbourhood operation (5x5 spikes, normals +-12 px, agreement radius); the speckle filter's
     connected components are joined across tile borders (union-find on border pixels).
   - Map files are read through memory mapping / row ranges, never whole into the heap.
5. **Tiled output**: points are written to spatial tiles (for example 500 m squares, LAS 1.2 each)
   as images are fused; a tile is closed when no later image overlaps it. DSM and report are built
   per tile.

## Memory and disk bounds (targets)

- Heap: one image tile's work (a few hundred MB) + resident map tiles of its overlapping images.
- Disk: depth maps of the window (about 8-16 bytes per pixel per image in the window) + open
  output tiles.

## Steps and checks

1. Measurement of today's code on the 4-image block at high (below).
2. Option, memory-plan detection, geometric overlap, window scheduler, direct map hand-off
   (mvsgm, normal-size images). Check: city2 with `--streamed-processing on` vs off (clouds agree;
   C2C and flat-surface metrics), heap and disk flat over a synthetic many-image block.
3. Tiling within an image (mvsgm, fusion). Check: tiled vs untiled identical on city2-size images
   (forced small tiles); 4-image large-format block at high on a small heap.
4. Tiled output, DSM/report per tile.
5. Test with a real block of dozens or hundreds of large-format images.

## Step 1 measurement (11 November 2026)

Pair SGM (`--dense-method sgm`, the reference settings) on the 4-image block at `--quality high`
(15,760 x 6,720 = 106 Mpx per image), supplied fixed poses, 7.9 GB heap, about 52 GB free disk:

- Every pair goes through windowed stereo (images too large for resident SGM), and windowed
  stereo has no TSGM bands, so it searches the full disparity range.
- Pair 0-1: 47,463,038 candidates after about 61 minutes (stereo stage 1:01:12).
- Pair 0-2: stopped before matching: "Windowed SGM needs approximately 55752 MiB temporary disk,
  available 52238 MiB". The run failed in the dense stage; fusion was never reached. Temporary files
  were removed.
- So today neither method processes these frames at high quality on a laptop: mvsgm has no tiled
  mode (its CPU fallback would take hours), pair SGM needs about 56 GB of disk per pair and about an
  hour per pair, and fusion at 106 Mpx per image (10-20 GB of work per image) is untested.

Implications for the plan: tiling inside an image (step 3) is the first hard requirement for
large frames on small machines, and banded (TSGM-style) search inside tiles is what keeps time and
disk per pair or image bounded; the window scheduler (step 2) then bounds the block size.

## Step 3a done (11 November 2026): mvsgm depth maps in tiles

`MultiViewSgm.tiledDepth`: images above 16 Mpx (or with `-Dsgm.mvsgm.tile=N`) are solved in
2,048-pixel tiles with a 64-pixel overlap. Per tile: a reference crop, and per neighbour a canvas
covering only what the tile can reach (`canvas` from the tile corners), resampled from only the
neighbour region it needs (`sourceRegion`, read as a sub-rectangle of the decoded image); the
plane-mapping table includes the tile origin (added in double before rounding; origin 0 leaves the
table bit-identical). The whole coarse-to-fine solve, selection and filters run per tile; tile
cores are written into the image's map files by position (`DepthMapFile.create/writeRegion`), so
no image-sized array is kept on the heap.

Results:
- city2 camera 6 with forced 512-pixel tiles (12 tiles) vs untiled: coverage 95.4 / 95.3%, flat
  RMS median 2.91 / 2.91 cm, robust sigma 2.49 / 2.50 cm, >25 cm outliers 1.26 / 1.25%.
- Untiled path unchanged: camera 6 map byte-identical to the stage-3 map; default output identity
  check SAME; mvsgm regression passes.
- Large-format block at `--quality high` (15,760 x 6,720 = 106 Mpx per image; tie points matched
  at 4,000 px with the supplied poses: 19,683 tracks), depth maps only, 6.5 GB heap: 32 tiles per
  image; 85.4 / 61.9 / 62.8 / 72.3 s per image (189-283 labels); 72-93% of pixels valid; live heap
  after GC at most 1.0 GB, no full GC; 6.4 GB of map files for 4 images. No visible tile seams.
  For comparison, pair SGM needed about an hour for one pair and then 56 GB of temporary disk.

Still needed before a large-format block runs end to end:
- phase 2 (consistency + candidates) reads whole neighbour maps (424 MB per 106 Mpx image) and
  writes the 55-byte-per-pixel candidate file: read maps by row bands / memory mapping and hand
  maps to fusion directly (step 2's hand-off);
- fusion tiles inside an image (step 3b);
- depth previews (`--export-disparity-images`) are not written for tiled images yet;
- a regression test for tiled solves (synthetic scene with forced small tiles vs untiled).

## Step 3b done (11 November 2026): mvsgm maps handed to fusion directly

No candidate file for `--dense-method mvsgm` any more.

- **Phase 2** (`MultiViewSgm.run`) memory-maps the camera's depth map and its neighbours' maps
  (`MappedFiles`, FFM API; nothing image-sized on the heap) and runs the same consistency check
  row-parallel. It writes two more maps per camera: `.lr` (agreement in labels; NaN = no depth or
  rejected) and `.rgb` (packed colour, copied from the frame in row bands). Each camera is still
  one single-view fusion pair (camera, best neighbour).
- **Hand-off** (`MvsgmMaps`): the map folder, each camera's fusion pair and the number of
  consistent pixels before each row. A candidate is a pixel with depth > 0 and agreement not NaN;
  candidates are numbered in raster order as the file numbered them, so candidate ids (pair start
  + rank) are unchanged. The maps are deleted after fusion (or after a failure) unless
  `--diagnostics` or `-Dsgm.mvsgm.keepMaps=true`.
- **ImageFusion** reads a camera's candidates for any row range straight from the maps
  (`mapPair`: bulk row copies; world point, evidence floats, packed colour and the triangulation
  term computed exactly as the former writer and reader did). Pass 1 (merge, spike, speckle and
  depth-outlier filters), the cross-check, graph averaging and emission are unchanged.

Results (city2, mvsgm, the reference fusion settings, `--quality high`, 6 GB heap, same machine):

| | before | after |
|---|---:|---:|
| 05 stereo | 27.2 s | 23.5 s |
| of which consistency (+ candidate writing before) | 6.7 s | 2.5 s |
| 06 fusion | 15.3 s | 15.5 s |
| fusion pass-1 reads (accumulated over 6 parallel cameras) | 7.2 s (file) | 2.9 s (maps) |
| dense step wall | 45 s | 42 s |
| temporary disk | 1.8 GB candidate file + maps | maps + 8 bytes per pixel (`.lr`, `.rgb`) |
| dense.las | | **byte-identical** (`cmp`: no difference) |

For a 106 Mpx image the candidate stream was about 5.8 GB per image; the hand-off adds 0.85 GB
of maps per image instead. A first version read the mapped files value by value: pass-1 reads
took 25.5 s (accumulated) because of that; one bulk copy per map and row fixed it.

Default output (pair SGM, identity check): dense.las SAME (points), dense.ply SAME,
point-confidence.bin SAME.

## Step 3c done (11 November 2026): image fusion in strips

`ImageFusion` can fuse a camera in **row strips** (full image width, `stripRows` rows plus a halo)
when one camera's work does not fit the heap. Strips are the "tiles" of the design: full-width
rows keep every neighbourhood operation one-dimensional across borders, and a 15,760-pixel row is
already long.

When it engages (memory plan, printed and stored in `06_fusion/image_fusion.json`):
1. everything resident, as before;
2. otherwise spilled maps with the resident set (FUSION_MEMORY.md), as before;
3. otherwise, with `--streamed-processing auto` (default), **strips**; with `off` the previous
   refusal ("needs at least about N MiB heap"), now naming the option.
`--streamed-processing on` or `-Dsgm.fusion.tileRows=N` (tests) force strips. With enough heap
nothing changes.

How strips work:
- **Maps memory-mapped**: in strip mode every camera's filtered depth, weights, local sigma and
  used/emitted marks are files under `06_fusion/map-spill` (`camera_<id>.z|w|best|used|emitted`),
  read and written in place through `MappedFiles`; nothing image-sized is on the heap. The
  cross-check reads neighbour cameras where its reprojections fall; the operating system's file
  cache holds what RAM allows. Files are unmapped and deleted at the end of fusion (also on
  failure).
- **Pass 1 per camera**: per strip, splat (for mvsgm: only the strip's map rows), merge and spike
  filter with a 3-row halo (5x5 window + the merge's 3x3 search); core rows go to the map file.
  Then over the mapped map: the **speckle filter with components joined across strips**
  (components labelled per strip; components touching a strip border become union-find nodes,
  joined where the border pixels connect, sizes summed at the root; a second sweep relabels and
  removes components below `--speckle-size`), depth outliers (same percentile samples), overlap
  samples (same pixels; subpixel offsets from a re-merge), local sigma (4-row halo).
- **Pass 2 per camera**: per strip with a 13-row halo (normal fits reach +-12 rows at stride 4, +1
  for the merge), re-merge, pass-1 mask, normal cache, then the cross-check in row bands as before.
  Marks are written to the mapped mark files between bands, exactly as before.
- Pixel indices: `Merged` carries its first image row (`row0`); the cross-check keeps image pixel
  numbers for marks, local sigma and emission, and strip-local numbers for the merged arrays.
  Image coordinates in back-projections are formed as before (integer row + float offset).
- Map reads in the cross-check hot loop go through `at(array, segment, index)` with both held in
  locals: the resident path reads the array directly. A first version used a `Raster.get` method
  per read and cost about 30% in the cross-check (10.7-12.0 s -> 10.7-14.4 s on the identity-check run); with
  the locals it is 10.0-10.1 s against 10.4-10.8 s for the baseline classes, back to back.

Plan in strips: per strip pixel about 132 bytes for mvsgm (merge 35 + splat 16 per pair +
candidate 56 + normal cache 24 + filter 1), times the strip with its halo, at most half of what is
left after the surface-patch caches and emission bands; at least 32 rows, otherwise a clear
refusal with the heap needed. Pair SGM can use strips too, but each strip reads its camera's whole
pairs again (and the pairs' candidates must fit), so it is slow; it is a fallback, not the
large-format path.

Checks (all byte-identical LAS; strips only change memory and time):

| run | fusion | result |
|---|---:|---|
| mvsgm city2, untiled (maps hand-off) | 16.1 s | `cmp` identical to the pre-step-1 reference |
| mvsgm city2, forced strips of 64 rows | 29.6 s | identical (296 MiB mapped) |
| mvsgm city2, forced strips of 200 rows | 24.1 s | identical |
| pair SGM identity check, default | 19.3-21.2 s | dense.las / dense.ply / point-confidence.bin SAME |
| pair SGM identity check, forced strips of 100 rows | 110.6 s | SAME |

Speckle sizes above 65,536 pixels: the untiled filter removes at most 65,536 members of a too
small component (its member buffer); the strip filter removes all. Only matters for
`--speckle-size` > 65,536.

## Step 3d done (11 November 2026): previews of tiled images, regression tests

- **Previews** (`--export-disparity-images` or `--diagnostics`) for tiled mvsgm images
  (`MultiViewSgm.exportDepthStreamed`): `depth.pfm` / `consistent_depth.pfm` at full resolution,
  written row by row from the memory-mapped maps; `.png` downscaled so the longer side is at most
  4,096 pixels (each preview pixel = mean of the valid depths in its block; 0 = none; a 15,760 x
  6,720 image gives a 3,940 x 1,680 PNG); `.json` records `preview_scale`. Untiled images keep the
  previous full-size export.
- **`LargeFormatRegression`** (new, in SelfTest, 53 checks, CPU, about 20 s): the synthetic
  five-camera scene of MultiViewSgmRegression written as TIFF files, run end to end through
  DenseReconstruction with mvsgm and image fusion (graph-average, edge-aware camera checks, surface
  normals):
  - untiled: maps handed to fusion (no candidate file), 43,401 points, at least 95% on the true
    planes;
  - fusion forced into 7-row strips (smaller than the halo): byte-identical LAS, strip files
    removed;
  - mvsgm forced into 96-pixel tiles: 170,405 of 170,405 common depths within 0.5% of the untiled
    maps, 7 of 170,412 pixels valid in only one; streamed previews at full resolution; strips on
    the tiled maps byte-identical too;
  - a 4,200-pixel-wide frame: PFM at full resolution, PNG at scale 2 with block means;
  - `--streamed-processing` parsing.
- **`ImageFusionAgreementRegression`** (forest_tiny, pair SGM) extended with strip reruns
  (37-row strips) of graph-average and owner-consume: byte-identical PLY, strip files removed;
  48 checks pass.

## Step 4 done (11 November 2026): the 4-image large-format block end to end at high

Command: the reference mvsgm command (`--quality high --alignment-max-dimension 4000`, supplied
`.prj` poses and calibration, `--dense-method mvsgm --fusion-mode image --fusion-strategy
graph-average --depth-outlier-filter edge-aware ...`, `--memory-mb 16000`) on the RTX 5070 laptop,
**6.5 GB heap** (`-Xmx6656m`, what the launcher's 65%-of-free-RAM cap gives with about 9-10 GB
free; 9.1 GB free at the start), new project folder. Exit 0, **9 min 17 s** wall.

| stage | time |
|---|---:|
| 00 inputs | 13.9 s |
| 02 features (4,000 px, supplied poses) | 14.9 s |
| 03 matches (19,683 tracks) | 1.5 s |
| 04 geometry | 17.1 s |
| 05 stereo: mvsgm depth maps, 32 tiles per image (86.6 / 63.2 / 62.6 / 63.8 s) | 276.1 s |
| 05 stereo: consistency (memory-mapped) | 8.1 s |
| 06 fusion (strips of 849 rows, chosen by the memory plan) | 222.5 s |
| - pass 1 reads / merge / speckle filter (union-find over strips) | 15.6 / 3.7 / 30.3 s |
| - pass 2 re-merge / cross-check, ownership, normals / output staging | 19.4 / 100.5 / 54.2 s |
| - final LAS export | 33.1 s |
| 07 results (report) | 14.1 s |

- Depth maps: 351,870,661 depths, 328,572,065 (93.4%) consistent with a neighbour map
  (per image 92.7-94.5%).
- Fusion plan: "heap budget 4,992 MiB; depth maps 3,698 MiB for 4 cameras (memory-mapped files;
  fusion in strips of 849 rows); largest camera's work 2,631 MiB". `auto` chose strips because
  the spilled plan needed about 19.6 GB. 3,282 MiB of maps were memory-mapped.
- **120,346,379 points** (merged valid pixels 349.9 M; graph averaging combines the copies; spike
  filter 0.98 M, speckle filter 0.22 M, cross-check about 1.4 M rejected). LAS 6.98 GB.
- **Heap** (GC log): peak used before a collection 6.45 GB (the heap is used fully, as expected),
  **live after collections at most 3.4 GB**; 216 pauses, no full GC, no out-of-memory.
- **Disk**: output folder at most 22.0 GB during the run (mvsgm maps 24 bytes per pixel x 4
  images, fusion map files, LAS staging); 6.6 GB at the end (the LAS); temporary maps and strip
  files removed. Free disk never below 38 GB.
- **Sanity check** (a streaming LAS preview, 4 m cells): top view in RGB and height continuous over
  the whole 7.9 x 4.1 km block (roads, field edges, the quarry and terrain relief intact), 91.5%
  of cells filled; no strip or tile seams visible; lakes are holes (no texture), as with any
  stereo method. Side views through the middle of the block follow terrain and canopy. Seen: a
  few stacked horizontal layers near the north end of the north-south section, and sparse
  outliers far outside the surface (z from -140 to 356 m against 85-200 m for the 1st-99th
  percentile); worth a look before large production blocks.

For comparison: pair SGM at high needed about an hour for one pair and then 56 GB of temporary
disk for the next (step 1 measurement), and the previous mvsgm code stopped in fusion asking for
about 19.6 GB of heap.

Still open (design steps 2, 4, 5): window scheduler and geometric overlap for blocks of hundreds
of images; tiled LAS output, DSM and report per tile (the single LAS is 7 GB for 4 images);
cross-check speed (100 s here; GPU port); a real block of dozens or hundreds of frames.

Tests after all of the above: full SelfTest 118,602 checks, all passed (including
LargeFormatRegression); ImageFusionAgreementRegression on forest_tiny 48 checks passed; default
output identity check SAME.
