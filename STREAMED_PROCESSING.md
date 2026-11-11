# Streamed processing for hundreds of large-format images (design, 11 November 2026)

Goal: process hundreds of large-format aerial frames (for example 31,520 x 13,440 pixels; 106 Mpx
per image at `--quality high`) on a machine whose RAM cannot hold the normal working set (an 8 GB
laptop), with memory and temporary disk that do not grow with the number of images. A machine with
enough RAM (for example a 270 GB server) keeps the normal path and its output unchanged.

Status: design only. Step 1 measurement results are at the end. Implementation starts in the next
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
