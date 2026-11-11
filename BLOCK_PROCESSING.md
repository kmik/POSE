# Block processing (`--block-processing`), 10 November 2026

Opt-in processing of image sets that do not fit this machine's memory as one block. The default
is `off`; without the option nothing changes (the block options are not part of run settings,
reports or cache keys, and the normal pipeline code is not touched).

```
photogrammetry -i IMAGES -o PROJECT [options] --block-processing auto
photogrammetry -i IMAGES -o PROJECT [options] --block-processing on --block-images 300
photogrammetry status -o PROJECT
```

Give it on the whole-run command (`-i ... -o ...`), not on step commands. Repeat the same command to
resume after an interruption: the shared calibration and finished blocks are reused.

## Options

| option | default | meaning |
|---|---|---|
| `--block-processing off\|auto\|on` | off | auto: split only when the memory forecast says the whole set does not fit; on: always split |
| `--block-images N` | 0 = from the forecast | core images per block |
| `--block-overlap F` | 1 | overlap margin around each block's core, in image ground footprints |

## What it does

1. **Forecast.** Alignment memory (sparse graph and bundle workspace, `AlignmentMemory`) and image
   fusion memory (the same admission rule as `ImageFusion`: compact maps of all cameras plus the
   largest camera's working set) are estimated as a function of the block size from the image
   count, `--features`, `--neighbors`, `--quality` (processed image size) and the fusion options.
   The heap a process gets is 65% of the RAM available now (as the launcher sizes it); a block
   must fit 70% of that. The largest block that fits is the memory limit; the core size is 2/3
   of it, leaving room for overlap images.
2. **Partition.** GPS positions (EXIF; required in every image) in a local metric frame are split
   recursively at the median along the longer axis (k-d) into cores of at most N images. Every
   image belongs to exactly one core. A block is its core plus the images whose position lies
   within `--block-overlap` footprints of the core; the margin is halved per block until the
   block fits the memory limit (at most twice its core). Footprint = DJI relative altitude x 36 /
   35 mm-equivalent focal length (median), or 4 x the median camera spacing without XMP.
3. **Calibration once.** The block nearest the centre runs `inputs calibrate match align
   georeference`; its final applied calibration (`camera.txt`) is passed to every block as a
   fixed `--calibration`, so all blocks share one lens model (the GPS-weighted adjustment keeps a
   fixed supplied lens). A user `--calibration` is used directly.
4. **Blocks.** Each block is an ordinary POSE project, `PROJECT/blocks/block_NNN/project`, run as
   a child process with the user's arguments and `-i` pointing at a folder of hard links to the
   block's images (symbolic links or copies when hard links are not possible). Blocks run one
   after another, each with the whole machine.
5. **Agreement.** Cameras shared by blocks have one georeferenced position per block. One 3D
   similarity per block (translation, rotation, scale about the block's camera centroid) is
   solved by linearized least squares over all shared cameras: positions (5 cm per coordinate) and
   orientations (0.02 deg; camera centres of a nadir block lie nearly in a plane and cannot fix its
   tilt on their own), with weak
   priors (2 m, 0.5 deg, 0.5%) that keep each block at its own GPS placement. Seam residuals
   before and after are reported.
6. **Merge.** Each block's `dense.las` is transformed with its similarity; a point is kept only by
   the block whose own (core) camera is nearest in plan view, so seams have no duplicates and
   overlap images still contribute matching support at the edges. Output in `PROJECT/merged`:
   `dense.las` (same point record layout and CRS records as the blocks), `cameras.csv` (every
   image from its home block, transformed), `block_report.md` / `.json` (blocks, points kept,
   transforms, seam residuals). The plan is `PROJECT/blocks/plan.json`.

Not merged: PLY, point-confidence, DSM and PDF report (each block keeps its own in its project).
Surface normals in extra bytes are copied as is (the block rotations are a few hundredths of a
degree at most).

## Results

Measured on this laptop (32 GB RAM, RTX 5070 8 GB), user settings with `--quality high`:

**forest_10, 89 images.** As one block the run fails at fusion: image fusion needs about 5.4 GB
of heap (compact maps 3.5 GB + largest camera 1.4 GB) and the heap is about 7 GB. The forecast
(6.6 GB) correctly says it does not fit, so `auto` splits. With `--block-processing on
--block-images 30`: 4 blocks of 47-49 images (cores of 22-23), calibration pass plus 4 blocks
plus merge in about 38 minutes; merged cloud about 20 M points.

Seams between blocks (shared cameras; RMS before -> after the agreement solve):

| seam | shared cameras | position (m) | orientation (deg) |
|---|---:|---|---|
| 0-1 | 36 | 0.62 -> 0.16 | 1.15 -> 0.10 |
| 0-2 | 24 | 0.38 -> 0.25 | 0.19 -> 0.16 |
| 0-3 | 18 | 0.41 -> 0.20 | 1.17 -> 0.13 |
| 1-2 | 18 | 0.48 -> 0.20 | 1.04 -> 0.13 |
| 1-3 | 24 | 0.40 -> 0.20 | 0.31 -> 0.14 |
| 2-3 | 37 | 0.46 -> 0.07 | 1.04 -> 0.04 |

Each block, georeferenced only by GPS, is tilted differently by up to about 1 degree; the
orientation constraints remove most of it. Dense clouds in the shared areas (nearest distance
from one block's cloud to its neighbour's, clouds thinned to 1/32 for the check): median
0.29-0.85 m raw, 0.20-0.35 m after the solve. Part of that is the sample spacing of the check,
but seams of a couple of decimetres remain: the blocks are independent reconstructions, each
with its own GPS fit (about 1.4 m RMS on this drone) and its own small internal deformation.

**city2, 17 images** (functional test, `--block-images 9 --block-overlap 0.5`): 2 blocks; the
area is about one footprint across, so both blocks contain all images, align identically, and the
seam is exactly 0. Resume: repeating the command reused the calibration and both blocks and only
merged (7 s).

## Limitations and possible improvements

- Seams of about 0.1-0.3 m with GPS-only georeferencing. Ground control points that appear in
  several blocks (the `gcp` step per block) would tie the blocks absolutely and remove most of it.
- A cloud-based refinement (multi-block ICP on the dense clouds of neighbouring blocks) was tried
  and removed: with both blocks free, point correspondences let blocks shrink or rotate together
  (16 degrees in one test). A correct version needs an incremental formulation with a fixed gauge
  and robust weights.
- Overlap is limited by memory: a block's margin is shrunk until the block fits, so on a machine
  with little RAM the overlap (and the number of shared cameras) is small.
- Each block runs the whole pipeline, including its own report and outputs (disk: about one
  ordinary project per block). Not merged: PLY, point confidence, DSM and the PDF report.
- Stops before any work when even a two-image block cannot fuse (e.g. --quality ultra on a
  machine with a ~7 GB heap: one camera's fusion working set alone is about 4 GB), and warns when
  memory leaves a block fewer than 3 shared cameras.
- Not supported with step commands (`inputs ...`, `dense ...`): give `--block-processing` on the
  whole-run command.

## Tests

- `BlockProcessingRegression` (in `SelfTest`): default off and package-private options, argument
  handling, k-d partition with overlap and with the memory limit, the agreement solve recovering
  known block offsets, rotations and scales from shared camera positions and orientations, nearest
  camera ownership, LAS merge (coordinates, ownership, header, extra bytes).
- Default workflow unchanged: the block code is only reached with `--block-processing auto|on`;
  see the city2 identity check in GPU_PORT_20261110.md's procedure (LAS points, PLY, confidence).

Note on block sizes: the forecast uses the RAM that is free when the command starts. A second
forest_10 run (blocks recomputed) started with less free RAM, so each block's overlap was shrunk
to 29-30 images (cores unchanged) and blocks shared only 2-10 cameras; seams after the solve were
0.08-0.35 m (orientation 0.07-0.21 deg). The forecast is conservative (image fusion predicted
6.6 GB where 5.4 GB was needed); closing other programs before a block run gives larger overlap.
