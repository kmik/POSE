# Orthomosaic and automatic check points (11 November 2026)

Two additions. Both are off unless asked for, so default outputs do not change.

## Full-resolution true orthomosaic (`--orthomosaic`)

The processing report already contained a small orthomosaic overview (at most 1,024 pixels a
side, from thumbnails). `--orthomosaic` writes the deliverable: the original photographs at full
resolution on the surface of the final cloud.

```
photogrammetry ... --orthomosaic                      # at the end of a run (finish step)
photogrammetry --export-orthomosaic RUN_DIRECTORY     # for a finished run, nothing reprocessed
               [--orthomosaic-output DIRECTORY]
```

Files in `07_results` (or the output directory):

- `orthomosaic.tif`: north-up GeoTIFF, 8-bit RGBA in 512-pixel tiles, Deflate compressed; alpha 0
  marks pixels without data. BigTIFF when the samples exceed about 3.5 GB. EPSG tag when the run
  is georeferenced.
- `orthomosaic.tfw`: world file (pixel centres).
- `orthomosaic.json`: size, pixel size and its basis, extent, surface cell, counts (painted, no
  surface, seen by no camera), photographs used and skipped, time.

It needs a projected coordinate frame, or `--report-local-dsm` when the local frame is meaningful
with Z up (for example orientations imported in map coordinates without GPS); the file then
carries no EPSG tag. The photographs must still be where the run found them (or archived in the
run) and have the calibrated size.

| Option | Default | Meaning |
| --- | --- | --- |
| `--orthomosaic` | off | Write the orthomosaic when the run finishes. |
| `--orthomosaic-resolution auto\|M` | auto | Pixel size in output units. Auto: the median ground sample distance of the photographs (height above the cloud's mean height over the focal length), rounded up to two significant digits. |
| `--orthomosaic-surface-cell auto\|M` | auto | Cell of the surface. Auto: twice the point spacing of the cloud (from the occupied area on a coarse grid), not finer than a pixel, coarser if the grid would exceed an eighth of the heap. |
| `--orthomosaic-fill auto\|M` | auto (32 cells) | Reach of the surface gap filling; 0 for none. |
| `--export-orthomosaic RUN_DIRECTORY` | | Standalone for a finished run. |
| `--orthomosaic-output DIRECTORY` | the run's `07_results` | Where the standalone export writes. |

### Method

1. **Surface.** The highest cloud point per cell of a north-up grid (three sequential passes
   over the final PLY or LAS: extent, point spacing, surface).
2. **Gap filling.** Cells without points up to the reach away from a known cell are flooded from
   the *lowest* rim of the gap first (cells taken in order of height), then smoothed among
   themselves. A gap beside a roof or a crown therefore continues the ground and the object does
   not grow sideways; roads and other surfaces without cloud points are still painted. Only
   enclosed cells are kept (cloud points within reach on both sides along at least two of the
   four axes), so the mosaic does not grow beyond the outer edge of the cloud. Wider
   gaps stay unknown.
3. **Height of a pixel.** Bilinear between the four nearest cell centres where the surface is
   continuous (heights within two cells), the nearest cell at a height step, so roof edges do not
   become ramps.
4. **Camera of a pixel.** Among the cameras that see the surface point inside their image and
   whose line of sight is not blocked by the surface, the one that sees it nearest the image
   centre. The line of sight is sampled every three quarters of a cell; close to the point a cell
   blocks only when it is one and a half cells above the line (rough ground does not hide
   itself), from four and a half cells on as soon as it is above the line. If no camera passes
   that, cameras passing with the near allowance all the way are used (rough canopy). If none
   does, the pixel is transparent. This is what makes it a *true* orthomosaic: roofs and crowns
   stay at their own position, and they never cover the ground beside them.
5. **Colour.** Bilinear in the original photograph, through the full camera model (Brown
   distortion) of the source image.

Not done: seam blending, radiometric balancing between photographs, overviews (pyramids),
colour for pixels no camera sees. Seams show where photographs differ in brightness, most
clearly on water.

### Memory and speed

The mosaic is produced in blocks of 2,048 x 2,048 pixels and written tile by tile; no raster of
the whole mosaic exists in memory or in a temporary file. The heap holds the surface grid (8
bytes per cell while filling), one block, and a cache of decoded photographs (a third of the
heap, least recently used first out). Photographs larger than a sixteenth of that cache are not
decoded whole but read by source window per block (large TIFF and BigTIFF frames). The result
does not depend on the thread count.

| Data | Mosaic | Pixel | Time | File | Heap after collections |
| --- | --- | --- | --- | --- | --- |
| city2, 17 photographs of 20 Mpx, 3.7 M points | 4,941 x 4,785 | 0.04 m | 10 s | 26 MiB | - |
| large format, 4 frames of 424 Mpx (2.3 GB TIFF each), 120 M points, 6.5 GB heap | 30,503 x 15,760 | 0.26 m | 139 s | 1,077 MiB | 0.34 GB |

city2: 51.8% of the bounding rectangle painted (45.3% has no surface: the block is a rotated
rectangle with tree gaps; 2.9% is seen by no camera, mostly ground behind trees). Large format:
90.9% painted, 7.6% no surface (open water), 1.5% seen by no camera; 386 source-window reads, no
photograph decoded whole.

Looked at by eye: buildings upright with sharp roof edges and no facades, cars, fences and road
markings continuous across photographs, transparent strips behind tall trees and at roof edges
where no camera sees the ground.

### Checks

`OrthomosaicRegression` (in SelfTest; 81 checks) uses a synthetic scene with known answers: flat
patterned ground, a 6 x 6 x 8 m box with a different roof pattern and magenta walls, nine nadir
cameras rendered by ray casting (two with lens distortion), and a cloud without ground points
beside the box.

- Surface: highest point per cell; the gap beside the box continues the ground; no ramp at the
  roof step; bilinear on a slope; line-of-sight cases.
- Mosaic of 2,500 x 2,300 pixels (four blocks): every one of 5.7 M ground and roof pixels away
  from the outline shows the pattern of its own position within 8 of 255 (the roof does not
  lean), none is transparent, no wall colour; identical tiles on 1 and 8 threads.
- Two cameras on one side only: the ground behind the box is transparent and nothing is wrong;
  with the visibility test switched off the same pixels show roof and wall (so the test can fail).
- The GeoTIFF is read back with the JDK decoder; size, tile, compression, alpha, pixel scale, tie
  point and CRS tags are parsed independently; the BigTIFF variant holds the same tiles and
  decodes to the same pixels; a file with a missing tile is refused.
- Photographs from files: whole with eviction, and by window.

`GcpRegression` runs `--orthomosaic` at the finish step of a real project (forest_tiny with
ground control) and checks the files, the EPSG code and that several photographs contribute.

## Automatic check points (`--gcp-check`)

Check points were already supported: a GCP typed `check` in the GCP file is left out of the
adjustment, triangulated from its markers with the final cameras, and reported with its own RMS
and rows in the report, with a warning when there are none. What was missing is holding points
back without editing the file.

| Option | Meaning |
| --- | --- |
| `--gcp-check none\|auto\|N` | `none` (default): the types in the file. `auto`: about one control point in five becomes a check point, keeping at least four control points (nothing is held back with fewer than five). `N`: that many; refused if fewer than three control points would remain. |
| `--gcp-check-ids ID,ID,...` | These GCPs become check points whatever their type in the file. |

Which points: the control points *kept* are chosen for spread (the two most distant points, then
repeatedly the point farthest from those already kept; ties go to the earlier line of the file).
The points held back therefore lie between control points, and the control network keeps its
extent. The choice depends only on the coordinates, so it is the same on every run and before
and after pinning. Points typed `check` in the file stay check points.

The choice is printed, stored in `gcp_report.json` (`check_selection`) and printed under the
point table of the report. The report's warning about missing check points now mentions
`--gcp-check auto`.

Checks: `OrthomosaicRegression` covers the selection on a 3 x 3 grid (auto holds two of nine
back, never a corner or the centre; counts; named ids; refusals; nothing with four points);
`GcpRegression` branches a real project at the gcp step with `--gcp-check-ids` and checks the
report (three check points, the point named, check RMS still small).

Not done: leave-one-out cross-validation (every control point held back in turn), which would
give an independent figure without giving up any control point, at the cost of one adjustment
per point.

## Verification of the whole

Full SelfTest: 121,869 checks, all passed (with `OrthomosaicRegression` and the extended
`GcpRegression`). Default dense output (pair SGM identity check on city2): dense.las SAME
(points), dense.ply SAME, point-confidence.bin SAME; neither addition touches reconstruction.
