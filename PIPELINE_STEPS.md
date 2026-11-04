# Pipeline steps

Collinea splits the reconstruction into separate
steps. Each step saves the project state of its run, so the next step can continue in a new
process, later, or with changed options. Without step names the whole pipeline runs in one
process as before (still saving the state after every step).

## Steps

| Step | Stages | Does | Reads | Writes to the state |
| --- | --- | --- | --- | --- |
| `inputs` | 00 | image inventory, orientation imports, resolution plan (`--estimate-memory` ends here) | images | image list |
| `calibrate` | 00, 01 | interior orientation (supplied, EXIF or automatic focal search, self-calibration), alignment images | images | frames (calibration, sizes), calibration |
| `match` | 02, 03 | features, pair matching, tracks; supplied poses are read here | frames | keypoints, feature colours, tracks, matched pairs (with relative poses) |
| `align` | 04 | SfM (or supplied/rectified poses), bundle adjustment, sparse cleanup, shared lens refinement | frames, tracks | cameras, triangulated tracks, final calibration |
| `georeference` | 04 | GPS similarity (single-line fit), GPS-weighted adjustment (lens, time offset, lever arm, rolling shutter), camera screening, final orientation exports | cameras, tracks | georeferenced cameras/tracks, CRS, report strings |
| `gcp` | 04 | ground control points from `--gcp` (skipped without it): predicted markers, `waiting` until pinned, datum onto the GCPs, adjustment with GCP priors, control/check residuals | cameras, tracks, markers | GCP-datum cameras/tracks, CRS, `gcp_summary` |
| `dense` | 05, 06 | dense matching and fusion, `dense.las` / `dense.ply`, `report.txt` | cameras, pairs, tracks | point counts |
| `ground` | 07 | `--classify-ground`, `--ground-pair-correction`, `--ground-aware-fusion` (skipped without them) | cloud, cameras | – |
| `finish` | 07 | file organisation, processing report (PDF/HTML), artifact catalogue | run folder | – |

Ground control points: see "Ground control points" below.

## Commands

```
photogrammetry -i IMAGES -o PROJECT [options]                 all steps (unchanged behaviour)
photogrammetry inputs -i IMAGES -o PROJECT [options]          start a new run
photogrammetry calibrate match align -o PROJECT               continue the latest run
photogrammetry georeference -o PROJECT --run ID               continue a given run
photogrammetry dense ground finish -o PROJECT --branch --min-views 2
                                                              new run from the state before 'dense'
photogrammetry status -o PROJECT [--run ID]                   steps done, next step
photogrammetry gcp-edit -o PROJECT [--run ID] [--gcp-port N]  pin GCP markers in the browser
```

- Later steps reuse the arguments saved by `inputs`; options given to a later step are appended
  and override them (they are saved too). An option that changes the alignment resolution after
  `calibrate` is refused (the saved keypoints would not fit); branch from `match` instead.
- A step needs all earlier steps complete (or skipped). A failed step can be run again on the
  same run. To redo a completed step, use `--branch`: the run history stays append-only.
- `--memory-mb` saved by `inputs` also sizes the Java heap of later steps.

## Saved state (`RUN/state`)

- `state.json`: step status and times, arguments, image list, coordinates description, CRS,
  georeference and geometry-quality text, dense point counts, branch origin.
- `frames.bin`: per image the calibration, processed size, camera (intrinsics, R, t), flags
  (registered, fixed, dense-excluded), velocity, keypoints (exact double coordinates, also after
  undistortion remapping), feature colours and the descriptor/pixel digests used by the cache.
- `tracks.bin`: tracks (observations, triangulated point), matched pairs (matches, relative
  pose, essential matrix, status) and the seed pair.
- `calibration.bin`: the main calibration and per-image orientation calibrations.
- `calibration_poses.bin`: camera poses of the self-calibration bootstrap; alignment uses them to
  choose its seed pair.
- `after-STEP/`: the state (plus `camera.txt` and `01_calibration/constraints.json`, which later
  steps rewrite) as it was after each step; `--branch` restores from the one before its first
  step. Unchanged files are hard-linked to the previous snapshot.

All values are written exactly (binary doubles), and every step starts from the same random seed,
so chaining or running the steps separately gives the same result.

## Run bookkeeping

- The first step creates the run (`runs/ID`); later steps attach to it: same run folder and id,
  stage history from `run.json`, console log appended, telemetry (`logs/processing.json`)
  merged with task and resource times offset from the run start. Between steps `run.json`
  status is `incomplete` with `next_step`; `finish` makes it `complete`.
- `--branch` creates a new run. Folders of the carried steps are copied, except stereo and fusion
  outputs (never rewritten later), which are hard-linked where the volume allows. The source run
  is never modified. `state.json` records `branched_from`.

## Verification (forest_tiny, `--quality low --cores 8 --classify-ground`, 4 Nov)

- Chained in one process, steps run one by one in eight processes, and the pre-split single-process
  pipeline: byte-identical `dense.las`, `dense.ply`, `point-confidence.bin`,
  `dtm.tif`, `04_geometry/final/exterior.csv`, `gps_bundle.json`, and calibration values
  (`camera.txt` differs only in its timestamp comment).
- `--branch` at match, align, georeference, dense and ground: identical `dense.las` and
  exterior orientations to the source run; the source run's files unchanged (hash of the whole
  folder before and after).
- Override on a branch: `--min-views 2` changes the cloud (20,167 → 29,405 points).
- Step run: `run.json` complete, all stages complete, report written, 237 telemetry tasks merged
  from eight processes (240 when chained).

## Which step reads an option

Give an option no later than the first step that reads it (it then holds for all later steps).
Options given too late are accepted but do not change what earlier steps already did.

| Step | Options it reads first |
| --- | --- |
| `inputs` | `-i`, `-o`, `--cores`, `--memory-mb`, `--gpu-memory-mb`, resolution (`--quality`, `--alignment-max-dimension`, `--max-dimension`); the memory plan written here also uses the dense options, so give those here too if the plan must be exact |
| `calibrate` | calibration options, and the matching options the automatic focal search already uses: `--features`, `--ratio`, `--neighbors`, `--descriptor-matcher`, `--descriptor-memory-mb`, `--bundle-iterations` (self-calibration) |
| `match` | `--min-forward-overlap` and the other pair-selection / overlap options |
| `align` | `--reprojection`, sparse-cleanup and lens-refinement options |
| `georeference` | `--crs`, `--gps*`, `--rolling-shutter`, `--camera-screening` |
| `gcp` | `--gcp`, `--gcp-markers`, `--gcp-sigma`, `--gcp-height-sigma` |
| `dense` | stereo and fusion options (`--step`, `--filtering`, `--min-views`, `--fusion-*`, `--pairwise-search`, ...) |
| `ground` | `--classify-ground`, `--ground-*` |

Verified on forest_tiny with the forest_4 option set of `examples/forest_4_steps.ps1` distributed like this: identical
orientations, `dense.las`, `dense.ply`, confidence sidecar and DTM to the single command.

## Ground control points

### GCP file (`--gcp FILE`)

```
gcp_id,x,y,z,sx,sy,sz,type
G1,605551.06,6938380.02,89.55,0.02,0.02,0.03,control
G7,605553.10,6938371.44,88.10,,,,check
```

- Columns: `gcp_id` (or `id`, `label`), `x`/`e`, `y`/`n`, `z`/`h`; optional `sx`, `sy`, `sz`
  (sigmas in metres; empty → `--gcp-sigma` 0.02 m and `--gcp-height-sigma` 0.03 m) and `type`
  (`control`, default, or `check`). Header optional; separator `,`, `;` or tab.
- Coordinates are in `--crs` (default EPSG:3067), heights in the vertical datum the GCPs were
  surveyed in; the output heights follow them.
- Check points are never used in the adjustment. Their errors are the independent accuracy
  measure in the report.

### Workflow

```
photogrammetry inputs -i IMAGES -o PROJECT [options]
photogrammetry calibrate match align georeference gcp -o PROJECT --gcp gcps.csv
      -> gcp: waiting (markers predicted, see 04_geometry/gcp/markers.json)
photogrammetry gcp-edit -o PROJECT                         pin the markers, press Done
photogrammetry gcp dense ground finish -o PROJECT
```

- Without enough pins the `gcp` step ends `waiting`: the run stays open (`status` shows it) and
  later steps are refused until `gcp` completes. Needed: 3 control points with 2+ pinned images
  each, or 1 when the block is already GPS-georeferenced.
- The editor (`gcp-edit`) runs a local web page (127.0.0.1, `--gcp-port`, default 8765) and opens
  the browser. Left: the GCP list with pin counts. Right: an image card per image the GCP should be
  visible in, centred on the predicted position. Click a card to zoom, click the exact position to
  pin; in the zoom view `R` rejects the image (not visible), `U` unpins, arrows step through the
  images, `+`/`-` zoom, Esc closes. Each pin is saved immediately and the
  predictions of the other images are recomputed from the pins (after two pins they follow the
  triangulated point). Done stops the editor.
- Markers are stored in original, distorted image pixels, so they stay valid when the lens model
  changes. `--gcp-markers FILE` starts a new run (or branch) from another run's markers.json.
- Running `gcp` again after more pins (or `--branch` from `gcp`) redoes the adjustment.

### What the step does

1. Triangulates each control point from its pins and moves the block onto the GCPs: a 7-parameter
   similarity with 3+ control points, otherwise a shift (GPS keeps scale and rotation). A block
   that was not georeferenced becomes georeferenced in `--crs`.
2. Adds each control point as a tie point with a position prior (its sigmas), observed by its pins.
3. With GPS (3+ cameras): reruns the GPS-weighted adjustment with the datum fixed by the GCPs. The
   GPS fixes get a common offset (reported as `gps_offset_to_gcp_datum_metres`, e.g. a geoid
   difference) instead of moving the block; lens, time offset, lever arm and rolling shutter are
   refitted as in `georeference`. Without GPS: bundle adjustment with the shared lens refit
   (`--gps-intrinsics`).
4. Writes `04_geometry/gcp/gcp_report.json` and `gcp_residuals.csv` (control and check errors in
   X, Y, Z, per-marker pixel residuals) and the GCP pages of the processing report.

## Processing report

`finish` writes `07_results/processing-report.pdf` (plus HTML and JSON). Sections:

| Section | Content |
| --- | --- |
| Title | orthomosaic, summary (images, tie and dense points, CRS, GCP RMS, ground, time), quality flags |
| Survey Data | camera and overlap maps; images, camera stations, flying altitude, ground resolution, coverage, tie points, projections, reprojection error; camera model table |
| Camera Calibration | image residual plot; F, Cx, Cy, K1–K3, P1, P2 (source pixels); GPS time offset, lever arm, rolling shutter |
| Camera Locations | GPS error plot (arrows XY, colour Z); X, Y, Z, XY, total RMS; per camera errors |
| Ground Control Points | GCP plot; control and check RMS tables; per point errors, images and marker pixel error |
| Digital Elevation Model | DSM, resolution, point density; ground classification and DTM when classified |
| Processing Parameters | per step status, time and main settings; software and hardware |

`--report-detail full` appends the detailed diagnostics (solver profiles, resources, hardware,
all settings).
