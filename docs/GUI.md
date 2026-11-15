# POSE desktop windows

POSE has native desktop windows (Java Swing, part of the bundled Java runtime; no browser and
no extra libraries). They work on Windows and Linux. Processing is unchanged: the windows only
call the same code as the command line or start the command line as a child process.

## Starting

```
pose-gui.bat [PROJECT]          (Windows; no console window)
./pose-gui.sh [PROJECT]         (Linux)
photogrammetry gui [PROJECT]    (any launcher)
```

The window has a **project folder** bar at the top, two tabs (**New run** and **Project**) and a
**console** at the bottom. Without a display it prints a short message and exits; use the
command line there. On Windows, set `POSE_GUI_CONSOLE=1` before `pose-gui.bat` to keep a
console for error messages.

The **Help** menu has this guide (**User guide**, F1), the full **Command-line options** with
a search box (as `photogrammetry --help TEXT`), **Open documentation folder**, and **About
POSE** with the version, the release of a portable package, the processing-engine
identifier, Java and system details (**Copy details** for a bug report).

The **Settings** menu:

- **Preferences...** (Ctrl+,):
  - **Interface scale:** automatic follows the system, or 100–200%. It applies at the next start,
    also to the GCP editor and Images windows. `-Dsun.java2d.uiScale` on the Java command line
    wins.
  - **New projects start with:** the last options you used (the default) or always the POSE
    defaults. A project folder used before keeps its own options either way.
  - **Open the report when a run completes:** off by default.
  - **Bring POSE to the front when a command ends:** on by default.
  - **Ask before closing POSE while a command runs:** on by default; when off, closing stops the
    command.
  - **Size limit of the image preview cache:** 1000 MB; the least recently used previews go
    first. **Clear now** empties it.
- **Fit compute options to this computer...** reads the CPU cores, RAM and the GPU (as numbered
  by `--list-gpus`, for the GPU index on the New run tab). It offers to set `--cores`,
  `--memory-mb` and `--gpu-memory-mb` by the same rule as the window's defaults (see New run),
  or `--compute cpu` without a GPU. Use it after changing the fields, or for another GPU index.
  It shows the old and new values first; other options are not touched.
- **Clear recent projects** empties the project folder list. **Forget saved options...** deletes
  the options remembered for each project folder and as your defaults; projects and runs are
  not touched. **Open settings folder** opens the folder these are kept in
  (`preferences.properties`, `defaults.properties`, `projects/`, `image-cache/`).

Every command the window starts is the normal command line (`photogrammetry ...`, same Java,
same class path, same `PHOTOGRAMMETRY_HEAP`), so `--memory-mb` still relaunches it with a larger
heap. The console shows the exact command first; you can copy it and run it in a terminal.

## New run

- **Input and project:** image folder, optional orientation import (`--orientation-input`, a
  file or folder), optional GCP file and check points, optional **Lidar reference (LAS/LAZ)**
  (`--lidar-reference`: one .las/.laz file or a folder of tiles; the "..." chooser shows LAS/LAZ
  files and accepts a folder), coordinate system. The lidar field is checked like the others
  (the file or folder must exist); the further `--lidar-*` settings are on the Advanced options
  tab. A run with a lidar reference gets the extra step `lidar` after `georeference` (see
  `photogrammetry --help lidar`); if the lidar does not cover the block, that step fails
  and its error (Project tab, step details) says which part is missing. The run summary then
  shows a "Lidar reference" line (estimated shifts and angles, stable-area RMS, whether fewer
  parameters were used) and the height datum; the report has a Lidar Reference page and the
  lidar quality flags appear with the others.
- **Processing:** quality, maximum image dimension, dense method, filtering, minimum views,
  quick calibration, tie-point cleanup (see below), ground classification, orthomosaic.
- **Compute and memory:** compute device, GPU index and memory, GPU normals in fusion, CPU
  cores, memory budget, RAM limit.
- **Outputs:** dense.ply, LAS tile size, point confidence file, verbose log.

Hover over a label for the option's help text from `photogrammetry --help`. A field left empty
(or "(default ...)") is not passed, so the command line default applies; the default is shown in
grey. **Extra arguments** takes options typed by hand, quoted like in a shell; they come after
all fields and win.

## Advanced options

Every other option of a new run has a field on the **Advanced options** tab (about 250),
grouped like the command-line help (inputs, calibration, block processing, feature matching,
dense stereo, filtering and fusion, georeferencing, GCPs, workflow controls, report, orthomosaic,
orientation import/export). The fields are made from the help text itself, so a new option
appears there by itself. Each row shows the option, its field (the default in grey), and the
first sentence of its help; hover the name for the full help. **Search** filters by name or help
text, **Show only the options I set** lists your changes, and the tab title counts them
(e.g. "Advanced options (2)"). **Clear advanced options** returns them all to the defaults;
**Reset options** on the New run tab clears both tabs. Advanced values go into the command after
the New run fields, are checked by the option parser and are remembered with the other
settings. Not listed: commands rather than options (help, exports, run analysis, chessboard
calibration mode), and second names of an option (e.g. `--epsg` for `--crs`).

Dense method **mvsgm** brings its required and tested settings with it (those that differ from
the command-line defaults): `--fusion-mode image` (required), `--surface-consolidation none`,
`--fusion-outliers robust`, `--fusion-strategy graph-average`, `--subpixel-fit equiangular`,
`--depth-outlier-filter edge-aware`, `--windowed-mode auto`, `--min-angle 2`, `--mvsgm-neighbors 8`
and `--mvsgm-optimal-neighbors`. Surface consolidation is off because image fusion does not need
it and, when on, mvsgm reserves disk for its records as if every pixel became a point (a large
block is then refused for lack of disk). Any of these that you set yourself (on the Advanced
options tab or in the extra arguments, also `--no-mvsgm-optimal-neighbors`) is kept as yours. The same applies to the
options typed on the Project tab for continuing or branching a run.

The **Command** box shows the exact command and whether the option parser accepts it (the same
parser as the command line). **Memory estimate** runs `--estimate-memory` in a temporary project
and shows the resolution and memory plan; nothing is kept. **Run** starts a new run in the
project folder with all steps. **Reset options** clears the options (the folders stay).

The window starts with its own defaults. The compute sizes come from this computer:
- `--cores`: three quarters of the processors.
- `--memory-mb`: half the RAM, rounded down to 1000.
- `--gpu-memory-mb`: 95% of GPU 0's memory (the most the command line accepts), rounded down
  to 100; `--compute cpu` without an OpenCL GPU.

The hardware is read once (`--list-gpus` in a child process) and kept in `machine.properties`
in the settings folder; it is read again in the background at each start. On a 16-core laptop
with 32 GB and an 8 GB GPU this gives 12 cores, 16000 MB and 7700 MB. Saved settings that still
have the earlier fixed sizes (12, 16000, 8000) take the computer's sizes; sizes you set
yourself stay. The other defaults are the tested mvsgm setup: `--quality high --quick-calibration --ply off --point-confidence off
--features 40000 --ratio 0.85 --neighbors 8 --descriptor-matcher flann --matching-algorithm sift
--descriptor-memory-mb 1600 --min-forward-overlap 50 --step 1 --filtering moderate --dense-method
mvsgm --min-views 2 --min-angle 2 --confidence-threshold 0.0 --surface-consolidation none
--windowed-mode auto --fusion-gpu-evidence auto --fusion-mode image --fusion-outliers robust
--subpixel-fit equiangular --fusion-strategy graph-average --mvsgm-optimal-neighbors
--surface-flatten 8 --depth-outlier-filter edge-aware --mvsgm-neighbors 8 --mvsgm-best-k 3
--mvsgm-census-window 5 --mvsgm-consistent-views 1 --mvsgm-consistency-labels 1` (the fields on
New run and Advanced options are filled with these). **Reset options** returns to them, keeping
the folders. The command line itself keeps its own defaults. The values are remembered per
project folder, and the last options used (without the folders) become your defaults for new
projects. They are kept in `%APPDATA%\POSE\gui` on Windows and
`~/.config/pose/gui` on Linux (`-Dpose.gui.settings=DIR` puts them elsewhere).

## Console

The step strip shows each step of the run (pending, running, complete, skipped, waiting,
failed) with its time; the bar under it shows the current stage and its percentage, as on the
command line. Then the live log, the elapsed time and **Stop**, which ends the command and every
process it started. A stopped step stays marked "running" in the run (the process had no
chance to record anything); run it again from the Project tab with **Run next step**. When the
command ends, **Open** shows the report (PDF or HTML), `07_results` or the
project folder. While idle, the console shows the newest run of the project.

## Project

- **Runs:** every run of the project folder with start time, status (complete, waiting,
  failed, incomplete), number of points and the next step. **Refresh** reads the folder again.
- **Steps of the selected run:** status, start, finish and time of each step.
  - **Run next step / Run remaining steps / Run selected steps** continue the run
    (`photogrammetry STEP ... -o PROJECT --run ID`).
  - **Redo steps in a new run...** redoes one or more steps with other options, e.g. only
    `align` after dense matching is done. Choose **Redo from** and **up to** (the selected rows
    by default) and the options. The run is copied as it was just before the first of them, so
    nothing earlier runs again; the chosen steps run in a new run (`--branch`), and the steps
    after them stay pending there, to be run later with **Run remaining steps** (or never). The
    source run, with its dense cloud, is not changed. Options only change the steps that use
    them: `--features` or `--ratio` belong to `match`, so to change them redo from `match`.
  - **Options for these steps** are passed to the steps run here (or are the changed options of
    a branch), e.g. `--filtering mild --min-views 2`; they are checked with the run's saved
    options.
  - **Edit GCPs** (when the run waits at the gcp step) opens the GCP marker editor; after
    **Done** there, the run continues from the gcp step. When a run in the console stops to wait
    for markers, the window offers to open the editor.
  - **Clean tie points...** (when the run waits at the cleanup step, see below) opens the
    tie-point cleanup window; its choice continues the run.
  - **Report again** writes the processing report again into `07_results/report` and
    **Orthomosaic again** makes the orthomosaic again, both without reprocessing.
- **Summary and quality flags:** the main numbers of the processing report (images aligned,
  tie points, reprojection error, ground resolution, area, point density, points, coordinate
  system) and its quality flags, plus the run's saved arguments. A failed run shows its error
  in red. **Copy run details** opens the run's status, steps, failed step, error, arguments and
  the end of its log as text to copy (for debugging or a bug report).
- **Step details:** double-click a step (or right-click > Show details) for its full message;
  for a failed step also the run's error, the error lines of `logs/console.log` and its end.
  The text can be selected and copied (**Copy**). Right-click > Copy (or Ctrl+C) copies the
  selected rows of the steps table.

## Tie-point cleanup

Like Metashape's gradual selection, the `cleanup` step (between `align` and `georeference`)
removes weak tie points and adjusts the block again. The **Tie-point cleanup** field on the New
run tab (`--tie-point-cleanup`) chooses how:

- **off** (the default): no cleanup.
- **auto:** cleanup with the automatic limits, without stopping:
  - largest reprojection error: three times the median of the points' largest errors, at
    least 0.5 px;
  - fewest images: 2, which removes none;
  - smallest intersection angle: 1 degree;
  - most removed: 20%.
- **custom:** cleanup with your own limits, without stopping. The four fields under the mode
  (enabled for custom and pause) set them; one left empty keeps its automatic value:
  - **Largest error (px)** (`--tie-point-max-error`);
  - **Fewest images** (`--tie-point-min-views`);
  - **Smallest angle (degrees)** (`--tie-point-min-angle`);
  - **Most removed (%)** (`--tie-point-max-removed`).

  With off or auto the fields are left out of the command; on the command line, auto together
  with a limit is refused (use custom).
- **pause:** the run stops after alignment (step `cleanup` waiting) and the window offers the
  **Tie-point cleanup** window. It has three histograms of the tie points:
  - their largest reprojection error;
  - the number of images that see them;
  - their largest intersection angle (a small angle means an uncertain depth).

  Each histogram has its limit, and red bars fail it; click a histogram or type a value to
  move the limit. The line below counts what the limits remove:
  - worst first: most limits failed, then the largest error;
  - at most the given share;
  - every camera keeps at least 15 points.

  It also shows the RMS error with and without the removed points; it starts from the four
  limit fields.

  Below the histograms, a **3D view of the tie points** shows the points the limits remove in
  **red** and the others in grey, together with the cameras; it changes with every limit. Untick
  **Kept points** to see only the red ones. Hover a point for its error, images and angle.
  Mouse, **Home** and **Top** work as in the 3D view. The positions are those of the alignment
  before georeferencing, turned so that the cameras look down.

  **Automatic limits** sets the auto values. **Apply and continue the run** removes the red
  points, adjusts the block again and runs the remaining steps. **Skip the cleanup and
  continue** keeps every point. **Cancel** leaves the run waiting (Project tab, **Clean tie
  points...**).

The selection in the window and in the step is the same code, so the window's count is exactly
what the step removes. Report: `04_geometry/tie_points/cleanup.json`. To try other limits later,
use **Redo steps in a new run...** from `cleanup`. On the command line the waiting run continues
with `photogrammetry cleanup -o PROJECT --tie-point-cleanup auto [limits]`, or
`--tie-point-cleanup off` to skip it.

## 3D view

**3D view...** on the Project tab shows the selected run in 3D (one window per run). What it
shows depends on how far the run got:
- the dense cloud (`07_results/dense.las` or LAS tiles), as every k-th point up to the point
  budget in the toolbar (6 M by default);
- the tie points and the cameras, available from the georeference step on.
- for a run made with `--lidar-reference`, the lidar points (the **Lidar** check box, drawn in
  orange) within the cloud's extent plus 10 m, every k-th point up to the same point budget,
  noise and withheld points left out. They are read from the files the run used (LAS or LAZ)
  when the box is first ticked, so the photogrammetric surface can be compared with the ALS
  directly. The box is disabled for runs without a lidar reference.
**Colour** by:
- RGB;
- height;
- classification (after ground classification);
- the cloud's surface attributes (vertical part of the normal, roughness, surface variation,
  planarity);
- with `--point-confidence on`, views per point and confidence.

Tie points are coloured by the number of images that see them. Cameras are drawn as small
pyramids pointing where they look; **red** ones are flagged by camera screening (hover for the
name, flag and reprojection error). Misaligned or turned cameras stand out at once.

- Left drag orbits, right drag (or Shift + drag) pans, the mouse wheel zooms towards the
  pointer, double-click centres on the point under the pointer.
- **Home** shows everything (stray tie points far away are left out), **Top** looks straight
  down like a map (orthographic), **Side** looks from the south.
- **Shading** (eye-dome lighting) darkens points behind nearer ones, so tree crowns and edges
  stand out.
- The bottom line shows the coordinates, class and attributes of the point under the pointer.
- While moving, a sample of the points is drawn; the full picture follows when the view rests.

Nothing is written into the project.
## Images

**Images...** on the Project tab opens the photographs of the selected run in their own window
(one per run).

- **Previews** on the left, with the reprojection error and any camera-screening flag of each
  photograph, or "not oriented". Click one to select it, double-click to open it on the
  **Image** tab at full resolution (mouse wheel zooms, drag pans, double-click fits).
- **Map** tab: the photographs projected to real-world coordinates through the run's final
  camera calibration and poses (`04_geometry/orientation_model.json`, written when the dense step
  starts) onto the run's surface model (`07_results/dsm.tif`; before it exists, a plane at the
  median tie-point height). Each photograph's **on map** box switches it on or off (**All**,
  **None**, **Flagged**). Lower the **Opacity** to let overlapping photographs show through each
  other: well-aligned ones look sharp, misaligned ones doubled. **Outlines** draws the
  footprints; the selected photograph has an orange outline and, with **Selected on top**, is
  drawn last. Click the map to select the photograph on top there, double-click to open it.
  The bottom line shows the coordinates and surface height under the pointer (in the run's
  coordinate system). The visible area is drawn again at screen resolution after each zoom or
  pan.
- Previews are decoded once and cached in the settings folder (`image-cache`); nothing is
  written into the project. Edges of buildings and trees can look ragged on the map where the
  surface model has gaps or steps; that is the surface, not the alignment.

Only one command runs at a time.

## GCP marker editor

```
photogrammetry gcp-edit -o PROJECT [--run ID] [--no-continue] [--browser]
```

Run it when the `gcp` step of a run says `waiting`. The window has three parts:

- **Left:** the ground control points with pins per GCP, the pin RMS and largest pin residual
  (green below 1 px, amber below 3 px, red above), and whether the run is ready for the gcp
  step. **Done** closes the editor.
- **Middle:** image crops of the selected GCP: pinned (green frame), refined by patch matching
  (blue, with the correlation score), predicted (red), rejected (faded). Images whose view of the
  point passes below the tie-point surface (canopy, roofs) are under **probably hidden**.
  Registered images without a prediction are listed at the bottom for placing a marker by hand.
  **Accept N refined** pins every refined marker with a score of at least 0.85.
- **Right:** the selected image. Click to pin the marker there. Drag to pan, mouse wheel to zoom
  (around the pointer). The yellow ring is the predicted position.

Keys: Left/Right previous/next image, Up/Down previous/next GCP, `+`/`-` zoom, `F` fit the whole
image, `A` or Enter accept (and go to the next image), `R` reject, `U` unpin.

Every change is saved at once to `04_geometry/gcp/markers.json` of the run. After a pin, the
residuals are recomputed and the other images are refined in the background ("Refining
markers..."); the crops update when it finishes.

**Done** with "Continue the run after Done" ticked (the default) runs the `gcp` step on the run
when enough markers are pinned, in the same console. Untick it, or give `--no-continue`, to only
save the markers; then run `photogrammetry gcp -o PROJECT` yourself. Closing the window with its
close button saves the markers but does not continue the run.

Without a display (a server, SSH without X forwarding), or with `--browser`, gcp-edit serves the
earlier web page on `http://127.0.0.1:8765/` (`--gcp-port N`) instead.

High-resolution screens: Windows scales the window by the display setting. On Linux set
`GDK_SCALE=2` (or start Java with `-Dsun.java2d.uiScale=2`) if the window looks too small.
