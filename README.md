<p align="center">
  <picture>
    <source media="(prefers-color-scheme: dark)" srcset="docs/images/pose-logo-dark.png">
    <img src="docs/images/pose-logo.png" alt="POSE" width="520">
  </picture>
</p>

# POSE

**Photogrammetric Orientation and Surface Estimation.** POSE turns overlapping photographs into
georeferenced point clouds, surface and terrain models. It is written in Java, uses OpenCL for dense stereo matching and depth fusion, and runs on
its own or as part of [LAS.utils](https://github.com/kmik/LAS.utils).

![Point cloud colored by elevation](docs/images/city-point-cloud.png)

## What it does

- **Alignment:** feature matching, camera alignment and bundle adjustment, with automatic focal
  length and lens distortion estimation, or a supplied calibration and camera poses.
- **Georeferencing:** from image GPS, with a GPS-weighted adjustment that also estimates the lens,
  the GPS time offset, the antenna lever arm and the rolling shutter.
- **Ground control points:** import a GCP file, pin the points in the images in a browser editor,
  and adjust the block onto them. Check points give an independent accuracy estimate.
- **Dense matching:** semi-global matching (Census, AD-Census, Census-gradient, ZNCC costs) with
  multi-view depth fusion and surface consolidation.
- **Ground classification:** ground points, a terrain model (DTM) and a canopy height model,
  without any reference terrain.
- **Outputs:** LAS and PLY clouds (with normals, surface metrics and point confidence), DSM, DTM,
  orthomosaic preview, and a PDF processing report in the style of survey software.
- **Step-by-step processing:** run the pipeline in one go or one step at a time, change settings
  between steps, and branch a run to try other settings.

## Build

For installations without administrator rights or a suitable system Java, use a
[portable POSE release](PORTABLE_INSTALL.md). It includes private Java 25; the Linux
x86-64 compatibility package targets glibc 2.28 and is built once by the release
maintainer. End users only extract and run it. See that guide for producing the
archive; adding these scripts alone does not publish a release.

The upstream OpenPnP native library used by a source build may require glibc 2.29
or newer. On glibc 2.28, use a validated portable compatibility release rather
than changing system libraries.

You need:

- a **64-bit JDK 25 or later** (`JAVA_HOME` set, or `java` and `javac` on PATH);
- internet access for the first build, which downloads OpenCV, Proj4J, TwelveMonkeys ImageIO and
  Apache PDFBox into `lib/`;
- for GPU processing, a vendor OpenCL runtime (GPU depth fusion needs double precision, FP64).
  Without OpenCL everything runs on the CPU.

Windows x64 and Linux x64 (glibc) are supported. No system OpenCV or CUDA installation is needed.

```powershell
# Windows
.\build.bat
.\photogrammetry.bat -h
```

```bash
# Linux
bash build.sh
bash photogrammetry.sh -h
```

`pose.bat` and `pose.sh` start the same program as `photogrammetry.bat` and `photogrammetry.sh`;
the examples below use `photogrammetry`.

The build scripts compile the program and run its test suite. `mvn package` only compiles and
copies the dependencies.

## Quick start

One command runs everything:

```powershell
.\photogrammetry.bat -i .\images -o .\projects\forest --quality medium --classify-ground
```

The results are in `projects\forest\runs\<run-id>\07_results\`: `dense.las`, `dense.ply`,
`dsm.tif`, `dtm.tif` and `processing-report.pdf`.

Run with no arguments or `-h` for every option.

## Step by step

The same pipeline can be run one step at a time. Each step saves its state, so the next step can
run later, in a new process, or with different settings.

| Step | Does |
| --- | --- |
| `inputs` | reads the images and starts a new run |
| `calibrate` | camera calibration (supplied, from EXIF, or estimated) |
| `match` | features, image pairs and tie points |
| `align` | camera alignment and bundle adjustment |
| `georeference` | GPS georeferencing and the GPS-weighted adjustment |
| `gcp` | ground control points (only with `--gcp`) |
| `dense` | dense matching and fusion: the point cloud |
| `ground` | ground classification (only with `--classify-ground`) |
| `finish` | report and final files |

```powershell
.\photogrammetry.bat inputs -i .\images -o .\projects\forest --quality medium
.\photogrammetry.bat calibrate match align georeference -o .\projects\forest
.\photogrammetry.bat dense ground finish -o .\projects\forest --min-views 2 --classify-ground
.\photogrammetry.bat status -o .\projects\forest
```

- Later steps reuse the options of `inputs`. Options given to a later step are added and hold for
  the steps after it.
- `--run ID` continues an older run instead of the latest one.
- `--branch` starts a new run from the saved state just before the first named step, so a step
  can be redone with other settings without touching the original run:

  ```powershell
  .\photogrammetry.bat dense ground finish -o .\projects\forest --branch --filtering mild
  ```

`examples/forest_4_steps.ps1` is a complete example with each option given at the step that uses
it. [Pipeline steps](PIPELINE_STEPS.md) lists which step reads which option.

## Ground control points

1. Write the GCP coordinates to a CSV file, in the output coordinate system (`--crs`, default
   EPSG:3067):

   ```text
   gcp_id,x,y,z,sx,sy,sz,type
   G1,605551.06,6938380.02,89.55,0.02,0.02,0.03,control
   G2,605569.64,6938375.92,85.33,,,,control
   G3,605579.60,6938372.18,85.86,,,,control
   G4,605553.10,6938371.44,88.10,,,,check
   ```

   `sx`, `sy`, `sz` are the coordinate accuracies in metres (defaults `--gcp-sigma 0.02` and
   `--gcp-height-sigma 0.03`). `type` is `control` (used in the adjustment) or `check` (only
   measured). The header line is optional.

2. Run the steps up to `gcp`. The step projects every GCP into the images and waits for you:

   ```powershell
   .\photogrammetry.bat calibrate match align georeference gcp -o .\projects\forest --gcp .\gcps.csv
   ```

3. Open the editor, which runs in your browser:

   ```powershell
   .\photogrammetry.bat gcp-edit -o .\projects\forest
   ```

   Pick a GCP, click an image, and click the exact position of the target. The editor then
   finds the target in the other images by itself (blue markers): check them and press `A` to
   accept, or accept them all at once. Images where trees or roofs probably hide the target are
   listed separately, and each pin shows its residual in pixels. `R` marks an image where the
   target is not visible. Press **Done** when finished.

4. Continue:

   ```powershell
   .\photogrammetry.bat gcp dense ground finish -o .\projects\forest
   ```

At least three control points pinned in two or more images each are needed (one is enough when
the images have GPS). The report lists the control and check point errors and the marker
residuals.

## Ground classification

`--classify-ground` marks ground points in `dense.las` (class 2 ground, 7 low noise, 1 other) and
writes `dtm.tif` (terrain), `chm.tif` (canopy height), `ground_support.tif` (where the ground was
seen and where it was interpolated) and `ground_uncertainty.tif`. It needs no reference terrain:
it combines a morphological filter with the camera rays, which pass through free space and so
cannot run below the terrain. Patches that look like ground but lie in the tree canopy are
rejected. Details: [ground classification](GROUND_CLASSIFICATION.md).

Two optional extras:

- `--ground-pair-correction` aligns the ground of each stereo pair before fusion, removing small
  height steps between pairs.
- `--ground-aware-fusion` merges the ground points of all pairs into one smooth ground surface.

## Common options

| Option | Default | Effect |
| --- | --- | --- |
| `--quality` | `custom` | `ultra`, `high`, `medium`, `low` process the images at full, 1/2, 1/4 or 1/8 size. |
| `--max-dimension` | `1600` | Longest image side for `custom` quality (also caps a named quality if given). |
| `--step` | `2` | Dense sample spacing in pixels. |
| `--filtering` | `moderate` | `mild`, `moderate` or `aggressive` depth filtering. |
| `--min-views` | `3` | Images that must agree on a point. |
| `--crs` | `EPSG:3067` | Output coordinate system for GPS and GCP georeferencing. |
| `--bundle-iterations` | until converged | Bundle adjustment iterations. |
| `--memory-mb` | `2048` | Processing buffer budget in MiB (not a total RAM limit). |
| `--gpu-memory-mb` | automatic | GPU working-set budget in MiB (default 70% of the VRAM). |
| `--compute` | `auto` | `auto` uses the GPU when suitable, `cpu` or `gpu` force one. |
| `--cores` | half the CPUs | CPU threads. |
| `--dense-method` | `sgm` | `mvsgm` (opt-in, needs `--fusion-mode image`): one depth map per image from several neighbouring images at once ([multi-view SGM](MULTIVIEW_SGM.md)). |
| `--block-processing` | `off` | `auto` or `on` splits very large image sets into overlapping blocks ([block processing](BLOCK_PROCESSING.md)). |

`--estimate-memory` shows the memory plan without processing. Large images and GPU settings:
[scaling](SCALING.md) and [performance and attributes](PERFORMANCE_AND_ATTRIBUTES.md).
Camera calibration and orientation files: [orientations](ORIENTATIONS.md).

## Output

Each run gets its own folder, `<project>/runs/<run-id>/`; earlier runs are kept.

```text
00_inputs/       image inventory
01_calibration/  camera calibration
02_features/     features
03_matches/      image pairs and tie points
04_geometry/     camera positions, georeferencing, ground control (gcp/)
05_stereo/       stereo pairs
06_fusion/       depth fusion
07_results/      point clouds, DSM, DTM, report
state/           saved step state
```

The processing report (`07_results/processing-report.pdf`, also HTML and JSON) has a summary with
quality flags, survey data (flying altitude, ground resolution, coverage, tie points, reprojection
error), camera calibration, camera location errors, ground control points, the elevation models
and the processing parameters of each step. `--report-detail full` adds the detailed diagnostics.
See [processing report](PROCESSING_REPORT.md) and [project layout](PROJECT_FORMAT.md).

## LAS.utils

Copy this repository into `sgm_java/` at the LAS.utils root, copy the wrappers from `las-utils/`
into that root, and build the module. POSE keeps its own classpath, so its OpenCV version does
not clash with the one in LAS.utils. The Java entry point is `photogrammetry`.

## Limitations

- Automatic calibration needs good overlap and camera geometry; otherwise supply a calibration.
- With GPS only, heights follow the EXIF GPS altitude (no geoid conversion). Use GCPs for a
  surveyed height datum.
- Under closed canopy the terrain is interpolated from the nearest visible ground.
- LAS output is version 1.2, point format 2, up to 4,294,967,295 points per file.

## Documentation

- [Pipeline steps and ground control points](PIPELINE_STEPS.md)
- [GPS-weighted adjustment](ORIENTATION_GPS_ADJUSTMENT.md)
- [Ground classification](GROUND_CLASSIFICATION.md)
- [Processing report](PROCESSING_REPORT.md)
- [Orientation formats](ORIENTATIONS.md)
- [Project layout](PROJECT_FORMAT.md)
- [Storage and GPU processing](STORAGE_GPU.md)
- [Large images and scaling](SCALING.md)
- [Multi-view SGM](MULTIVIEW_SGM.md)
- [Block processing](BLOCK_PROCESSING.md)
- [Image fusion memory](FUSION_MEMORY.md)
- [Third-party dependencies](THIRD_PARTY_NOTICES.md)

Author: [Mikko Kukkonen](https://github.com/kmik). No project license has been selected;
dependencies keep their own licenses.
