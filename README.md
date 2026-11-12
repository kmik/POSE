<p align="center">
  <picture>
    <source media="(prefers-color-scheme: dark)" srcset="docs/images/pose-logo-dark.png">
    <img src="docs/images/pose-logo.png" alt="POSE" width="520">
  </picture>
</p>

# POSE

**Photogrammetric Orientation and Surface Estimation.** POSE turns overlapping photographs into
georeferenced point clouds, surface models, terrain models and orthomosaics. It is written in
Java, uses OpenCL for dense matching and depth fusion, and runs on its own or as part of
[LAS.utils](https://github.com/kmik/LAS.utils).

![Point cloud colored by elevation](docs/images/city-point-cloud.png)

## Features

- Camera alignment and bundle adjustment, with automatic or supplied calibration and poses.
- Georeferencing from image GPS, and from ground control points pinned in a browser editor.
- Dense matching (semi-global matching, optionally multi-view) with multi-view depth fusion.
- Ground classification with a terrain model and a canopy height model.
- Full-resolution true orthomosaic as a tiled GeoTIFF.
- LAS and PLY point clouds, DSM, DTM and a PDF processing report.
- The pipeline runs in one go or one step at a time.

## Build

You need a 64-bit JDK 25 or later. The first build downloads the dependencies into `lib/`.
A vendor OpenCL runtime enables GPU processing; without one everything runs on the CPU.
Windows x64 and Linux x64 are supported.

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

The build scripts compile the program and run its tests. `pose.bat` and `pose.sh` start the same
program. For machines without a suitable Java, see the [portable release](PORTABLE_INSTALL.md).

## Quick start

```powershell
.\photogrammetry.bat -i .\images -o .\projects\forest --quality medium --classify-ground
```

The results are in `projects\forest\runs\<run-id>\07_results\`: `dense.las`, `dense.ply`,
`dsm.tif`, `dtm.tif` and `processing-report.pdf`. Each run gets its own folder; earlier runs are
kept. Run with `-h` for every option.

## Step by step

Each step saves its state, so the next one can run later or with different settings.

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
```

`--run ID` continues an older run, and `--branch` redoes a step with other settings in a new run
without touching the original. `examples/forest_4_steps.ps1` is a complete example.

## Ground control points

Write the points to a CSV file in the output coordinate system (`--crs`, default EPSG:3067).
`type` is `control` (used in the adjustment) or `check` (only measured):

```text
gcp_id,x,y,z,sx,sy,sz,type
G1,605551.06,6938380.02,89.55,0.02,0.02,0.03,control
G2,605553.10,6938371.44,88.10,,,,check
```

```powershell
.\photogrammetry.bat calibrate match align georeference gcp -o .\projects\forest --gcp .\gcps.csv
.\photogrammetry.bat gcp-edit -o .\projects\forest     # pin the points in the browser
.\photogrammetry.bat gcp dense ground finish -o .\projects\forest
```

At least three control points pinned in two or more images each are needed (one is enough when
the images have GPS). `--gcp-check auto` holds some points back as check points automatically.

## Common options

| Option | Default | Effect |
| --- | --- | --- |
| `--quality` | `custom` | `ultra`, `high`, `medium`, `low`: images at full, 1/2, 1/4 or 1/8 size. |
| `--max-dimension` | `1600` | Longest image side for `custom` quality. |
| `--step` | `2` | Dense sample spacing in pixels. |
| `--filtering` | `moderate` | `mild`, `moderate` or `aggressive` depth filtering. |
| `--min-views` | `3` | Images that must agree on a point. |
| `--crs` | `EPSG:3067` | Output coordinate system. |
| `--classify-ground` | off | Ground classes, `dtm.tif` and `chm.tif`. |
| `--orthomosaic` | off | Full-resolution true orthomosaic. |
| `--dense-method` | `sgm` | `mvsgm` matches several neighbouring images at once (needs `--fusion-mode image`). |
| `--block-processing` | `off` | `auto` or `on` splits very large image sets into overlapping blocks. |
| `--compute` | `auto` | `cpu` or `gpu` force one. |
| `--cores` | half the CPUs | CPU threads. |
| `--memory-mb` | `2048` | Processing buffer budget in MiB (not a total RAM limit). |

`--estimate-memory` shows the memory plan without processing.

## LAS.utils

Copy this repository into `sgm_java/` at the LAS.utils root, copy the wrappers from `las-utils/`
into that root, and build the module. The Java entry point is `photogrammetry`.

## Limitations

- Automatic calibration needs good overlap and camera geometry; otherwise supply a calibration.
- With GPS only, heights follow the EXIF GPS altitude (no geoid conversion). Use GCPs for a
  surveyed height datum.
- Under closed canopy the terrain is interpolated from the nearest visible ground.

## Documentation

- [Pipeline steps and ground control points](docs/PIPELINE_STEPS.md)
- [Project layout](docs/PROJECT_FORMAT.md)
- [Orientation formats](docs/ORIENTATIONS.md)
- [Ground classification](docs/GROUND_CLASSIFICATION.md)
- [Orthomosaic and automatic check points](docs/ORTHOMOSAIC.md)
- [Multi-view SGM](docs/MULTIVIEW_SGM.md)
- [Thin surfaces](docs/THIN_SURFACES_FUSION.md)
- [Quick calibration](docs/QUICK_CALIBRATION.md)
- [Portable releases](PORTABLE_INSTALL.md)
- [Third-party dependencies](THIRD_PARTY_NOTICES.md)

Author: [Mikko Kukkonen](https://github.com/kmik). No project license has been selected;
dependencies keep their own licenses.
