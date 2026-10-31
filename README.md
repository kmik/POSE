# Collinea

Collinea reconstructs point clouds from overlapping photographs. It is written in Java, with OpenCL acceleration for dense stereo matching and depth fusion, and can run on its own or as part of [LAS.utils](https://github.com/kmik/LAS.utils).

![Point cloud colored by elevation](docs/images/city-point-cloud.png)

## Features

- Feature matching, camera alignment and bundle adjustment.
- Automatic focal length and radial distortion estimation, or supplied camera calibration and poses.
- Semi-global stereo matching with Census, AD-Census, Census-gradient and ZNCC costs.
- Per-pair disparity range estimation and multi-view depth filtering.
- Reprojection-based depth refinement and normal-aware surface consolidation.
- GPS georeferencing and import/export of interior and exterior orientations.
- LAS and PLY output, including normals, roughness, planarity and point confidence.
- Self-contained processing reports with all parameter values, stage timings, resource measurements and a DSM overview.

## Build

Requirements:

- **64-bit JDK 25 or later**, with `JAVA_HOME` set or `java` and `javac` on PATH.
- Internet access for the first build, which downloads OpenCV 4.9.0-0, Proj4J 1.4.3 and TwelveMonkeys ImageIO 3.15.3 into `lib/`.
- A vendor OpenCL runtime for GPU processing. GPU depth fusion also requires double-precision support (FP64).

Windows x64 is tested locally on NVIDIA and AMD OpenCL devices. Windows and Linux x64/glibc build-and-test jobs have passed CI. Alpine/musl is unsupported. CPU reconstruction does not require OpenCL.

From the repository root:

```powershell
# Windows PowerShell
.\build.bat
.\photogrammetry.bat -h
```

```bash
# Linux
bash build.sh
bash photogrammetry.sh -h
```

The build scripts compile the program and run the test suite. Test files go under `target/`. `mvn package` compiles and copies dependencies without running this suite. No system OpenCV installation or CUDA toolkit is needed.

## Run

```powershell
.\photogrammetry.bat `
  -i .\images `
  -o .\projects\city `
  --max-dimension 1600 `
  --step 2 `
  --filtering moderate `
  --neighbors 5 `
  --min-views 3 `
  --memory-mb 4096
```

```bash
bash photogrammetry.sh \
  -i ./images \
  -o ./projects/city \
  --max-dimension 1600 \
  --step 2 \
  --filtering moderate \
  --neighbors 5 \
  --min-views 3 \
  --memory-mb 4096
```

Without a calibration file, Collinea estimates calibration from image metadata and geometry. If the images do not constrain it well enough, supply a measured calibration. Supplied calibration and poses are fixed by default. Use `--intrinsics-mode prior` or `initial`, and `--extrinsics-mode prior` or `initial`, to refine them. The [orientation documentation](ORIENTATIONS.md) covers these modes and supported file formats.

Run with no arguments or `-h` for the complete option list.

## Resolution and filtering

| Option | Default | Effect |
| --- | --- | --- |
| `--quality` | `custom` | `ultra`, `high`, `medium` and `low` divide the original width and height by 1, 2, 4 and 8. `custom` uses the longest-side limit below. |
| `--max-dimension` | `1600` | Longest-side limit in pixels for `custom` quality. If explicitly supplied, it also caps a named quality preset. Aspect ratio is preserved; images are never upsampled. |
| `--step` | `2` | Sample one dense candidate every N pixels along each axis of the processed image. Step 2 samples about one candidate per four pixels. It does not reduce the stereo matching resolution. |
| `--filtering` | `moderate` | `mild`, `moderate` or `aggressive` depth filtering. This changes acceptance thresholds, not image size. |

For a 6000 × 4000 image, the default is **1600 × 1067**. `--quality high` gives **3000 × 2000**: the implicit 1600 limit does not apply to named presets. Adding `--max-dimension 2000` caps that result at **2000 × 1333**.

Lowering `--step` produces more candidates for fusion, but the disparity map is calculated at the same resolution. Change `--quality` or `--max-dimension` to change that resolution.

## Memory and processing

Memory arguments use **MiB**: 1024 MiB is 1 GiB.

| Option | Default | Effect |
| --- | --- | --- |
| `--memory-mb` | `2048` | Processing-buffer budget, not a limit on total process RAM. An explicit value raises the command-line JVM heap if needed; a larger existing heap is retained. |
| `PHOTOGRAMMETRY_HEAP` (environment variable) | `8g` | Initial maximum Java heap used by the launch scripts. `--memory-mb` can raise it further. |
| `--compute` | `auto` | `auto` selects GPU stereo when suitable and falls back to CPU. `cpu` selects CPU stereo; `gpu` requires a compatible GPU plan. Fusion inherits this choice unless overridden below. |
| `--fusion-compute` | `inherit` | Override depth fusion separately with `cpu`, `auto` or `gpu`. Auto allows CPU fallback; GPU mode requires FP64 support. |
| `--gpu-memory-mb` | `0` (automatic) | GPU working-set budget. Automatic uses 70% of reported VRAM. Explicit values are capped at 95%; available memory and device allocation limits can reduce what fits. |
| `--gpu-sgm` | `auto` | Auto tries resident GPU volumes, exact streaming with a RAM aggregate, then compact Census with exact verification or bounded recomputation. Paths continue across strips. `resident`, `streamed` and `esgm` select a backend explicitly. |
| `--gpu-aggregate` | `auto` | Streamed aggregate storage: `ram`, `disk` or `recompute`. Auto avoids the full aggregate disk spool. Exact recomputation trades additional path sweeps for lower RAM. |
| `--esgm-candidates` | `2` | Compact proposals per path, from 1 to 4. Unresolved pixels always receive exact full-range verification. |
| `--sgm-aggregate` | `auto` | Use 16-bit aggregate storage when the configured costs and penalties fit; otherwise use 32-bit. `16` rejects an unsafe configuration, while `32` forces 32-bit storage. |
| `--point-attributes` | `surface` | Include normals and surface metrics in LAS/PLY. `none` writes compact XYZ/RGB cloud records. The confidence CSV remains available. |

Native OpenCV memory is additional to the Java heap. For example, `--memory-mb 4096` requests a 4 GiB processing budget, not a 4 GiB total RAM limit. Feature extraction and bundle adjustment run on CPU. See [performance and storage details](PERFORMANCE_AND_ATTRIBUTES.md) for the remaining CPU/GPU stages and memory estimates.

Fusion depth evidence has a separate bounded CPU page cache; see [fusion and consolidation controls](FOLLOWUP_PERFORMANCE.md).

Use `--estimate-memory` with the input directory and intended resolution settings to inspect the plan before reconstructing a project. See [large-image processing and measured memory use](SCALING.md) for backend selection, commands and limits.

### Image and feature storage

| Option | Default | Effect |
| --- | --- | --- |
| `--image-reader` | `auto` | Uses windowed decoding for sources above 64 MiB of RGB pixels and for BigTIFF. Smaller sources retain OpenCV decoding. Explicit `windowed` and `opencv` modes are available. |
| `--image-window` | `1024` | Maximum decoded window side in pixels. Codec buffers are additional. |
| `--image-memory-mb` | `0` (automatic) | Processed raster cache: automatic uses one quarter of `--memory-mb`, capped at 2048 MiB. The active processed source pair must fit. |
| `--descriptor-memory-mb` | `256` | Native descriptor cache cap, also limited to one quarter of `--memory-mb`. Evicted descriptors use run-local temporary files; active matching pairs must fit. |

Windowed TIFF/BigTIFF input supports grayscale and RGB sources, including 16-bit inputs converted to 8-bit sRGB for matching. It avoids a full decoded source image on the Java heap. Processed rasters, keypoints, detector scratch and stereo input buffers still need memory.

## Output

Each run is saved under `<output>/runs/<run-id>/`:

```text
00_inputs/       Image inventory and processing metadata
01_calibration/  Source and processed interior orientations
02_features/     Feature diagnostics
03_matches/      Pair and track diagnostics
04_geometry/     Camera poses and orientation archive
05_stereo/       Stereo ranges and depth/disparity diagnostics
06_fusion/       Fusion and consolidation reports
07_results/      LAS/PLY clouds and point-confidence.csv.gz
```

The amount of diagnostic data depends on the storage profile. `--storage standard` is the default; `--storage debug` keeps detailed intermediates. Stage caching is enabled with `--cache`. Temporary depth and surface files are removed after use. Previous runs are retained.

The default report is `07_results/processing-report.html`, with all effective processing options, supplied arguments, timings/resources and an embedded DSM preview when the coordinate frame permits it. [Report controls and regeneration](PROCESSING_REPORT.md) explain the JSON and GeoTIFF outputs.

Interior and exterior orientations are saved automatically. Additional formats can be exported during a run or afterward with `--export-orientations RUN_DIRECTORY`, without processing the photos again. See [project layout](PROJECT_FORMAT.md) and [orientation formats](ORIENTATIONS.md).

## LAS.utils

Copy this repository into `sgm_java/` at the LAS.utils root, then copy the wrappers from `las-utils/` into that root and build the module. Collinea uses its own classpath to keep OpenCV 4.9 separate from LAS.utils' older OpenCV jar. The Java entry point is `photogrammetry`.

## Limitations

Automatic calibration needs sufficient overlap and camera geometry. Estimated stereo uncertainty does not include errors in calibration, camera poses or GPS.

The default horizontal output CRS for GPS georeferencing is EPSG:3067. Heights retain EXIF GPS altitude; no geoid or N2000 conversion is applied. Negative Z values can be valid.

Windowed source loading and bounded raster/descriptor caches reduce large-image memory use. Full native-resolution aerial blocks still need further work on stereo input tiling, camera groups and detector memory. BigTIFF EXIF GPS/focal metadata is not currently extracted; supply calibration and orientations when needed. LAS output uses version 1.2, point format 2, with optional Extra Bytes and a maximum of 4,294,967,295 points per file. Reader support for extra attributes varies; the same attributes are available in the CSV sidecar.

## Documentation

- [Processing report, complete parameter values and DSM](PROCESSING_REPORT.md)
- [Fusion/consolidation throughput and progress](FOLLOWUP_PERFORMANCE.md)
- [Surface processing, performance and exported attributes](PERFORMANCE_AND_ATTRIBUTES.md)
- [Large-image processing, exact streaming and compact SGM](SCALING.md)
- [Earlier disk-free SGM research](ESGM_RESEARCH.md)
- [Orientation formats and conventions](ORIENTATIONS.md)
- [Project artifacts](PROJECT_FORMAT.md)
- [Storage and GPU processing](STORAGE_GPU.md)
- [Thin structures and SGM penalties](THIN_STRUCTURES.md)
- [Validation](VALIDATION.md)
- [Third-party dependencies](THIRD_PARTY_NOTICES.md)

Author: [Mikko Kukkonen](https://github.com/kmik). No project license has been selected; dependencies retain their own licenses.
