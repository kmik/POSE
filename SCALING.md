# Large images and GPU memory

The subsequent [fusion/consolidation update](FOLLOWUP_PERFORMANCE.md) adds exact routed partition preparation, evidence-cache controls and explicit progress. [Processing reports](PROCESSING_REPORT.md) include all effective options, resource measurements, stage times and a DSM.

The production matcher now handles stereo volumes that exceed VRAM without requiring a full aggregate file on disk. This changes storage and scheduling; image resolution, filtering thresholds, path directions and subpixel estimates are retained.

## Backend selection

With `--compute auto --gpu-sgm auto --gpu-aggregate auto`:

1. Use banked resident GPU volumes when they fit.
2. Otherwise stream strips through the GPU and keep the exact full aggregate in native RAM when the processing budget and available OS RAM permit it.
3. If that aggregate does not fit, use compact path-minimum proposals for supported grayscale Census/16-bit configurations. Certify a result only when omitted competitors cannot change its winner or acceptance decision. Recompute unresolved pixels over the full disparity interval with continuous global path state.
4. For other supported GPU configurations, use bounded exact recomputation directly. If no GPU plan fits, automatic compute may fall back to CPU; explicit GPU mode reports failure.

`--gpu-sgm streamed` requests ordinary exact streaming. `--gpu-aggregate ram` requires the full native aggregate to fit; `recompute` retains only bounded groups and repeats predecessor paths from the image boundary. `disk` explicitly retains the earlier aggregate-file approach. Independent tiles do not replace continuous paths.

`--gpu-sgm esgm` explicitly selects compact proposals with mandatory exact verification. This is an eSGM-style implementation, not the paper's optimized three-pass schedule. `--esgm-candidates 1..4` controls proposals per path (default 2); the planner can reduce the pool to fit. A smaller pool can require more verification. It never relaxes uniqueness to recover coverage. Other costs or unsafe 16-bit bounds use ordinary exact recomputation in auto mode; explicit `esgm` requires a compatible plan.

The coarse-to-fine research fork remains experimental. It is not enabled by this update: previous tests showed unreliable coarse predictions could lose structures or admit wrong depths.

## Commands

From this repository root, the usual command can use the new automatic selection:

```powershell
.\photogrammetry.bat `
  -i .\images -o .\projects\large-images `
  --quality high --step 1 --filtering moderate --min-views 3 `
  --memory-mb 16000 --gpu-memory-mb 8000
```

Use `sgm_java\photogrammetry.bat` instead when launching from the original C++ repository root. On Linux, replace the launcher with `bash photogrammetry.sh` and use shell line continuations.

For a separate compact-backend comparison, retain all geometry/resolution/filter settings and change the output directory and these controls:

```text
--compute gpu --gpu-sgm esgm --memory-mb 2048 --gpu-memory-mb 8000
```

For exact ordinary streaming, use `--gpu-sgm streamed --gpu-aggregate ram`. To bound aggregate RAM even more aggressively, use `--gpu-aggregate recompute`. Disk streaming remains available explicitly with `--gpu-aggregate disk`.

Pair reports describe aggregate storage, planned device/host bytes and recomputation bounds. Compact runs also write `esgm_left.json` and `esgm_right.json`, including certified outcomes, exact verification counts and candidate device buffers. Progress identifies completed paths/groups and left/right matching.

## What the budgets mean

`--memory-mb` is a processing-buffer allowance, not a total process RAM limit. The command-line launcher raises heap capacity when needed; the native aggregate is counted in the stereo plan. RAM selection also checks free OS memory, and an automatic full-aggregate allocation failure retries bounded exact recomputation. Explicit `ram` fails clearly instead.

Image rasters, descriptors, camera/keypoint objects, detector/codec scratch, native OpenCV allocations and concurrent driver allocations are additional. Both CPU and GPU retain processed image arrays. A single GPU input buffer must still respect the device's allocation maximum. The maximum disparity count remains subject to local memory and implementation limits. No setting makes arbitrarily large native-resolution stereo inputs fit.

The default avoids the full SGM aggregate disk spool. Run-local descriptor spill files, depth evidence, fusion/surface staging and optional diagnostics still use disk. `--storage minimal` reduces retained diagnostics independently of matching.

## Source windows and caches

`--image-reader auto` uses windows when source RGB pixels exceed 64 MiB or the file is BigTIFF. Smaller images use the original OpenCV path. `--image-reader windowed` forces the windowed path for supported ImageIO formats; `opencv` explicitly selects the old reader. Source dimensions are read before decoding pixels.

`--image-window 1024` bounds the returned decoder windows. Resampling integrates global pixel areas, including fractional boundaries, so window boundaries do not restart the resize grid. Images are never upsampled. The processed color/gray/mask rasters remain full-sized native matrices.

Classic TIFF windows use the JDK TIFF codec. BigTIFF metadata uses TwelveMonkeys; a bounded virtual classic-TIFF view maps only the requested compressed strips/tiles to their original 64-bit file offsets, then the JDK codec decodes those windows. It does not copy the complete source file or allocate a full source raster. Support depends on compression and photometric codecs. Old-style JPEG compression 6, incomplete/sparse zero-length blocks, and a single requested compressed window exceeding the virtual 32-bit address space are rejected explicitly.

Grayscale/PAN and RGB 8/16-bit sources are converted through the image color model to **8-bit sRGB**. Native 16-bit matching, arbitrary multispectral bands and proprietary sensor SDKs are not implemented. BigTIFF EXIF GPS/focal extraction is not implemented by the existing classic-TIFF EXIF parser; supplied calibration/poses remain appropriate for aerial data.

`--image-memory-mb` bounds cached processed rasters (color, gray, mask), defaulting to one quarter of the processing allowance, capped at 2048 MiB. The active pair must fit; eviction preserves dimensions, keypoints and feature colors. `--descriptor-memory-mb 256` bounds cached native descriptor matrices, also capped at one quarter of the processing allowance. Evicted descriptors reload bit-exactly from run-local files, removed on normal cleanup. Both active descriptor sets must fit together. Persistent `--cache` is independent.

For first aerial trials, cap feature extraction with `--alignment-max-dimension 3000` or `4000`, inspect `--estimate-memory`, and start with a reduced dense resolution. SIFT scratch and keypoint metadata are not bounded by the descriptor cache. Windowed *source* reads do not yet provide fully windowed stereo, sparse alignment, bundle adjustment or point-cloud export across arbitrary large flight blocks. Camera groups/rig constraints and surveyed aerial validation remain future work.

## Measured comparison

Windows, NVIDIA GeForce RTX 5070 Laptop GPU; a recreated real forest pair, 1978 x 2640 pixels, minimum disparity 131, 1037 hypotheses, default eight-path Census P1/P2=4/24 and full stereo post-filtering. Both directions were computed. These are stereo-stage measurements, not whole-project timings.

| Measurement | Exact RAM streaming | Compact + sparse exact verification |
| --- | --- | --- |
| Processing allowance | 16000 MiB | 2048 MiB |
| Stereo + filtering time | 54.93 s | 104.22 s |
| Peak process working set | 10.85 GiB | 1.94 GiB |
| Sampled peak private bytes | 11.99 GiB | 2.85 GiB |
| Explicit GPU buffers | 618.9 MiB | 1015.9 MiB candidate phase; 618.9 MiB verification strips |
| Full aggregate disk spool | 0 | 0 |
| Accepted depth pixels | 128,905 | 128,905 |

Complete filtered disparity arrays and all rejection counters were identical, including subpixel bits. SHA-256 of canonical float bits: `af9cbbdd4d5f2d301d3e8d2d98d3f3825ddf44692a9a75f1d8f93b5df80ed74c`. The compact approach required exact verification for about 70% of pixels here, explaining the extra work. Measurements used the same 24 GiB heap ceiling to compare native storage; this is not a minimum heap requirement. Working set and private bytes measure different resources and include more than the planned aggregate. Sampling was every 100 ms.

RAM/disk/recompute parity passed 258 checks on each NVIDIA and AMD device; compact production parity passed 36 checks on each, covering signed ranges, repeated textures, masks, four/eight paths and candidate pools 1/2/4. Image/cache regressions passed 37 checks, including 16-bit conversion, eviction/reload, fractional resampling and BigTIFF offsets beyond 4 GiB. Standalone 6000 x 4000 tiled TIFF and BigTIFF decoding passed at native size and 1600 x 1067 with a 64 MiB Java heap. Native processed matrices and codec allocations are outside that heap limit.

Machine-readable evidence: [results-2026-10-31.json](research/scaling/results-2026-10-31.json). These tests establish scheduling/output parity and bounded source decoding on the fixtures. They do not establish Vexcel sensor compatibility, field accuracy or improvement in surface noise. Full native-scale Vexcel reconstruction has not been tested.

## References

- [Hirschmuller, Buder and Ernst, Memory Efficient Semi-Global Matching (2012)](https://isprs-annals.copernicus.org/articles/I-3/371/2012/isprsannals-I-3-371-2012.pdf).
- [Java ImageReadParam source-region API](https://docs.oracle.com/en/java/javase/25/docs/api/java.desktop/javax/imageio/ImageReadParam.html).
- [TwelveMonkeys ImageIO](https://github.com/haraldk/TwelveMonkeys), pinned to [3.15.3](https://github.com/haraldk/TwelveMonkeys/releases/tag/twelvemonkeys-3.15.3).
