# Processing report and DSM

Every normal run writes `07_results/processing-report.html` and `processing-report.json`. HTML is self-contained: figures are embedded and it needs no web service. It can be printed from a browser. `--report none` disables the HTML/DSM export; processing telemetry remains in `logs/processing.json`.

## Parameters and measurements

The report includes every public processing option with its effective value, including defaults, explicit overrides and unset values. It also preserves the supplied argument array, initial effective settings, resolved per-image source/processed camera orientations and per-pair disparity ranges. Parameters used to regenerate the report are separate from the original reconstruction parameters.

The stage table records elapsed wall times. Detailed tasks cover source decoding, feature extraction, pair matching, calibration/refinement, bundle adjustment, rectification, stereo attempts, triangulation, fusion/backend preparation, surface partition preparation and exports where those operations run. Existing fusion and consolidation profiles provide normal-estimation, CPU audit/guard, GPU kernel/transfer, neighborhood, validation and output timings. Nested and overlapping measurements must not be added together. Blocking GPU reads can include waits for queued kernels.

Resources distinguish configured budgets, maximum heap, sampled heap use, sampled process resident/private memory and tracked OpenCL buffer allocations. Sampling is approximately once per second. Process RAM includes heap and native allocations; these quantities are not additive. OpenCL buffer tracking excludes driver/display memory. Unavailable measurements remain unavailable. Wall times include suspension/scheduling gaps; the largest resource-sampling gap is recorded. CPU time is cumulative process time since launch.

## Regenerate a report

From the original repository root on Windows:

```powershell
.\sgm_java\photogrammetry.bat `
  --export-report .\sgm_java\output\forest\runs\RUN_ID `
  --report-output .\sgm_java\output\forest-report
```

From a standalone checkout on Linux:

```bash
bash photogrammetry.sh --export-report ./projects/forest/runs/RUN_ID \
  --report-output ./forest-report
```

This reads archived geometry and the final PLY; reconstruction is not repeated. By default standalone export writes under `RUN_DIRECTORY/07_results/report/`. A run still marked running is rejected. Older runs use their archived settings and stage-event timestamps when available. Missing historical settings or resource peaks are not reconstructed from current defaults or invented.

## Surface model

For an established projected coordinate frame, report generation produces:

- `dsm.tif`: north-up, Float32 surface elevations, GeoTIFF horizontal CRS tags and NaN nodata.
- `dsm.tfw`: pixel-center world-file transform.
- `dsm.png`: elevation-color overview, also embedded in HTML.
- `dsm.json`: resolution, extent, aggregation, coverage/nodata, units and datum notes.

The default uses the maximum observed final-cloud elevation in each cell. It includes roofs and vegetation; it is not a classified bare-earth terrain model. Remaining outliers can influence maxima. Empty cells stay nodata: there is no interpolation or gap filling. The source point cloud is unchanged.

| Argument | Default | Meaning |
| --- | --- | --- |
| `--report-dsm-resolution` | `0` (auto) | Cell size in final coordinate units. Auto fits the allowed grid size. |
| `--report-dsm-max-dimension` | `1024` | Maximum grid side; 16..4096. The effective cap can be lower to respect the report's RAM allowance. |
| `--report-dsm-statistic` | `max` | `max` or `mean` of observed cell elevations. |
| `--report-dsm-min-points` | `1` | Cells with fewer samples become nodata. |
| `--report-local-dsm` | off | Explicitly allow a local XY/Z raster when that projection is meaningful. It receives no invented EPSG reference. |

For example, use `--report-dsm-resolution 0.2` for 20 cm cells in a metre CRS. If an explicitly requested cell size exceeds the grid/RAM cap, the DSM is reported unavailable with a diagnostic; it is never silently coarsened. Automatic rasterization bounds grid storage and reads the PLY sequentially. Camera-center plots show archived XY coordinates, not inferred image footprints.

Heights retain the cloud's vertical reference. EXIF GPS altitude is not converted to N2000 or another geoid datum. A horizontal EPSG tag does not establish a vertical datum. Float32 raster elevations are rounded from the final double-precision cloud.

GeoTIFF tag conventions follow the [OGC GeoTIFF standard](https://docs.ogc.org/is/19-008r4/19-008r4.html). Generated samples are checked with an independent TIFF decoder.
