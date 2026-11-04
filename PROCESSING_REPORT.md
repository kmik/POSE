# PDF processing report and maps

Every normal run writes **`07_results/processing-report.pdf`**, with self-contained HTML and JSON companions. PDF generation uses Apache PDFBox 3.0.8 directly; fonts and figures are embedded and no browser or external printing utility is required. Build scripts/Maven supply the dependencies on Windows and Linux. Use `--report pdf` (default), `--report html` to omit the PDF, or `--report none` to disable automatic report/DSM export. Processing telemetry remains in `logs/processing.json`.

The cover shows a genuine report-resolution orthomosaic when calibrated source photographs and a usable surface are available. Source files must match their archived SHA-256 identities. Brown projection and DSM visibility checks select photographed colors; holes remain transparent. This is a nearest-camera overview, without seam blending, inferred terrain or photogrammetric repair. A poor reconstruction can produce a poor overview. Missing inputs are labeled unavailable; a DSM or point-cloud rendering is never substituted under the orthomosaic label.

Camera maps show registered centers/IDs, XY viewing directions and approximate image footprints on the median DSM-height plane. Unique EXIF capture times establish acquisition-order lines: supplied UTC offsets are normalized, and otherwise a common camera clock is explicitly assumed. Mixed local/UTC timestamps or different uncorrected camera models do not establish a flight order. Missing/ambiguous times do not get filename-based lines. Failed images without a pose are counted, rather than plotted at invented positions. North is shown only for an established georeferenced frame; local maps use +Y.

The overlap map counts calibrated camera-frustum coverage at observed DSM cells. It is **not reconstructed depth support or point confidence**, and source-file availability does not alter the geometric count. Unobserved terrain remains unknown. Orthomosaic/coverage grids are bounded to 1024 pixels per side; source thumbnails to 1600 pixels, using decoder subsampling. Larger DSMs are sampled at calibrated world pixel centers for this overview. The full DSM remains a separate output.

## Parameters and measurements

The report includes every public processing option with its effective value, including defaults, explicit overrides and unset values. It also preserves the supplied argument array, initial effective settings, resolved per-image source/processed camera orientations and per-pair disparity ranges. Parameters used to regenerate the report are separate from the original reconstruction parameters.

The stage table records elapsed wall times. Detailed tasks cover source decoding, feature extraction, pair matching, calibration/refinement, bundle adjustment, rectification, stereo attempts, triangulation, fusion/backend preparation, surface partition preparation and exports where those operations run. Existing fusion and consolidation profiles provide normal-estimation, CPU audit/guard, GPU kernel/transfer, neighborhood, validation and output timings. Nested and overlapping measurements must not be added together. Blocking GPU reads can include waits for queued kernels.

Resources distinguish configured budgets, maximum heap, sampled heap use, sampled process resident/private memory and tracked OpenCL buffer allocations. Sampling is approximately once per second; timelines retain at most 4096 decimated samples while scalar peaks retain full-rate measurements. CPU utilization is process CPU time per interval, normalized to all logical processors. CPU time is cumulative since launch. Process RAM includes heap and native allocations; these quantities are not additive. Exclusive native-memory use is unavailable. OpenCL buffer tracking excludes driver/display memory.

Actual program kernel durations, blocking transfer times and transferred bytes are recorded for SGM, PatchMatch, compact matching, fusion and surface checks when they execute. Optional NVML supplies selected NVIDIA device-wide utilization and VRAM, including other programs. Unsupported devices/runtimes and absent historical measurements remain unavailable; kernel time is not relabeled as utilization. GPU occupancy is not currently measured. Wall times include suspension/scheduling gaps; the largest resource-sampling gap is recorded. Nested/overlapping tasks and device operations must not be summed into total elapsed time.

The PDF labels its measurement cutoff. During normal processing, its snapshot includes map generation and precedes document formatting; final logs also record the report-generation task. Regeneration uses original processing measurements, with its own settings separate. Report failure is recorded in `07_results/report-status.json` and does not turn a successful reconstruction into a failed run.

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

This reads archived geometry and the final PLY; reconstruction is not repeated. Orthorectification additionally needs the original or archived photographs matching the input inventory. By default standalone export writes under `RUN_DIRECTORY/07_results/report/`; the human-readable deliverable is `processing-report.pdf`. A run still marked running is rejected. Completed and failed archived runs can be reported with explicit missing-product diagnostics. Older runs use their archived settings and stage-event timestamps when available. Missing historical settings or resource peaks are not reconstructed from current defaults or invented.

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

For example, use `--report-dsm-resolution 0.2` for 20 cm cells in a metre CRS. If an explicitly requested cell size exceeds the grid/RAM cap, the DSM is reported unavailable with a diagnostic; it is never silently coarsened. Automatic rasterization bounds grid storage and reads the PLY sequentially. Camera centers use archived XY coordinates; dashed footprints are explicitly approximate median-height intersections, not surveyed terrain boundaries.

Heights retain the cloud's vertical reference. EXIF GPS altitude is not converted to N2000 or another geoid datum. A horizontal EPSG tag does not establish a vertical datum. Float32 raster elevations are rounded from the final double-precision cloud.

GeoTIFF tag conventions follow the [OGC GeoTIFF standard](https://docs.ogc.org/is/19-008r4/19-008r4.html). Generated samples are checked with an independent TIFF decoder.
