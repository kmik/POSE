# Layered-ground investigation

The ground-thickness report was reproduced against the existing DJI project, `output/example-test_dji5x/`, using the seven photographs in `example_data/5x_test/`. That original output remains untouched. New results are in `output/layer-fusion-dji5x/`, with a separate controlled comparison in `output/layer-fusion-dji5x-fixed-geometry/`.

## Why layers survived

The earlier fusion filter allowed a fixed 2% relative depth difference and 1.5 pixels of reprojection error. At 50 metres camera distance, 2% is a one-metre interval. Its support count also credited both endpoints of another stereo pair if either endpoint agreed, including an endpoint with no valid measurement. A few agreeing pair estimates could survive despite a competing depth mode. Accepted per-pair estimates were then exported at their original positions, retaining separate bands within that tolerance.

These are dense-fusion issues. Calibration/pose bias and insufficient image resolution can also thicken a cloud, so the comparison below separates a fusion-only change from the latest complete pipeline.

## Changes

- Default relative depth agreement is now 0.5%, with 0.75-pixel back-projection tolerance.
- A candidate must agree with a dominant per-ray depth cluster in both producing cameras. The cluster needs at least 60% of available pair measurements in that ray. There is no global elevation clipping, ground flattening, or assumption that only one surface can occupy an XY location.
- Independent support credits only the camera whose own measurement passed. The producing pair cannot create a third view.
- Evidence retains packed subpixel X/Y offsets alongside depth. Back-projecting a rounded pixel center previously added artificial geometric disagreement.
- Consistent independent-pair estimates refine the candidate's depth along its reference-camera ray. Approximate inverse depth-variance weights favor better parallax, with capped weights and a weighted-median neighborhood to exclude other depth modes. Refined points must pass source-image and independent-view checks again.
- Estimated relative stereo uncertainty is limited by `--max-depth-uncertainty` (default 0.1). The heuristic assumes 0.5-pixel disparity uncertainty and limits the benefit attributed to correlated pairs. It excludes calibration, pose and GPS error; it is not an absolute accuracy estimate.

`--no-depth-refinement` disables the coordinate adjustment while retaining the new acceptance checks. `--voxel` remains optional density control; this change does not collapse all samples into a single terrain model. Stricter checks leave holes when evidence is ambiguous, including at occlusion boundaries and on small vegetation.

## Controlled regression cases

`LayeringRegression` includes the former filter as a test-only ablation (`LegacyLayerFusion`). It demonstrates:

1. A false 20.25-unit layer next to a true 20-unit surface passes the old 2% rule but is rejected by the new consensus rule; the true surface remains.
2. Ten mildly noisy measurements of one plane all survive. RMS depth error decreases from 0.0200 to 0.00229 source units after multi-pair refinement.
3. Two genuine surfaces at depths 16 and 20, visible along different rays, both remain at their original depths.
4. An off-image camera cannot count as a supporting view just because its stereo partner agreed. The legacy filter falsely accepted that case.
5. Numerically agreeing measurements with negligible parallax still fail the uncertainty limit.
6. Packed subpixel evidence preserves the sampled image position within 0.000016 pixels.

The existing adverse-texture, independent-view, real-plane retention, calibrated-cloud, binary export and project-provenance tests are also retained. Project tests verify that accepted LAS coordinates correspond to `refinements.csv.gz`, while raw candidate coordinates stay unchanged.

## DJI comparison

| Measurement | Existing DJI run | Fusion-only, fixed geometry | Latest complete pipeline |
|---|---:|---:|---:|
| Sampled stereo candidates | 168,316 | 168,316 | 302,917 |
| Exported points | 132,402 | 39,896 | 115,035 |
| Per-ray consensus rejects | not implemented | 94,176 | 156,058 |
| Points with refined depth | not implemented | 39,896 | 115,035 |

The fusion-only run imports the exact previous calibration and world-to-camera poses and uses the same fixed 128 disparities, maximum dimension 800 and step 3. Identical raw candidate counts isolate the changed acceptance/refinement behavior. The latest complete run also uses the previously added shared radial calibration and automatic per-pair disparity ranges, so its counts are not a pure fusion ablation.

To quantify thickness without claiming surveyed ground truth, `LayerCloudMetrics` selects bright/neutral-coloured points, groups them into one-metre XY cells and fits a local plane within each cloud's cell. Thickness is the orthogonal residual 90th-to-10th percentile spread. Only cells with at least eight selected points in **both** compared clouds are used.

- **Fixed geometry:** 368 common cells; median thickness 0.803 to 0.199 metres, approximately 75% lower. The 95th percentile decreases from 1.364 to 0.554 metres. Overall point retention is 30.1%, demonstrating the coverage cost of rejecting weak estimates without improving the input geometry/search.
- **Latest complete pipeline:** 221 common cells; median thickness 0.698 to 0.152 metres, and 95th percentile 1.271 to 0.330 metres. Overall point retention is 86.9%. The calibration and footprint changed, and the common-cell set differs, so these numbers must not be compared directly with the preceding bullet as if they measured the same patches.

Colour selection is a material/road proxy, not ground classification. Local planarity is an analysis assumption only; no such plane is imposed on the exported cloud. These measures assess local consistency, not survey accuracy or the correctness of all retained vegetation/terrain.

Per-cell measurements and summaries are saved under `target/layer-comparison-fixed/` and `target/layer-comparison-full/`. `CloudDiagnostics` projections/statistics are under `target/layer-debug-before/` and `target/layer-debug-after/`. Every candidate's raw and refined coordinates and rejection reason are retained in its project run under `06_fusion/`.

## Reproduce

From the original repository root:

```powershell
sgm_java\photogrammetry.bat -i example_data/5x_test -o sgm_java/output/layer-fusion-dji5x --max-dimension 800 --step 3 --cores 2 --bundle-iterations 10

sgm_java\photogrammetry.bat -i example_data/5x_test -o sgm_java/output/layer-fusion-dji5x-fixed-geometry --calibration sgm_java/output/example-test_dji5x/camera.txt --poses sgm_java/output/example-test_dji5x/cameras.csv --max-dimension 800 --disparities 128 --step 3 --cores 2 --no-gps
```

The controlled import retains the old numerical world coordinates and units, but does not assert a CRS because supplied poses alone do not assign EPSG metadata. The latest complete run fits image GPS and assigns the selected horizontal CRS, default EPSG:3067. The old and new projects preserve their own immutable run histories.
