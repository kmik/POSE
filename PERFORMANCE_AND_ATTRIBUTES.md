# Surface processing, SGM storage and point attributes

For current aggregate storage, source-window loading and bounded image/descriptor caches, see [SCALING.md](SCALING.md). The original disk-streaming and full-source-residency statements below describe the earlier implementation; surface processing and attribute definitions remain applicable.

This update preserves the existing geometric acceptance rules. It changes how work is scheduled and stored, and makes the existing local surface estimates available in point-cloud exports.

## Consolidation

`--surface-validation auto` reuses the resident FP64 GPU fusion evidence for batched checks of proposed positions and merges. Depth refinement is **disabled** during these checks: a proposed surface point is checked where it is, rather than moved back to a depth estimate. Threshold guards replay ambiguous GPU decisions on the CPU. Sparse CPU audits compare status, distinct-camera support, pair support and coordinates before results are used. Surface checks audit up to eight evenly distributed points per call (approximately one per 512 points, at least one for a nonempty short call).

An additional bounded GPU kernel evaluates oriented-normal compatibility, normal offsets/tangent distances and the producing cameras' displacement predicates. An exact inexpensive CPU broad phase reduces transfer volume. The first eight neighborhood batches in auto mode compare observed GPU query time against sampled CPU query time; CPU neighborhoods are selected if the GPU path costs more. This affects execution only. GPU depth revalidation can continue after neighborhood processing switches to CPU. `--surface-validation cpu` forces both reference paths; `gpu` requires prepared GPU fusion evidence and keeps GPU neighborhood processing enabled. Use `--fusion-compute gpu` when explicitly requiring it. CPU guards remain enabled in GPU mode.

Weighted medians, resolved-layer separation, greedy representative selection and output remain ordered CPU work. Proposals use original positions and normals. Later points cannot merge into already emitted points, opposite-facing layers remain separate, and grouping does not invent supporting cameras. Local geometric tests continue to protect thin structures; this is not a terrain-flattening operation.

Partition membership and representative priority are retained. The halo now uses the largest querying footprint **within its owner partition**, rather than the global maximum, with the conservative cylinder bound `footprint * hypot(surfaceRadius * step, surfaceMaxShift)` and an outward numerical margin. Unreliable-normal samples do not enlarge this query bound. A bounded cache holds decoded immutable partition bytes, and grid membership uses primitive index arrays. Depth page caches and OpenCL argument/queue ownership remain single-threaded during consolidation.

Smaller halos can prevent an earlier memory-overflow passthrough. In that case the new run can consolidate points that the old run left untouched; `budget_passthrough_samples` records this behavior. Partition-boundary duplicates and extremely crowded/scale-degenerate passthrough remain possible. RAM settings must be held fixed when comparing runs.

`06_fusion/surface_consolidation.json` reports partitioning, loading, grid construction, neighborhood queries, revalidation and output times; decoded bytes/cache hits; retained halo records; validation counts; GPU kernel/transfers and guards; backend trials; and temporary-file peaks. Query counts include bounded speculative work that greedy selection may later skip. The existing total `seconds` includes initial accepted-sample staging; individual stage timings isolate consolidation work. GPU utilization alone does not measure end-to-end benefit.

## SGM aggregates and streaming

`--sgm-aggregate auto|16|32` defaults to `auto`. The classic normalized recurrence has the bound `L <= C + P2`. All supported costs are at most 120 for valid hypotheses or **512 for invalid cost**, so the aggregate is bounded by `directions * (512 + effectiveP2)`. Edge adaptation never increases effective P2. Auto uses unsigned 16-bit storage when this bound is at most 65535 and otherwise uses signed 32-bit storage. Forced 16-bit mode rejects an unsafe configuration; forced 32-bit mode provides a reference for comparisons. Default 5x5, eight-path P1/P2=4/24 gives a bound of 4288.

Path/carry arithmetic stays 32-bit. CPU resident and paged aggregates, GPU banks, host transfers and streamed disk pages all use the selected storage width. Raw costs retain their original unsigned 16-bit representation and invalid value 512. No value is clipped, rescaled or quantized to fit a smaller aggregate. Planners and per-pair reports use the selected width; old low-level estimate overloads without Options conservatively assume 32-bit.

The streamed GPU selects a strip's disparities immediately after its last path contribution. Future strips need the continuous **path carry**, not that strip's aggregate. Consequently eight-path matching needs eight cost-generation passes instead of nine, and seven aggregate-volume writes/reads instead of eight each. All paths, signed ranges, masks, confidence calculations and filters are retained. At eligible settings the aggregate spool is also half the old size. These are traffic/storage reductions, not a promised runtime multiplier. Input images/descriptors still reside in full; this is not native-scale aerial windowed processing or per-pixel disparity-band storage.

## Exported attributes

`--point-attributes surface` is the default. It appends 32 bytes to each LAS/PLY point. `none` retains compact XYZ/RGB cloud records; if consolidation is also disabled, it avoids normal estimation. Existing public CloudWriter constructors keep their compact layout.

PLY uses conventional `nx`, `ny`, `nz` float properties. LAS uses scalar Extra Bytes descriptors in `LASF_Spec`, record 4, with `normal_x`, `normal_y`, `normal_z`. This is an extensible LAS 1.2 format-2 record; base XYZ/RGB remain unchanged, and header point-record lengths/VLR counts include the extensions. Readers must honor the declared record length. Extra Bytes display support varies by reader; the compressed CSV remains available to LAS.utils or other consumers that do not expose those fields.

The attributes describe the robust **source-depth neighborhood**, before spatial consolidation. They are carried with the final representative; they are not recalculated from the resulting cloud. Normals are unit vectors in the cloud's world axes and point toward the producing pair's first camera, so signs are not a globally outward-facing orientation. Translation and positive uniform output scaling do not change normals. Surface metrics use the retained neighborhood covariance, with ascending eigenvalues `lambda0 <= lambda1 <= lambda2`:

| Attribute | Definition |
| --- | --- |
| `roughness_rms` | `sqrt(lambda0)`, perpendicular plane-fit RMS in output coordinate units |
| `surface_variation` | `lambda0 / (lambda0 + lambda1 + lambda2)`; dimensionless |
| `planarity` | `(lambda1 - lambda0) / lambda2`; dimensionless; distinguishes planar from line-like neighborhoods |
| `normal_radius` | Maximum retained neighbor distance from the candidate, in output units |
| `normal_neighbors` | Number of retained depth samples, not number of cameras |
| `surface_status` | 0: valid metrics/normal; 1: unreliable/unavailable; 2: normal only, metrics unavailable |
| `normal_trimmed` | 1 if robust trimming removed neighborhood samples |

Normal and metric floats are NaN when unavailable. LAS float no-data descriptors use the required double-width NaN representation. The CSV stores the same attributes at greater numerical precision and keeps its original point-ID/XYZ/support columns, with new fields appended. `normal_trimmed` is a boolean string in CSV and an unsigned byte in binary clouds. Neighborhood counts/radii and robust trimming must accompany interpretation of “flatness.”

These attributes reuse existing covariance fits. They do **not** introduce a new default rejection threshold: a fence/corner/vegetation sample can have low planarity, and a wrong surface can have high planarity. Noise evaluation should combine these diagnostics with distinct-camera evidence, reprojection/depth agreement and uncertainty, and compare thickness **and** coverage. No normals or local-plane fits establish absolute survey accuracy.

Temporary surface records use 109 bytes when metrics are unavailable and 146 bytes for valid metrics, versus 108 before this update. The planner uses a conservative 148-byte upper bound; actual decoded/input sizes are reported. It remains gzip-compressed and is removed on close. LAS+PLY together add 64 uncompressed output bytes per retained point, plus 32 temporary cloud-spool bytes; use `--point-attributes none` when that output cost is unwanted. This does not change stereo resolution or filtering strength.

Extra Bytes layout follows the [ASPRS specification](https://www.asprs.org/wp-content/uploads/2019/07/LAS_1_4_r15.pdf). See [VALIDATION.md](VALIDATION.md) for measured checks and limitations.
