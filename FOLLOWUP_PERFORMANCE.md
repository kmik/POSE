# Fusion, consolidation and progress update

This update targets the slow fusion/consolidation stages observed in the forest run. It retains geometric filtering thresholds, distinct-camera requirements, refinement rules and ordered surface representatives. It does not claim a reduction in surface noise or surveyed accuracy.

## Fusion evidence access

CPU audits, numerical guards and source-depth normal estimation still run. Their depth/offset evidence now uses interleaved exact 32-bit channels and smaller pages, reducing unrelated data read for sparse lookups. The shared cache remains bounded, with up to 128 open evidence files. GPU evidence remains resident when it fits.

| Argument | Default | Meaning |
| --- | --- | --- |
| `--fusion-page-side` | `64` | Power of two from 16 to 512; page storage contains side-squared 32-bit cells with interleaved depth/offset values. |
| `--fusion-cache-mb` | `0` (auto) | Auto uses `min(512, memory-mb/8)` MiB, at least 1 MiB. This CPU evidence cache is additional to GPU buffers and other native allocations. |
| `--fusion-batch-size` | `16384` | Pipeline/GPU batch ceiling, 32..65536. Device batches shrink when memory/allocation limits require it. |
| `--surface-batch-size` | `64` | Surface proposal look-ahead ceiling, 1..1024, further bounded by surface memory. CPU representative ordering is retained. Larger values are experimental tuning, not a guaranteed speedup. |

Fusion CPU audit density is retained at up to eight samples per 4096-candidate block, including partial blocks. Surface validation retains its existing per-call sparse audit rules. No ambiguity guard or acceptance predicate is disabled to improve throughput.

## Exact spatial partition routing

`--surface-partition routed` is the default; `recursive` retains the previous reference strategy. Both use the same midpoint splits, longest-axis tie handling, maximum depth, leaf ordering and stable record order.

Routed preparation records a 28-byte coordinate/routing entry per accepted sample. It plans the exact split tree by scanning that compact file, then routes each full record to its final partition once. A bounded eight-writer cache reuses gzip encoders and buffers; concatenated gzip members remain readable by the standard decoder. It avoids decompressing/recompressing full surface records at every tree level.

This trades additional coordinate scratch and sometimes larger partition files for less repeated compression. Peak temporary disk space can increase: the report records coordinate scratch and peak partition bytes. Metadata planning is bounded; excessive planning metadata falls back to the recursive reference. Neither path removes earlier runs or input photographs.

## Terminal progress

Stage transitions and tasks announce start, periodic activity (roughly five seconds) and completion. Fusion prints final candidate counts before surface preparation. Spool finalization and spatial partitioning are announced before recursive work; partition preparation reports record visits/pass activity instead of a misleading fixed percentage. Consolidation reports fixed partition progress and the current subtask.

Heartbeats also cover long native source/feature/matching/calibration/bundle operations, dense pair work and cloud exports. Updates use ordinary lines suitable for redirected logs. Unknown totals have activity counts and elapsed time rather than invented percentages/ETA. Scheduling uses fixed delays so machine resume does not flood the terminal with missed timer ticks.

## Similarity support

`--similarity-window` is a clearer alias for `--ws_census`. Both retain the default 5x5 support and accept odd sizes 3..11. This controls the Census descriptor and dense ZNCC cost support. `--correlation-window` separately controls photometric validation/refinement; `--ws_mFilter` controls disparity median filtering. Windows are in processed-image pixels. Penalties retain their existing area scaling from 5x5 reference units.

## Conditional regional-disparity experiment

The test-only `GuardedRegionalInvestigation` requires at least 95% coarse prediction coverage, broad spatial occupancy and low disparity spread before restricting a region. It retains full-range fallback for uncertain regions. Production dispatch and pyramid defaults are unchanged.

The experiment failed its promotion gate on 512 x 256 / 128-hypothesis controls. A plane took 32.15 s versus 1.95 s for ordinary matching; fences/layers took 29.98 s versus 0.93 s and increased wrong-depth acceptance from 321 to 335 eligible pixels. Repeated texture also admitted additional wrong depths. Small cropped regions repeated substantial path/halo work. These are synthetic controls, not an aerial speed benchmark.

The regional fork is not enabled in normal processing. Packed per-pixel bands are deferred until a regional/search design demonstrates worthwhile savings without unacceptable depth or coverage changes. See the [earlier investigation](ESGM_RESEARCH.md).

## Validation and limits

Machine-readable measurements and comparison notes are in `research/followup/results-2026-10-31.json`. The report implementation is documented in [PROCESSING_REPORT.md](PROCESSING_REPORT.md). Existing source images and completed runs are preserved; tests use isolated builds and output directories.

Results distinguish synthetic known-plane fusion/consolidation controls from a read-only sample of a completed real forest cloud. That sample exercises partition routing and ordered passthrough; it is not the original pre-consolidation evidence and does not benchmark complete real multi-view fusion. Phase timings and full-stage timings must be considered separately. Larger batching and a proposed projection rewrite were not promoted after inconsistent or slower large-workload results.

## Measured observations

| Control | Reference | Updated | Result |
| --- | ---: | ---: | --- |
| Synthetic fusion, 131,072 candidates | 2.054 s | 0.365 s | Same 130,168 decisions and quantized geometry hash; includes CPU audits, excludes backend preparation. |
| Synthetic consolidation, 6,503,432 samples | 223.181 s | 206.507 s | Same ordered geometry, support and merge membership; 893,665 representatives. |
| Real-cloud partition-only replay, 497,034 samples | 3.887 s | 1.907 s | Identical output; intentionally excludes neighborhood/refinement. |
| Three-image fixed-camera city reconstruction | 35,089 points | 35,089 points | Final PLY is byte-for-byte identical. |

These are local observations, not whole-project speedup guarantees. Consolidation trials varied; larger batching and a projection rewrite were rejected. The final long benchmark had no material sampling gaps. A separate run interrupted by a 976-second scheduling/suspension gap is retained in the evidence but excluded from performance comparisons. Synthetic consolidation peak partition scratch increased from 197.7 to 404.9 MiB, including the routed coordinate file; select `--surface-partition recursive` when that tradeoff is unsuitable.
