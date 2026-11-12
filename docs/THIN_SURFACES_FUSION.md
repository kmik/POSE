# Thin surfaces in image fusion (`--thin-surfaces`)

Status: implemented 2026-11-07, opt-in. With every option below at its default the output is
unchanged (the agreement test reduces to the previous relative rule bit for bit).

## Why

city2 (17 DJI images, 88 m above ground, GSD 8.7 cm at `--quality high`) compared with Metashape
on the same images (`example_data/forest/city/metashape.las`, 1 m cells that Metashape fits with
a plane to under 2 cm):

- Our robust core was thinner than Metashape's (1.0 vs 1.6 cm), but about 8% of points lay more
  than 10 cm off the surface. Metashape has almost none.
- The tail rate follows the stereo baseline. Pairs of neighbouring images (5–10°, 8–16 m) put
  30–67% of their points off the surface; pairs above 20° put 1–5%. Per-pair median offsets are a
  few cm, so it is matching noise (about 1 m depth per disparity pixel for a close pair), not pose
  error.
- Every fusion check used `--fusion-relative` x depth = 0.005 x 88 m = ±44 cm, including the
  "precise" band of the edge-aware vote. Noisy close-pair depths were therefore "confirmed" and
  emitted at their own depth: 18.9M agreements were not averaged, only counted as support.
- The fused sigma predicted the tail well (top quintile: 33% off the surface, best 60%: under 2%),
  but `--max-depth-uncertainty` is relative (0.1) and never fired.
- `--min-angle 12` confirmed the cause (flat RMS median 3.8 → 2.8 cm, p90 11.3 → 7.4 cm) but cost
  39% of the points and about half the coverage in vegetation and at building edges.

## What the options do

All require `--fusion-mode image`. `--thin-surfaces` sets all of them; options after it override.

| Option | `--thin-surfaces` | Effect |
|---|---|---|
| `--fusion-tolerance sigma` | sigma | Agreement within K x combined stereo sigma (inverse-variance weights from triangulation angles, 0.5 px), at least one footprint, at most the relative band. Used in the per-camera merge and in cross-checks. |
| `--fusion-sigma-k K` | 3 | Multiplier for the two sigma tests. |
| `--fusion-precision-check on` | on | Cameras with at least 2x smaller sigma that see the same surface within the relative band but not within K sigma are near-miss votes; the point is rejected when they outnumber precise confirmations. Confirmed points are averaged with the confirming depths along their ray for every `--fusion-strategy` (snap). |
| `--fusion-seed-order precision` | precision | Pass 2 runs in tiers over all cameras: merged sigma within one footprint first, then the rest, then narrow-only pixels. Precise points absorb noisier copies before those can seed. About 3x pass-2 time. |
| `--fusion-seed-min-angle DEG` | off (0) | Pixels whose merged depth comes only from candidates with triangulation angle below DEG still confirm and average, but seed a point only with two confirming cameras outside their own pairs. Coverage where no wide pair sees is kept. |
| `--fusion-local-sigma K` | 3 | Reject a point whose final sigma is above K x the best merged sigma within 4 pixels in its camera. |
| `--surface-consolidation normal`, `--surface-flatten 8` | set | Normal-aware consolidation with weak-layer removal (previously `--surface-flatten` was a no-op together with `--surface-consolidation none`). |
| `--subpixel-refinement zncc` | set | Photometric sub-pixel refinement against pixel locking (the integer bin held 2.2x the average count). |

New counters in `06_fusion/image_fusion.json` → `thin_surface_checks`: `precision_rejected`,
`narrow_unconfirmed_rejected`, `local_sigma_rejected` (all included in the fusion rejection total).

## city2 results (2026-11-07)

Dense step branched from the baseline city2 run with its dense settings
(image fusion, graph-average, edge-aware outlier filter, ground classification + ground-aware
fusion), then `ground finish`. Compared with Metashape on 1 m cells; our cloud shifted by about
(+0.5, −0.5, −2.39) m to align. "Flat" = Metashape plane fit under 2 cm, slope under 10°.

| | Metashape | before | `--min-angle 12` | `--thin-surfaces` + seed-min-angle 12 | **`--thin-surfaces`** |
|---|---|---|---|---|---|
| Flat RMS median / p90 | 1.6 / 2.0 cm | 3.8 / 11.3 | 2.8 / 7.4 | 0.78 / 3.4 | **0.81 / 3.9** |
| Flat P95 p75 / p90 | 3.6 / 4.2 cm | 15.0 / 27.8 | 8.8 / 19.3 | 2.2 / 4.5 | **2.3 / 4.8** |
| Flat share > 10 cm p75 / p90 | ~0 | 6.1 / 14.7 % | 4.6 / 10.4 % | 0.5 / 2.4 % | **0.7 / 3.0 %** |
| Flat points per m² (median) | 155 | 193 | 186 | 134 | **133** |
| Total points | 5.60M | 3.45M | 2.11M | 2.31M | **2.47M** |
| Common rough cells | – | 5,611 | 2,613 | 5,006 | **5,719** |
| Fusion stage | – | 58 s | – | 213 s | **197 s** |

- The flat-surface tail is gone: P95 now matches Metashape; RMS is below Metashape's at the median
  and about 2 cm above it at p90.
- Fewer points per m² on flats mainly come from consolidation merging duplicate copies of one
  surface (690k observations merged), not from lost coverage. Rough-area coverage is slightly
  above the old run.
- Counters (preset run): precision_rejected 93, local_sigma_rejected 6,459. The precision check
  rarely fires because sigma agreement plus precise-first seeding already pull most noisy seeds
  onto the surface (members averaged 28.8M → 33.2M) or consume them.
- Stereo time is unchanged with zncc sub-pixel refinement. Fusion is about 3.4x slower: mostly
  surface consolidation, plus the second pass-2 tier.
- The `--fusion-seed-min-angle 12` rule gained little flatness for 1.1M rejected points and about
  14% fewer rough cells, so it is not in the preset.

Remaining gap to Metashape: point density (133 vs 155 per m² on flats, 2.5M vs 5.6M in total) and
an about 14 cm spatially varying vertical difference after the global shift, which is a
georeferencing/bundle difference, not surface noise.
