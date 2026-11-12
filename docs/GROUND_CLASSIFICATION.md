# Ground classification (`--classify-ground`, C1)

Opt-in. Runs after dense fusion on the published cloud; needs georeferenced (z-up) output,
otherwise `07_results/ground.json` says why it was skipped. No reference terrain is used.

## Method (`GroundClassification.java`)

1. **Inputs:** `dense.las` plus, when aligned with it, `07_results/point-confidence.bin`
   (supporting cameras, views, normal and source pair of every point).
2. **Grid:** cell = 3 × point spacing, clamped to 0.25–2 m (`--ground-cell`).
3. **Candidates:** the second-lowest point per cell (lowest if alone) among points with two or
   more views and a normal more than 30° from horizontal. Candidates more than 1 m + slope ×
   2 cells below their 5 × 5 median are low outliers.
4. **Progressive morphological filter (after SMRF):** openings with windows up to
   `--ground-max-object` (30 m); a cell is an object when an opening step lowers it by more than
   tan(`--ground-max-slope`, 35°) × window radius.
5. **Slope envelope:** a ground cell may not stand more than slope × distance + tolerance above
   any other ground cell (two-pass chamfer). This removes crown sides at the cloud edge and
   canopy patches beside real ground.
5b. **Canopy islands** (revised 4 Nov 2026). Under a closed stand the lowest points of a region
   can be crown tops; the morphological filter and the slope envelope then accept them, since
   nothing lower is near. The ground cells are grouped into connected patches.
   - *Trusted ground:* patches of at least 25 m² that are smooth within 4 m tiles (median tile
     plane RMS ≤ max(0.25 m, tolerance); a single plane over a large patch would reject real,
     curving terrain), plus patches within 30 m of them that lie at their coarse (5 m) terrain
     trend (within max(1 m, 2 × tolerance)).
   - *Every other patch* is rejected as canopy when its median height above the terrain
     interpolated from the trusted ground exceeds max(1 m, 2 × tolerance) or 0.25 m per metre of
     distance to the trusted ground (about 14° average slope), whichever is larger. Ground seen
     through a gap lies at the terrain level; a crown stands metres above it, however deep it is
     below the surrounding canopy top.
   - Rejected cells become interpolated terrain. `ground.json` → `canopy_islands`: patches
     tested and rejected, their area and median rise. Without any trusted ground nothing is
     rejected. `-Dsgm.islandsDebug=FILE` writes one CSV row per patch for diagnosis.

   The first version (same day) kept patches lying more than a "gap depth" (3–8 m) below the
   surrounding canopy. In tall forest the lower crowns are deeper than that, so on forest_2
   blocks of crowns 8–12 m above the terrain stayed ground and raised the DTM there. That rule
   and `--ground-gap-depth` were removed.
6. **Free space:** every camera ray reached its point, so terrain cannot stand above it. Each
   cell records the heights of the rays crossing it (one entry per ray, stopping 3 cells above
   the point); the second-lowest, which needs two independent rays, caps the terrain. Candidates
   above the cap are rejected, and the interpolated terrain is clamped to it. Points far below a
   preliminary terrain (4σ, at least 1 m) are low noise and do not carve.
7. **Self-tuning:** σ = 1.4826 · MAD of points within 1 m of the terrain over ground cells;
   ground tolerance = max(0.1 m, 3σ) + slope × cell.
8. **Classes:** written in place into `dense.las` (classification byte): 2 ground, 7 low noise,
   1 other. `dense.ply` is unchanged.

## Checks without a reference

| Check | Meaning |
| --- | --- |
| Split-half | Terrain from even vs odd stereo pairs; median and p95 \|ΔZ\| where both observe ground; `ground_uncertainty.tif` = half the difference |
| Blocked rays | Camera rays the final terrain still blocks; cells with two or more are coded 3 in `ground_support.tif` |
| Support | `ground_support.tif`: 1 observed ground, 2 interpolated, 3 blocks rays; distance to the nearest observed ground |
| Low noise | Points far below the terrain (class 7) |
| DJI takeoff | EXIF altitude minus XMP relative altitude, reported for orientation only (the takeoff spot may lie outside the area) |

Outputs in `07_results`: `dtm.tif`, `chm.tif` (highest point minus terrain),
`ground_support.tif`, `ground_uncertainty.tif` (Float32 GeoTIFF + .tfw), `dtm.png`,
`ground_support.png`, `ground.json`. The PDF summary gets a Ground line, plus quality flags
when observed ground is below 10% of cells, split-half p95 exceeds 1 m, or more than 5% of rays
are blocked.

## Results

- **forest_10** (run 2af2c8b5's 33.0 M-point cloud, classified on a copy, 34 s): 30.4% ground,
  ground observed in 35.6% of 0.25 m cells, split-half median 0.038 m (p95 0.37 m), 0.00% rays
  blocked, σ-based tolerance 0.47 m, 1,324 low-noise points. Profiles: open ground classified,
  crowns and crown edges excluded. Without the slope envelope, the crown side at the cloud edge
  and canopy patches in a large closed stand were taken as ground (split-half p95 0.46 m,
  17,597 low-noise points). The 0.5–1 m second ground sheet from fusion layering stays
  class 1.
- **forest_tiny** end to end (`--quality low`): 71.1% ground, observed in 47% of 1.6 m cells,
  split-half median 0.13 m.
- **Synthetic** (`GroundClassificationRegression`, SelfTest): sloped bumpy terrain, 120 conical
  trees, a 45 m closed block with 3 m crown depressions and a 40 m tall stand (canopy 24–27 m) with
  lower crowns 10–12 m up seen through gaps: recall > 95%, precision > 98%, DTM RMSE < 0.25 m where ground is
  visible and < 1.5 m interpolated under both stands, no ground inside either stand. Unit checks cover the ray bound (one
  ray alone does not set it) and the slope envelope.

## Limits and next steps

- Under closed canopy the DTM is interpolated (code 2, distance in `ground.json`). The
  interpolation is a push-pull pyramid with bilinear upsampling plus relaxation; until 4 Nov it
  copied coarse cells into blocks, which left flat squares up to 16 m wide where little ground is
  seen (forest_11, 7% observed).
- Closed stands larger than `--ground-max-object` used to leave crown tops classified as ground.
  With the revised canopy island filter (5b), on the same clouds: forest_2 16.0% → 14.0% ground (crown blocks
  8–12 m above the terrain gone), forest_8 14.0% → 11.0%, forest_11 5.4% → 3.7% (before: no
  filter for forest_8 and forest_11, the first filter version for forest_2); profiles show
  the remaining ground at the foot of the trees. A block without any large smooth ground has no
  trusted ground, so the filter does nothing there.
- The DTM inherits orientation height errors.
- C2: per-pair ground labels on depth maps, voted in fusion. C3: keep the second, farther
  matching minimum in gaps.

- **forest_4** (one flight line; `examples/forest_4_steps.ps1` options plus `--classify-ground`, 10.5 M points, 16 s):
  42.4% ground, observed in 55.1% of cells, split-half median 0.035 m (p95 0.20 m). Before the
  single-line georeferencing fix this run was skipped (no georeferenced frame).

## GS1 `--ground-pair-correction` and GS2 `--ground-aware-fusion` (5 Nov)

Both imply `--classify-ground` and need `point-confidence.bin` (pair ids).

- **Pair levels (shared):** each stereo pair's own ground height per 1 m cell, from all of its
  points near the terrain (not only class 2, so a lifted copy is still seen): the median of its
  points within the ground tolerance of their 10th percentile. Consensus per cell: weighted mean
  over pairs within 1 m of the weighted median (`GroundSheet.levels`).
- **GS1 (`PairOffsets`):** per pair a robust Huber plane (offset, plus tilt when its cells span
  10 m both ways) of level minus consensus, three rounds, block mean kept. Always measured and
  reported in `ground.json` → `pair_offsets` (per pair: offset, tilt, RMS before/after). With the
  option, each pair's correction is subtracted from all its points (ground and canopy) in
  dense.las and dense.ply, and classification runs again. `point-confidence.bin` keeps the
  original heights unless GS2 rewrites it.
- **GS2 (`GroundSheet`):** each pair's ground copy is shifted onto the consensus (shift
  interpolated bilinearly within the pair), copies merge to one point per point-spacing cell at
  the view-weighted mean height (best-supported point kept), then **sheet fusion**: one robust
  weighted plane per `--ground-fusion-radius` bin (auto 2 × terrain cell; 0.5 m on forest_10),
  blended bilinearly at every ground point, so per-pair depth noise averages across pairs.
  Shrubs and canopy above a pair's own ground are not moved. dense.las, dense.ply and
  point-confidence.bin are rewritten in step (counts, bounds, ids).

### Results

**Synthetic** (`GroundStuffRegression`, SelfTest; four pairs at +0.0, +0.6, −0.2 m with a
4 mm/m tilt, +0.1 m): GS1 recovers the relative offsets within 0.06 m and the tilt within
1.5 mm/m, two-sheet cells 99.7% → 0.0%. GS2 alone: two-sheet ground cells 74.2% → 0.0%,
near-ground thickness 0.81 → 0.08 m (the simulated noise), trees untouched, files aligned.

**forest_10** (run 2af2c8b5, 33.0 M points, copies in the scratchpad):

| | GS1 (measure) | GS2 only | GS1 + GS2 |
| --- | --- | --- | --- |
| Pair offsets | median 0.071 m, p95 0.199 m | – | applied |
| Near-ground thickness (p90−p10 per 1 m cell, shrubs included) | 0.43 m | **0.15 m** | 0.13 m |
| Points | 32.95 M | 29.07 M (3.88 M duplicates merged) | 28.90 M |
| Time (classification + step) | 40 s | +54 s | +~60 s |

- Pairs 0–4 (start of the flight) sit 0.21–0.29 m high with 5–7 mm/m tilts and dominate the
  upper sheets; GS1 cuts their RMS about the consensus from 0.24–0.28 m to 0.11–0.15 m. Over the
  whole block, though, pair-level disagreement is mostly local, so GS1 alone barely changes the
  two-sheet fraction of pair levels (60% → 59%). The thick ground band on forest_10 is mainly
  per-pair depth noise (σ about 0.15 m at 100 m), which only the GS2 sheet fusion removes.
- Profiles (30 m × 0.5 m sections): the 0.5–1 m double sheets become one thin continuous
  sheet; shrubs stay above it.
- Metric notes: `StripConsistency` LAS mode counts any 0.3 m gap with three points on each side
  in the lowest 3 m (shrubs, stray points), so it cannot judge ground layering (26.1% before,
  24.2% after GS1+GS2). The ground-only gap test in `ground.json` (`two_sheet_cells_*`) rises
  slightly after GS2 (5.4% → 7.1%): once the ground is thin, shrubs above it are separated by a
  clean gap. Near-ground thickness is the meaningful number.
- Caveat: sheet fusion smooths ground relief smaller than the radius (stones, small ditches);
  lower `--ground-fusion-radius` to keep more micro-relief.
