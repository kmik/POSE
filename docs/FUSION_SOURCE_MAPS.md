# Scoped MVSGM source maps

Use `--fusion-source-maps scoped` with `--dense-method mvsgm --fusion-mode image`.
The default is `retained`, preserving the original source-map lifetime. The option is
independent of `--fusion-host-memory-mb`; to test source lifetime alone, use:

```sh
--fusion-source-maps scoped --fusion-host-memory-mb 0
```

This setting appears in the GUI and is remembered in preferences, saved run settings,
and checkpoint-continuation arguments. Restore the original lifetime explicitly with
`--fusion-source-maps retained`. If the bounded host-memory option is enabled, its page
cache takes precedence and already provides scoped source access.

## What changes

`ImageFusion.mapPair` obtains a source-camera reader, resolves the needed ordinary
memory mappings once, and copies rows into the existing owned candidate arrays with
the same bulk-copy operations and reconstruction arithmetic. All copying tasks finish
before the reader closes, including on failure or cancellation. The last active reader
of a camera closes its mapping arena. Another reader can reopen the same files later.

Closing source mappings does not retire cameras or delete files. File retirement still
follows the existing complete dependency graph, later stereo consistency readers,
precision tiers, and diagnostic/retention settings.

There is no new page cache or memory allowance in this option. Fusion planning, strip
selection, stereo preparation, cameras in flight, output queueing, ownership, camera
order and point IDs retain their existing behavior. Source-copy workers use a joined
task barrier to protect their mappings on exceptional exits; other fusion work keeps
its existing scheduling.

The fusion report's `memory.source_maps` records current/peak source mapping extents,
file mapping opens/closes and active reader scopes. These are mapping sizes, **not RAM
measurements**. OS file cache can retain reclaimable pages after unmapping. Other
fusion maps, candidate arrays, native allocations and output structures are unchanged;
this is a source-retention fix, not a total-RAM bound.

## Building and continuing the server project

Build with JDK 25 in a new engine installation, keeping the existing project and engine:

```sh
bash scripts/verify-fusion-host-memory.sh --install
```

This verification entry point excludes the two existing suites that execute ULTRA
fixtures and reports those exclusions. The original build scripts remain unchanged.
The launcher uses `target/classes`; copying source alone does not install an update.
Restart the launcher GUI after rebuilding so it discovers the new option.
An engine upgrade changes implementation fingerprints, so old reconstruction-cache
entries can miss even when their feature/matching algorithms are unchanged. Use the
saved pipeline-state checkpoint below to preserve completed alignment.

To reuse the supplied server run's completed alignment and redo dense reconstruction:

```sh
PROJECT=/work3/RESEARCH/ENNAS/POSE/POSE_output_mvsgm
SOURCE_RUN=20261117T101444909Z_633a5299
./photogrammetry.sh status -o "$PROJECT" --run "$SOURCE_RUN"
./photogrammetry.sh dense ground finish -o "$PROJECT" --run "$SOURCE_RUN" --branch \
  --quality high --fusion-source-maps scoped --fusion-host-memory-mb 0
```

The new branch inherits existing matching, memory, GPU, stereo and filtering settings.
It recomputes stereo/fusion and preserves the original run. Check disk space for both.
For a failed step on the new run, retry that new run ID without `--branch`. Source-map
scoping does not resume partial temporary maps. The full 3,013-image workload has not
been validated locally.

## Validation

The source and compiled baseline before this change are saved in the new
`target/source-map-scopes-*` validation directory. The dedicated regression covers
32, 256 and 3,013 retained disk dependencies, repeated reopening, shared-reader leases,
actual arena invalidation, failure/cancellation joins, GUI discovery and option copies.
Ownership/precision variants, mixed dimensions, voxel output, tiled LAS and surface
consolidation compare ordered cloud/confidence bytes with retained source mappings.

The final source regression passes **10,264 checks**. Related checks also pass:
`FusionHostMemoryRegression` 11,270; `MvsgmCameraPoolRegression` 94;
`CameraWindowRegression` 1,686; `GuiRegression` 1,115. These were focused runs, not a new
full SelfTest run. No ULTRA reconstruction was executed.

### Forest HIGH comparison, 18 November 2026

The 24 images in `../example_data/forest/forest_9` were processed at HIGH quality,
2640 x 1978 largest processed dimensions, eight cores, 4,324 MiB heap, 2,800 MiB
processing budget and 4,000 MiB GPU allowance. Both fresh full-pipeline runs use the
same final binary and settings; only `--fusion-source-maps retained|scoped` differs.
The shared host-memory option is zero in both. Separate new output directories retain
their complete results. Exact arguments and measurements are under
`target/source-map-scopes-20261118T082021Z/`.

| Measurement | Retained | Scoped |
| --- | ---: | ---: |
| End-to-end seconds | 243.98 | 245.67 |
| CPU seconds | 1,212.56 | 1,227.77 |
| Peak sampled process RSS, GiB | 8.233 | 5.526 |
| Peak sampled private bytes, GiB | 7.135 | 6.915 |
| API read transfer, GiB | 7.5167 | 7.5168 |
| API write transfer, GiB | 7.3069 | 7.3078 |
| Peak source mapping extents, bytes (not RAM) | 3,007,825,920 | 125,326,080 |
| Peak source file mappings | 144 | 6 |
| Source mapping opens / closes | 144 / 144 | 192 / 192 |

The pair shows **32.9% lower peak sampled RSS**, with **0.69% longer elapsed time**.
That small timing difference is not a statistically established overhead. API transfer
totals are effectively unchanged; they include cached operations and do not measure
physical disk traffic or all mapped-page-fault I/O. Private/committed memory changes
much less than RSS. Source mapping extents are reduced by 24x, and both cases finish
with zero source mappings and zero active reader scopes.

Both runs emit **16,582,058 points**. SHA-256 comparisons match for all five artifacts:
LAS, PLY, binary point confidence, saved georeferenced frames and saved tracks. They
also match the original pre-host-memory-change legacy hashes. This verifies point
geometry, attributes, candidate IDs and order as well as alignment preservation.

The complete fusion-plan strings are identical: all 24 cameras' fusion maps stay
resident, strip rows are zero, and the same emission bands, normal cache and worker
counts are used. Both saved effective settings and pipeline continuation arguments
record the requested source lifetime with `fusionHostMemoryMb=0`.

Evidence: `byte-comparison.json`, `original-legacy-comparison.json`,
`measurement-summary.json`, `retained-args.json`, `scoped-args.json`, individual
`*-measurement.json` files, source manifest, regression logs, and
`install-verification.json`. All 194 engine source files were checked against the
compiled snapshot and all 595 installed class hashes match the tested build.

### Limits of these measurements

These are one paired trial on the Windows desktop, not the 260 GB server. Memory
pressure differed: available host RAM started at 5.69 / 8.43 GiB and reached minima of
0.275 / 2.905 GiB in retained/scoped order. Desktop activity, compression, caching and
run order can affect timing and RSS. Consequently the observed 0.69% timing difference
does not establish production performance. The earlier bounded-page-cache timing
trials used different code paths and a different desktop-memory state; do not compare
their elapsed times directly with this pair.

Post-processing settings-path and PowerShell serialization errors were corrected;
the raw reconstruction measurements and artifact hashes were unaffected. Both
reconstruction runs and all listed regression runs passed. The full 3,013-image
reconstruction remains untested. This option specifically removes long-lived source
mapping retention; it does not guarantee a bound on total process or system RAM.
