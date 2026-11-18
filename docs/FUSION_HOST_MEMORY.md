# Opt-in bounded host working sets for overlap-window MVSGM

Enable with `--fusion-host-memory-mb N`, where N is a MiB accounting allowance across
retained project data, stereo, fusion, caches and output buffers. The default is **0**,
which preserves the legacy implementation when source maps use their default `retained`
lifetime. For the independent source-lifetime-only option, see
[scoped source maps](FUSION_SOURCE_MAPS.md). `--fusion-host-cache-mb N` sets the shared
source/fusion page-cache payload cap (default **512 MiB**, additionally capped at one
eighth of the effective host allowance). The GUI discovers both fields from CLI help,
remembers them, and writes them into run settings and pipeline continuation arguments.

Example additions to an existing high-quality MVSGM image-fusion command:

```sh
--quality high --dense-method mvsgm --fusion-mode image --disparity-refinement off \
--streamed-processing auto --fusion-host-memory-mb 40000 --fusion-host-cache-mb 4096
```

`--fusion-window-mb` continues to control regional **GPU evidence**. It has no new host
meaning. `--memory-mb`, the launcher heap sizing and `--ram-limit-mb` remain separate.
The new allowance must leave room after retained data/native/image reservations for the
largest unchanged stereo camera. Lower the cache or increase the allowance/job allocation
if admission refuses that camera. Reducing stereo tile size automatically is deliberately
avoided: it can change SGM results at tile borders.

## Implementation and lifetime

* `MvsgmMaps.Reader` is scoped to `ImageFusion.mapPair`. Every parallel row task joins
  before the reader closes, including on interruption/failure. Candidate positions,
  evidence, colour and IDs are copied into owned `PairData` arrays. Subsequent fusion and
  output tasks do not retain source-file views.
* Source maps, consistency maps, merged maps, weights, local sigma and ownership bits use
  a shared `HostMapCache`. Its 64 KiB heap pages are evicted **before** a replacement page
  is allocated. Mutable pages flush before eviction; immutable pages support concurrent
  reads without a per-sample global lock. A separate 64-channel LRU limits file handles.
  Page count is bounded as well as payload, so many partial/tiny maps cannot bypass the
  metadata reservation. Source row scratch is reused once per worker instead of leaving
  per-call ThreadLocal values waiting for collection.
  No map-sized mapping or array is retained by this opt-in storage path.
* Cache eviction keeps files intact. Camera retirement follows all exact/conservative
  fusion readers plus later raw-depth consistency readers. Diagnostics/retention still
  keep their files. Precision seed tiers retain dependencies through the last tier.
* Stereo camera concurrency becomes one and preparation does not overlap fusion or the
  next camera. The existing bounded processed-image cache remains reusable. Stereo and
  fusion share one allowance. Pending output is drained before preparing another camera,
  so emission arrays cannot overlap a newly admitted stereo workspace.
* Fusion uses the existing exact strip algorithm and halos. Strip height, emission band
  size, speckle concurrency and surface-model caches use the shared plan. Emission costs
  include dependency-sized agreement arrays and camera bitsets. Compact bitsets replace
  boxed dependency-reader sets in this path. Overlap geometry/selection is unchanged.
* Fixed cloud, tiled-output and optional surface-consolidation buffers are reserved in
  the shared plan. Growing voxel sets and tile/spool metadata are admitted before each
  new entry; exceeding the allowance stops with an actionable error. Output accounting is
  also recorded in `06_fusion/host_output_memory.json`.
* Camera headers, dependency graphs, row prefixes and reports still scale with project
  size. Their retained/future metadata is reserved; this is a bounded raster working set,
  not an assertion that all application data is O(1) in camera count.

Every approximately 30 seconds, `Host fusion memory:` prints heap use, available host/job
memory, RSS/private bytes, Linux RssAnon/RssFile/RssShmem where available, mapped file
extents, cache use, active/retired cameras and evictions. Mapped extents are **not resident
RAM**. Cache statistics and the shared plan are included in the fusion/window JSON reports.
Cache counters measure application page I/O, not physical disk traffic.

Pressure trims pages and lowers row concurrency; pre-allocation guards check the shared
plan, available heap and current host/job room. Cancellation joins active work before
retirement/deletion. The cache payload cap is explicit; page metadata, file-slot metadata
and bounded transient active-reader pages are additionally reserved. JVM collection,
OpenCV/OpenCL allocations, allocator fragmentation, OS file cache, competing processes
and job accounting make a strict total-RAM guarantee impossible from this accounting.
Use the server's cgroup/Slurm limit for enforcement and leave operating headroom.

## Deployment and resuming the interrupted server run

Copy the updated Java source tree and documentation into a **new engine installation**
on the server; retain the existing engine and project. Build there with JDK 25:

```sh
cd /path/to/new/sgm_java_revision_claude_sequential
bash scripts/verify-fusion-host-memory.sh --install
```

This compiles the engine and tests, runs `SelfTest` through a high-only harness omitting
the two existing suites that invoke ultra-quality fixtures, and installs classes only after
the checks pass. It reports both omissions. The ordinary `build.sh`/`build.ps1` remain
unchanged and run the full original suite. Windows-generated Java class
files are portable to JDK 25, but keep the server's platform-specific native libraries.
The launcher reads `target/classes`; copying only source files does not update the engine.

The supplied log identifies the existing high-quality project/run as:

```sh
PROJECT=/work3/RESEARCH/ENNAS/POSE/POSE_output_mvsgm
SOURCE_RUN=20261117T101444909Z_633a5299
./photogrammetry.sh status -o "$PROJECT" --run "$SOURCE_RUN"
test -f "$PROJECT/runs/$SOURCE_RUN/state/state.json"
```

When the predecessor alignment/georeferencing steps are complete, branch at dense. This
loads exact saved binary cameras/tracks and preserves the interrupted run and its files:

```sh
./photogrammetry.sh dense ground finish -o "$PROJECT" --run "$SOURCE_RUN" --branch \
  --quality high --memory-mb 56000 --streamed-processing auto \
  --fusion-host-memory-mb 40000 --fusion-host-cache-mb 4096
```

The command inherits the saved stereo, GPU, filtering, ownership and output settings.
`ground` is skipped if the saved command did not request it. Record the **new run ID**.
Retry a failed dense step on that new run without `--branch`; branch again to redo a
completed dense step. Do not use `--rerun-from` as a substitute for these pipeline-state
checkpoints: reconstruction caches have implementation fingerprints and may invalidate
after an engine upgrade. A valid `state/after-...` checkpoint avoids recomputing alignment.
The branch recomputes stereo/fusion; it does not resume partial map files. The interrupted
run's temporary maps still occupy disk, so check free space for both runs. Preserve `state`
and completed outputs when archiving any old temporary data.

The 40,000/4,096 MiB values are a starting configuration, not a validated admission result
for that server. Inspect the printed retained/native reserves and effective work room;
increase the allowance only within the actual job allocation if the unchanged stereo
workspace cannot fit. Very wide overlap windows need more cache locality than the 24-camera
fixture; a tiny cache can cause severe I/O thrashing even while its byte bound holds.
Disk dependency lifetimes and output-space guards still apply.
The supplied estimates (heap, processing allowance, planned maps, resident maps and disk)
overlap and must not be summed into a RAM estimate. The log alone does not identify the
original growth as anonymous RAM, mapped pages or system file cache.

See [validation measurements and limitations](FUSION_HOST_MEMORY_VALIDATION.md).
