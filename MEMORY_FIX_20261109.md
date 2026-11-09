# Large alignment memory safety, 9 November 2026

This revision addresses the reported 3,000-plus-image Linux alignment that reached 260.9 GB RSS
and exited with status 137. That report is consistent with an external memory kill; the actual
scheduler/kernel record is still needed to establish the cause of that particular termination.
The changes fix independently verified allocation/lifetime risks and missing limits handling.

The changes are in the working copy. The original `sgm_java` is unchanged.
Pre-change source and compiled classes are retained in `target/memory-20261109/baseline-src` and
`baseline-classes`. Existing saved frame/track formats and reconstruction thresholds are retained.

## Host and job limits

Linux detection reads `/proc/meminfo`, the process's cgroup memberships and mount information.
It supports cgroup v1 and v2, walks visible ancestors, handles namespace-rebased paths and escaped
mount paths, and preserves known limits even when usage counters cannot be read. A finite quota
with an unreadable usage counter assumes zero safe headroom and refuses allocation. An unlimited
leaf does not hide a finite parent limit. Reclaimable inactive file cache is considered separately
from live controller usage. Strict host commit accounting is enforced when overcommit mode is 2;
heuristic-mode Committed_AS is not treated as a hard limit.

Slurm per-node and per-CPU allocation variables provide an additional bound when running inside
a Slurm job. An optional `--ram-limit-mb N` supplies a total host-RAM planning ceiling in MiB.
It is intersected with detected limits; it cannot enlarge the job allocation. Actual limits and
their source are written to `logs/processing.json` under `host_memory_limits`.

The CLI now caps heap at 65% of the effective host/job ceiling and accounts for available RAM,
leaving native/JVM headroom. It can reduce an oversized existing heap by relaunching, including
removing an incompatible initial-heap setting. It relaunches only once. Embedded `Main.run` calls
cannot resize their host JVM and reject a heap that exceeds the detected total-memory envelope.

`--memory-mb` retains its meaning as a processing allowance, **not a hard RSS cap**. Requested
allowances may be reduced by safe heap sizing. For a 228 GiB job allocation, the total-limit heap
ceiling is approximately 148 GiB, rather than allowing a 192 GiB heap plus unbounded native growth.
Current availability can lower that ceiling further. Neither planning flag replaces scheduler/
cgroup hard enforcement; other processes and untracked native allocations can still create pressure.

## Geometry native lifetime

OpenCV 4.9's Java `Mat.release()` releases matrix data, while deletion of its native header depends
on Java finalization. Allocating six Mats for every triangulation attempt could therefore generate
large numbers of native headers before a large heap triggered collection.

Triangulation now reuses one thread-local SVD workspace and writes projection coefficients directly
from camera arrays. The same four native matrix headers serve repeated point attempts. PnP also
reuses its matrix wrappers across registrations. Geometry and pipeline cleanup release these
workspaces, including failure paths. No reflective native deletion or forced finalization is used.

## Bounded alignment intermediates

Bundle adjustment materializes per-observation Jacobians in bounded batches instead of retaining
them for the entire graph. Batches are selected by observation count, with a separate bound on
empty/untriangulated track entries. A single longer track receives its own admission check.
Shared camera/lens contributions are added in original track/observation order across batches.
Point/lens fitting, prior terms, robust objective and solver thresholds are unchanged.

Robust-objective cost buffers and conditional calibration-validation residuals are also batched.
Ordinary pose/point bundles no longer allocate unused lens-cross arrays for every point. Damped
and undamped point blocks remain resident and are included in workspace estimates.

Before restoring a saved track graph, a streaming preflight counts tracks, observations, pairs and
matches without creating their object graph. Keypoint restoration, shared-calibration measured
points and joint bundle workspace also have conservative heap/RAM admission checks. A fitting
allocation is retried once after collection when a large heap may retain unreachable prior-step
garbage; this is separate from the native workspace lifetime fix.

The pipeline releases its reference to the previous step's sparse graph before the next step
loads it. Oversized known allocations fail with an actionable memory message and project failure
state. The full sparse graph and Schur point blocks are still resident: this is not out-of-core SfM,
and datasets genuinely larger than the allocation must use fewer features or smaller blocks.

## Verification and server use

The complete Windows/OpenCV regression suite passed **116,450 checks** (exit 0), logged in
`target/memory-20261109/self-test.log`. After that suite's build was frozen, additional guards
for unreadable usage, over-quota cache reclamation and untriangulated long tracks were added.
The final build passed **1,885 memory checks**, **286 launcher checks** and **925 bundle checks**;
logs are `final-memory-controls.log`, `final-launcher-controls.log` and `final-bundle-controls.log`
in the same directory. Final compiled classes were installed in `target/classes` and
`target/test-classes`. These are correctness checks, not a controlled performance benchmark.

`AlignmentMemoryRegression` covers cgroup v1/v2 ancestors, namespaces, escaped paths, missing usage,
Slurm allocations, strict commit accounting, heap ceiling calculations and rejected graph loads.
It performs 100,000 triangulations using the same native headers, compares 1,000 with the original
per-point SVD bit for bit, and restores a 3,001-camera-ID graph fixture. Small and large Jacobian/
residual batches give bit-identical poses, points, shared lens values and RMS.

`LauncherRegression` checks both raising a small heap and reducing a 1 GiB heap under a 512 MiB
explicit RAM envelope. The normal bundle controls retain the matrix-free fallback test without
using an invalid zero processing budget to force that solver path.

A branch of the real 17-image city project was aligned with a 4,000 MiB processing allowance and
8,192 MiB total RAM planning ceiling. Saved `frames.bin` and `tracks.bin` after alignment are
byte-identical to the pre-change alignment. Evidence is in
[`target/memory-20261109/native-city-case.json`](target/memory-20261109/native-city-case.json).

Linux limit-file parsing is tested using fixtures on Windows. This environment has no Linux/WSL
runtime, and the server's full 3,000-image dataset has not been executed here. On the Linux server,
with the existing dependency jars and JDK 25, run this isolated verification:

```bash
bash scripts/verify-memory.sh
```

Rebuild/deploy this revision before resuming the failed alignment. Existing matching state can be
reused; no change to image dimensions or feature settings is required by the fix:

```bash
./photogrammetry.sh align -o /work3/RESEARCH/ENNAS/POSE/POSE_output3 --memory-mb 128000
```

Detected cgroup/Slurm limits apply automatically. If the scheduler allocation is not visible,
add `--ram-limit-mb` with the actual allocation in MiB. For example, **only if the allocation is
228 GiB**, use `--ram-limit-mb 233472`. Confirm allocation units rather than treating GB and GiB as
interchangeable. The CLI reports any heap reduction before opening the project.
