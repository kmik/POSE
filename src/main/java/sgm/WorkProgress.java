package sgm;

import java.util.*;
import java.util.concurrent.*;

/** Rate-limited activity, including native calls and unknown-length preparation. */
final class WorkProgress implements AutoCloseable {
  private static final ScheduledExecutorService CLOCK =
      Executors.newSingleThreadScheduledExecutor(
          r -> {
            Thread t = new Thread(r, "processing-progress");
            t.setDaemon(true);
            return t;
          });
  private final String label, units;
  private final long started = System.nanoTime();
  // Total work; a task may revise it (see total(long, boolean)). 0 = unknown.
  private volatile long total;
  private volatile boolean estimated, revised;
  private final RunTelemetry owner = RunTelemetry.current();
  private final ScheduledFuture<?> heartbeat;
  private volatile long count;
  private volatile String detail = "";
  private boolean complete, closed;

  static WorkProgress start(String label, long total, String units) {
    return new WorkProgress(label, total, units);
  }

  private WorkProgress(String label, long total, String units) {
    this.label = label;
    this.total = total;
    this.units = units;
    System.out.println(label + ": starting" + (total > 0 ? " (" + total + " " + units + ")" : ""));
    if (owner != null) owner.started(this, label, started);
    Terminal.started(this);
    heartbeat =
        CLOCK.scheduleWithFixedDelay(
            () -> {
              try {
                print("working");
              } catch (RuntimeException ignored) {
              }
            },
            5,
            5,
            TimeUnit.SECONDS);
  }

  String label() {
    return label;
  }

  String units() {
    return units;
  }

  long total() {
    return total;
  }

  long count() {
    return count;
  }

  /** True when the total is an estimate (shown with "~"). */
  boolean estimated() {
    return estimated;
  }

  /**
   * Sets or revises the total (0 = unknown, no bar). A revised total is exact or an upper bound
   * when estimated is false. On done() a revised total becomes the count actually reached.
   */
  void total(long total, boolean estimated) {
    this.estimated = estimated && total > 0;
    this.total = Math.max(0, total);
    revised = true;
  }

  String detailText() {
    return detail;
  }

  long startedNanos() {
    return started;
  }

  void update(long count) {
    this.count = count;
  }

  void detail(String detail) {
    this.detail = detail;
  }

  void done() {
    detail = "";
    if (revised) {
      // An early stop or an estimate: report the work actually done.
      total = count;
      estimated = false;
    }
    complete = true;
  }

  private synchronized void print(String state) {
    if (closed && state.equals("working")) return;
    double seconds = (System.nanoTime() - started) / 1e9;
    long total = this.total;
    String amount =
        total > 0
            ? String.format(
                Locale.ROOT,
                "%,d/%s%,d %s (%.1f%%)",
                count,
                estimated ? "~" : "",
                total,
                units,
                Math.min(100, 100. * count / total))
            : count > 0 ? String.format(Locale.ROOT, "%,d %s", count, units) : "";
    System.out.printf(
        Locale.ROOT,
        "%s: %s%s; %.1f s%s%s%n",
        label,
        state,
        amount.isEmpty() ? "" : "; " + amount,
        seconds,
        count > 0 && seconds > 0
            ? String.format(Locale.ROOT, "; %,.2f %s/s", count / seconds, units)
            : "",
        detail.isEmpty() ? "" : "; " + detail);
  }

  public synchronized void close() {
    if (closed) return;
    closed = true;
    heartbeat.cancel(false);
    if (complete && total > 0) count = total;
    print(complete ? "complete" : "stopped before completion");
    Terminal.finished(this);
    if (owner != null) owner.finished(this, label, started, complete);
  }
}
