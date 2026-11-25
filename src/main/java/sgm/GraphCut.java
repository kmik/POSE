package sgm;

import java.util.Arrays;

/**
 * Minimum s-t cut of a sparse graph (Boykov-Kolmogorov augmenting paths with search-tree reuse),
 * with the binary-energy interface of Kolmogorov and Zabih: variable i is 0 on the source side, 1
 * on the sink side. Used by the orthomosaic seam search (OrthoSeams). Deterministic: nodes and
 * arcs are visited in the order they were added.
 */
final class GraphCut {
  private static final int TERMINAL = -2, ORPHAN = -3, NONE = -1;

  private final int n;
  private final double[] terminal;
  private final int[] first, parent, nextActive, ts, dist;
  private final boolean[] sink;
  private int[] head, next;
  private double[] capacity;
  private int arcs;
  private double flow;
  private int activeFirst = NONE, activeLast = NONE;
  private int[] orphans = new int[64];
  private int orphanHead, orphanTail;
  private int time;

  GraphCut(int nodes, int arcEstimate) {
    n = nodes;
    terminal = new double[n];
    first = new int[n];
    parent = new int[n];
    nextActive = new int[n];
    ts = new int[n];
    dist = new int[n];
    sink = new boolean[n];
    Arrays.fill(first, NONE);
    int size = Math.max(16, 2 * arcEstimate);
    head = new int[size];
    next = new int[size];
    capacity = new double[size];
  }

  /** Adds cost0 to x_i = 0 and cost1 to x_i = 1. */
  void unary(int i, double cost0, double cost1) {
    flow += Math.min(cost0, cost1);
    terminal[i] += cost1 - cost0;
  }

  /** Adds the pairwise energy E(0,0) = a, E(0,1) = b, E(1,0) = c, E(1,1) = d (needs a + d <= b + c). */
  void pairwise(int i, int j, double a, double b, double c, double d) {
    unary(i, a, d);
    b -= a;
    c -= d;
    if (b < 0) {
      unary(i, b, 0);
      unary(j, -b, 0);
      edge(i, j, 0, b + c);
    } else if (c < 0) {
      unary(i, -c, 0);
      unary(j, c, 0);
      edge(i, j, b + c, 0);
    } else edge(i, j, b, c);
  }

  private void edge(int i, int j, double forward, double backward) {
    if (arcs + 2 > head.length) {
      int size = 2 * head.length;
      head = Arrays.copyOf(head, size);
      next = Arrays.copyOf(next, size);
      capacity = Arrays.copyOf(capacity, size);
    }
    int a = arcs++, b = arcs++;
    head[a] = j;
    capacity[a] = forward;
    next[a] = first[i];
    first[i] = a;
    head[b] = i;
    capacity[b] = backward;
    next[b] = first[j];
    first[j] = b;
  }

  /** 1 when variable i is on the sink side after maxflow(). */
  int label(int i) {
    return parent[i] != NONE && sink[i] ? 1 : 0;
  }

  private void activate(int i) {
    if (nextActive[i] != NONE) return;
    nextActive[i] = i; // marks "in the list"; the real link is set below
    if (activeLast == NONE) activeFirst = i;
    else nextActive[activeLast] = i;
    activeLast = i;
    nextActive[i] = i;
  }

  private int nextActiveNode() {
    while (activeFirst != NONE) {
      int i = activeFirst;
      int following = nextActive[i];
      activeFirst = following == i ? NONE : following;
      if (activeFirst == NONE) activeLast = NONE;
      nextActive[i] = NONE;
      if (parent[i] != NONE) return i;
    }
    return NONE;
  }

  private void orphan(int i) {
    parent[i] = ORPHAN;
    if (orphanTail == orphans.length) {
      int live = orphanTail - orphanHead;
      if (orphanHead > orphans.length / 2) System.arraycopy(orphans, orphanHead, orphans, 0, live);
      else orphans = Arrays.copyOf(Arrays.copyOfRange(orphans, orphanHead, orphanTail), 2 * orphans.length);
      orphanHead = 0;
      orphanTail = live;
    }
    orphans[orphanTail++] = i;
  }

  double maxflow() {
    Arrays.fill(nextActive, NONE);
    for (int i = 0; i < n; i++) {
      ts[i] = 0;
      dist[i] = 1;
      if (terminal[i] > 0) {
        sink[i] = false;
        parent[i] = TERMINAL;
        activate(i);
      } else if (terminal[i] < 0) {
        sink[i] = true;
        parent[i] = TERMINAL;
        activate(i);
      } else parent[i] = NONE;
    }
    int current = NONE;
    while (true) {
      int i = current != NONE && parent[current] != NONE ? current : nextActiveNode();
      current = NONE;
      if (i == NONE) break;
      int found = NONE;
      if (!sink[i]) {
        for (int a = first[i]; a != NONE; a = next[a]) {
          if (!(capacity[a] > 0)) continue;
          int j = head[a];
          if (parent[j] == NONE) {
            sink[j] = false;
            parent[j] = a ^ 1;
            ts[j] = ts[i];
            dist[j] = dist[i] + 1;
            activate(j);
          } else if (sink[j]) {
            found = a;
            break;
          } else if (ts[j] <= ts[i] && dist[j] > dist[i]) {
            parent[j] = a ^ 1;
            ts[j] = ts[i];
            dist[j] = dist[i] + 1;
          }
        }
      } else {
        for (int a = first[i]; a != NONE; a = next[a]) {
          if (!(capacity[a ^ 1] > 0)) continue;
          int j = head[a];
          if (parent[j] == NONE) {
            sink[j] = true;
            parent[j] = a ^ 1;
            ts[j] = ts[i];
            dist[j] = dist[i] + 1;
            activate(j);
          } else if (!sink[j]) {
            found = a ^ 1;
            break;
          } else if (ts[j] <= ts[i] && dist[j] > dist[i]) {
            parent[j] = a ^ 1;
            ts[j] = ts[i];
            dist[j] = dist[i] + 1;
          }
        }
      }
      time++;
      if (found == NONE) continue;
      // Keep i active: it may have more paths.
      current = i;
      augment(found);
      while (orphanHead < orphanTail) {
        int o = orphans[orphanHead++];
        if (sink[o]) adoptSink(o);
        else adoptSource(o);
      }
      orphanHead = orphanTail = 0;
    }
    return flow;
  }

  /** a: arc from a source-tree node to a sink-tree node. */
  private void augment(int middle) {
    double bottleneck = capacity[middle];
    int i = head[middle ^ 1];
    while (parent[i] != TERMINAL) {
      int a = parent[i];
      bottleneck = Math.min(bottleneck, capacity[a ^ 1]);
      i = head[a];
    }
    bottleneck = Math.min(bottleneck, terminal[i]);
    i = head[middle];
    while (parent[i] != TERMINAL) {
      int a = parent[i];
      bottleneck = Math.min(bottleneck, capacity[a]);
      i = head[a];
    }
    bottleneck = Math.min(bottleneck, -terminal[i]);
    capacity[middle ^ 1] += bottleneck;
    capacity[middle] -= bottleneck;
    i = head[middle ^ 1];
    while (parent[i] != TERMINAL) {
      int a = parent[i];
      capacity[a] += bottleneck;
      capacity[a ^ 1] -= bottleneck;
      int up = head[a];
      if (!(capacity[a ^ 1] > 0)) orphan(i);
      i = up;
    }
    terminal[i] -= bottleneck;
    if (!(terminal[i] > 0)) orphan(i);
    i = head[middle];
    while (parent[i] != TERMINAL) {
      int a = parent[i];
      capacity[a ^ 1] += bottleneck;
      capacity[a] -= bottleneck;
      int up = head[a];
      if (!(capacity[a] > 0)) orphan(i);
      i = up;
    }
    terminal[i] += bottleneck;
    if (!(terminal[i] < 0)) orphan(i);
    flow += bottleneck;
  }

  /** Distance of j to its tree's terminal (marking the path), or MAX_VALUE when j hangs off an orphan. */
  private int origin(int j) {
    int d = 0, k = j;
    while (true) {
      if (ts[k] == time) {
        d += dist[k];
        break;
      }
      int a = parent[k];
      d++;
      if (a == TERMINAL) {
        ts[k] = time;
        dist[k] = 1;
        break;
      }
      if (a == ORPHAN) return Integer.MAX_VALUE;
      k = head[a];
    }
    int mark = d;
    for (k = j; ts[k] != time; k = head[parent[k]]) {
      ts[k] = time;
      dist[k] = mark--;
    }
    return d;
  }

  private void adoptSource(int i) {
    int best = NONE, bestDistance = Integer.MAX_VALUE;
    for (int a = first[i]; a != NONE; a = next[a]) {
      if (!(capacity[a ^ 1] > 0)) continue;
      int j = head[a];
      if (sink[j] || parent[j] == NONE) continue;
      int d = origin(j);
      if (d < bestDistance) {
        bestDistance = d;
        best = a;
      }
    }
    if (best != NONE) {
      parent[i] = best;
      ts[i] = time;
      dist[i] = bestDistance + 1;
      return;
    }
    for (int a = first[i]; a != NONE; a = next[a]) {
      int j = head[a];
      if (sink[j] || parent[j] == NONE) continue;
      if (capacity[a ^ 1] > 0) activate(j);
      int pa = parent[j];
      if (pa != TERMINAL && pa != ORPHAN && head[pa] == i) orphan(j);
    }
    parent[i] = NONE;
  }

  private void adoptSink(int i) {
    int best = NONE, bestDistance = Integer.MAX_VALUE;
    for (int a = first[i]; a != NONE; a = next[a]) {
      if (!(capacity[a] > 0)) continue;
      int j = head[a];
      if (!sink[j] || parent[j] == NONE) continue;
      int d = origin(j);
      if (d < bestDistance) {
        bestDistance = d;
        best = a;
      }
    }
    if (best != NONE) {
      parent[i] = best;
      ts[i] = time;
      dist[i] = bestDistance + 1;
      return;
    }
    for (int a = first[i]; a != NONE; a = next[a]) {
      int j = head[a];
      if (!sink[j] || parent[j] == NONE) continue;
      if (capacity[a] > 0) activate(j);
      int pa = parent[j];
      if (pa != TERMINAL && pa != ORPHAN && head[pa] == i) orphan(j);
    }
    parent[i] = NONE;
  }
}
