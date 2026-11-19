package sgm;

import java.util.*;

/** Immutable display-only predicates. Coordinates are relative to the data's double origin. */
record CloudDisplay(CloudDisplay.Box box, Map<String, CloudDisplay.Range> ranges,
                    Set<Integer> classes, Integer denseColour, int lidarColour, double shading) {
  static final CloudDisplay DEFAULT = new CloudDisplay(null, Map.of(), Set.of(), null, 0xff9800, 12);

  record Range(double low, double high) {
    boolean contains(double v) { return Double.isFinite(v) && v >= low && v <= high; }
    boolean containsFloat(float v) { return Float.isFinite(v) && v >= (float)low && v <= (float)high; }
  }

  record Box(double x0, double y0, double z0, double x1, double y1, double z1) {
    Box {
      if (Double.isNaN(x0) || Double.isNaN(y0) || Double.isNaN(z0) || Double.isNaN(x1) || Double.isNaN(y1) || Double.isNaN(z1) || x0 > x1 || y0 > y1 || z0 > z1)
        throw new IllegalArgumentException("Clip minimum must not exceed maximum");
    }
    boolean contains(double x, double y, double z) {
      return x >= x0 && x <= x1 && y >= y0 && y <= y1 && z >= z0 && z <= z1;
    }
    boolean intersects(Box b) {
      return x0 <= b.x1 && x1 >= b.x0 && y0 <= b.y1 && y1 >= b.y0 && z0 <= b.z1 && z1 >= b.z0;
    }
    Box shift(double x, double y, double z) { return new Box(x0+x,y0+y,z0+z,x1+x,y1+y,z1+z); }
    double[] centre() { return new double[] {(x0+x1)/2,(y0+y1)/2,(z0+z1)/2}; }
  }

  CloudDisplay {
    ranges = Map.copyOf(ranges);
    classes = Set.copyOf(classes);
  }

  boolean position(double x, double y, double z) {
    return Double.isFinite(x) && Double.isFinite(y) && Double.isFinite(z) && (box == null || box.contains(x,y,z));
  }

  boolean dense(CloudData d, int i) {
    if (!position(d.x[i],d.y[i],d.z[i]) || !classes.isEmpty() && !classes.contains(d.cls[i]&255)) return false;
    for (var entry : ranges.entrySet()) {
      if (entry.getKey().equals("height")) {
        if (!entry.getValue().contains(d.z[i]+d.origin[2])) return false;
      } else {
        CloudData.Attribute a = d.attribute(entry.getKey());
        if (a == null || !entry.getValue().containsFloat(a.values()[i])) return false;
      }
    }
    return true;
  }
}
