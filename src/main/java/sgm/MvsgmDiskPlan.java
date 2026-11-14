package sgm;

/** Separates predictable map work from the unknown final cloud's all-pixels upper bound. */
record MvsgmDiskPlan(long maps, long spoolUpper, long exportsUpper, long recordsUpper,
    long deferredUpper, long diagnosticsUpper, long headroom, long startup, long worstCase) {
  static MvsgmDiskPlan of(long pixels, int cameras, long maps, long retained, Options o) {
    long attributes = o.pointAttributes.equals("surface") ? SurfaceAttributes.BYTES : 0;
    // The cloud is written once, straight to dense.las / dense.ply (no staging spool since
    // 14 November 2026); the field stays for the disk_plan.json layout.
    long spool = 0;
    long exports = Math.multiplyExact(pixels, 26 + attributes + (o.writePly ? 27 + attributes : 0));
    long confidence = o.writePointConfidence ? Math.multiplyExact(pixels, 224L + Math.ceilDiv(cameras, 8)) : 0;
    long deferred = o.fusionEmission.equals("camera") || o.surfaceConsolidation.equals("normal")
        ? Math.multiplyExact(pixels, 256L + Math.ceilDiv(cameras, 8)) : 0;
    long diagnostics = o.diagnostics ? Math.multiplyExact(pixels, 2048L) : 0;
    long records = Math.addExact(confidence, deferred);
    long worst = Math.addExact(Math.max(Math.addExact(maps, spool),
        Math.addExact(Math.addExact(retained, diagnostics), Math.addExact(spool, exports))), records);
    worst = Math.addExact(worst, Math.max(256L << 20, worst / 20));
    long fixed = Math.addExact(maps, Math.addExact(deferred, diagnostics));
    long headroom = Math.max(256L << 20, fixed / 20);
    // Streamed confidence/cloud files grow under actual-space checks. Deferred sort/surface
    // intermediates keep their earlier conservative reservation until they too are streamable.
    long batch = Math.min(pixels, OutputDiskSpace.BATCH);
    long initial = batch * (26 + attributes + (o.writePly ? 27 + attributes : 0)
        + (o.writePointConfidence ? 256L + Math.ceilDiv(cameras, 8) : 0));
    long startup = Math.addExact(Math.addExact(fixed, headroom), initial + 4096);
    if (deferred != 0 || o.diagnostics) startup = Math.max(startup, worst);
    return new MvsgmDiskPlan(maps, spool, exports, records, deferred, diagnostics,
        headroom, startup, worst);
  }
}
