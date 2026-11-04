package sgm;

import java.io.*;
import java.nio.file.*;
import java.util.*;
import javax.imageio.*;
import javax.imageio.stream.ImageInputStream;

/**
 * Header-only resolution planning. Automatic disparity ranges remain estimates until rectification.
 */
final class ResolutionPlan {
  static int[] originalSize(Path path) throws IOException {
    try (ImageInputStream in = ImageIO.createImageInputStream(path.toFile())) {
      if (in == null) throw new IOException("Cannot inspect image header: " + path);
      Iterator<ImageReader> readers = ImageIO.getImageReaders(in);
      if (!readers.hasNext()) throw new IOException("No image header reader for " + path);
      ImageReader reader = readers.next();
      try {
        reader.setInput(in, true, true);
        return new int[] {reader.getWidth(0), reader.getHeight(0)};
      } finally {
        reader.dispose();
      }
    }
  }

  static void write(List<Path> paths, Options o, Project project) throws IOException {
    List<Object> images = new ArrayList<>();
    long pixels = 0, alignmentPixels = 0, maxPixels = 0, sourceMax = 0, evidence = 0;
    int longest = 0, shortest = 0, minimumSide = Integer.MAX_VALUE;
    long[] sizes = new long[paths.size()];
    for (int i = 0; i < paths.size(); i++) {
      int[] original = originalSize(paths.get(i)),
          processed = o.processedSize(original[0], original[1]);
      long n = (long) processed[0] * processed[1];
      if (!o.estimateMemory
          && n > (o.windowedMode.equals("off") ? Integer.MAX_VALUE / 3 : Integer.MAX_VALUE - 8L))
        throw new IllegalArgumentException(
            "Processed image exceeds the dense color-array index limit; select --quality or"
                + " --max-dimension explicitly: "
                + paths.get(i));
      int[] aligned = o.alignmentOptions().processedSize(original[0], original[1]);
      alignmentPixels += (long) aligned[0] * aligned[1];
      minimumSide = Math.min(minimumSide, Math.min(processed[0], processed[1]));
      sizes[i] = n;
      pixels += n;
      sourceMax = Math.max(sourceMax, (long) original[0] * original[1]);
      if (n > maxPixels) {
        maxPixels = n;
        longest = Math.max(processed[0], processed[1]);
        shortest = Math.min(processed[0], processed[1]);
      }
      images.add(
          Json.object(
              "image_id",
              i,
              "filename",
              paths.get(i).getFileName().toString(),
              "original_size",
              original,
              "processed_size",
              processed,
              "alignment_size",
              aligned,
              "scale_xy",
              new double[] {
                processed[0] / (double) original[0], processed[1] / (double) original[1]
              }));
    }
    int pairs = 0;
    for (int i = 0; i < paths.size(); i++)
      for (int j = i + 1; j < Math.min(paths.size(), i + o.neighbors + 1); j++) {
        pairs++;
        evidence += 8 * (sizes[i] + sizes[j]);
      }
    if (!o.pairSelection.equals("sequential")) {
      pairs = paths.size() * (paths.size() - 1) / 2;
      evidence = 0;
      for (int i = 0; i < paths.size(); i++)
        for (int j = i + 1; j < paths.size(); j++) evidence += 8 * (sizes[i] + sizes[j]);
    }
    if (o.rectified) {
      pairs = 1;
      evidence = 8 * (sizes[0] + sizes[1]);
    }
    long imagesNative = Math.min(5 * Math.max(pixels, alignmentPixels), RasterCache.limit(o)),
        features =
            (long) paths.size() * o.features * 64
                + Math.min(
                    (long) paths.size() * o.features * 512,
                    Math.min(o.descriptorMemoryMb * 1048576L, o.memoryMb * 1048576L / 4)),
        decodeNative =
            o.imageReader.equals("windowed")
                    || o.imageReader.equals("auto") && 3 * sourceMax > 64L * 1048576
                ? 32L * o.imageWindow * o.imageWindow
                : 3 * sourceMax,
        rectifyNative = 32 * maxPixels;
    long residentEvidence =
        o.diskDepth
            ? Math.min(
                evidence,
                Math.max(1, o.fusionCacheMb > 0 ? o.fusionCacheMb : Math.min(512, o.memoryMb / 8))
                    * 1048576L)
            : evidence;
    if (o.denseMethod.equals("patchmatch")) {
      long atlas = maxPixels * Math.min(paths.size(), o.patchmatchViews + 1L);
      long deviceBytes = 2 * atlas + 64 * maxPixels, hostBytes = 4 * atlas + 160 * maxPixels;
      Json.write(
          o.output.resolve("00_inputs/resolution_plan.json"),
          Json.object(
              "dense_method",
              "patchmatch",
              "experimental",
              true,
              "compute",
              o.compute,
              "images",
              images,
              "maximum_source_views",
              o.patchmatchViews,
              "coarse_to_fine_levels",
              o.patchmatchLevels,
              "estimated_device_working_bytes",
              deviceBytes,
              "estimated_host_working_bytes",
              hostBytes,
              "stored_depth_normal_evidence_bytes",
              32 * pixels,
              "potential_pair_evidence_bytes",
              evidence,
              "source_raster_cache_limit_bytes",
              RasterCache.limit(o),
              "source_decode_native_bytes",
              decodeNative,
              "confidence_threshold",
              o.confidenceThreshold,
              "sgm_cost_volume_bytes",
              0,
              "scope",
              "Conservative camera-atlas/plane buffers; excludes codec/native raster caches, fusion"
                  + " and JVM overhead. Actual source graph, allocation/device and depth bounds"
                  + " checked before estimation."));
      System.out.printf(
          Locale.ROOT,
          "PatchMatch plan: up to %d source views; %.0f MiB device, %.0f MiB host buffers; no"
              + " disparity cost volume%n",
          o.patchmatchViews,
          deviceBytes / 1048576.,
          hostBytes / 1048576.);
      return;
    }
    List<Integer> counts = new ArrayList<>(Arrays.asList(64, 128, 256, 512));
    if (!counts.contains(o.disparities)) counts.add(o.disparities);
    Collections.sort(counts);
    List<Object> estimates = new ArrayList<>();
    long budget = Math.min(o.memoryMb * 1048576L, (long) (Runtime.getRuntime().maxMemory() * .65));
    if (WindowedStereo.shouldUse(longest, shortest, o)) {
      int limit = WindowedStereo.limit(longest, shortest, o);
      List<Object> scenarios = new ArrayList<>();
      for (int d : counts)
        scenarios.add(
            Json.object(
                "disparities",
                d,
                "host_minimum_strip_bytes",
                WindowedStereo.hostEstimate(longest, d, 1, o),
                "aggregate_disk_bytes",
                maxPixels * d * StereoCost.aggregateBytes(o),
                "raster_temporary_disk_estimate_bytes",
                maxPixels * 112,
                "fits_minimum_strip",
                d <= limit));
      Json.write(
          o.output.resolve("00_inputs/resolution_plan.json"),
          Json.object(
              "dense_method",
              "sgm",
              "windowed_mode",
              o.windowedMode,
              "effective_windowed",
              true,
              "images",
              images,
              "processing_budget_bytes",
              budget,
              "scenarios",
              scenarios,
              "feasible_disparity_limit",
              limit,
              "potential_pair_evidence_bytes",
              evidence,
              "scope",
              "Conservative header-only window/carry estimate; actual rectified width, range,"
                  + " device, native staging, safe heap/commit and temporary disk checked at"
                  + " execution"));
      System.out.printf(
          Locale.ROOT,
          "Windowed stereo plan: largest image %dx%d; minimum-strip disparity limit %d; pair"
              + " evidence %.0f MiB remains separately paged%n",
          longest,
          shortest,
          limit,
          evidence / 1048576.);
      if (!o.estimateMemory && (limit < 3 || o.disparitiesExplicit && o.disparities > limit))
        throw new IllegalArgumentException(
            "Windowed stereo minimum strip/carry cannot fit requested search; see"
                + " resolution_plan.json");
      return;
    }
    int gpuLimit = GpuSgm.disparityLimit(longest, shortest, o);
    long gpuHost = maxPixels * (116 + 32 * ((o.census * o.census - 1 + 63) / 64));
    if (gpuHost > budget) gpuLimit = 0;
    for (int d : counts) {
      Options scenario = o.copy();
      scenario.disparities = d;
      long sgm = CensusSgm.memoryEstimate(longest, shortest, d, o);
      boolean gpuSelected =
          d <= gpuLimit && (!o.compute.equals("auto") || maxPixels * d >= 1000000L);
      GpuStereo.Plan selectedPlan =
          gpuSelected
              ? GpuStereo.plan(
                  GpuSgm.selectedDevice(o), GpuSgm.selectedGroup(o), longest, shortest, d, scenario)
              : null;
      estimates.add(
          Json.object(
              "disparities",
              d,
              "sgm_bytes",
              sgm,
              "gpu_estimated_bytes",
              GpuSgm.memoryEstimate(longest, shortest, d, o),
              "fits_gpu_limits",
              d <= gpuLimit,
              "gpu_memory_plan",
              GpuStereo.report(longest, shortest, scenario),
              "tiled_sgm_resident_bytes",
              CensusSgm.tiledMemoryEstimate(longest, shortest, d, o),
              "tiled_cost_volume_disk_bytes",
              maxPixels * d * (2 + StereoCost.aggregateBytes(o)),
              "fits_sgm_budget",
              d <= gpuLimit
                  || (o.tiled
                      ? CensusSgm.tiledMemoryEstimate(longest, shortest, d, o) <= budget
                      : sgm <= budget && maxPixels * d <= Integer.MAX_VALUE - 8L),
              "potential_depth_evidence_bytes",
              evidence,
              "approximate_known_process_bytes",
              (gpuSelected
                      ? selectedPlan == null ? gpuHost : selectedPlan.hostBytes()
                      : o.tiled ? CensusSgm.tiledMemoryEstimate(longest, shortest, d, o) : sgm)
                  + residentEvidence
                  + imagesNative
                  + features
                  + decodeNative
                  + rectifyNative));
    }
    int limit = DisparityRange.memoryLimit(longest, shortest, o);
    Path file = o.output.resolve("00_inputs/resolution_plan.json");
    Files.createDirectories(file.getParent());
    Json.write(
        file,
        Json.object(
            "quality",
            o.quality,
            "compute",
            o.compute,
            "gpu_device_index",
            o.gpuDevice,
            "gpu_feasible_disparities",
            gpuLimit,
            "storage",
            o.storage,
            "cache_enabled",
            o.cache,
            "alignment_quality",
            o.alignmentQuality,
            "alignment_max_dimension",
            o.alignmentMaxDimension,
            "disk_depth",
            o.diskDepth,
            "tiled",
            o.tiled,
            "pyramid_levels",
            o.pyramidLevels,
            "downsample_factor",
            o.qualityDownsample(),
            "max_dimension_cap",
            o.quality.equals("custom") || o.maxDimensionExplicit ? o.maxDimension : null,
            "images",
            images,
            "potential_neighbor_pairs",
            pairs,
            "resident_processed_image_native_bytes",
            imagesNative,
            "source_pair_raster_cache_limit_bytes",
            RasterCache.limit(o),
            "largest_pair_source_raster_bytes",
            10 * maxPixels,
            "largest_pair_fits_source_cache",
            10 * maxPixels <= RasterCache.limit(o),
            "descriptor_resident_limit_bytes",
            Math.min(o.descriptorMemoryMb * 1048576L, o.memoryMb * 1048576L / 4),
            "image_reader",
            o.imageReader,
            "image_window",
            o.imageWindow,
            "largest_source_decode_native_bytes",
            decodeNative,
            "approximate_feature_bytes",
            features,
            "potential_depth_evidence_heap_bytes",
            residentEvidence,
            "potential_depth_evidence_total_bytes",
            evidence,
            "potential_depth_evidence_disk_bytes",
            o.diskDepth ? evidence : 0,
            "surface_consolidation",
            o.surfaceConsolidation,
            "surface_working_set_budget_bytes",
            o.surfaceConsolidation.equals("normal")
                ? Math.min(o.surfaceMemoryMb, Math.max(8, o.memoryMb / 4)) * 1048576L
                : 0,
            "potential_surface_input_uncompressed_bytes",
            o.surfaceConsolidation.equals("normal")
                ? maxPixels
                    * pairs
                    / ((long) o.step * o.step)
                    * (SurfaceConsolidator.RECORD_BYTES + (paths.size() + 7L) / 8)
                : 0,
            "surface_temporary_disk_scope",
            "Gzip level-1 partitions of accepted candidates only; splitting may temporarily hold"
                + " parent plus children. Actual compressed/peak bytes are recorded in"
                + " 06_fusion/surface_consolidation.json; all work partitions removed on close.",
            "rectification_native_estimate_bytes",
            rectifyNative,
            "jvm_max_heap_bytes",
            Runtime.getRuntime().maxMemory(),
            "sgm_budget_bytes",
            budget,
            "largest_pair_feasible_disparities",
            limit,
            "sgm_scenarios",
            estimates,
            "disparity_mode",
            o.disparitiesExplicit
                ? "explicit"
                : "automatic; actual range determined after rectification",
            "estimate_scope",
            "Known buffers and potential pair evidence; approximate process RAM, not a reservation"
                + " or guarantee. OpenCV/SIFT, bundle workspace, object overhead, image codecs and"
                + " project compression add memory. Matching-row estimate permits vertical"
                + " transposition. No silent quality downgrade or upsampling."));
    System.out.printf(
        Locale.ROOT,
        "Quality=%s; downsample=%d; largest processed image=%dx%d; SGM budget=%.0f MiB, feasible"
            + " disparities up to %d; potential depth evidence=%.0f MiB%n",
        o.quality,
        o.qualityDownsample(),
        longest,
        shortest,
        budget / 1048576.,
        limit,
        evidence / 1048576.);
    for (int d : counts) {
      long sgm = CensusSgm.memoryEstimate(longest, shortest, d, o);
      boolean gpuSelected =
          d <= gpuLimit && (!o.compute.equals("auto") || maxPixels * d >= 1000000L);
      long resident =
          gpuSelected
              ? GpuStereo.plan(
                      GpuSgm.selectedDevice(o), GpuSgm.selectedGroup(o), longest, shortest, d, o)
                  .hostBytes()
              : o.tiled ? CensusSgm.tiledMemoryEstimate(longest, shortest, d, o) : sgm;
      System.out.printf(
          Locale.ROOT,
          "  %4d disparities: SGM resident %.0f MiB; known process components ~%.0f MiB%s%n",
          d,
          resident / 1048576.,
          (resident + residentEvidence + imagesNative + features + decodeNative + rectifyNative)
              / 1048576.,
          resident > budget || d > gpuLimit && !o.tiled && maxPixels * d > Integer.MAX_VALUE - 8L
              ? " [exceeds SGM budget/array limit]"
              : "");
      if (gpuSelected) {
        Options scenario = o.copy();
        scenario.disparities = d;
        GpuStereo.Plan plan =
            GpuStereo.plan(
                GpuSgm.selectedDevice(o), GpuSgm.selectedGroup(o), longest, shortest, d, scenario);
        System.out.printf(
            Locale.ROOT,
            "       GPU %s working set ~%.0f MiB; temporary aggregate disk ~%.0f MiB%n",
            plan.streamed() ? "streamed " + plan.aggregateStorage() : "banked resident",
            plan.deviceBytes() / 1048576.,
            plan.aggregateStorage().equals("disk") ? plan.aggregateBytes() / 1048576. : 0.);
      } else if (o.tiled)
        System.out.printf(
            Locale.ROOT,
            "       temporary cost/aggregate disk ~%.0f MiB%n",
            maxPixels * d * (2 + StereoCost.aggregateBytes(o)) / 1048576.);
    }
    System.out.println(
        "Estimates exclude additional native/temporary allocations; see"
            + " 00_inputs/resolution_plan.json.");
    if (minimumSide < o.census + 2)
      throw new IllegalArgumentException(
          "Selected quality leaves an image too small for the census window; choose a higher"
              + " quality");
    if (!o.estimateMemory && (limit < 3 || o.disparitiesExplicit && o.disparities > limit))
      throw new IllegalArgumentException(
          "Selected quality/resolution cannot fit the "
              + (o.disparitiesExplicit ? o.disparities : "minimum")
              + " disparity search in the SGM budget. Choose lower --quality/--max-dimension or"
              + " raise --memory-mb and heap; see 00_inputs/resolution_plan.json");
  }
}
