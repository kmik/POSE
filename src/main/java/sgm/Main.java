package sgm;

import java.io.*;
import java.nio.file.*;
import java.util.*;
import org.opencv.core.*;

public final class Main {
  private Main() {}

  public static void main(String[] args) {
    MemoryLauncher.main(args);
  }

  public static int run(String[] args) {
    Project project = null;
    Project.LogCapture capture = null;
    Throwable failure = null;
    int status = 1;
    try {
      Options options = Options.parse(args);
      if (options.help) {
        System.out.print(Options.help());
        return 0;
      }
      if (options.listGpus) {
        GpuSgm.list();
        return 0;
      }
      nu.pattern.OpenCV.loadLocally();
      if (options.analyzeRun != null) {
        SurfaceDiagnostics.analyze(options);
        return 0;
      }
      if (options.exportOrientations != null) {
        OrientationIO.exportRun(options);
        return 0;
      }
      Core.setNumThreads(options.cores);
      Core.setRNGSeed(1);
      project = Project.open(options, args);
      capture = project.capture();
      System.out.println("Project: " + project.root + "\nRun: " + project.id);
      MemoryLauncher.report(options);
      if (!options.calibrate) System.out.println(options.filteringSummary());
      if (options.calibrate) Calibration.calibrate(options, project);
      else
        try (Reconstruction reconstruction = new Reconstruction(options, project)) {
          reconstruction.run();
        }
      System.out.println(
          options.estimateMemory
              ? "Memory planning completed; no reconstruction performed."
              : "Reconstruction stages completed.");
      status = 0;
    } catch (IllegalArgumentException | IOException e) {
      System.err.println("photogrammetry: " + e.getMessage());
      failure = e;
      status = 2;
    } catch (Exception | LinkageError e) {
      System.err.println("photogrammetry failed: " + e.getMessage());
      e.printStackTrace(System.err);
      failure = e;
      status = 1;
    } finally {
      GpuSgm.closeSession();
      try {
        if (capture != null) capture.close();
      } catch (IOException e) {
        System.err.println("Cannot close project log: " + e);
        if (failure == null) failure = e;
        status = 2;
      }
      if (project != null)
        try {
          project.finish(status == 0, failure);
          System.out.println(
              "Project run " + (status == 0 ? "complete" : "failed") + ": " + project.run);
        } catch (IOException e) {
          System.err.println("Cannot finalize project artifacts: " + e);
          status = 2;
        }
    }
    return status;
  }

  static List<Path> images(Options o) throws IOException {
    Set<String> extensions =
        new HashSet<>(Arrays.asList(o.format.toLowerCase(Locale.ROOT).split(",")));
    List<Path> result = new ArrayList<>();
    try (java.util.stream.Stream<Path> files = Files.list(o.input)) {
      files
          .filter(Files::isRegularFile)
          .forEach(
              p -> {
                String n = p.getFileName().toString().toLowerCase(Locale.ROOT);
                int dot = n.lastIndexOf('.');
                if (dot >= 0 && extensions.contains(n.substring(dot + 1))) result.add(p);
              });
    }
    result.sort(Comparator.comparing(p -> p.getFileName().toString()));
    return result;
  }
}
