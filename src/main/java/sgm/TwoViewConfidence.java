package sgm;

import java.io.IOException;
import java.nio.file.Path;

/**
 * Experimental acceptance gate; final legacy confidence and 3+ camera acceptance stay unchanged.
 */
final class TwoViewConfidence {
  static final String MODEL = "sgm-bidirectional-two-view-v1";

  static double effective(double requested) {
    return Math.max(PointConfidence.TWO_VIEW_MINIMUM, requested);
  }

  static DepthFusion.Result apply(
      DepthFusion.Result result, double left, double right, double requested) {
    result.rightMatchingConfidence = right;
    result.twoViewMatchingConfidence =
        PointConfidence.valid(left) && PointConfidence.valid(right)
            ? Math.min(left, right)
            : Double.NaN;
    if (!result.accepted() || result.views > 2) return result;
    String status = null;
    if (!PointConfidence.valid(left) || left < PointConfidence.TWO_VIEW_MINIMUM)
      status = "two_view_confidence_rejected";
    else if (Double.isFinite(requested)) {
      if (!PointConfidence.valid(right)) status = "two_view_right_confidence_unavailable";
      else if (Math.min(left, right) < effective(requested))
        status = "two_view_pair_confidence_rejected";
    }
    if (status == null) return result;
    var rejected =
        new DepthFusion.Result(
            result.point,
            result.views,
            result.independentPairs,
            result.relativeSigma,
            result.shift,
            status);
    rejected.supportingCameras = result.supportingCameras;
    rejected.rightMatchingConfidence = right;
    rejected.twoViewMatchingConfidence = result.twoViewMatchingConfidence;
    return rejected;
  }

  static final class Diagnostics {
    private final Options options;
    private long legacyRejected, unavailable, rejected, accepted, threePlus;
    private final long[] left = new long[21], right = new long[21], pair = new long[21];

    Diagnostics(Options options) {
      this.options = options;
    }

    void record(DepthFusion.Result r) {
      if (r.views > 2) {
        if (r.accepted()) threePlus++;
        return;
      }
      switch (r.status) {
        case "two_view_confidence_rejected" -> legacyRejected++;
        case "two_view_right_confidence_unavailable" -> unavailable++;
        case "two_view_pair_confidence_rejected" -> rejected++;
        case "accepted" -> accepted++;
        default -> {
          return;
        }
      }
      bin(left, r.matchingConfidence);
      bin(right, r.rightMatchingConfidence);
      bin(pair, r.twoViewMatchingConfidence);
    }

    private void bin(long[] bins, double score) {
      if (PointConfidence.valid(score)) bins[Math.min(20, (int) (score * 20))]++;
    }

    long additionalRejected() {
      return unavailable + rejected;
    }

    void write(Path output) throws IOException {
      Json.write(
          output.resolve("06_fusion/two_view_confidence.json"),
          Json.object(
              "model",
              MODEL,
              "experimental",
              true,
              "enabled",
              PointConfidence.bidirectional(options),
              "requested_threshold",
              options.twoViewConfidenceThreshold,
              "threshold",
              PointConfidence.bidirectional(options)
                  ? effective(options.twoViewConfidenceThreshold)
                  : null,
              "effective_threshold",
              PointConfidence.bidirectional(options)
                  ? effective(options.twoViewConfidenceThreshold)
                  : null,
              "existing_left_matching_floor",
              PointConfidence.TWO_VIEW_MINIMUM,
              "legacy_left_rejected",
              legacyRejected,
              "right_unavailable_rejected",
              unavailable,
              "pair_threshold_rejected",
              rejected,
              "additional_rejected",
              additionalRejected(),
              "two_view_accepted",
              accepted,
              "three_plus_accepted_unchanged",
              threePlus,
              "histogram_bin_width",
              .05,
              "left_score_histogram",
              left,
              "right_score_histogram",
              right,
              "pair_score_histogram",
              pair,
              "comparison",
              "minimum of left/right matching scores; equality passes",
              "scope",
              "Acceptance gate only for exactly two supporting cameras. Existing final confidence"
                  + " formula and 3+ view rules remain unchanged. Histograms cover points reaching"
                  + " a two-view confidence gate, not geometric failures. Missing right evidence"
                  + " rejects two-view points when enabled; no extra SGM passes."));
    }
  }
}
