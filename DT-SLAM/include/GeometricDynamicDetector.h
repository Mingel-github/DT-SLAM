/**
* This file is part of DT-SLAM.
*
* G0-1 computes single-reference RGB-D residuals, G0-2 classifies
* diagnostic evidence, and G0-3 forms depth-connected shadow candidates.
* None of these stages modifies SLAM state.
*/

#ifndef GEOMETRIC_DYNAMIC_DETECTOR_H
#define GEOMETRIC_DYNAMIC_DETECTOR_H

#include <cstddef>
#include <vector>

#include <opencv2/core/core.hpp>

namespace ORB_SLAM2
{

struct GeometricDepthRegionStats
{
    std::size_t pixels = 0;
    std::size_t positiveSeedPixels = 0;
    std::size_t negativeDiagnosticPixels = 0;
    double positiveSeedRatio = 0.0;
    double negativeDiagnosticRatio = 0.0;
    float signedResidualMedian = 0.0f;
};

struct GeometricWarpStats
{
    std::size_t referenceValidPixels = 0;
    std::size_t projectedSamples = 0;
    std::size_t zbufferValidPixels = 0;
    std::size_t currentValidPixels = 0;
    std::size_t validComparisons = 0;
    std::size_t consistentEvidencePixels = 0;
    std::size_t positiveSeedPixels = 0;
    std::size_t negativeDiagnosticPixels = 0;
    std::size_t depthRegionCount = 0;
    std::size_t regionCandidatePixels = 0;
    std::size_t largestRegionPixels = 0;

    double predictionCoverageRatio = 0.0;
    double comparisonCoverageRatio = 0.0;
    double consistentEvidenceRatio = 0.0;
    double positiveSeedRatio = 0.0;
    double negativeDiagnosticRatio = 0.0;
    double regionGrowthRatio = 0.0;
    double residualMean = 0.0;
    double residualMeanAbs = 0.0;
    double residualMaxAbs = 0.0;

    double warpMs = 0.0;
    double residualMs = 0.0;
    double evidenceMs = 0.0;
    double regionGrowMs = 0.0;
    double totalMs = 0.0;
};

struct GeometricWarpResult
{
    // CV_32FC1 meters. Zero means no projected reference surface.
    cv::Mat predictedDepth;

    // CV_8UC1. 255 means both predicted and current depth are valid.
    cv::Mat validComparisonMask;

    // CV_32FC1 meters. Read values only where validComparisonMask is non-zero.
    cv::Mat signedDepthResidual;

    // G0-2 masks are mutually exclusive and only valid where
    // validComparisonMask is non-zero.
    cv::Mat consistentEvidenceMask; // |residual| <= threshold
    cv::Mat positiveSeedMask;       // residual > threshold
    cv::Mat negativeDiagnosticMask; // residual < -threshold

    // G0-3 CV_8UC1 shadow output. 255 means the pixel is depth-connected
    // to at least one positive seed under the configured local threshold.
    cv::Mat regionCandidateMask;

    // G0-3R CV_32FC1 diagnostic image. Each grown region contains its
    // positiveSeedPixels / pixels support value in [0,1].
    cv::Mat regionPositiveSupport;
    std::vector<GeometricDepthRegionStats> depthRegions;

    GeometricWarpStats stats;
};

struct GeometricReferenceFrame
{
    cv::Mat depthMeters;
    // G2-3R3 cached boundary-preserving 2x depth-pyramid level.
    cv::Mat pyramidDepthMeters;
    cv::Mat Tcw;
    // G2-2S raw RGB/depth-domain ORB pixels with valid static depth.
    std::vector<cv::Point2i> featureDepthPixels;
    // G2-2G raw RGB/depth-domain regular-grid pixels with valid static depth.
    std::vector<cv::Point2i> gridDepthPixels;
    long unsigned int frameId = 0;
    double timestampSeconds = 0.0;
};

enum class GeometricReferenceSamplingPolicy
{
    Dense,
    OrbDepth,
    GridDepth,
    PyramidDense
};

struct GeometricReferenceSelectionStats
{
    std::size_t candidateCount = 0;
    std::size_t cachedReferenceMatchCount = 0;
    std::size_t selectedReferenceCount = 0;
};

struct GeometricReferenceSelectionResult
{
    std::vector<GeometricReferenceFrame> references;
    GeometricReferenceSelectionStats stats;
};

struct GeometricPerReferenceStats
{
    long unsigned int frameId = 0;
    GeometricWarpStats warp;
};

struct GeometricMultiReferenceStats
{
    std::size_t referenceCount = 0;
    std::size_t pixelsWithComparison = 0;
    std::size_t totalComparisons = 0;
    std::size_t pixelsWithPositiveEvidence = 0;
    std::size_t totalPositiveVotes = 0;
    std::size_t totalNegativeVotes = 0;
    std::size_t totalConsistentVotes = 0;
    double warpAndEvidenceMs = 0.0;
    double aggregateMs = 0.0;
    double preprocessMs = 0.0;
    double expandMs = 0.0;
    double totalMs = 0.0;
};

struct GeometricMultiReferenceResult
{
    // CV_8UC1 vote counts. Zero comparisonCount means no geometry evidence.
    cv::Mat comparisonCount;
    cv::Mat positiveCount;
    cv::Mat negativeCount;
    cv::Mat consistentCount;

    // G2-3R4 native pyramid-domain state. These fields are populated only
    // by ComputePyramidMultiReferenceEvidence. Each native count cell is one
    // measurement; the expanded full-resolution images above are only a
    // coordinate-domain compatibility view.
    cv::Mat nativeDepthMeters;
    cv::Mat nativeComparisonCount;
    cv::Mat nativePositiveCount;
    cv::Mat nativeNegativeCount;
    cv::Mat nativeConsistentCount;
    int nativeScale = 1;

    std::vector<GeometricPerReferenceStats> perReference;
    GeometricMultiReferenceStats stats;
};

struct GeometricRegionPartitionStats
{
    int domainScale = 1;
    std::size_t validDepthPixels = 0;
    std::size_t boundaryPixels = 0;
    std::size_t assignedRegionPixels = 0;
    std::size_t regionCount = 0;
    std::size_t largestRegionPixels = 0;
    std::size_t topFiveRegionPixels = 0;
    std::size_t singletonRegionCount = 0;
    std::size_t smallRegionCount = 0;
    double boundaryValidRatio = 0.0;
    double assignedValidRatio = 0.0;
    double largestRegionValidRatio = 0.0;
    double topFiveRegionValidRatio = 0.0;
    double totalMs = 0.0;
    double mappingMs = 0.0;
    double onlineTotalMs = 0.0;
};

struct GeometricRegionPartitionResult
{
    // CV_8UC1. 255 is a depth-discontinuity boundary.
    cv::Mat boundaryMask;

    // CV_32SC1. -1 is invalid depth, -2 is a boundary, and non-negative
    // values are connected region labels. No label is a motion decision.
    cv::Mat labels;
    std::vector<std::size_t> regionSizes;
    GeometricRegionPartitionStats stats;
};

struct GeometricRegionRiskBandStats
{
    std::size_t regionPixels = 0;
    std::size_t comparisonPixels = 0;
    std::size_t positivePresencePixels = 0;
    std::size_t comparisonVotes = 0;
    std::size_t positiveVotes = 0;
};

struct GeometricRegionEvidenceStats
{
    int regionLabel = -1;
    std::size_t regionPixels = 0;
    std::size_t semanticProxyPixels = 0;
    std::size_t semanticComparisonPixels = 0;
    std::size_t semanticPositivePresencePixels = 0;
    std::size_t semanticNegativePresencePixels = 0;
    std::size_t semanticConsistentPresencePixels = 0;
    std::size_t comparisonPixels = 0;
    std::size_t positivePresencePixels = 0;
    std::size_t negativePresencePixels = 0;
    std::size_t consistentPresencePixels = 0;
    std::size_t comparisonVotes = 0;
    std::size_t positiveVotes = 0;
    std::size_t negativeVotes = 0;
    std::size_t consistentVotes = 0;
    std::size_t singleReferenceComparisonPixels = 0;
    std::size_t multiReferenceComparisonPixels = 0;
    std::size_t singleReferencePositivePresencePixels = 0;
    std::size_t multiReferencePositivePresencePixels = 0;
    std::size_t unanimousPositivePixels = 0;
    GeometricRegionRiskBandStats boundaryWithinOnePixel;
    GeometricRegionRiskBandStats boundaryWithinTwoPixels;
    GeometricRegionRiskBandStats invalidWithinOnePixel;
    GeometricRegionRiskBandStats invalidWithinTwoPixels;
    double semanticProxyRegionRatio = 0.0;
    double semanticComparisonCoverage = 0.0;
    double semanticPositiveComparedPixelRatio = 0.0;
    double comparisonCoverage = 0.0;
    double positiveComparedPixelRatio = 0.0;
    double negativeComparedPixelRatio = 0.0;
    double consistentComparedPixelRatio = 0.0;
    double positiveVoteRatio = 0.0;
    double negativeVoteRatio = 0.0;
    double consistentVoteRatio = 0.0;
};

struct GeometricRegionEvidenceAggregationStats
{
    std::size_t regionCount = 0;
    std::size_t regionsWithComparison = 0;
    std::size_t regionsWithPositiveEvidence = 0;
    std::size_t regionPixels = 0;
    std::size_t comparisonPixels = 0;
    std::size_t comparisonVotes = 0;
    double totalMs = 0.0;
};

struct GeometricRegionEvidenceAggregationResult
{
    std::vector<GeometricRegionEvidenceStats> regions;
    GeometricRegionEvidenceAggregationStats stats;
};

class GeometricDynamicDetector
{
public:
    GeometricDynamicDetector();

    void SetCameraMatrix(const cv::Mat &K);
    void SetResidualThresholdMeters(const float thresholdMeters);
    float ResidualThresholdMeters() const;
    void SetRegionGrowEnabled(const bool enabled);
    bool RegionGrowEnabled() const;
    void SetRegionDepthThresholdMeters(const float thresholdMeters);
    float RegionDepthThresholdMeters() const;

    void UpdateReference(const cv::Mat &depthMeters,
                         const cv::Mat &Tcw,
                         const long unsigned int frameId,
                         const double timestampSeconds);

    void ResetReference();
    bool HasReference() const;
    long unsigned int ReferenceFrameId() const;
    double ReferenceTimestampSeconds() const;

    bool Compute(const cv::Mat &currentDepthMeters,
                 const cv::Mat &TcwCurrent,
                 GeometricWarpResult &result) const;

    // Public pure computation entry point for deterministic G0-1 tests.
    static GeometricWarpResult ComputeWarp(const cv::Mat &referenceDepthMeters,
                                           const cv::Mat &currentDepthMeters,
                                           const cv::Mat &TcwReference,
                                           const cv::Mat &TcwCurrent,
                                           const cv::Mat &K);

    // G0-2 evidence classification. This does not modify SLAM state.
    static void ClassifyEvidence(GeometricWarpResult &result,
                                 const float thresholdMeters);

    // G0-3 depth-continuity candidate generation. This is diagnostic only.
    static void GrowDepthRegions(const cv::Mat &currentDepthMeters,
                                 GeometricWarpResult &result,
                                 const float depthThresholdMeters);

    // G2-1 multi-reference evidence accumulation. This produces vote counts
    // only and deliberately makes no binary dynamic decision.
    static GeometricMultiReferenceResult ComputeMultiReferenceEvidence(
        const std::vector<GeometricReferenceFrame> &references,
        const cv::Mat &currentDepthMeters,
        const cv::Mat &TcwCurrent,
        const cv::Mat &K,
        const float residualThresholdMeters,
        const GeometricReferenceSamplingPolicy samplingPolicy =
            GeometricReferenceSamplingPolicy::Dense);

    // G2-3R3 boundary-preserving depth-pyramid primitives and shadow
    // evidence approximation. Expanded count cells are not independent
    // full-resolution measurements and must not drive SLAM decisions.
    static cv::Mat DownsampleDepthBoundaryAware(
        const cv::Mat &depthMeters,
        const int scale,
        const float relativeThreshold,
        const float absoluteThresholdMeters);

    static cv::Mat ScaleCameraMatrix(
        const cv::Mat &K,
        const int scale);

    // G2-3R4 semantic proxy projection into the native pyramid-cell domain.
    // A cell is non-zero when any source pixel in its scale-by-scale block is
    // non-zero. This is diagnostic-only and is not a dynamic decision.
    static cv::Mat DownsampleMaskAny(
        const cv::Mat &mask,
        const int scale);

    static GeometricMultiReferenceResult
        ComputePyramidMultiReferenceEvidence(
            const std::vector<GeometricReferenceFrame> &references,
            const cv::Mat &currentDepthMeters,
            const cv::Mat &TcwCurrent,
            const cv::Mat &K,
            const float residualThresholdMeters,
            const int scale,
            const float relativeThreshold,
            const float absoluteThresholdMeters);

    // G2-2R pure selection helper. Candidate frame ids must already be
    // ordered by the caller's reference policy. Missing cache entries remain
    // unavailable and are never replaced by a different reference.
    static GeometricReferenceSelectionResult SelectCachedReferences(
        const std::vector<GeometricReferenceFrame> &cachedReferences,
        const std::vector<long unsigned int> &orderedCandidateFrameIds,
        const std::size_t maximumReferences);

    // G2-3R0 shadow-only region representation. The depth boundary follows
    // the SInDSLAM relative-plus-absolute threshold form, while connected
    // components are a lightweight adaptation rather than a reproduction of
    // its full K-means, plane-edge, and re-clustering pipeline.
    static GeometricRegionPartitionResult PartitionDepthByDiscontinuity(
        const cv::Mat &depthMeters,
        const float relativeThreshold,
        const float absoluteThresholdMeters,
        const std::size_t smallRegionMaximumPixels = 64);

    // G2-3R1 shadow-only aggregation. It records evidence distributions
    // inside fixed regions and deliberately emits no dynamic decision.
    static GeometricRegionEvidenceAggregationResult
        AggregateMultiReferenceEvidenceByRegion(
            const GeometricRegionPartitionResult &partition,
            const GeometricMultiReferenceResult &evidence,
            const cv::Mat &semanticProxyMask = cv::Mat(),
            const bool collectRiskDiagnostics = false);

private:
    cv::Mat mK;
    cv::Mat mReferenceDepthMeters;
    cv::Mat mTcwReference;
    float mResidualThresholdMeters;
    float mRegionDepthThresholdMeters;
    long unsigned int mnReferenceFrameId;
    double mReferenceTimestampSeconds;
    bool mbHasReference;
    bool mbRegionGrowEnabled;
};

} // namespace ORB_SLAM2

#endif // GEOMETRIC_DYNAMIC_DETECTOR_H
