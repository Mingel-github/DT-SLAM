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
    cv::Mat Tcw;
    long unsigned int frameId = 0;
    double timestampSeconds = 0.0;
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
    double totalMs = 0.0;
};

struct GeometricMultiReferenceResult
{
    // CV_8UC1 vote counts. Zero comparisonCount means no geometry evidence.
    cv::Mat comparisonCount;
    cv::Mat positiveCount;
    cv::Mat negativeCount;
    cv::Mat consistentCount;
    std::vector<GeometricPerReferenceStats> perReference;
    GeometricMultiReferenceStats stats;
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
        const float residualThresholdMeters);

    // G2-2R pure selection helper. Candidate frame ids must already be
    // ordered by the caller's reference policy. Missing cache entries remain
    // unavailable and are never replaced by a different reference.
    static GeometricReferenceSelectionResult SelectCachedReferences(
        const std::vector<GeometricReferenceFrame> &cachedReferences,
        const std::vector<long unsigned int> &orderedCandidateFrameIds,
        const std::size_t maximumReferences);

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
