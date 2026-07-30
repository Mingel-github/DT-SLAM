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

struct GeometricFeatureEvidenceSample
{
    std::size_t featureIndex = 0;
    int imageU = -1;
    int imageV = -1;
    int nativeU = -1;
    int nativeV = -1;
    int nativeScale = 1;
    unsigned char comparisonCount = 0;
    unsigned char positiveCount = 0;
    unsigned char negativeCount = 0;
    unsigned char consistentCount = 0;
};

enum class GeometricSparseFlowEvidenceState
{
    Measured,
    LkInvalid,
    DepthInvalid,
    ProjectionInvalid,
    DomainInvalid,
    ReferenceUnavailable
};

struct GeometricSparseFlowSample
{
    std::size_t featureIndex = 0;
    cv::Point2f currentPixel;
    cv::Point2f referencePixel;
    cv::Point2f forwardBackPixel;
    float backwardLkError = 0.0f;
    float forwardLkError = 0.0f;
    float forwardBackwardErrorPixels = 0.0f;
    float referenceDepthMeters = 0.0f;
    cv::Point2f slamEgoPixel;
    cv::Point2f slamResidualPixels;
    float slamResidualMagnitudePixels = 0.0f;
    cv::Point2f groundTruthEgoPixel;
    cv::Point2f groundTruthResidualPixels;
    float groundTruthResidualMagnitudePixels = 0.0f;
    bool referenceDepthBoundaryWithinOnePixel = false;
    bool referenceDepthBoundaryWithinTwoPixels = false;
    bool referenceInvalidDepthWithinOnePixel = false;
    bool referenceInvalidDepthWithinTwoPixels = false;
    bool backwardLkValid = false;
    bool forwardLkValid = false;
    bool referenceDepthValid = false;
    bool slamProjectionValid = false;
    bool groundTruthPoseAvailable = false;
    bool groundTruthProjectionValid = false;
    GeometricSparseFlowEvidenceState evidenceState =
        GeometricSparseFlowEvidenceState::ReferenceUnavailable;
};

struct GeometricSparseFlowStats
{
    std::size_t featureCount = 0;
    std::size_t backwardLkValidCount = 0;
    std::size_t forwardLkValidCount = 0;
    std::size_t referenceDepthValidCount = 0;
    std::size_t slamResidualValidCount = 0;
    std::size_t groundTruthResidualValidCount = 0;
    double slamResidualMedianPixels = 0.0;
    double slamResidualP90Pixels = 0.0;
    double slamResidualP95Pixels = 0.0;
    double groundTruthResidualMedianPixels = 0.0;
    double groundTruthResidualP90Pixels = 0.0;
    double groundTruthResidualP95Pixels = 0.0;
    double backwardLkMs = 0.0;
    double forwardLkMs = 0.0;
    double depthAndProjectionMs = 0.0;
    double totalMs = 0.0;
};

struct GeometricSparseFlowResult
{
    std::vector<GeometricSparseFlowSample> samples;
    GeometricSparseFlowStats stats;
};

enum class GeometricRigidityNodeState
{
    Measured,
    SparseFlowInvalid,
    ForwardBackwardRejected,
    SemanticExcluded,
    CurrentDepthInvalid,
    UncertaintyInvalid,
    OutsideImage,
    DuplicateImagePoint,
    NoGraphEdge
};

struct GeometricRigidityNodeSample
{
    std::size_t featureIndex = 0;
    cv::Point2f currentPixel;
    cv::Point2f referencePixel;
    cv::Point3f currentPointMeters;
    cv::Point3f referencePointMeters;
    // G2-4F3U axial depth uncertainty from the fixed Kinect-style
    // depth-square model and the 3x3 Gaussian depth mixture. These are
    // continuous shadow measurements, not confidence or motion labels.
    float currentDepthUncertaintyStdMeters = 0.0f;
    float referenceDepthUncertaintyStdMeters = 0.0f;
    float currentDepthNeighborhoodValidWeight = 0.0f;
    float referenceDepthNeighborhoodValidWeight = 0.0f;
    float forwardBackwardErrorPixels = 0.0f;
    float flowResidualMagnitudePixels = 0.0f;
    std::size_t validNeighborCount = 0;
    float incidentAbsoluteStrainMedianMeters = 0.0f;
    float incidentAbsoluteStrainP90Meters = 0.0f;
    float incidentRelativeStrainMedian = 0.0f;
    float incidentRelativeStrainP90 = 0.0f;
    float incidentUncertaintyNormalizedStrainMedian = 0.0f;
    float incidentUncertaintyNormalizedStrainP90 = 0.0f;
    GeometricRigidityNodeState state =
        GeometricRigidityNodeState::SparseFlowInvalid;
};

struct GeometricRigidityEdgeSample
{
    std::size_t featureIndexA = 0;
    std::size_t featureIndexB = 0;
    float referenceDistanceMeters = 0.0f;
    float currentDistanceMeters = 0.0f;
    float absoluteStrainMeters = 0.0f;
    float relativeStrain = 0.0f;
    float deltaLengthUncertaintyStdMeters = 0.0f;
    float uncertaintyNormalizedStrain = 0.0f;
    float flowResidualMagnitudePixelsA = 0.0f;
    float flowResidualMagnitudePixelsB = 0.0f;
    float forwardBackwardErrorPixelsA = 0.0f;
    float forwardBackwardErrorPixelsB = 0.0f;
};

struct GeometricRigidityStats
{
    std::size_t inputFeatureCount = 0;
    std::size_t sparseFlowMeasuredCount = 0;
    std::size_t forwardBackwardRejectedCount = 0;
    std::size_t semanticExcludedCount = 0;
    std::size_t currentDepthInvalidCount = 0;
    std::size_t uncertaintyInvalidCount = 0;
    std::size_t outsideImageCount = 0;
    std::size_t duplicateImagePointCount = 0;
    std::size_t eligibleNodeCount = 0;
    std::size_t nodeWithEdgeCount = 0;
    std::size_t validEdgeCount = 0;
    std::size_t uncertaintyNormalizedEdgeCount = 0;
    std::size_t uncertaintyFloorUseCount = 0;
    // Khoshelham and Elberink (2012), converted to z in meters. This is a
    // literature-model transfer for shadow diagnostics, not a per-device
    // calibration for the TUM/Bonn cameras.
    float axialDepthNoiseCoefficientPerMeter = 0.001425f;
    float uncertaintyDenominatorFloorMeters = 1e-6f;
    double graphMs = 0.0;
    double metricMs = 0.0;
    double totalMs = 0.0;
};

struct GeometricRigidityResult
{
    std::vector<GeometricRigidityNodeSample> nodes;
    std::vector<GeometricRigidityEdgeSample> edges;
    GeometricRigidityStats stats;
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

    // G2-4F0 shadow-only mapping from raw RGB/depth-domain feature centers
    // to existing multi-reference evidence. Pyramid-expanded cells are
    // sampled in their native domain so a 2x2 expansion remains one
    // measurement.
    static std::vector<GeometricFeatureEvidenceSample>
        SampleMultiReferenceEvidenceAtFeatures(
            const GeometricMultiReferenceResult &evidence,
            const std::vector<cv::Point2f> &featurePixels);

    // G2-4F1 shadow-only adjacent-frame sparse observed-flow minus
    // camera-induced RGB-D/SE(3) flow. Raw LK validity and continuous
    // residuals are returned; no threshold or motion class is produced.
    static GeometricSparseFlowResult ComputeSparseEgoFlow(
        const cv::Mat &currentGray,
        const cv::Mat &referenceGray,
        const cv::Mat &referenceDepthMeters,
        const std::vector<cv::Point2f> &currentFeaturePixels,
        const cv::Mat &TcwReference,
        const cv::Mat &TcwCurrent,
        const cv::Mat &K,
        const cv::Mat &TcwGroundTruthReference = cv::Mat(),
        const cv::Mat &TcwGroundTruthCurrent = cv::Mat(),
        const float depthBoundaryRelativeThreshold = 0.025f,
        const float depthBoundaryAbsoluteThresholdMeters = 0.08f);

    static const char *SparseFlowEvidenceStateName(
        const GeometricSparseFlowEvidenceState state);

    // G2-4F3 shadow-only local rigidity measurement. Delaunay adjacency is
    // built in the current image and 3-D edge-length change is measured
    // between adjacent RGB-D frames. It emits no threshold or motion class.
    static GeometricRigidityResult ComputeLocalRigidity(
        const cv::Mat &referenceDepthMeters,
        const cv::Mat &currentDepthMeters,
        const cv::Mat &K,
        const GeometricSparseFlowResult &sparseFlow,
        const std::vector<unsigned char> &semanticDynamic =
            std::vector<unsigned char>(),
        const float maximumForwardBackwardErrorPixels = 0.25f,
        const float relativeDenominatorFloorMeters = 1e-4f,
        const float axialDepthNoiseCoefficientPerMeter = 0.001425f,
        const float uncertaintyDenominatorFloorMeters = 1e-6f);

    static const char *RigidityNodeStateName(
        const GeometricRigidityNodeState state);

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
