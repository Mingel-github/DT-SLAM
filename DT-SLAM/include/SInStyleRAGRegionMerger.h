/**
 * Clean-room gradient-only adaptation of the SInDSLAM RAG merge.
 *
 * The output is a region representation only. It does not classify any
 * region or pixel as static or dynamic.
 */

#ifndef SIN_STYLE_RAG_REGION_MERGER_H
#define SIN_STYLE_RAG_REGION_MERGER_H

#include <cstddef>

#include <opencv2/core/core.hpp>

namespace ORB_SLAM2
{

struct SInStyleRAGMergeConfig
{
    bool enabled = false;
    float maximumDepthMeters = 6.0f;
    int adjacencyDilationRadius = 3;
    int histogramBins = 256;
    float adjacencyThresholdPixels = 200.0f;
    float areaDepthScoreWeight = 0.05f;
    float fakeEdgeWeight = 0.01f;
    float largeRegionWeight = 0.7f;
    float middleRegionWeight = 1.0f;
    float smallRegionWeight = 2.0f;
    float mergeThreshold = 0.9f;
    float depthRejectThreshold = 0.2f;
    float highMiddleFraction = 0.7f;
    float largeFraction = 0.5f;
    float smallFraction = 0.7f;
};

struct SInStyleRAGMergeStats
{
    bool enabled = false;
    bool available = false;
    bool dynamicStateAvailable = false;
    bool planeRejectionAvailable = false;
    std::size_t imagePixels = 0;
    std::size_t inputCorePixels = 0;
    std::size_t outputCorePixels = 0;
    int inputComponentCount = 0;
    int outputRegionCount = 0;
    std::size_t totalPairCount = 0;
    // Initial-component graph counts. Merge-stage scores are recomputed from
    // current group attributes and are summarized separately by merge counts.
    std::size_t spatialAdjacentPairCount = 0;
    std::size_t sharedFakeEdgePairCount = 0;
    std::size_t depthRejectedPairCount = 0;
    std::size_t eligiblePairCount = 0;
    int highMiddleMergeCount = 0;
    int lowScoreMergeCount = 0;
    int unmergedLowScoreRegionCount = 0;
    int crossGradientMergeViolationCount = 0;
    double meanHistogramSimilarityOnAdjacentPairs = 0.0;
    double maximumHistogramSimilarityOnAdjacentPairs = 0.0;
    double meanTotalScoreOnEligiblePairs = 0.0;
    double maximumTotalScoreOnEligiblePairs = 0.0;
    double medianMergedGroupComponents = 0.0;
    int maximumMergedGroupComponents = 0;
    std::size_t smallestMergedRegionPixels = 0;
    std::size_t largestMergedRegionPixels = 0;
    double attributeMs = 0.0;
    double ragMs = 0.0;
    double mergeMs = 0.0;
    double totalMs = 0.0;
};

struct SInStyleRAGMergeResult
{
    // CV_32SC1: -1 invalid/unmeasured, 0 retained boundary, >0 region.
    cv::Mat mergedLabels;
    cv::Mat mergedValidMask;
    SInStyleRAGMergeStats stats;
};

class SInStyleRAGRegionMerger
{
public:
    SInStyleRAGRegionMerger();

    void Configure(const SInStyleRAGMergeConfig &config);

    SInStyleRAGMergeResult Compute(
        const cv::Mat &depthMeters,
        const cv::Mat &initialLabels,
        const cv::Mat &splitCoreLabels,
        const cv::Mat &realEdgeMask) const;

private:
    SInStyleRAGMergeConfig mConfig;
};

} // namespace ORB_SLAM2

#endif // SIN_STYLE_RAG_REGION_MERGER_H
