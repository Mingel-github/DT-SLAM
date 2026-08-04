/**
 * Clean-room SIn-style initial 3D region partition.
 *
 * This is an S1 shadow provider, not a dynamic detector. It temporarily
 * reuses the project's tested JiDepth K-means implementation only for XYZ
 * clustering; no Ji reprojection decision is used.
 */

#ifndef SIN_STYLE_INITIAL_REGION_CLUSTERER_H
#define SIN_STYLE_INITIAL_REGION_CLUSTERER_H

#include "JiGeometryBaseline.h"

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include <opencv2/core/core.hpp>

namespace ORB_SLAM2
{

struct SInStyleInitialRegionConfig
{
    bool enabled = false;
    double clusterPixelDivisor = 25600.0;
    float maximumDepthMeters = 6.0f;
    int maximumIterations = 20;
    double epsilon = 1e-3;
    int attempts = 1;
    std::uint64_t randomSeed = 2025;
    bool coarseToFine = false;
    int pyramidLevels = 4;
    bool temporalInitialization = true;
    long unsigned int temporalCommitStartInputIndex = 0;
};

struct SInStyleInitialRegionLevelStats
{
    int level = 0;
    int rows = 0;
    int cols = 0;
    std::size_t validSamples = 0;
    std::size_t priorInitializedSamples = 0;
    std::size_t gridFallbackSamples = 0;
    double compactness = 0.0;
    double prepareMs = 0.0;
    double kmeansMs = 0.0;
    double labelMs = 0.0;
};

struct SInStyleInitialRegionStats
{
    bool enabled = false;
    bool available = false;
    bool dynamicStateAvailable = false;
    std::size_t imagePixels = 0;
    std::size_t inputDepthValidPixels = 0;
    std::size_t clusteringDepthValidPixels = 0;
    std::size_t excludedFarDepthPixels = 0;
    int requestedClusters = 0;
    int producedClusters = 0;
    std::size_t smallestRegionPixels = 0;
    std::size_t largestRegionPixels = 0;
    double compactness = 0.0;
    double prepareMs = 0.0;
    double kmeansMs = 0.0;
    double labelConversionMs = 0.0;
    double totalMs = 0.0;
    bool coarseToFine = false;
    int pyramidLevels = 1;
    std::string initializationSource = "from_scratch";
    std::size_t previousPriorSamples = 0;
    std::size_t gridFallbackSamples = 0;
    double previousPriorCoverage = 0.0;
    bool temporalPriorCommitted = false;
    std::vector<SInStyleInitialRegionLevelStats> levels;
};

struct SInStyleInitialRegionResult
{
    // CV_32SC1: -1 invalid/excluded, positive values are initial regions.
    cv::Mat labels;
    // CV_8UC1: 255 only where labels are positive.
    cv::Mat validMask;
    SInStyleInitialRegionStats stats;
};

class SInStyleInitialRegionClusterer
{
public:
    SInStyleInitialRegionClusterer();

    void Configure(const SInStyleInitialRegionConfig &config,
                   const cv::Mat &cameraMatrix);
    void Reset();

    SInStyleInitialRegionResult Compute(
        const cv::Mat &depthMeters,
        long unsigned int inputIndex = 0);

private:
    // Temporal state is confined to the Tracking thread; Compute and Reset
    // are not intended for concurrent calls.
    SInStyleInitialRegionConfig mConfig;
    cv::Mat mCameraMatrix;
    cv::Mat mPreviousInitialLabels;
    long unsigned int mPreviousInputIndex = 0;
    bool mbHasPreviousInitialLabels = false;
};

} // namespace ORB_SLAM2

#endif // SIN_STYLE_INITIAL_REGION_CLUSTERER_H
