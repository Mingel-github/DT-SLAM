/**
 * Clean-room SIn-style gradient-depth region split.
 *
 * This S1 component applies the depth-discontinuity test described by the
 * SInDSLAM paper inside an existing initial partition. It exposes region
 * evidence only: no region is classified as static or dynamic.
 */

#ifndef SIN_STYLE_GRADIENT_REGION_SPLITTER_H
#define SIN_STYLE_GRADIENT_REGION_SPLITTER_H

#include <cstddef>

#include <opencv2/core/core.hpp>

namespace ORB_SLAM2
{

struct SInStyleGradientSplitConfig
{
    bool enabled = false;
    float maximumDepthMeters = 6.0f;
    float relativeThreshold = 0.025f;
    float absoluteThresholdMeters = 0.08f;
    int medianRadius = 2;
    int minimumMedianSupport = 5;
    int connectivity = 8;
    std::size_t smallComponentAuditPixels = 80;
};

struct SInStyleGradientSplitStats
{
    bool enabled = false;
    bool available = false;
    bool dynamicStateAvailable = false;
    std::size_t imagePixels = 0;
    std::size_t inputDepthValidPixels = 0;
    std::size_t initialRegionPixels = 0;
    std::size_t medianValidPixels = 0;
    std::size_t insufficientSupportPixels = 0;
    std::size_t rawGradientEdgePixels = 0;
    std::size_t splitBoundaryPixels = 0;
    std::size_t splitCorePixels = 0;
    int initialRegionCount = 0;
    int splitComponentCount = 0;
    int splitInitialRegionCount = 0;
    int fullyConsumedInitialRegionCount = 0;
    double medianFragmentation = 0.0;
    int maximumFragmentation = 0;
    int smallComponentCount = 0;
    std::size_t smallComponentPixels = 0;
    double medianFilterMs = 0.0;
    double gradientEdgeMs = 0.0;
    double connectedComponentsMs = 0.0;
    double totalMs = 0.0;
};

struct SInStyleGradientSplitResult
{
    // CV_32FC1 metric depth; zero denotes no filtered-depth evidence.
    cv::Mat filteredDepth;
    // CV_16UC1 number of valid samples used by the masked median.
    cv::Mat medianSupport;
    cv::Mat medianValidMask;
    cv::Mat insufficientSupportMask;
    cv::Mat rawGradientEdgeMask;
    cv::Mat splitBoundaryMask;
    // CV_32SC1: -1 invalid/unmeasured, 0 boundary, >0 split component.
    cv::Mat splitCoreLabels;
    cv::Mat splitValidMask;
    SInStyleGradientSplitStats stats;
};

class SInStyleGradientRegionSplitter
{
public:
    SInStyleGradientRegionSplitter();

    void Configure(const SInStyleGradientSplitConfig &config);

    SInStyleGradientSplitResult Compute(
        const cv::Mat &depthMeters,
        const cv::Mat &initialLabels) const;

private:
    SInStyleGradientSplitConfig mConfig;
};

} // namespace ORB_SLAM2

#endif // SIN_STYLE_GRADIENT_REGION_SPLITTER_H
