/**
 * Clean-room SIn-style plane-edge region split.
 *
 * The SInDSLAM paper uses PEAC plane contours. This component deliberately
 * uses OpenCV RgbdPlane instead, then applies the paper-described gradient
 * endpoint support rule. It is an S1 shadow measurement only.
 */

#ifndef SIN_STYLE_PLANE_EDGE_REGION_SPLITTER_H
#define SIN_STYLE_PLANE_EDGE_REGION_SPLITTER_H

#include <cstddef>

#include <opencv2/core/core.hpp>

namespace ORB_SLAM2
{

struct SInStylePlaneEdgeSplitConfig
{
    bool enabled = false;
    float maximumDepthMeters = 6.0f;
    int blockSize = 16;
    int minimumPlanePixels = 2000;
    double distanceThresholdMeters = 0.01;
    double sensorErrorA = 0.0075;
    double sensorErrorB = 0.0;
    double sensorErrorC = 0.0;
    int endpointRadius = 2;
    int endpointMaximumSupportExclusive = 5;
    int endpointAssociationRadius = 2;
    int minimumEndpointCountExclusive = 1;
    int connectivity = 8;
};

struct SInStylePlaneEdgeSplitStats
{
    bool enabled = false;
    bool available = false;
    bool dynamicStateAvailable = false;
    bool opencvPlaneSubstitute = true;
    std::size_t imagePixels = 0;
    std::size_t inputDepthValidPixels = 0;
    std::size_t initialRegionPixels = 0;
    std::size_t planePixels = 0;
    int planeCount = 0;
    std::size_t rawPlaneBoundaryPixels = 0;
    std::size_t gradientOverlapPixels = 0;
    std::size_t planeCandidateBoundaryPixels = 0;
    std::size_t gradientEndpointPixels = 0;
    int planeBoundarySegmentCount = 0;
    int retainedPlaneBoundarySegmentCount = 0;
    int unsupportedPlaneBoundarySegmentCount = 0;
    std::size_t retainedPlaneBoundaryPixels = 0;
    std::size_t combinedEdgePixels = 0;
    std::size_t combinedCorePixels = 0;
    int initialRegionCount = 0;
    int combinedComponentCount = 0;
    int splitInitialRegionCount = 0;
    int fullyConsumedInitialRegionCount = 0;
    int maximumFragmentation = 0;
    double planeExtractionMs = 0.0;
    double boundaryBuildMs = 0.0;
    double endpointFilterMs = 0.0;
    double connectedComponentsMs = 0.0;
    double totalMs = 0.0;
};

struct SInStylePlaneEdgeSplitResult
{
    // CV_32SC1: -1 non-plane/invalid, >=0 OpenCV plane id.
    cv::Mat planeLabels;
    cv::Mat rawPlaneBoundaryMask;
    cv::Mat gradientEndpointMask;
    cv::Mat planeCandidateBoundaryMask;
    cv::Mat retainedPlaneBoundaryMask;
    cv::Mat combinedEdgeMask;
    // CV_32SC1: -1 invalid/unmeasured, 0 boundary, >0 component.
    cv::Mat combinedCoreLabels;
    cv::Mat combinedValidMask;
    SInStylePlaneEdgeSplitStats stats;
};

class SInStylePlaneEdgeRegionSplitter
{
public:
    SInStylePlaneEdgeRegionSplitter();

    void Configure(const SInStylePlaneEdgeSplitConfig &config,
                   const cv::Mat &cameraMatrix);

    SInStylePlaneEdgeSplitResult Compute(
        const cv::Mat &depthMeters,
        const cv::Mat &initialLabels,
        const cv::Mat &rawGradientEdgeMask) const;

    // Deterministic seam used by synthetic tests and offline audits. The
    // supplied plane labels use -1 for non-plane/invalid and >=0 for planes.
    SInStylePlaneEdgeSplitResult ComputeFromPlaneLabels(
        const cv::Mat &depthMeters,
        const cv::Mat &initialLabels,
        const cv::Mat &rawGradientEdgeMask,
        const cv::Mat &planeLabels) const;

private:
    SInStylePlaneEdgeSplitConfig mConfig;
    cv::Mat mCameraMatrix;
};

} // namespace ORB_SLAM2

#endif // SIN_STYLE_PLANE_EDGE_REGION_SPLITTER_H
