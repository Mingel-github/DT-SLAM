/**
* This file is part of DT-SLAM.
*
* G0-1 extracts single-reference RGB-D geometric inconsistency evidence.
* It does not classify dynamic pixels or modify SLAM state.
*/

#ifndef GEOMETRIC_DYNAMIC_DETECTOR_H
#define GEOMETRIC_DYNAMIC_DETECTOR_H

#include <cstddef>

#include <opencv2/core/core.hpp>

namespace ORB_SLAM2
{

struct GeometricWarpStats
{
    std::size_t referenceValidPixels = 0;
    std::size_t projectedSamples = 0;
    std::size_t zbufferValidPixels = 0;
    std::size_t currentValidPixels = 0;
    std::size_t validComparisons = 0;

    double predictionCoverageRatio = 0.0;
    double comparisonCoverageRatio = 0.0;
    double residualMean = 0.0;
    double residualMeanAbs = 0.0;
    double residualMaxAbs = 0.0;

    double warpMs = 0.0;
    double residualMs = 0.0;
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

    GeometricWarpStats stats;
};

class GeometricDynamicDetector
{
public:
    GeometricDynamicDetector();

    void SetCameraMatrix(const cv::Mat &K);

    void UpdateReference(const cv::Mat &depthMeters,
                         const cv::Mat &Tcw,
                         const long unsigned int frameId);

    void ResetReference();
    bool HasReference() const;
    long unsigned int ReferenceFrameId() const;

    bool Compute(const cv::Mat &currentDepthMeters,
                 const cv::Mat &TcwCurrent,
                 GeometricWarpResult &result) const;

    // Public pure computation entry point for deterministic G0-1 tests.
    static GeometricWarpResult ComputeWarp(const cv::Mat &referenceDepthMeters,
                                           const cv::Mat &currentDepthMeters,
                                           const cv::Mat &TcwReference,
                                           const cv::Mat &TcwCurrent,
                                           const cv::Mat &K);

private:
    cv::Mat mK;
    cv::Mat mReferenceDepthMeters;
    cv::Mat mTcwReference;
    long unsigned int mnReferenceFrameId;
    bool mbHasReference;
};

} // namespace ORB_SLAM2

#endif // GEOMETRIC_DYNAMIC_DETECTOR_H
