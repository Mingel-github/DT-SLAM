#ifndef SIN_STYLE_DENSE_FLOW_RESIDUAL_ESTIMATOR_H
#define SIN_STYLE_DENSE_FLOW_RESIDUAL_ESTIMATOR_H

#include <opencv2/core/core.hpp>

#include <cstddef>
#include <string>

namespace ORB_SLAM2
{

struct SInStyleDenseFlowResidualConfig
{
    bool enabled = false;
    std::string backend = "reference_replay";
    std::string referenceDirectory;
    bool requireReference = true;
    bool useTemporalRegionPrior = false;
    double residualRecomputeTolerancePx = 1e-4;
    int normalizedResidualTolerance = 1;
};

struct SInStyleDenseFlowResidualStats
{
    bool enabled = false;
    bool available = false;
    bool dynamicStateAvailable = false;
    std::string failureReason = "disabled";
    std::string backend;
    std::size_t frameIndex = 0;
    int intendedReferenceLag = 0;
    int referenceIndex = -1;
    int actualReferenceLag = 0;
    bool largeMotion = false;
    double imageScale = 0.0;
    int homographySampleCount = 0;
    bool temporalRegionPriorUsed = false;
    int temporalUnknownSamples = 0;
    int temporalStaticSamples = 0;
    int temporalDynamicSamples = 0;
    double maxObservedFlowPx = 0.0;
    double maxResidualPx = 0.0;
    double lowThresholdU8 = 0.0;
    double highThresholdU8 = 0.0;
    double otsuThresholdU8 = 0.0;
    double triangleThresholdU8 = 0.0;
    double lowThresholdPx = 0.0;
    double highThresholdPx = 0.0;
    std::size_t lowPixels = 0;
    std::size_t highPixels = 0;
    double residualRecomputeMaxAbsPx = 0.0;
    int normalizedRecomputeMaxAbs = 0;
    double loadMs = 0.0;
    double validateMs = 0.0;
    double totalMs = 0.0;
};

struct SInStyleDenseFlowResidualResult
{
    cv::Mat rawBackendFlowNative;
    cv::Mat observedFlowFull;
    cv::Mat homographyCurrentToReference;
    cv::Mat inducedFlowFull;
    cv::Mat residualFlowFull;
    cv::Mat residualMagnitudePx;
    cv::Mat normalizedResidual;
    // Source-compatible name: this is actually residual > low threshold,
    // i.e. high plus intermediate/uncertain support, not low residual.
    cv::Mat lowResidualMask;
    cv::Mat highResidualMask;
    SInStyleDenseFlowResidualStats stats;
};

class SInStyleDenseFlowResidualEstimator
{
public:
    void Configure(const SInStyleDenseFlowResidualConfig &config);
    void Reset();
    void CommitTemporalRegionPrior(
        std::size_t frameIndex,
        const cv::Mat &detectorStateMask,
        const cv::Mat &regionLabels);
    SInStyleDenseFlowResidualResult Process(
        std::size_t frameIndex,
        const cv::Mat &currentGray = cv::Mat());

private:
    SInStyleDenseFlowResidualResult ProcessReferenceReplay(
        std::size_t frameIndex) const;
    SInStyleDenseFlowResidualResult ProcessNativeDeepFlow(
        std::size_t frameIndex, const cv::Mat &currentGray);
    void CacheNativeFrame(std::size_t frameIndex,
                          const cv::Mat &currentGray);
    static cv::Mat ReadFlowFile(const std::string &path);
    static std::string FrameStem(const std::string &directory,
                                 std::size_t frameIndex);

    SInStyleDenseFlowResidualConfig mConfig;
    cv::Mat mNativeLastGray;
    cv::Mat mNativeSecondLastGray;
    std::size_t mNativeLastFrameIndex = 0;
    std::size_t mNativeSecondLastFrameIndex = 0;
    bool mbNativeLastFrameValid = false;
    bool mbNativeSecondLastFrameValid = false;
    cv::Mat mTemporalDetectorStateMask;
    cv::Mat mTemporalRegionLabels;
    std::size_t mTemporalPriorFrameIndex = 0;
    bool mbTemporalRegionPriorValid = false;
};

} // namespace ORB_SLAM2

#endif
