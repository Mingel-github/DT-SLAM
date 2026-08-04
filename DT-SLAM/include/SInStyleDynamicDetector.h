/**
 * SIn-style region evidence interface for DT-SLAM.
 *
 * S1 is shadow-only. This interface deliberately does not accept Frame,
 * MapPoint, semantic masks, or camera poses, so it cannot mutate SLAM state.
 */

#ifndef SIN_STYLE_DYNAMIC_DETECTOR_H
#define SIN_STYLE_DYNAMIC_DETECTOR_H

#include <cstddef>
#include <string>
#include <vector>

#include <opencv2/core/core.hpp>

namespace ORB_SLAM2
{

struct SInStyleRegionStats
{
    int label = -1;
    std::size_t labelPixels = 0;
    std::size_t depthSupportedPixels = 0;
    std::size_t staticPixels = 0;
    std::size_t dynamicPixels = 0;
};

struct SInStyleRuntimeStats
{
    double loadMs = 0.0;
    double stateConversionMs = 0.0;
    double regionStatisticsMs = 0.0;
    double totalMs = 0.0;
};

struct SInStyleShadowStats
{
    std::size_t pixelCount = 0;
    std::size_t depthValidPixels = 0;
    std::size_t rawUnknownPixels = 0;
    std::size_t rawStaticPixels = 0;
    std::size_t rawDynamicPixels = 0;
    std::size_t validPixels = 0;
    std::size_t staticPixels = 0;
    std::size_t dynamicPixels = 0;
    std::size_t unknownPixels = 0;
    // Derived from the complete reference label image. Label zero is an
    // invalid/unassigned bucket, not evidence of static background.
    std::size_t positiveLabelCount = 0;
    std::size_t positiveLabelPixels = 0;
    std::size_t depthSupportedPositiveLabelPixels = 0;
    std::size_t positiveLabelComponentCount = 0;
    std::size_t authorDynamicPixelsOnPositiveLabels = 0;
    std::size_t authorDynamicPixelsOnLabelZero = 0;
    std::size_t authorDynamicPixelsWithLabelsUnavailable = 0;
    bool referenceAvailable = false;
    bool labelsAvailable = false;
    bool regionValidityAvailable = false;
};

struct SInStyleShadowResult
{
    // Original SIn reference convention: 0 unknown, 125 static, 255 dynamic.
    cv::Mat rawStateMask; // CV_8UC1

    // Raw reference codes and DT-SLAM depth support remain separate. The
    // public SIn tracking rule consumes authorDynamicMask directly.
    cv::Mat referenceKnownCodeMask;
    cv::Mat referenceUnknownMask;
    cv::Mat inputDepthValidMask;
    // Exact author imgTotalArea domain when replayed. It is independent of
    // positive region labels and of DT-SLAM's finite-depth validity.
    cv::Mat referenceRegionValidMask;

    // Project-level geometry evidence. All masks are CV_8UC1 with 0/255.
    cv::Mat validMask;
    cv::Mat staticMask;
    // Exact author tracking candidate before DT-SLAM depth-validity modeling.
    cv::Mat authorDynamicMask;
    // Dynamic evidence supported by both author state and valid current depth.
    cv::Mat dynamicMask;
    cv::Mat unknownMask;

    // CV_32SC1. -1 means unavailable; zero is the reference
    // invalid/unassigned label bucket; positive values are region labels.
    cv::Mat regionLabels;

    std::vector<SInStyleRegionStats> regions;
    SInStyleShadowStats stats;
    SInStyleRuntimeStats runtime;
};

struct SInStyleDetectorConfig
{
    bool enabled = false;
    std::string backend = "reference_replay";
    std::string referenceDirectory;
    std::string referenceMaskSuffix = "_mask_final.png";
    std::string referenceRegionValidSuffix = "_region_valid.png";
    bool requireLabels = true;
    bool requireRegionValidity = false;
};

class SInStyleDynamicDetector
{
public:
    SInStyleDynamicDetector();
    explicit SInStyleDynamicDetector(const SInStyleDetectorConfig &config);

    void Configure(const SInStyleDetectorConfig &config);
    void Reset();

    SInStyleShadowResult Process(const cv::Mat &image,
                                 const cv::Mat &depthMeters,
                                 long unsigned int frameId,
                                 double timestampSeconds) const;

    // Public for deterministic in-memory tests and future native backends.
    static SInStyleShadowResult ConvertReferenceState(
        const cv::Mat &rawStateMask,
        const cv::Mat &regionLabels,
        const cv::Mat &depthMeters);

private:
    std::string FrameStem(long unsigned int frameId) const;

    SInStyleDetectorConfig mConfig;
};

} // namespace ORB_SLAM2

#endif // SIN_STYLE_DYNAMIC_DETECTOR_H
