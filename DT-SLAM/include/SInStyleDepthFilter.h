#ifndef SIN_STYLE_DEPTH_FILTER_H
#define SIN_STYLE_DEPTH_FILTER_H

#include <opencv2/core/core.hpp>

#include <cstddef>
#include <string>

namespace ORB_SLAM2
{

struct SInStyleDepthFilterConfig
{
    bool enabled = false;
    std::string maskMode = "semantic_or_geometry";
};

struct SInStyleDepthFilterStats
{
    bool enabled = false;
    bool available = false;
    bool geometryEvidenceAvailable = false;
    std::string maskMode = "semantic_or_geometry";
    std::string state = "disabled";
    std::size_t inputValidDepthPixels = 0;
    std::size_t semanticDynamicPixels = 0;
    std::size_t geometryDynamicPixels = 0;
    std::size_t unionDynamicPixels = 0;
    std::size_t rejectedValidDepthPixels = 0;
    std::size_t outputValidDepthPixels = 0;
    double totalMs = 0.0;
};

struct SInStyleDepthFilterResult
{
    cv::Mat dynamicDepthMask;
    cv::Mat staticDepthMeters;
    SInStyleDepthFilterStats stats;
};

class SInStyleDepthFilter
{
public:
    void Configure(const SInStyleDepthFilterConfig &config);

    SInStyleDepthFilterResult Filter(
        const cv::Mat &depthMeters,
        const cv::Mat &semanticDynamicMask,
        const cv::Mat &geometryDynamicMask,
        bool geometryEvidenceAvailable) const;

private:
    SInStyleDepthFilterConfig mConfig;
};

} // namespace ORB_SLAM2

#endif
