#include "SInStyleDepthFilter.h"

#include <opencv2/imgproc/imgproc.hpp>

#include <chrono>
#include <stdexcept>

namespace ORB_SLAM2
{

void SInStyleDepthFilter::Configure(
    const SInStyleDepthFilterConfig &config)
{
    if(config.maskMode!="semantic_only" &&
       config.maskMode!="geometry_only" &&
       config.maskMode!="semantic_or_geometry")
    {
        throw std::invalid_argument(
            "SInStyle.DepthFilterMaskMode must be semantic_only, "
            "geometry_only, or semantic_or_geometry");
    }
    mConfig = config;
}

SInStyleDepthFilterResult SInStyleDepthFilter::Filter(
    const cv::Mat &depthMeters,
    const cv::Mat &semanticDynamicMask,
    const cv::Mat &geometryDynamicMask,
    bool geometryEvidenceAvailable) const
{
    SInStyleDepthFilterResult result;
    result.stats.enabled = mConfig.enabled;
    result.stats.maskMode = mConfig.maskMode;
    result.stats.geometryEvidenceAvailable =
        geometryEvidenceAvailable;
    if(!mConfig.enabled)
        return result;

    const std::chrono::steady_clock::time_point start =
        std::chrono::steady_clock::now();
    if(depthMeters.empty() || depthMeters.type()!=CV_32FC1)
    {
        throw std::invalid_argument(
            "SIn depth filter requires a non-empty CV_32FC1 depth image");
    }
    const cv::Size imageSize = depthMeters.size();
    if(!semanticDynamicMask.empty() &&
       (semanticDynamicMask.type()!=CV_8UC1 ||
        semanticDynamicMask.size()!=imageSize))
    {
        throw std::invalid_argument(
            "SIn depth filter semantic mask must be CV_8UC1 and match depth");
    }
    if(!geometryDynamicMask.empty() &&
       (geometryDynamicMask.type()!=CV_8UC1 ||
        geometryDynamicMask.size()!=imageSize))
    {
        throw std::invalid_argument(
            "SIn depth filter geometry mask must be CV_8UC1 and match depth");
    }

    cv::Mat semanticMask(imageSize,CV_8UC1,cv::Scalar(0));
    if(!semanticDynamicMask.empty())
        cv::compare(semanticDynamicMask,0,semanticMask,cv::CMP_NE);
    cv::Mat geometryMask(imageSize,CV_8UC1,cv::Scalar(0));
    if(geometryEvidenceAvailable && !geometryDynamicMask.empty())
        cv::compare(geometryDynamicMask,0,geometryMask,cv::CMP_NE);

    if(mConfig.maskMode=="semantic_only")
    {
        result.dynamicDepthMask = semanticMask;
        result.stats.state = semanticDynamicMask.empty() ?
            "semantic_unavailable_no_filter" : "applied";
    }
    else if(mConfig.maskMode=="geometry_only")
    {
        if(!geometryEvidenceAvailable || geometryDynamicMask.empty())
        {
            result.stats.state = "geometry_unavailable";
            return result;
        }
        result.dynamicDepthMask = geometryMask;
        result.stats.state = "applied";
    }
    else
    {
        cv::bitwise_or(semanticMask,geometryMask,
                       result.dynamicDepthMask);
        if(!geometryEvidenceAvailable || geometryDynamicMask.empty())
        {
            result.stats.state = semanticDynamicMask.empty() ?
                "geometry_and_semantic_unavailable_no_filter" :
                "geometry_unavailable_semantic_only";
        }
        else
        {
            result.stats.state = "applied";
        }
    }

    cv::Mat validDepthMask;
    cv::compare(depthMeters,0.0f,validDepthMask,cv::CMP_GT);
    cv::Mat rejectedValidDepthMask;
    cv::bitwise_and(validDepthMask,result.dynamicDepthMask,
                    rejectedValidDepthMask);

    result.staticDepthMeters = depthMeters.clone();
    result.staticDepthMeters.setTo(0.0f,result.dynamicDepthMask);
    result.stats.inputValidDepthPixels =
        static_cast<std::size_t>(cv::countNonZero(validDepthMask));
    result.stats.semanticDynamicPixels =
        static_cast<std::size_t>(cv::countNonZero(semanticMask));
    result.stats.geometryDynamicPixels =
        static_cast<std::size_t>(cv::countNonZero(geometryMask));
    result.stats.unionDynamicPixels =
        static_cast<std::size_t>(
            cv::countNonZero(result.dynamicDepthMask));
    result.stats.rejectedValidDepthPixels =
        static_cast<std::size_t>(
            cv::countNonZero(rejectedValidDepthMask));
    result.stats.outputValidDepthPixels =
        result.stats.inputValidDepthPixels>=
            result.stats.rejectedValidDepthPixels ?
        result.stats.inputValidDepthPixels-
            result.stats.rejectedValidDepthPixels : 0;
    result.stats.available = true;
    result.stats.totalMs =
        std::chrono::duration<double,std::milli>(
            std::chrono::steady_clock::now()-start).count();
    return result;
}

} // namespace ORB_SLAM2
