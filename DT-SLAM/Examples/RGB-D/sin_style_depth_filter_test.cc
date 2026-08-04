#include "SInStyleDepthFilter.h"

#include <opencv2/core/core.hpp>

#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>

namespace
{

void Require(bool condition, const std::string &message)
{
    if(!condition)
        throw std::runtime_error(message);
}

ORB_SLAM2::SInStyleDepthFilter MakeFilter(
    const std::string &mode, bool enabled = true)
{
    ORB_SLAM2::SInStyleDepthFilter filter;
    ORB_SLAM2::SInStyleDepthFilterConfig config;
    config.enabled = enabled;
    config.maskMode = mode;
    filter.Configure(config);
    return filter;
}

} // namespace

int main()
{
    try
    {
        cv::Mat depth = (cv::Mat_<float>(2,3) <<
            1.0f,2.0f,0.0f,
            3.0f,4.0f,5.0f);
        const cv::Mat originalDepth = depth.clone();
        cv::Mat semantic(2,3,CV_8UC1,cv::Scalar(0));
        semantic.at<unsigned char>(0,0) = 1;
        cv::Mat geometry(2,3,CV_8UC1,cv::Scalar(0));
        geometry.at<unsigned char>(0,1) = 255;
        geometry.at<unsigned char>(1,2) = 7;

        ORB_SLAM2::SInStyleDepthFilter unionFilter =
            MakeFilter("semantic_or_geometry");
        const ORB_SLAM2::SInStyleDepthFilterResult unionResult =
            unionFilter.Filter(depth,semantic,geometry,true);
        Require(unionResult.stats.available,"union result unavailable");
        Require(unionResult.stats.state=="applied",
                "union state is not applied");
        Require(unionResult.dynamicDepthMask.type()==CV_8UC1,
                "dynamic mask type mismatch");
        Require(cv::countNonZero(unionResult.dynamicDepthMask)==3,
                "union mask pixel count mismatch");
        Require(unionResult.stats.inputValidDepthPixels==5,
                "input valid-depth count mismatch");
        Require(unionResult.stats.rejectedValidDepthPixels==3,
                "rejected valid-depth count mismatch");
        Require(unionResult.stats.outputValidDepthPixels==2,
                "output valid-depth count mismatch");
        Require(unionResult.staticDepthMeters.at<float>(0,0)==0.0f &&
                unionResult.staticDepthMeters.at<float>(0,1)==0.0f &&
                unionResult.staticDepthMeters.at<float>(1,2)==0.0f,
                "dynamic depth was not zeroed");
        Require(unionResult.staticDepthMeters.at<float>(1,0)==3.0f &&
                unionResult.staticDepthMeters.at<float>(1,1)==4.0f,
                "static depth was changed");
        Require(cv::norm(depth,originalDepth,cv::NORM_INF)==0.0,
                "input depth was mutated");

        ORB_SLAM2::SInStyleDepthFilter semanticFilter =
            MakeFilter("semantic_only");
        const ORB_SLAM2::SInStyleDepthFilterResult semanticResult =
            semanticFilter.Filter(depth,semantic,cv::Mat(),false);
        Require(semanticResult.stats.available &&
                semanticResult.stats.rejectedValidDepthPixels==1,
                "semantic-only mode mismatch");

        ORB_SLAM2::SInStyleDepthFilter geometryFilter =
            MakeFilter("geometry_only");
        const ORB_SLAM2::SInStyleDepthFilterResult geometryResult =
            geometryFilter.Filter(depth,semantic,geometry,true);
        Require(geometryResult.stats.available &&
                geometryResult.stats.rejectedValidDepthPixels==2,
                "geometry-only mode mismatch");
        const ORB_SLAM2::SInStyleDepthFilterResult noGeometryResult =
            geometryFilter.Filter(depth,semantic,cv::Mat(),false);
        Require(!noGeometryResult.stats.available &&
                noGeometryResult.stats.state=="geometry_unavailable" &&
                noGeometryResult.dynamicDepthMask.empty(),
                "geometry-unavailable behavior mismatch");

        const ORB_SLAM2::SInStyleDepthFilterResult fallbackResult =
            unionFilter.Filter(depth,semantic,cv::Mat(),false);
        Require(fallbackResult.stats.available &&
                fallbackResult.stats.state==
                    "geometry_unavailable_semantic_only" &&
                fallbackResult.stats.rejectedValidDepthPixels==1,
                "union semantic fallback mismatch");

        ORB_SLAM2::SInStyleDepthFilter disabledFilter =
            MakeFilter("semantic_or_geometry",false);
        const ORB_SLAM2::SInStyleDepthFilterResult disabledResult =
            disabledFilter.Filter(depth,semantic,geometry,true);
        Require(!disabledResult.stats.available &&
                disabledResult.stats.state=="disabled" &&
                disabledResult.staticDepthMeters.empty(),
                "disabled behavior mismatch");

        bool typeRejected = false;
        try
        {
            unionFilter.Filter(cv::Mat(2,3,CV_16UC1),semantic,
                               geometry,true);
        }
        catch(const std::invalid_argument &)
        {
            typeRejected = true;
        }
        Require(typeRejected,"invalid depth type was accepted");

        bool sizeRejected = false;
        try
        {
            unionFilter.Filter(depth,cv::Mat(1,1,CV_8UC1),
                               geometry,true);
        }
        catch(const std::invalid_argument &)
        {
            sizeRejected = true;
        }
        Require(sizeRejected,"invalid semantic-mask size was accepted");

        std::cout << "SIn-style mapping depth filter tests passed"
                  << std::endl;
        return 0;
    }
    catch(const std::exception &error)
    {
        std::cerr << "SIn-style mapping depth filter test failed: "
                  << error.what() << std::endl;
        return 1;
    }
}
