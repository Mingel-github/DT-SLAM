#include "SInStyleRegionDynamicClassifier.h"

#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <set>
#include <stdexcept>
#include <vector>

namespace ORB_SLAM2
{
namespace
{

double Milliseconds(const std::chrono::steady_clock::time_point &start,
                    const std::chrono::steady_clock::time_point &end)
{
    return std::chrono::duration<double,std::milli>(end-start).count();
}

void RequireMask(const cv::Mat &mask, const cv::Size &size,
                 const char *name)
{
    if(mask.empty() || mask.type()!=CV_8UC1 || mask.size()!=size)
        throw std::invalid_argument(std::string(name)+
            " must be CV_8UC1 and match region labels");
}

cv::Mat BinaryMask(const cv::Mat &mask)
{
    cv::Mat binary;
    cv::compare(mask,0,binary,cv::CMP_NE);
    return binary;
}

} // namespace

void SInStyleRegionDynamicClassifier::Configure(
    const SInStyleRegionDynamicConfig &config)
{
    if(config.minimumClusterHighPixels<0 ||
       !std::isfinite(config.minimumContourAreaPixels) ||
       config.minimumContourAreaPixels<0.0 ||
       !std::isfinite(config.minimumContourRoundness) ||
       config.minimumContourRoundness<0.0 ||
       !std::isfinite(config.largeContourAreaPixels) ||
       config.largeContourAreaPixels<0.0 ||
       !std::isfinite(config.wholeRegionFillFraction) ||
       config.wholeRegionFillFraction<0.0 ||
       config.wholeRegionFillFraction>1.0 ||
       config.lowResidualDilationSize<=0 ||
       config.lowResidualDilationSize%2==0 ||
       config.outputDilationSize<=0 ||
       config.outputDilationSize%2==0)
    {
        throw std::invalid_argument(
            "invalid SIn-style region dynamic configuration");
    }
    mConfig = config;
    Reset();
}

void SInStyleRegionDynamicClassifier::Reset()
{
    mPreviousHighResidualMask.release();
    mPreviousFrameIndex = 0;
    mbPreviousHighResidualValid = false;
}

SInStyleRegionDynamicResult
SInStyleRegionDynamicClassifier::Compute(
    std::size_t frameIndex,
    const cv::Mat &regionLabels,
    const cv::Mat &regionValidMask,
    const cv::Mat &lowResidualMask,
    const cv::Mat &highResidualMask)
{
    SInStyleRegionDynamicResult result;
    result.stats.enabled = mConfig.enabled;
    result.stats.frameIndex = frameIndex;
    if(!mConfig.enabled)
        return result;

    const std::chrono::steady_clock::time_point totalStart =
        std::chrono::steady_clock::now();
    result.stats.failureReason = "evidence_unavailable";
    if(regionLabels.empty() || regionValidMask.empty() ||
       lowResidualMask.empty() || highResidualMask.empty())
    {
        result.stats.totalMs = Milliseconds(
            totalStart,std::chrono::steady_clock::now());
        return result;
    }
    if(regionLabels.type()!=CV_32SC1)
        throw std::invalid_argument("region labels must be CV_32SC1");
    RequireMask(regionValidMask,regionLabels.size(),"region valid mask");
    RequireMask(lowResidualMask,regionLabels.size(),"low residual mask");
    RequireMask(highResidualMask,regionLabels.size(),"high residual mask");
    if(mbPreviousHighResidualValid && frameIndex!=mPreviousFrameIndex+1)
        Reset();

    const std::chrono::steady_clock::time_point prepareStart =
        std::chrono::steady_clock::now();
    result.validRegionMask = BinaryMask(regionValidMask);
    cv::bitwise_not(result.validRegionMask,result.unknownMask);
    cv::Mat lowKnown;
    cv::Mat highEvidence = BinaryMask(highResidualMask);
    cv::bitwise_and(BinaryMask(lowResidualMask),result.validRegionMask,
                    lowKnown);
    result.lowResidualSupportMask = lowKnown.clone();
    if(mConfig.usePreviousHighResidual && mbPreviousHighResidualValid &&
       mPreviousHighResidualMask.size()==regionLabels.size())
    {
        cv::Mat previousKnown;
        cv::bitwise_and(mPreviousHighResidualMask,result.validRegionMask,
                        previousKnown);
        cv::Mat newlyAdded;
        cv::bitwise_and(previousKnown,result.lowResidualSupportMask==0,
                        newlyAdded);
        result.stats.temporalHighPixelsAdded =
            static_cast<std::size_t>(cv::countNonZero(newlyAdded));
        cv::bitwise_or(result.lowResidualSupportMask,previousKnown,
                       result.lowResidualSupportMask);
    }
    const cv::Mat lowElement = cv::getStructuringElement(
        cv::MORPH_ELLIPSE,
        cv::Size(mConfig.lowResidualDilationSize,
                 mConfig.lowResidualDilationSize));
    cv::dilate(result.lowResidualSupportMask,
               result.lowResidualSupportMask,lowElement);
    result.stats.validRegionPixels = static_cast<std::size_t>(
        cv::countNonZero(result.validRegionMask));
    result.stats.unknownPixels = static_cast<std::size_t>(
        cv::countNonZero(result.unknownMask));
    result.stats.lowResidualPixels = static_cast<std::size_t>(
        cv::countNonZero(lowKnown));
    result.stats.highResidualPixels = static_cast<std::size_t>(
        cv::countNonZero(highEvidence));
    result.stats.prepareMs = Milliseconds(
        prepareStart,std::chrono::steady_clock::now());

    const std::chrono::steady_clock::time_point classifyStart =
        std::chrono::steady_clock::now();
    result.filledDynamicMaskBeforeDilation = cv::Mat(
        regionLabels.size(),CV_8UC1,cv::Scalar(0));
    std::set<int> labels;
    for(int row=0; row<regionLabels.rows; ++row)
    {
        const int *values = regionLabels.ptr<int>(row);
        for(int col=0; col<regionLabels.cols; ++col)
        {
            if(values[col]>0)
                labels.insert(values[col]);
        }
    }
    result.stats.regionCount = static_cast<int>(labels.size());
    for(const int label : labels)
    {
        cv::Mat regionMask;
        cv::compare(regionLabels,label,regionMask,cv::CMP_EQ);
        const int regionPixels = cv::countNonZero(regionMask);
        if(regionPixels<=0)
            continue;
        cv::Mat regionHigh;
        cv::bitwise_and(regionMask,highEvidence,regionHigh);
        if(cv::countNonZero(regionHigh)<=mConfig.minimumClusterHighPixels)
        {
            ++result.stats.regionsWithoutHighSupport;
            continue;
        }
        ++result.stats.regionsWithHighSupport;

        std::vector<std::vector<cv::Point>> contours;
        std::vector<cv::Vec4i> hierarchy;
        cv::findContours(regionHigh,contours,hierarchy,
                         cv::RETR_CCOMP,cv::CHAIN_APPROX_NONE);
        cv::Mat floodMask;
        cv::copyMakeBorder(regionMask,floodMask,1,1,1,1,
                           cv::BORDER_CONSTANT,0);
        cv::bitwise_not(floodMask,floodMask);
        for(const std::vector<cv::Point> &contour : contours)
        {
            const double area = cv::contourArea(contour);
            const double perimeter = cv::arcLength(contour,true);
            const double roundness = perimeter>0.0 ?
                4.0*CV_PI*area/(perimeter*perimeter) : 0.0;
            if(!((area>mConfig.minimumContourAreaPixels &&
                  roundness>mConfig.minimumContourRoundness) ||
                 area>mConfig.largeContourAreaPixels))
            {
                continue;
            }
            ++result.stats.eligibleContourCount;
            cv::Point seed;
            bool foundSeed = false;
            for(const cv::Point &point : contour)
            {
                if(result.lowResidualSupportMask.at<unsigned char>(
                        point.y,point.x)!=0)
                {
                    seed = point;
                    foundSeed = true;
                    break;
                }
            }
            if(!foundSeed)
                continue;
            ++result.stats.validSeedContourCount;
            cv::floodFill(result.lowResidualSupportMask,floodMask,seed,0,
                          nullptr,cv::Scalar(0),cv::Scalar(5),
                          8 | cv::FLOODFILL_MASK_ONLY | (50 << 8));
        }
        cv::Mat filled = floodMask(cv::Rect(
            1,1,regionLabels.cols,regionLabels.rows)).clone();
        cv::compare(filled,50,filled,cv::CMP_EQ);
        const int filledPixels = cv::countNonZero(filled);
        if(filledPixels>mConfig.wholeRegionFillFraction*regionPixels)
        {
            cv::bitwise_or(result.filledDynamicMaskBeforeDilation,
                           regionMask,
                           result.filledDynamicMaskBeforeDilation);
            ++result.stats.wholeDynamicRegionCount;
        }
        else if(filledPixels>0)
        {
            cv::bitwise_or(result.filledDynamicMaskBeforeDilation,
                           filled,
                           result.filledDynamicMaskBeforeDilation);
            ++result.stats.partialDynamicRegionCount;
        }
    }
    result.stats.dynamicPixelsBeforeDilation = static_cast<std::size_t>(
        cv::countNonZero(result.filledDynamicMaskBeforeDilation));
    const cv::Mat outputElement = cv::getStructuringElement(
        cv::MORPH_ELLIPSE,
        cv::Size(mConfig.outputDilationSize,mConfig.outputDilationSize));
    cv::dilate(result.filledDynamicMaskBeforeDilation,
               result.authorStyleDynamicMask,outputElement);
    cv::bitwise_and(result.authorStyleDynamicMask,
                    result.validRegionMask,result.dynamicMask);
    cv::Mat notDynamic;
    cv::bitwise_not(result.authorStyleDynamicMask,notDynamic);
    cv::bitwise_and(result.validRegionMask,notDynamic,result.staticMask);
    result.rawStateMask = cv::Mat(
        regionLabels.size(),CV_8UC1,cv::Scalar(0));
    result.rawStateMask.setTo(125,result.staticMask);
    result.rawStateMask.setTo(255,result.authorStyleDynamicMask);

    result.stats.authorStyleDynamicPixels = static_cast<std::size_t>(
        cv::countNonZero(result.authorStyleDynamicMask));
    result.stats.depthSupportedDynamicPixels = static_cast<std::size_t>(
        cv::countNonZero(result.dynamicMask));
    result.stats.staticPixels = static_cast<std::size_t>(
        cv::countNonZero(result.staticMask));
    result.stats.classifyMs = Milliseconds(
        classifyStart,std::chrono::steady_clock::now());
    result.stats.totalMs = Milliseconds(
        totalStart,std::chrono::steady_clock::now());
    result.stats.available = true;
    result.stats.dynamicStateAvailable = true;
    result.stats.failureReason = "none";

    mPreviousHighResidualMask = highEvidence.clone();
    mPreviousFrameIndex = frameIndex;
    mbPreviousHighResidualValid = true;
    return result;
}

} // namespace ORB_SLAM2
