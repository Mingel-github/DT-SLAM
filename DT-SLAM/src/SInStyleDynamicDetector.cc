/**
 * Clean-room SIn-style shadow interface.
 *
 * The first S1 backend replays independently generated SInDSLAM outputs. It
 * establishes coordinate, state, and audit invariants before a native detector
 * is implemented. It is not itself a dynamic detection algorithm.
 */

#include "SInStyleDynamicDetector.h"

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <map>
#include <sstream>
#include <stdexcept>

namespace ORB_SLAM2
{
namespace
{

double ElapsedMilliseconds(const int64 startTicks)
{
    return 1000.0*static_cast<double>(cv::getTickCount()-startTicks)/
           cv::getTickFrequency();
}

cv::Mat BuildDepthValidityMask(const cv::Mat &depthMeters)
{
    if(depthMeters.empty() || depthMeters.type()!=CV_32FC1)
        throw std::invalid_argument(
            "SIn-style depth must be non-empty CV_32FC1 meters");

    cv::Mat valid(depthMeters.size(),CV_8UC1,cv::Scalar(0));
    for(int row=0; row<depthMeters.rows; ++row)
    {
        const float *depth = depthMeters.ptr<float>(row);
        unsigned char *output = valid.ptr<unsigned char>(row);
        for(int col=0; col<depthMeters.cols; ++col)
        {
            if(std::isfinite(depth[col]) && depth[col]>0.0f)
                output[col] = 255;
        }
    }
    return valid;
}

void ValidateReferenceStateValues(const cv::Mat &state)
{
    for(int row=0; row<state.rows; ++row)
    {
        const unsigned char *values = state.ptr<unsigned char>(row);
        for(int col=0; col<state.cols; ++col)
        {
            const unsigned char value = values[col];
            if(value!=0 && value!=125 && value!=255)
            {
                std::ostringstream message;
                message << "SIn reference state contains unsupported value "
                        << static_cast<int>(value)
                        << "; expected only 0, 125, or 255";
                throw std::invalid_argument(message.str());
            }
        }
    }
}

} // namespace

SInStyleDynamicDetector::SInStyleDynamicDetector()
{
}

SInStyleDynamicDetector::SInStyleDynamicDetector(
    const SInStyleDetectorConfig &config)
{
    Configure(config);
}

void SInStyleDynamicDetector::Configure(
    const SInStyleDetectorConfig &config)
{
    if(config.enabled)
    {
        if(config.backend!="reference_replay")
            throw std::invalid_argument(
                "SInStyle.Backend currently supports only reference_replay");
        if(config.referenceDirectory.empty())
            throw std::invalid_argument(
                "SInStyle.ReferenceDirectory is required for reference_replay");
        if(config.referenceMaskSuffix.empty())
            throw std::invalid_argument(
                "SInStyle.ReferenceMaskSuffix must not be empty");
        if(config.requireRegionValidity &&
           config.referenceRegionValidSuffix.empty())
        {
            throw std::invalid_argument(
                "SInStyle.ReferenceRegionValidSuffix must not be empty");
        }
    }
    mConfig = config;
}

void SInStyleDynamicDetector::Reset()
{
    // The replay backend has no temporal state. The method is part of the
    // stable S1 interface because the native backend will require it.
}

std::string SInStyleDynamicDetector::FrameStem(
    const long unsigned int frameId) const
{
    std::ostringstream path;
    path << mConfig.referenceDirectory;
    if(!mConfig.referenceDirectory.empty() &&
       mConfig.referenceDirectory[mConfig.referenceDirectory.size()-1]!='/')
    {
        path << "/";
    }
    path << "frame_" << std::setw(6) << std::setfill('0') << frameId;
    return path.str();
}

SInStyleShadowResult SInStyleDynamicDetector::Process(
    const cv::Mat &image,
    const cv::Mat &depthMeters,
    const long unsigned int frameId,
    const double timestampSeconds) const
{
    if(!mConfig.enabled)
        throw std::logic_error(
            "SIn-style detector Process called while disabled");
    if(image.empty() || image.size()!=depthMeters.size())
        throw std::invalid_argument(
            "SIn-style image and depth must be non-empty and have equal size");
    if(!std::isfinite(timestampSeconds))
        throw std::invalid_argument(
            "SIn-style timestamp must be finite");

    const int64 totalStart = cv::getTickCount();
    const int64 loadStart = cv::getTickCount();
    const std::string stem = FrameStem(frameId);
    const std::string maskPath = stem+mConfig.referenceMaskSuffix;
    const std::string labelPath = stem+"_labels.png";
    const std::string regionValidPath =
        stem+mConfig.referenceRegionValidSuffix;

    cv::Mat rawState = cv::imread(maskPath,cv::IMREAD_UNCHANGED);
    cv::Mat labels;
    cv::Mat regionValidity;
    if(!rawState.empty())
    {
        labels = cv::imread(labelPath,cv::IMREAD_UNCHANGED);
        if(mConfig.requireRegionValidity)
        {
            regionValidity = cv::imread(
                regionValidPath,cv::IMREAD_GRAYSCALE);
        }
        if(mConfig.requireLabels && labels.empty())
        {
            throw std::runtime_error(
                "SIn reference mask exists but required labels are missing: "+
                labelPath);
        }
        if(mConfig.requireRegionValidity && regionValidity.empty())
        {
            throw std::runtime_error(
                "SIn reference mask exists but required region validity is "
                "missing: "+regionValidPath);
        }
    }
    const double loadMs = ElapsedMilliseconds(loadStart);

    SInStyleShadowResult result;
    if(rawState.empty())
    {
        result.rawStateMask = cv::Mat(
            depthMeters.size(),CV_8UC1,cv::Scalar(0));
        result.regionLabels = cv::Mat(
            depthMeters.size(),CV_32SC1,cv::Scalar(-1));
    }
    else
    {
        if(rawState.type()!=CV_8UC1 || rawState.size()!=depthMeters.size())
            throw std::runtime_error(
                "SIn reference mask must be CV_8UC1 and match input size: "+
                maskPath);
        result.rawStateMask = rawState;

        if(labels.empty())
        {
            result.regionLabels = cv::Mat(
                depthMeters.size(),CV_32SC1,cv::Scalar(-1));
        }
        else
        {
            if(labels.channels()!=1 || labels.size()!=depthMeters.size())
                throw std::runtime_error(
                    "SIn reference labels must be single-channel and match input size: "+
                    labelPath);
            labels.convertTo(result.regionLabels,CV_32SC1);
        }
    }

    const int64 conversionStart = cv::getTickCount();
    ValidateReferenceStateValues(result.rawStateMask);
    result.inputDepthValidMask = BuildDepthValidityMask(depthMeters);
    if(!regionValidity.empty())
    {
        if(regionValidity.type()!=CV_8UC1 ||
           regionValidity.size()!=depthMeters.size())
        {
            throw std::runtime_error(
                "SIn reference region validity must be CV_8UC1 and match "
                "input size: "+regionValidPath);
        }
        cv::compare(regionValidity,0,result.referenceRegionValidMask,
                    cv::CMP_NE);
    }

    cv::compare(result.rawStateMask,0,result.referenceKnownCodeMask,cv::CMP_NE);
    cv::compare(result.rawStateMask,0,result.referenceUnknownMask,cv::CMP_EQ);
    cv::bitwise_and(result.referenceKnownCodeMask,
                    result.inputDepthValidMask,result.validMask);

    cv::Mat rawStatic;
    cv::compare(result.rawStateMask,125,rawStatic,cv::CMP_EQ);
    cv::bitwise_and(rawStatic,result.validMask,result.staticMask);

    cv::Mat rawDynamic;
    cv::compare(result.rawStateMask,255,rawDynamic,cv::CMP_EQ);
    result.authorDynamicMask = rawDynamic;
    cv::bitwise_and(rawDynamic,result.validMask,result.dynamicMask);
    cv::bitwise_not(result.validMask,result.unknownMask);
    const double conversionMs = ElapsedMilliseconds(conversionStart);

    const int64 statsStart = cv::getTickCount();
    result.stats.pixelCount = static_cast<std::size_t>(depthMeters.total());
    result.stats.depthValidPixels =
        static_cast<std::size_t>(cv::countNonZero(result.inputDepthValidMask));
    result.stats.rawUnknownPixels =
        static_cast<std::size_t>(cv::countNonZero(result.referenceUnknownMask));
    result.stats.rawStaticPixels =
        static_cast<std::size_t>(cv::countNonZero(rawStatic));
    result.stats.rawDynamicPixels =
        static_cast<std::size_t>(cv::countNonZero(rawDynamic));
    result.stats.validPixels =
        static_cast<std::size_t>(cv::countNonZero(result.validMask));
    result.stats.staticPixels =
        static_cast<std::size_t>(cv::countNonZero(result.staticMask));
    result.stats.dynamicPixels =
        static_cast<std::size_t>(cv::countNonZero(result.dynamicMask));
    result.stats.unknownPixels =
        static_cast<std::size_t>(cv::countNonZero(result.unknownMask));
    result.stats.referenceAvailable = !rawState.empty();
    result.stats.labelsAvailable = !labels.empty();
    result.stats.regionValidityAvailable = !regionValidity.empty();

    if(result.stats.labelsAvailable)
    {
        std::map<int,SInStyleRegionStats> byLabel;
        for(int row=0; row<result.regionLabels.rows; ++row)
        {
            const int *labelValues = result.regionLabels.ptr<int>(row);
            const unsigned char *validValues =
                result.validMask.ptr<unsigned char>(row);
            const unsigned char *dynamicValues =
                result.dynamicMask.ptr<unsigned char>(row);
            const unsigned char *authorDynamicValues =
                result.authorDynamicMask.ptr<unsigned char>(row);
            for(int col=0; col<result.regionLabels.cols; ++col)
            {
                if(labelValues[col]==0 && authorDynamicValues[col]!=0)
                    ++result.stats.authorDynamicPixelsOnLabelZero;
                if(labelValues[col]<=0)
                    continue;
                SInStyleRegionStats &region = byLabel[labelValues[col]];
                region.label = labelValues[col];
                ++region.labelPixels;
                ++result.stats.positiveLabelPixels;
                if(authorDynamicValues[col]!=0)
                    ++result.stats.authorDynamicPixelsOnPositiveLabels;
                if(validValues[col]!=0)
                {
                    ++region.depthSupportedPixels;
                    ++result.stats.depthSupportedPositiveLabelPixels;
                    if(dynamicValues[col]!=0)
                        ++region.dynamicPixels;
                    else
                        ++region.staticPixels;
                }
            }
        }
        for(std::map<int,SInStyleRegionStats>::const_iterator iterator =
                byLabel.begin(); iterator!=byLabel.end(); ++iterator)
        {
            result.regions.push_back(iterator->second);
            cv::Mat labelMask;
            cv::compare(result.regionLabels,iterator->first,labelMask,cv::CMP_EQ);
            cv::Mat components;
            const int componentCount = cv::connectedComponents(
                labelMask,components,8,CV_32S);
            if(componentCount>1)
            {
                result.stats.positiveLabelComponentCount +=
                    static_cast<std::size_t>(componentCount-1);
            }
        }
        result.stats.positiveLabelCount = result.regions.size();
    }
    else
    {
        result.stats.authorDynamicPixelsWithLabelsUnavailable =
            result.stats.rawDynamicPixels;
    }
    const double statsMs = ElapsedMilliseconds(statsStart);

    result.runtime.loadMs = loadMs;
    result.runtime.stateConversionMs = conversionMs;
    result.runtime.regionStatisticsMs = statsMs;
    result.runtime.totalMs = ElapsedMilliseconds(totalStart);
    return result;
}

SInStyleShadowResult SInStyleDynamicDetector::ConvertReferenceState(
    const cv::Mat &rawStateMask,
    const cv::Mat &regionLabels,
    const cv::Mat &depthMeters)
{
    if(rawStateMask.empty() || rawStateMask.type()!=CV_8UC1 ||
       rawStateMask.size()!=depthMeters.size())
    {
        throw std::invalid_argument(
            "reference state must be CV_8UC1 and match depth size");
    }

    SInStyleShadowResult result;
    result.rawStateMask = rawStateMask.clone();
    if(regionLabels.empty())
    {
        result.regionLabels = cv::Mat(
            depthMeters.size(),CV_32SC1,cv::Scalar(-1));
    }
    else
    {
        if(regionLabels.channels()!=1 || regionLabels.size()!=depthMeters.size())
            throw std::invalid_argument(
                "reference labels must be single-channel and match depth size");
        regionLabels.convertTo(result.regionLabels,CV_32SC1);
    }

    ValidateReferenceStateValues(result.rawStateMask);
    result.inputDepthValidMask = BuildDepthValidityMask(depthMeters);
    cv::compare(result.rawStateMask,0,result.referenceKnownCodeMask,cv::CMP_NE);
    cv::compare(result.rawStateMask,0,result.referenceUnknownMask,cv::CMP_EQ);
    cv::bitwise_and(result.referenceKnownCodeMask,
                    result.inputDepthValidMask,result.validMask);
    cv::Mat rawStatic;
    cv::compare(result.rawStateMask,125,rawStatic,cv::CMP_EQ);
    cv::bitwise_and(rawStatic,result.validMask,result.staticMask);
    cv::Mat rawDynamic;
    cv::compare(result.rawStateMask,255,rawDynamic,cv::CMP_EQ);
    result.authorDynamicMask = rawDynamic;
    cv::bitwise_and(rawDynamic,result.validMask,result.dynamicMask);
    cv::bitwise_not(result.validMask,result.unknownMask);

    result.stats.pixelCount = depthMeters.total();
    result.stats.depthValidPixels = cv::countNonZero(result.inputDepthValidMask);
    result.stats.rawUnknownPixels =
        cv::countNonZero(result.referenceUnknownMask);
    result.stats.rawStaticPixels = cv::countNonZero(rawStatic);
    result.stats.rawDynamicPixels = cv::countNonZero(rawDynamic);
    result.stats.validPixels = cv::countNonZero(result.validMask);
    result.stats.staticPixels = cv::countNonZero(result.staticMask);
    result.stats.dynamicPixels = cv::countNonZero(result.dynamicMask);
    result.stats.unknownPixels = cv::countNonZero(result.unknownMask);
    result.stats.referenceAvailable = true;
    result.stats.labelsAvailable = !regionLabels.empty();

    if(result.stats.labelsAvailable)
    {
        std::map<int,SInStyleRegionStats> byLabel;
        for(int row=0; row<result.regionLabels.rows; ++row)
        {
            const int *labelValues = result.regionLabels.ptr<int>(row);
            const unsigned char *validValues =
                result.validMask.ptr<unsigned char>(row);
            const unsigned char *dynamicValues =
                result.dynamicMask.ptr<unsigned char>(row);
            const unsigned char *authorDynamicValues =
                result.authorDynamicMask.ptr<unsigned char>(row);
            for(int col=0; col<result.regionLabels.cols; ++col)
            {
                if(labelValues[col]==0 && authorDynamicValues[col]!=0)
                    ++result.stats.authorDynamicPixelsOnLabelZero;
                if(labelValues[col]<=0)
                    continue;
                SInStyleRegionStats &region = byLabel[labelValues[col]];
                region.label = labelValues[col];
                ++region.labelPixels;
                ++result.stats.positiveLabelPixels;
                if(authorDynamicValues[col]!=0)
                    ++result.stats.authorDynamicPixelsOnPositiveLabels;
                if(validValues[col]!=0)
                {
                    ++region.depthSupportedPixels;
                    ++result.stats.depthSupportedPositiveLabelPixels;
                    if(dynamicValues[col]!=0)
                        ++region.dynamicPixels;
                    else
                        ++region.staticPixels;
                }
            }
        }
        for(std::map<int,SInStyleRegionStats>::const_iterator iterator =
                byLabel.begin(); iterator!=byLabel.end(); ++iterator)
        {
            result.regions.push_back(iterator->second);
            cv::Mat labelMask;
            cv::compare(result.regionLabels,iterator->first,labelMask,cv::CMP_EQ);
            cv::Mat components;
            const int componentCount = cv::connectedComponents(
                labelMask,components,8,CV_32S);
            if(componentCount>1)
            {
                result.stats.positiveLabelComponentCount +=
                    static_cast<std::size_t>(componentCount-1);
            }
        }
        result.stats.positiveLabelCount = result.regions.size();
    }
    else
    {
        result.stats.authorDynamicPixelsWithLabelsUnavailable =
            result.stats.rawDynamicPixels;
    }
    return result;
}

} // namespace ORB_SLAM2
