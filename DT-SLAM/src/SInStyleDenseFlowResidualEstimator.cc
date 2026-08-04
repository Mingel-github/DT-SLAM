#include "SInStyleDenseFlowResidualEstimator.h"

#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/optflow.hpp>
#include <opencv2/video/tracking.hpp>

#include <algorithm>
#include <chrono>
#include <cfloat>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <limits>
#include <sstream>
#include <stdexcept>

namespace ORB_SLAM2
{
namespace
{

double Milliseconds(const std::chrono::steady_clock::time_point &start,
                    const std::chrono::steady_clock::time_point &end)
{
    return std::chrono::duration<double,std::milli>(end-start).count();
}

bool IsFiniteMatrix(const cv::Mat &matrix)
{
    return !matrix.empty() && cv::checkRange(matrix,true,nullptr,
                                              -DBL_MAX,DBL_MAX);
}

std::size_t CountMask(const cv::Mat &mask)
{
    return static_cast<std::size_t>(cv::countNonZero(mask));
}

struct WeightedFlowSample
{
    WeightedFlowSample(int xValue, int yValue, float weightValue)
        : x(xValue), y(yValue), weight(weightValue)
    {
    }

    int x;
    int y;
    float weight;
};

bool SourceStyleInBorder(float row, float col, int height, int width)
{
    const int integerRow = static_cast<int>(row);
    const int integerCol = static_cast<int>(col);
    return static_cast<unsigned int>(integerRow)<=
               static_cast<unsigned int>(height) &&
           static_cast<unsigned int>(integerCol)<=
               static_cast<unsigned int>(width);
}

} // namespace

void SInStyleDenseFlowResidualEstimator::Configure(
    const SInStyleDenseFlowResidualConfig &config)
{
    if(config.backend!="reference_replay" &&
       config.backend!="deepflow_cpu")
        throw std::invalid_argument(
            "unsupported SIn dense-flow residual backend: "+config.backend);
    if(config.enabled && config.backend=="reference_replay" &&
       config.referenceDirectory.empty())
        throw std::invalid_argument(
            "SIn dense-flow residual replay requires a reference directory");
    if(!std::isfinite(config.residualRecomputeTolerancePx) ||
       config.residualRecomputeTolerancePx<0.0 ||
       config.normalizedResidualTolerance<0)
    {
        throw std::invalid_argument(
            "invalid SIn dense-flow residual validation tolerance");
    }
    mConfig = config;
    Reset();
}

void SInStyleDenseFlowResidualEstimator::Reset()
{
    mNativeLastGray.release();
    mNativeSecondLastGray.release();
    mNativeLastFrameIndex = 0;
    mNativeSecondLastFrameIndex = 0;
    mbNativeLastFrameValid = false;
    mbNativeSecondLastFrameValid = false;
    mTemporalDetectorStateMask.release();
    mTemporalRegionLabels.release();
    mTemporalPriorFrameIndex = 0;
    mbTemporalRegionPriorValid = false;
}

void SInStyleDenseFlowResidualEstimator::CommitTemporalRegionPrior(
    const std::size_t frameIndex,
    const cv::Mat &detectorStateMask,
    const cv::Mat &regionLabels)
{
    if(!mConfig.useTemporalRegionPrior)
        return;
    if(detectorStateMask.empty() || detectorStateMask.type()!=CV_8UC1 ||
       regionLabels.empty() || regionLabels.type()!=CV_32SC1 ||
       detectorStateMask.size()!=regionLabels.size())
    {
        throw std::invalid_argument(
            "invalid SIn temporal region prior");
    }
    mTemporalDetectorStateMask = detectorStateMask.clone();
    mTemporalRegionLabels = regionLabels.clone();
    mTemporalPriorFrameIndex = frameIndex;
    mbTemporalRegionPriorValid = true;
}

std::string SInStyleDenseFlowResidualEstimator::FrameStem(
    const std::string &directory, std::size_t frameIndex)
{
    std::ostringstream stream;
    stream << directory;
    if(directory[directory.size()-1]!='/')
        stream << '/';
    stream << "frame_" << std::setfill('0') << std::setw(6) << frameIndex;
    return stream.str();
}

cv::Mat SInStyleDenseFlowResidualEstimator::ReadFlowFile(
    const std::string &path)
{
    std::ifstream stream(path.c_str(),std::ios::binary);
    if(!stream.is_open())
        return cv::Mat();
    float magic = 0.0f;
    std::int32_t width = 0;
    std::int32_t height = 0;
    stream.read(reinterpret_cast<char*>(&magic),sizeof(magic));
    stream.read(reinterpret_cast<char*>(&width),sizeof(width));
    stream.read(reinterpret_cast<char*>(&height),sizeof(height));
    if(!stream.good() || magic!=202021.25f || width<=0 || height<=0 ||
       width>10000 || height>10000)
    {
        throw std::runtime_error("invalid .flo header: "+path);
    }
    cv::Mat flow(height,width,CV_32FC2);
    const std::streamsize byteCount = static_cast<std::streamsize>(
        flow.total()*flow.elemSize());
    stream.read(reinterpret_cast<char*>(flow.ptr<float>()),byteCount);
    if(stream.gcount()!=byteCount)
        throw std::runtime_error("truncated .flo payload: "+path);
    char trailing = 0;
    if(stream.read(&trailing,1))
        throw std::runtime_error("unexpected trailing .flo bytes: "+path);
    return flow;
}

SInStyleDenseFlowResidualResult
SInStyleDenseFlowResidualEstimator::Process(
    std::size_t frameIndex, const cv::Mat &currentGray)
{
    if(mConfig.backend=="deepflow_cpu")
        return ProcessNativeDeepFlow(frameIndex,currentGray);
    return ProcessReferenceReplay(frameIndex);
}

SInStyleDenseFlowResidualResult
SInStyleDenseFlowResidualEstimator::ProcessReferenceReplay(
    std::size_t frameIndex) const
{
    SInStyleDenseFlowResidualResult result;
    result.stats.enabled = mConfig.enabled;
    result.stats.backend = mConfig.backend;
    result.stats.frameIndex = frameIndex;
    if(!mConfig.enabled)
        return result;
    result.stats.failureReason = "history_unavailable";
    if(frameIndex==0)
        return result;

    const std::chrono::steady_clock::time_point start =
        std::chrono::steady_clock::now();
    try
    {
        const std::string stem = FrameStem(
            mConfig.referenceDirectory,frameIndex);
        cv::FileStorage metadata(stem+"_meta.yml",cv::FileStorage::READ);
        if(!metadata.isOpened())
        {
            result.stats.failureReason = "reference_missing";
            if(mConfig.requireReference)
                throw std::runtime_error(
                    "missing dense-flow metadata: "+stem);
            return result;
        }

        int metadataFrame = -1;
        int available = 0;
        std::string metadataFailureReason;
        metadata["frame_index"] >> metadataFrame;
        metadata["available"] >> available;
        const cv::FileNode failureReasonNode = metadata["failure_reason"];
        if(!failureReasonNode.empty())
            failureReasonNode >> metadataFailureReason;
        if(metadataFrame!=static_cast<int>(frameIndex))
            throw std::runtime_error(
                "dense-flow metadata frame index mismatch");
        if(available!=1)
        {
            result.stats.failureReason = metadataFailureReason.empty() ?
                "reference_unavailable" : metadataFailureReason;
            metadata.release();
            const std::chrono::steady_clock::time_point end =
                std::chrono::steady_clock::now();
            result.stats.loadMs = Milliseconds(start,end);
            result.stats.totalMs = result.stats.loadMs;
            return result;
        }

        result.rawBackendFlowNative = ReadFlowFile(
            stem+"_brox_internal_s06.flo");
        result.observedFlowFull = ReadFlowFile(
            stem+"_flow_refined_full.flo");
        result.residualFlowFull = ReadFlowFile(stem+"_residual_full.flo");
        result.normalizedResidual = cv::imread(
            stem+"_residual_normalized.png",cv::IMREAD_UNCHANGED);
        result.lowResidualMask = cv::imread(
            stem+"_threshold_low.png",cv::IMREAD_UNCHANGED);
        result.highResidualMask = cv::imread(
            stem+"_threshold_high.png",cv::IMREAD_UNCHANGED);

        metadata["backend"] >> result.stats.backend;
    int internalSign = 0;
    int homographyValid = 0;
    std::string flowUnits;
    std::string homographyDirection;
    metadata["internal_sign"] >> internalSign;
    metadata["flow_units"] >> flowUnits;
    metadata["homography_direction"] >> homographyDirection;
    metadata["homography_valid"] >> homographyValid;
    if(internalSign!=-1 || flowUnits!="full_resolution_pixels" ||
       homographyDirection!="current_to_reference" || homographyValid!=1)
    {
        throw std::runtime_error(
            "dense-flow replay identity metadata mismatch");
    }
    metadata["image_scale"] >> result.stats.imageScale;
    metadata["intended_reference_lag"] >>
        result.stats.intendedReferenceLag;
    metadata["reference_index"] >> result.stats.referenceIndex;
    metadata["actual_reference_lag"] >> result.stats.actualReferenceLag;
    int largeMotion = 0;
    metadata["large_motion"] >> largeMotion;
    result.stats.largeMotion = largeMotion!=0;
    metadata["homography_sample_count"] >>
        result.stats.homographySampleCount;
    metadata["homography"] >> result.homographyCurrentToReference;
    metadata["max_flow_px"] >> result.stats.maxObservedFlowPx;
    metadata["max_residual_px"] >> result.stats.maxResidualPx;
    const cv::FileNode otsuNode = metadata["otsu_threshold_u8"];
    if(!otsuNode.empty())
        otsuNode >> result.stats.otsuThresholdU8;
    const cv::FileNode triangleNode = metadata["triangle_threshold_u8"];
    if(!triangleNode.empty())
        triangleNode >> result.stats.triangleThresholdU8;
    metadata["low_threshold_u8"] >> result.stats.lowThresholdU8;
    metadata["high_threshold_u8"] >> result.stats.highThresholdU8;
    metadata["low_threshold_px"] >> result.stats.lowThresholdPx;
    metadata["high_threshold_px"] >> result.stats.highThresholdPx;
    int lowPixels = 0;
    int highPixels = 0;
    metadata["low_pixels"] >> lowPixels;
    metadata["high_pixels"] >> highPixels;
    metadata.release();
    result.stats.lowPixels = static_cast<std::size_t>(std::max(0,lowPixels));
    result.stats.highPixels = static_cast<std::size_t>(std::max(0,highPixels));
    const std::chrono::steady_clock::time_point loadEnd =
        std::chrono::steady_clock::now();
    result.stats.loadMs = Milliseconds(start,loadEnd);

    const cv::Size fullSize(640,480);
    if(metadataFrame!=static_cast<int>(frameIndex) || available!=1 ||
       result.rawBackendFlowNative.type()!=CV_32FC2 ||
       result.rawBackendFlowNative.size()!=cv::Size(384,288) ||
       result.observedFlowFull.type()!=CV_32FC2 ||
       result.observedFlowFull.size()!=fullSize ||
       result.residualFlowFull.type()!=CV_32FC2 ||
       result.residualFlowFull.size()!=fullSize ||
       result.normalizedResidual.type()!=CV_8UC1 ||
       result.normalizedResidual.size()!=fullSize ||
       result.lowResidualMask.type()!=CV_8UC1 ||
       result.lowResidualMask.size()!=fullSize ||
       result.highResidualMask.type()!=CV_8UC1 ||
       result.highResidualMask.size()!=fullSize ||
       result.homographyCurrentToReference.rows!=3 ||
       result.homographyCurrentToReference.cols!=3 ||
       !IsFiniteMatrix(result.rawBackendFlowNative) ||
       !IsFiniteMatrix(result.observedFlowFull) ||
       !IsFiniteMatrix(result.residualFlowFull) ||
       !IsFiniteMatrix(result.homographyCurrentToReference))
    {
        throw std::runtime_error(
            "invalid dense-flow replay data for frame "+
            std::to_string(frameIndex));
    }
    result.homographyCurrentToReference.convertTo(
        result.homographyCurrentToReference,CV_64FC1);

    result.inducedFlowFull = cv::Mat(fullSize,CV_32FC2);
    cv::Mat recomputedResidual(fullSize,CV_32FC2);
    const cv::Mat &H = result.homographyCurrentToReference;
    for(int row=0; row<fullSize.height; ++row)
    {
        cv::Vec2f *induced = result.inducedFlowFull.ptr<cv::Vec2f>(row);
        cv::Vec2f *recomputed = recomputedResidual.ptr<cv::Vec2f>(row);
        const cv::Vec2f *observed =
            result.observedFlowFull.ptr<cv::Vec2f>(row);
        for(int col=0; col<fullSize.width; ++col)
        {
            const double denominator =
                H.at<double>(2,0)*col+H.at<double>(2,1)*row+
                H.at<double>(2,2);
            if(!std::isfinite(denominator) || std::fabs(denominator)<1e-12)
                throw std::runtime_error(
                    "invalid homography denominator in dense-flow replay");
            const double referenceX =
                (H.at<double>(0,0)*col+H.at<double>(0,1)*row+
                 H.at<double>(0,2))/denominator;
            const double referenceY =
                (H.at<double>(1,0)*col+H.at<double>(1,1)*row+
                 H.at<double>(1,2))/denominator;
            induced[col] = cv::Vec2f(
                static_cast<float>(col-referenceX),
                static_cast<float>(row-referenceY));
            recomputed[col] = observed[col]-induced[col];
        }
    }
    result.stats.residualRecomputeMaxAbsPx = cv::norm(
        recomputedResidual,result.residualFlowFull,cv::NORM_INF);
    if(result.stats.residualRecomputeMaxAbsPx>
       mConfig.residualRecomputeTolerancePx)
    {
        throw std::runtime_error(
            "dense-flow residual reconstruction tolerance exceeded");
    }

    cv::Mat residualParts[2];
    cv::split(result.residualFlowFull,residualParts);
    cv::magnitude(residualParts[0],residualParts[1],
                  result.residualMagnitudePx);
    double maximumResidual = 0.0;
    cv::minMaxLoc(result.residualMagnitudePx,nullptr,&maximumResidual);
    if(std::fabs(maximumResidual-result.stats.maxResidualPx)>1e-4)
        throw std::runtime_error("dense-flow maximum residual mismatch");
    cv::Mat observedParts[2];
    cv::Mat observedMagnitude;
    cv::split(result.observedFlowFull,observedParts);
    cv::magnitude(observedParts[0],observedParts[1],observedMagnitude);
    double maximumObservedFlow = 0.0;
    cv::minMaxLoc(observedMagnitude,nullptr,&maximumObservedFlow);
    if(std::fabs(maximumObservedFlow-result.stats.maxObservedFlowPx)>1e-4)
        throw std::runtime_error("dense-flow maximum observed flow mismatch");
    const double expectedLowThresholdPx =
        result.stats.lowThresholdU8*maximumResidual/255.0;
    const double expectedHighThresholdPx =
        result.stats.highThresholdU8*maximumResidual/255.0;
    if(std::fabs(expectedLowThresholdPx-result.stats.lowThresholdPx)>1e-4 ||
       std::fabs(expectedHighThresholdPx-result.stats.highThresholdPx)>1e-4)
    {
        throw std::runtime_error(
            "dense-flow physical threshold metadata mismatch");
    }
    cv::Mat expectedNormalized;
    if(maximumResidual>0.0)
    {
        result.residualMagnitudePx.convertTo(
            expectedNormalized,CV_8UC1,255.0/maximumResidual);
    }
    else
    {
        expectedNormalized = cv::Mat(fullSize,CV_8UC1,cv::Scalar(0));
    }
    result.stats.normalizedRecomputeMaxAbs = static_cast<int>(cv::norm(
        expectedNormalized,result.normalizedResidual,cv::NORM_INF));
    if(result.stats.normalizedRecomputeMaxAbs>
       mConfig.normalizedResidualTolerance)
    {
        throw std::runtime_error(
            "dense-flow normalized residual tolerance exceeded");
    }

    cv::Mat expectedLow =
        result.normalizedResidual>result.stats.lowThresholdU8;
    cv::Mat expectedHigh =
        result.normalizedResidual>result.stats.highThresholdU8;
    cv::Mat mismatch;
    cv::bitwise_xor(expectedLow,result.lowResidualMask,mismatch);
    if(cv::countNonZero(mismatch)!=0)
        throw std::runtime_error("dense-flow low threshold mask mismatch");
    cv::bitwise_xor(expectedHigh,result.highResidualMask,mismatch);
    if(cv::countNonZero(mismatch)!=0)
        throw std::runtime_error("dense-flow high threshold mask mismatch");
    cv::Mat highOutsideLow;
    cv::bitwise_and(result.highResidualMask,
                    result.lowResidualMask==0,highOutsideLow);
    if(cv::countNonZero(highOutsideLow)!=0 ||
       CountMask(result.lowResidualMask)!=result.stats.lowPixels ||
       CountMask(result.highResidualMask)!=result.stats.highPixels)
    {
        throw std::runtime_error("dense-flow threshold support invariant failed");
    }
    if(result.stats.referenceIndex<0 ||
       result.stats.actualReferenceLag!=
           static_cast<int>(frameIndex)-result.stats.referenceIndex ||
       (result.stats.intendedReferenceLag!=1 &&
        result.stats.intendedReferenceLag!=2))
    {
        throw std::runtime_error("dense-flow reference metadata mismatch");
    }

    const std::chrono::steady_clock::time_point end =
        std::chrono::steady_clock::now();
    result.stats.validateMs = Milliseconds(loadEnd,end);
    result.stats.totalMs = Milliseconds(start,end);
        result.stats.available = true;
        result.stats.dynamicStateAvailable = false;
        result.stats.failureReason = "none";
        return result;
    }
    catch(const std::exception &)
    {
        if(mConfig.requireReference)
            throw;
        SInStyleDenseFlowResidualResult unavailable;
        unavailable.stats.enabled = true;
        unavailable.stats.backend = mConfig.backend;
        unavailable.stats.frameIndex = frameIndex;
        unavailable.stats.failureReason = "reference_invalid";
        const std::chrono::steady_clock::time_point end =
            std::chrono::steady_clock::now();
        unavailable.stats.totalMs = Milliseconds(start,end);
        return unavailable;
    }
}

void SInStyleDenseFlowResidualEstimator::CacheNativeFrame(
    std::size_t frameIndex, const cv::Mat &currentGray)
{
    if(mbNativeLastFrameValid)
    {
        mNativeSecondLastGray = mNativeLastGray;
        mNativeSecondLastFrameIndex = mNativeLastFrameIndex;
        mbNativeSecondLastFrameValid = true;
    }
    mNativeLastGray = currentGray.clone();
    mNativeLastFrameIndex = frameIndex;
    mbNativeLastFrameValid = true;
}

SInStyleDenseFlowResidualResult
SInStyleDenseFlowResidualEstimator::ProcessNativeDeepFlow(
    std::size_t frameIndex, const cv::Mat &currentGray)
{
    SInStyleDenseFlowResidualResult result;
    result.stats.enabled = mConfig.enabled;
    result.stats.backend = "DeepFlow_CPU";
    result.stats.frameIndex = frameIndex;
    if(!mConfig.enabled)
        return result;

    const std::chrono::steady_clock::time_point start =
        std::chrono::steady_clock::now();
    result.stats.failureReason = "history_unavailable";
    if(currentGray.empty() || currentGray.type()!=CV_8UC1)
    {
        result.stats.failureReason = "invalid_gray_input";
        result.stats.totalMs = Milliseconds(start,
            std::chrono::steady_clock::now());
        return result;
    }
    if(mbNativeLastFrameValid && frameIndex!=mNativeLastFrameIndex+1)
    {
        Reset();
        CacheNativeFrame(frameIndex,currentGray);
        result.stats.failureReason = "frame_discontinuity_reset";
        result.stats.totalMs = Milliseconds(start,
            std::chrono::steady_clock::now());
        return result;
    }
    if(!mbNativeLastFrameValid)
    {
        CacheNativeFrame(frameIndex,currentGray);
        result.stats.totalMs = Milliseconds(start,
            std::chrono::steady_clock::now());
        return result;
    }

    try
    {
        const float imageScale = 0.6f;
        const cv::Size fullSize = currentGray.size();
        if(fullSize.width<=0 || fullSize.height<=0)
            throw std::runtime_error("invalid native DeepFlow image size");
        const cv::Size scaledSize(
            static_cast<int>(imageScale*fullSize.width),
            static_cast<int>(imageScale*fullSize.height));
        if(scaledSize.width<=0 || scaledSize.height<=0)
            throw std::runtime_error("invalid native DeepFlow scaled size");

        const cv::Mat &initialReference = mbNativeSecondLastFrameValid ?
            mNativeSecondLastGray : mNativeLastGray;
        std::size_t referenceIndex = mbNativeSecondLastFrameValid ?
            mNativeSecondLastFrameIndex : mNativeLastFrameIndex;
        cv::Mat currentScaled;
        cv::Mat referenceScaled;
        cv::resize(currentGray,currentScaled,scaledSize);
        cv::resize(initialReference,referenceScaled,scaledSize);

        cv::Ptr<cv::DenseOpticalFlow> deepFlow =
            cv::optflow::createOptFlow_DeepFlow();
        cv::Mat selectedFlowNative;
        deepFlow->calc(currentScaled,referenceScaled,selectedFlowNative);
        selectedFlowNative *= -1.0f;
        if(selectedFlowNative.type()!=CV_32FC2 ||
           !IsFiniteMatrix(selectedFlowNative))
        {
            throw std::runtime_error("invalid native DeepFlow output");
        }

        cv::Mat selectedParts[2];
        cv::Mat selectedMagnitude;
        cv::split(selectedFlowNative,selectedParts);
        cv::magnitude(selectedParts[0],selectedParts[1],selectedMagnitude);
        double maximumNativeFlow = 0.0;
        cv::minMaxLoc(selectedMagnitude,nullptr,&maximumNativeFlow);
        if(!std::isfinite(maximumNativeFlow) || maximumNativeFlow<=0.0)
            throw std::runtime_error("native DeepFlow has zero motion scale");
        cv::Mat normalizedNativeMagnitude =
            selectedMagnitude*(255.0/maximumNativeFlow);
        normalizedNativeMagnitude.convertTo(
            normalizedNativeMagnitude,CV_8UC1);
        const int histogramSize = 256;
        const float histogramRangeValues[] = {0.0f,256.0f};
        const float *histogramRange = histogramRangeValues;
        cv::Mat histogram;
        cv::calcHist(&normalizedNativeMagnitude,1,nullptr,cv::Mat(),
                     histogram,1,&histogramSize,&histogramRange,true,false);
        float accumulatedPixels = 0.0f;
        int percentileBin = 0;
        const int tenPixelBin = static_cast<int>(
            10.0f*imageScale*255.0f/
            static_cast<float>(maximumNativeFlow));
        const float scaledPixelCount = static_cast<float>(
            fullSize.width*fullSize.height)*imageScale*imageScale;
        for(int bin=0; bin<255; ++bin)
        {
            accumulatedPixels += histogram.at<float>(bin);
            if(accumulatedPixels>0.3f*scaledPixelCount)
            {
                percentileBin = bin;
                break;
            }
        }
        bool largeMotion = percentileBin>tenPixelBin;
        if(largeMotion)
        {
            referenceIndex = mNativeLastFrameIndex;
            cv::resize(mNativeLastGray,referenceScaled,scaledSize);
            deepFlow->calc(currentScaled,referenceScaled,
                           selectedFlowNative);
            selectedFlowNative *= -1.0f;
            if(selectedFlowNative.type()!=CV_32FC2 ||
               !IsFiniteMatrix(selectedFlowNative))
            {
                throw std::runtime_error(
                    "invalid native DeepFlow fallback output");
            }
        }
        result.rawBackendFlowNative = selectedFlowNative.clone();

        cv::Ptr<cv::VariationalRefinement> refinement =
            cv::VariationalRefinement::create();
        refinement->calc(currentScaled,referenceScaled,selectedFlowNative);
        cv::resize(selectedFlowNative,result.observedFlowFull,fullSize);
        result.observedFlowFull *= 1.0f/imageScale;
        if(!IsFiniteMatrix(result.observedFlowFull))
            throw std::runtime_error("invalid refined native DeepFlow");

        cv::Mat observedParts[2];
        cv::Mat observedMagnitude;
        cv::split(result.observedFlowFull,observedParts);
        cv::magnitude(observedParts[0],observedParts[1],observedMagnitude);
        cv::minMaxLoc(observedMagnitude,nullptr,
                      &result.stats.maxObservedFlowPx);

        cv::RNG random(12345);
        std::vector<WeightedFlowSample> samples;
        samples.reserve(static_cast<std::size_t>(
            fullSize.width*fullSize.height*0.02f));
        const bool useTemporalPrior =
            mConfig.useTemporalRegionPrior &&
            mbTemporalRegionPriorValid &&
            mTemporalPriorFrameIndex+1==frameIndex &&
            mTemporalDetectorStateMask.size()==fullSize &&
            mTemporalRegionLabels.size()==fullSize;
        std::vector<float> regionDynamicFractions;
        if(useTemporalPrior)
        {
            double maximumLabelValue = 0.0;
            cv::minMaxLoc(mTemporalRegionLabels,nullptr,
                          &maximumLabelValue);
            const int maximumLabel = std::max(
                0,static_cast<int>(maximumLabelValue));
            regionDynamicFractions.assign(
                static_cast<std::size_t>(maximumLabel+1),0.0f);
            cv::Mat previousDynamic;
            cv::compare(mTemporalDetectorStateMask,255,
                        previousDynamic,cv::CMP_EQ);
            for(int label=1; label<=maximumLabel; ++label)
            {
                cv::Mat region;
                cv::compare(mTemporalRegionLabels,label,region,cv::CMP_EQ);
                cv::Mat regionDynamic;
                cv::bitwise_and(region,previousDynamic,regionDynamic);
                regionDynamicFractions[static_cast<std::size_t>(label)] =
                    static_cast<float>(cv::countNonZero(regionDynamic))/
                    static_cast<float>(cv::countNonZero(region)+1);
            }
        }
        for(int row=10; row<fullSize.height; row+=10)
        {
            for(int col=10; col<fullSize.width; col+=10)
            {
                const float randomWeight =
                    static_cast<float>(random.gaussian(0.5));
                float weight = randomWeight+1.0f;
                if(useTemporalPrior)
                {
                    const unsigned char state =
                        mTemporalDetectorStateMask.at<unsigned char>(row,col);
                    if(state<20)
                    {
                        ++result.stats.temporalUnknownSamples;
                    }
                    else if(static_cast<unsigned int>(state-20)<=210u)
                    {
                        ++result.stats.temporalStaticSamples;
                        const int label =
                            mTemporalRegionLabels.at<int>(row,col);
                        const float dynamicFraction =
                            label>=0 && label<static_cast<int>(
                                regionDynamicFractions.size()) ?
                            regionDynamicFractions[
                                static_cast<std::size_t>(label)] : 0.0f;
                        weight = randomWeight+
                            1.2f*(1.0f-dynamicFraction);
                    }
                    else
                    {
                        ++result.stats.temporalDynamicSamples;
                        weight = randomWeight+0.4f;
                    }
                }
                samples.emplace_back(col,row,weight);
            }
        }
        std::sort(samples.begin(),samples.end(),
            [](const WeightedFlowSample &left,
               const WeightedFlowSample &right)
            {
                return left.weight>right.weight;
            });
        std::vector<cv::Point2f> currentPoints;
        std::vector<cv::Point2f> referencePoints;
        currentPoints.reserve(samples.size());
        referencePoints.reserve(samples.size());
        for(const WeightedFlowSample &sample : samples)
        {
            const cv::Vec2f flow = result.observedFlowFull.at<cv::Vec2f>(
                sample.y,sample.x);
            const float referenceX = sample.x-flow[0];
            const float referenceY = sample.y-flow[1];
            if(SourceStyleInBorder(referenceY,referenceX,
                                   fullSize.height,fullSize.width))
            {
                currentPoints.emplace_back(
                    static_cast<float>(sample.x),
                    static_cast<float>(sample.y));
                referencePoints.emplace_back(referenceX,referenceY);
            }
        }
        if(currentPoints.size()<4)
            throw std::runtime_error("insufficient homography samples");
        result.homographyCurrentToReference = cv::findHomography(
            currentPoints,referencePoints,cv::noArray(),cv::RHO);
        if(result.homographyCurrentToReference.rows!=3 ||
           result.homographyCurrentToReference.cols!=3 ||
           !IsFiniteMatrix(result.homographyCurrentToReference))
        {
            throw std::runtime_error("native DeepFlow homography failed");
        }
        result.homographyCurrentToReference.convertTo(
            result.homographyCurrentToReference,CV_64FC1);

        result.inducedFlowFull.create(fullSize,CV_32FC2);
        result.residualFlowFull.create(fullSize,CV_32FC2);
        const cv::Mat &H = result.homographyCurrentToReference;
        for(int row=0; row<fullSize.height; ++row)
        {
            cv::Vec2f *induced =
                result.inducedFlowFull.ptr<cv::Vec2f>(row);
            cv::Vec2f *residual =
                result.residualFlowFull.ptr<cv::Vec2f>(row);
            const cv::Vec2f *observed =
                result.observedFlowFull.ptr<cv::Vec2f>(row);
            for(int col=0; col<fullSize.width; ++col)
            {
                const double denominator =
                    H.at<double>(2,0)*col+H.at<double>(2,1)*row+
                    H.at<double>(2,2);
                if(!std::isfinite(denominator) ||
                   std::fabs(denominator)<1e-12)
                {
                    throw std::runtime_error(
                        "invalid native homography denominator");
                }
                const double inducedX =
                    col-(H.at<double>(0,0)*col+
                         H.at<double>(0,1)*row+H.at<double>(0,2))/
                        denominator;
                const double inducedY =
                    row-(H.at<double>(1,0)*col+
                         H.at<double>(1,1)*row+H.at<double>(1,2))/
                        denominator;
                induced[col] = cv::Vec2f(
                    static_cast<float>(inducedX),
                    static_cast<float>(inducedY));
                residual[col][0] = observed[col][0]-
                    static_cast<float>(inducedX);
                residual[col][1] = observed[col][1]-
                    static_cast<float>(inducedY);
            }
        }

        cv::Mat residualParts[2];
        cv::split(result.residualFlowFull,residualParts);
        cv::magnitude(residualParts[0],residualParts[1],
                      result.residualMagnitudePx);
        cv::minMaxLoc(result.residualMagnitudePx,nullptr,
                      &result.stats.maxResidualPx);
        if(!std::isfinite(result.stats.maxResidualPx) ||
           result.stats.maxResidualPx<=0.0)
        {
            throw std::runtime_error("native residual has zero scale");
        }
        cv::Mat normalizedFloat = result.residualMagnitudePx*
            (255.0/result.stats.maxResidualPx);
        normalizedFloat.convertTo(result.normalizedResidual,CV_8UC1);

        cv::Mat otsuMask;
        cv::Mat triangleMask;
        const float otsuThreshold = static_cast<float>(cv::threshold(
            result.normalizedResidual,otsuMask,80,255,cv::THRESH_OTSU));
        const float triangleThreshold = static_cast<float>(cv::threshold(
            result.normalizedResidual,triangleMask,80,255,
            cv::THRESH_TRIANGLE));
        result.stats.otsuThresholdU8 = otsuThreshold;
        result.stats.triangleThresholdU8 = triangleThreshold;
        float lowThreshold = std::min(otsuThreshold,triangleThreshold);
        float highThreshold = std::max(otsuThreshold,triangleThreshold);
        const float maximumResidualFloat =
            static_cast<float>(result.stats.maxResidualPx);
        const float minimumLow =
            1.7f*255.0f/maximumResidualFloat;
        const float maximumLow =
            3.0f*255.0f/maximumResidualFloat;
        if(lowThreshold<minimumLow)
            lowThreshold = minimumLow;
        else if(lowThreshold>maximumLow)
            lowThreshold = maximumLow;
        result.lowResidualMask = result.normalizedResidual>lowThreshold;
        if(cv::countNonZero(result.lowResidualMask)>
           0.5*fullSize.width*fullSize.height)
        {
            lowThreshold += 0.2f*255.0f/maximumResidualFloat;
            result.lowResidualMask = result.normalizedResidual>lowThreshold;
        }
        const float minimumHigh = std::max(
            3.0f*255.0f/maximumResidualFloat,lowThreshold*1.2f);
        const float maximumHigh =
            10.0f*255.0f/maximumResidualFloat;
        if(highThreshold<minimumHigh)
            highThreshold = minimumHigh;
        else if(highThreshold>maximumHigh)
            highThreshold = maximumHigh;
        result.highResidualMask = result.normalizedResidual>highThreshold;

        result.stats.available = true;
        result.stats.dynamicStateAvailable = false;
        result.stats.failureReason = "none";
        result.stats.imageScale = imageScale;
        result.stats.intendedReferenceLag = largeMotion ? 1 : 2;
        result.stats.referenceIndex = static_cast<int>(referenceIndex);
        result.stats.actualReferenceLag = static_cast<int>(
            frameIndex-referenceIndex);
        result.stats.largeMotion = largeMotion;
        result.stats.homographySampleCount =
            static_cast<int>(currentPoints.size());
        result.stats.temporalRegionPriorUsed = useTemporalPrior;
        result.stats.lowThresholdU8 = lowThreshold;
        result.stats.highThresholdU8 = highThreshold;
        result.stats.lowThresholdPx =
            lowThreshold*result.stats.maxResidualPx/255.0;
        result.stats.highThresholdPx =
            highThreshold*result.stats.maxResidualPx/255.0;
        result.stats.lowPixels = CountMask(result.lowResidualMask);
        result.stats.highPixels = CountMask(result.highResidualMask);
        result.stats.totalMs = Milliseconds(start,
            std::chrono::steady_clock::now());
        CacheNativeFrame(frameIndex,currentGray);
        return result;
    }
    catch(const std::exception &)
    {
        Reset();
        CacheNativeFrame(frameIndex,currentGray);
        result.stats.failureReason = "native_computation_failed";
        result.stats.totalMs = Milliseconds(start,
            std::chrono::steady_clock::now());
        return result;
    }
}

} // namespace ORB_SLAM2
