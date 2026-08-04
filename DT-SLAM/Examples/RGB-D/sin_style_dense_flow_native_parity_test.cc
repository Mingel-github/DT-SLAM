#include "SInStyleDenseFlowResidualEstimator.h"

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

using ORB_SLAM2::SInStyleDenseFlowResidualConfig;
using ORB_SLAM2::SInStyleDenseFlowResidualEstimator;
using ORB_SLAM2::SInStyleDenseFlowResidualResult;

namespace
{

void Require(bool condition, const std::string &message)
{
    if(!condition)
        throw std::runtime_error(message);
}

std::vector<std::string> LoadRgbPaths(const std::string &associationPath)
{
    std::ifstream stream(associationPath.c_str());
    if(!stream.is_open())
        throw std::runtime_error("cannot open association file");
    std::vector<std::string> paths;
    std::string line;
    while(std::getline(stream,line))
    {
        if(line.empty() || line[0]=='#')
            continue;
        std::istringstream row(line);
        double rgbTimestamp = 0.0;
        double depthTimestamp = 0.0;
        std::string rgbPath;
        std::string depthPath;
        if(row >> rgbTimestamp >> rgbPath >> depthTimestamp >> depthPath)
            paths.push_back(rgbPath);
    }
    return paths;
}

double MatrixMaxAbsDifference(const cv::Mat &left, const cv::Mat &right)
{
    Require(!left.empty() && left.type()==right.type() &&
            left.size()==right.size(),"matrix shape mismatch");
    return cv::norm(left,right,cv::NORM_INF);
}

} // namespace

int main(int argc, char **argv)
{
    if(argc!=6 && argc!=7)
    {
        std::cerr << "usage: " << argv[0]
                  << " dataset association reference_directory first_frame"
                     " last_frame [temporal_region_prior]\n";
        return 2;
    }
    const std::string dataset = argv[1];
    const std::vector<std::string> rgbPaths = LoadRgbPaths(argv[2]);
    const int firstFrame = std::atoi(argv[4]);
    const int lastFrame = std::atoi(argv[5]);
    const bool useTemporalRegionPrior =
        argc==7 && std::string(argv[6])=="temporal_region_prior";
    Require(firstFrame==1 && lastFrame>=firstFrame &&
            lastFrame<static_cast<int>(rgbPaths.size()),
            "native parity range must start at frame 1");

    SInStyleDenseFlowResidualEstimator nativeEstimator;
    SInStyleDenseFlowResidualConfig nativeConfig;
    nativeConfig.enabled = true;
    nativeConfig.backend = "deepflow_cpu";
    nativeConfig.useTemporalRegionPrior = useTemporalRegionPrior;
    nativeEstimator.Configure(nativeConfig);

    SInStyleDenseFlowResidualEstimator referenceEstimator;
    SInStyleDenseFlowResidualConfig referenceConfig;
    referenceConfig.enabled = true;
    referenceConfig.backend = "reference_replay";
    referenceConfig.referenceDirectory = argv[3];
    referenceEstimator.Configure(referenceConfig);

    double maximumRawFlowError = 0.0;
    double maximumObservedFlowError = 0.0;
    double maximumHomographyError = 0.0;
    double maximumResidualError = 0.0;
    int maximumNormalizedError = 0;
    std::size_t totalLowMaskMismatch = 0;
    std::size_t totalHighMaskMismatch = 0;
    int referenceSelectionMismatch = 0;
    int thresholdMismatch = 0;

    for(int index=0; index<=lastFrame; ++index)
    {
        const cv::Mat color = cv::imread(
            dataset+"/"+rgbPaths[static_cast<std::size_t>(index)],
            cv::IMREAD_UNCHANGED);
        Require(!color.empty(),"cannot read RGB frame");
        cv::Mat gray;
        if(color.channels()==3)
            cv::cvtColor(color,gray,cv::COLOR_BGR2GRAY);
        else if(color.channels()==4)
            cv::cvtColor(color,gray,cv::COLOR_BGRA2GRAY);
        else
            gray = color;

        const SInStyleDenseFlowResidualResult nativeResult =
            nativeEstimator.Process(static_cast<std::size_t>(index),gray);
        if(index==0)
        {
            Require(!nativeResult.stats.available &&
                    nativeResult.stats.failureReason=="history_unavailable",
                    "native first frame did not remain unknown");
            continue;
        }
        if(index<firstFrame)
            continue;
        const SInStyleDenseFlowResidualResult referenceResult =
            referenceEstimator.Process(static_cast<std::size_t>(index));
        Require(nativeResult.stats.available &&
                !nativeResult.stats.dynamicStateAvailable &&
                referenceResult.stats.available,
                "native or reference evidence unavailable");

        maximumRawFlowError = std::max(maximumRawFlowError,
            MatrixMaxAbsDifference(nativeResult.rawBackendFlowNative,
                                   referenceResult.rawBackendFlowNative));
        maximumObservedFlowError = std::max(maximumObservedFlowError,
            MatrixMaxAbsDifference(nativeResult.observedFlowFull,
                                   referenceResult.observedFlowFull));
        maximumHomographyError = std::max(maximumHomographyError,
            MatrixMaxAbsDifference(
                nativeResult.homographyCurrentToReference,
                referenceResult.homographyCurrentToReference));
        maximumResidualError = std::max(maximumResidualError,
            MatrixMaxAbsDifference(nativeResult.residualFlowFull,
                                   referenceResult.residualFlowFull));
        maximumNormalizedError = std::max(maximumNormalizedError,
            static_cast<int>(MatrixMaxAbsDifference(
                nativeResult.normalizedResidual,
                referenceResult.normalizedResidual)));
        cv::Mat mismatch;
        cv::bitwise_xor(nativeResult.lowResidualMask,
                        referenceResult.lowResidualMask,mismatch);
        totalLowMaskMismatch += static_cast<std::size_t>(
            cv::countNonZero(mismatch));
        cv::bitwise_xor(nativeResult.highResidualMask,
                        referenceResult.highResidualMask,mismatch);
        totalHighMaskMismatch += static_cast<std::size_t>(
            cv::countNonZero(mismatch));
        if(nativeResult.stats.referenceIndex!=
               referenceResult.stats.referenceIndex ||
           nativeResult.stats.largeMotion!=
               referenceResult.stats.largeMotion ||
           nativeResult.stats.homographySampleCount!=
               referenceResult.stats.homographySampleCount)
        {
            ++referenceSelectionMismatch;
        }
        if(std::fabs(nativeResult.stats.lowThresholdU8-
                     referenceResult.stats.lowThresholdU8)>1e-4 ||
           std::fabs(nativeResult.stats.highThresholdU8-
                     referenceResult.stats.highThresholdU8)>1e-4)
        {
            ++thresholdMismatch;
        }
        if(useTemporalRegionPrior)
        {
            const std::string stem = std::string(argv[3])+
                "/frame_"+(index<10 ? "00000" :
                index<100 ? "0000" : index<1000 ? "000" : "")+
                std::to_string(index);
            const cv::Mat state = cv::imread(
                stem+"_mask_pre_runner_dilate.png",cv::IMREAD_GRAYSCALE);
            const cv::Mat labels8 = cv::imread(
                stem+"_labels.png",cv::IMREAD_GRAYSCALE);
            Require(!state.empty() && !labels8.empty(),
                    "cannot read temporal region prior");
            cv::Mat labels32;
            labels8.convertTo(labels32,CV_32SC1);
            nativeEstimator.CommitTemporalRegionPrior(
                static_cast<std::size_t>(index),state,labels32);
        }
    }

    std::cout << "SIn native CPU DeepFlow parity "
              << (lastFrame-firstFrame+1)
              << " frames: raw_flow_max=" << maximumRawFlowError
              << " observed_flow_max=" << maximumObservedFlowError
              << " homography_max=" << maximumHomographyError
              << " residual_max=" << maximumResidualError
              << " normalized_max=" << maximumNormalizedError
              << " low_mask_mismatch=" << totalLowMaskMismatch
              << " high_mask_mismatch=" << totalHighMaskMismatch
              << " reference_selection_mismatch="
              << referenceSelectionMismatch
              << " threshold_mismatch=" << thresholdMismatch
              << " temporal_region_prior=" << useTemporalRegionPrior
              << " dynamic_decision=none direct_slam_state_mutation=none\n";

    const double homographyTolerance =
        useTemporalRegionPrior ? 3e-6 : 1e-6;
    Require(maximumRawFlowError<=1e-4 &&
            maximumObservedFlowError<=1e-4 &&
            maximumHomographyError<=homographyTolerance &&
            maximumResidualError<=1e-3 &&
            maximumNormalizedError<=1 &&
            totalLowMaskMismatch==0 && totalHighMaskMismatch==0 &&
            referenceSelectionMismatch==0 && thresholdMismatch==0,
            "native CPU DeepFlow parity tolerance exceeded");
    return 0;
}
