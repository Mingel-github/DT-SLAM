#include "SInStyleRegionDynamicClassifier.h"

#include <opencv2/imgcodecs.hpp>

#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>

using ORB_SLAM2::SInStyleRegionDynamicClassifier;
using ORB_SLAM2::SInStyleRegionDynamicConfig;
using ORB_SLAM2::SInStyleRegionDynamicResult;

namespace
{

void Require(bool condition, const std::string &message)
{
    if(!condition)
        throw std::runtime_error(message);
}

std::string FrameStem(const std::string &directory, int frameIndex)
{
    std::ostringstream stream;
    stream << directory << "/frame_" << std::setw(6)
           << std::setfill('0') << frameIndex;
    return stream.str();
}

cv::Mat ReadGray(const std::string &path)
{
    const cv::Mat image = cv::imread(path,cv::IMREAD_GRAYSCALE);
    Require(!image.empty(),"cannot read audit image: "+path);
    return image;
}

std::size_t PixelMismatch(const cv::Mat &left, const cv::Mat &right)
{
    Require(!left.empty() && left.type()==CV_8UC1 &&
            right.type()==CV_8UC1 && left.size()==right.size(),
            "mask shape mismatch");
    cv::Mat mismatch;
    cv::compare(left,right,mismatch,cv::CMP_NE);
    return static_cast<std::size_t>(cv::countNonZero(mismatch));
}

void RunSyntheticStateTests()
{
    const cv::Size size(80,60);
    cv::Mat labels(size,CV_32SC1,cv::Scalar(1));
    labels(cv::Rect(40,0,40,60)).setTo(2);
    const cv::Mat valid(size,CV_8UC1,cv::Scalar(255));
    cv::Mat aboveLow(size,CV_8UC1,cv::Scalar(0));
    aboveLow(cv::Rect(5,5,70,50)).setTo(255);
    cv::Mat high(size,CV_8UC1,cv::Scalar(0));
    high(cv::Rect(10,10,20,20)).setTo(255);

    SInStyleRegionDynamicClassifier classifier;
    SInStyleRegionDynamicConfig config;
    config.enabled = true;
    classifier.Configure(config);
    const SInStyleRegionDynamicResult first = classifier.Compute(
        0,labels,valid,aboveLow,high);
    Require(first.stats.available && first.stats.dynamicStateAvailable,
            "synthetic first result unavailable");
    cv::Mat rightRegion;
    cv::compare(labels,2,rightRegion,cv::CMP_EQ);
    cv::Mat coreAcrossLabel;
    cv::bitwise_and(first.filledDynamicMaskBeforeDilation,
                    rightRegion,coreAcrossLabel);
    Require(cv::countNonZero(coreAcrossLabel)==0,
            "cluster-confined core crossed region label");

    const cv::Mat emptyEvidence(size,CV_8UC1,cv::Scalar(0));
    const SInStyleRegionDynamicResult second = classifier.Compute(
        1,labels,valid,emptyEvidence,emptyEvidence);
    Require(second.stats.available &&
            second.stats.temporalHighPixelsAdded>0 &&
            second.stats.dynamicPixelsBeforeDilation==0,
            "previous high residual incorrectly became a current seed");

    classifier.Reset();
    const SInStyleRegionDynamicResult afterReset = classifier.Compute(
        7,labels,valid,emptyEvidence,emptyEvidence);
    Require(afterReset.stats.available &&
            afterReset.stats.temporalHighPixelsAdded==0 &&
            afterReset.stats.dynamicPixelsBeforeDilation==0,
            "classifier reset retained temporal evidence");
}

} // namespace

int main(int argc, char **argv)
{
    if(argc!=4)
    {
        std::cerr << "usage: " << argv[0]
                  << " reference_directory first_frame last_frame\n";
        return 2;
    }
    const std::string directory = argv[1];
    const int firstFrame = std::atoi(argv[2]);
    const int lastFrame = std::atoi(argv[3]);
    Require(firstFrame>=1 && lastFrame>=firstFrame,
            "invalid frame range");

    RunSyntheticStateTests();

    SInStyleRegionDynamicClassifier classifier;
    SInStyleRegionDynamicConfig config;
    config.enabled = true;
    classifier.Configure(config);

    std::size_t totalRawStateMismatch = 0;
    std::size_t maximumRawStateMismatch = 0;
    int mismatchingFrames = 0;
    std::size_t totalAuthorDynamicPixels = 0;
    std::size_t totalValidDynamicPixels = 0;
    std::size_t totalUnknownPixels = 0;

    for(int frameIndex=firstFrame; frameIndex<=lastFrame; ++frameIndex)
    {
        const std::string stem = FrameStem(directory,frameIndex);
        const cv::Mat labels8 = ReadGray(stem+"_labels.png");
        cv::Mat labels32;
        labels8.convertTo(labels32,CV_32SC1);
        const cv::Mat valid = ReadGray(stem+"_region_valid.png");
        const cv::Mat low = ReadGray(stem+"_threshold_low.png");
        const cv::Mat high = ReadGray(stem+"_threshold_high.png");
        const cv::Mat expected = ReadGray(
            stem+"_mask_pre_runner_dilate.png");

        const SInStyleRegionDynamicResult result = classifier.Compute(
            static_cast<std::size_t>(frameIndex),labels32,valid,low,high);
        Require(result.stats.available &&
                result.stats.dynamicStateAvailable &&
                result.stats.failureReason=="none",
                "classifier evidence unavailable at frame "+
                std::to_string(frameIndex));

        const std::size_t mismatch = PixelMismatch(
            result.rawStateMask,expected);
        totalRawStateMismatch += mismatch;
        maximumRawStateMismatch = std::max(maximumRawStateMismatch,mismatch);
        if(mismatch!=0)
        {
            ++mismatchingFrames;
            std::cout << "frame=" << frameIndex
                      << " raw_state_mismatch=" << mismatch
                      << " expected_dynamic="
                      << cv::countNonZero(expected==255)
                      << " actual_author_dynamic="
                      << result.stats.authorStyleDynamicPixels
                      << " valid_dynamic="
                      << result.stats.depthSupportedDynamicPixels << '\n';
        }
        totalAuthorDynamicPixels += result.stats.authorStyleDynamicPixels;
        totalValidDynamicPixels += result.stats.depthSupportedDynamicPixels;
        totalUnknownPixels += result.stats.unknownPixels;
    }

    std::cout << "SIn region classifier parity "
              << (lastFrame-firstFrame+1)
              << " frames: raw_state_mismatch_total="
              << totalRawStateMismatch
              << " raw_state_mismatch_max=" << maximumRawStateMismatch
              << " mismatching_frames=" << mismatchingFrames
              << " author_dynamic_pixels=" << totalAuthorDynamicPixels
              << " valid_dynamic_pixels=" << totalValidDynamicPixels
              << " unknown_pixels=" << totalUnknownPixels
              << " dynamic_decision=shadow_only"
              << " direct_slam_state_mutation=none\n";

    Require(totalRawStateMismatch==0,
            "region classifier parity mismatch");
    return 0;
}
