#include "SInStyleDenseFlowResidualEstimator.h"

#include <opencv2/core/core.hpp>

#include <algorithm>
#include <cstdlib>
#include <iostream>
#include <stdexcept>
#include <string>

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

} // namespace

int main(int argc, char **argv)
{
    if(argc!=4)
    {
        std::cerr << "usage: " << argv[0]
                  << " reference_directory first_frame last_frame\n";
        return 2;
    }
    const int first = std::atoi(argv[2]);
    const int last = std::atoi(argv[3]);
    if(first<1 || last<first)
        throw std::invalid_argument("invalid replay frame range");

    SInStyleDenseFlowResidualEstimator estimator;
    SInStyleDenseFlowResidualConfig disabled;
    estimator.Configure(disabled);
    Require(!estimator.Process(0).stats.available,
            "disabled residual replay became available");

    SInStyleDenseFlowResidualConfig config;
    config.enabled = true;
    config.referenceDirectory = argv[1];
    estimator.Configure(config);
    const SInStyleDenseFlowResidualResult historyUnavailable =
        estimator.Process(0);
    Require(!historyUnavailable.stats.available &&
            historyUnavailable.stats.failureReason=="history_unavailable",
            "frame zero did not remain explicitly unavailable");

    SInStyleDenseFlowResidualConfig nonStrictMissing = config;
    nonStrictMissing.requireReference = false;
    nonStrictMissing.referenceDirectory =
        std::string(argv[1])+"/intentionally_missing";
    estimator.Configure(nonStrictMissing);
    const SInStyleDenseFlowResidualResult missing = estimator.Process(1);
    Require(!missing.stats.available &&
            !missing.stats.dynamicStateAvailable &&
            missing.stats.failureReason=="reference_missing",
            "non-strict missing reference did not remain unknown");
    estimator.Configure(config);

    int largeMotionFrames = 0;
    double maximumResidualRecomputeError = 0.0;
    int maximumNormalizedError = 0;
    for(int index=first; index<=last; ++index)
    {
        const SInStyleDenseFlowResidualResult result =
            estimator.Process(static_cast<std::size_t>(index));
        const SInStyleDenseFlowResidualResult repeat =
            estimator.Process(static_cast<std::size_t>(index));
        Require(result.stats.available &&
                !result.stats.dynamicStateAvailable &&
                result.stats.failureReason=="none",
                "replay did not expose evidence-only state");
        Require(result.stats.highPixels<=result.stats.lowPixels,
                "high support exceeds low support");
        Require(cv::norm(result.residualFlowFull,repeat.residualFlowFull,
                         cv::NORM_INF)==0.0 &&
                cv::norm(result.lowResidualMask,repeat.lowResidualMask,
                         cv::NORM_INF)==0.0 &&
                cv::norm(result.highResidualMask,repeat.highResidualMask,
                         cv::NORM_INF)==0.0,
                "reference replay is not deterministic");
        if(result.stats.largeMotion)
            ++largeMotionFrames;
        maximumResidualRecomputeError = std::max(
            maximumResidualRecomputeError,
            result.stats.residualRecomputeMaxAbsPx);
        maximumNormalizedError = std::max(
            maximumNormalizedError,
            result.stats.normalizedRecomputeMaxAbs);
    }
    std::cout << "SIn dense-flow replay passed " << (last-first+1)
              << " frames; large_motion=" << largeMotionFrames
              << " max_residual_recompute_px="
              << maximumResidualRecomputeError
              << " max_normalized_error=" << maximumNormalizedError
              << " dynamic_decision=none direct_slam_state_mutation=none\n";
    return 0;
}
