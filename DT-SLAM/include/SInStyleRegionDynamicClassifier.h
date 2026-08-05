/**
 * SIn-style cluster-confined residual classification for S1 shadow mode.
 *
 * This component converts low/high dense-flow residual evidence into a
 * three-state region result. It owns detector history only; it cannot access
 * or mutate Frame, MapPoint, Optimizer, semantic masks, or SLAM state.
 */

#ifndef SIN_STYLE_REGION_DYNAMIC_CLASSIFIER_H
#define SIN_STYLE_REGION_DYNAMIC_CLASSIFIER_H

#include <cstddef>
#include <string>
#include <vector>

#include <opencv2/core/core.hpp>

namespace ORB_SLAM2
{

struct SInStyleRegionDynamicConfig
{
    bool enabled = false;
    bool usePreviousHighResidual = true;
    int minimumClusterHighPixels = 100;
    double minimumContourAreaPixels = 100.0;
    double minimumContourRoundness = 0.2;
    double largeContourAreaPixels = 2000.0;
    double wholeRegionFillFraction = 0.5;
    int lowResidualDilationSize = 5;
    int outputDilationSize = 9;
};

struct SInStyleRegionDynamicStats
{
    bool enabled = false;
    bool available = false;
    bool dynamicStateAvailable = false;
    std::string failureReason = "disabled";
    std::size_t frameIndex = 0;
    std::size_t validRegionPixels = 0;
    std::size_t unknownPixels = 0;
    std::size_t lowResidualPixels = 0;
    std::size_t highResidualPixels = 0;
    std::size_t temporalHighPixelsAdded = 0;
    int regionCount = 0;
    int regionsWithoutHighSupport = 0;
    int regionsWithHighSupport = 0;
    int eligibleContourCount = 0;
    int validSeedContourCount = 0;
    int wholeDynamicRegionCount = 0;
    int partialDynamicRegionCount = 0;
    std::size_t dynamicPixelsBeforeDilation = 0;
    std::size_t authorStyleDynamicPixels = 0;
    std::size_t depthSupportedDynamicPixels = 0;
    std::size_t staticPixels = 0;
    double prepareMs = 0.0;
    double classifyMs = 0.0;
    double totalMs = 0.0;
};

// Read-only explanation of the existing classifier branch taken for one
// region.  These fields must never be consumed by the classifier itself.
struct SInStyleRegionDecisionAudit
{
    int regionLabel = 0;
    int regionPixels = 0;
    int currentHighPixels = 0;
    bool passedMinimumHighPixels = false;
    int highContourCount = 0;
    int eligibleContourCount = 0;
    int validSeedContourCount = 0;
    int filledPixels = 0;
    double filledFraction = 0.0;
    std::string outputState = "none";
    std::string decisionReason = "not_evaluated";
};

struct SInStyleRegionDynamicResult
{
    // Author-compatible raw state: 0 unknown, 125 static, 255 dynamic.
    cv::Mat rawStateMask;
    cv::Mat validRegionMask;
    cv::Mat unknownMask;
    cv::Mat staticMask;
    // Dilation is retained for behavior audit, including pixels outside the
    // region-valid domain. Consumers must use dynamicMask for valid evidence.
    cv::Mat authorStyleDynamicMask;
    cv::Mat dynamicMask;
    // R1 audit matrices. They are snapshots of evidence already used by the
    // existing decision and do not feed back into classification.
    cv::Mat currentAboveLowMask;
    cv::Mat currentHighResidualMask;
    cv::Mat previousHighResidualMask;
    cv::Mat temporalHighAddedMask;
    cv::Mat aboveLowSupportBeforeDilation;
    cv::Mat lowResidualSupportMask;
    cv::Mat filledDynamicMaskBeforeDilation;
    std::vector<SInStyleRegionDecisionAudit> regionDecisionAudits;
    SInStyleRegionDynamicStats stats;
};

class SInStyleRegionDynamicClassifier
{
public:
    void Configure(const SInStyleRegionDynamicConfig &config);
    void Reset();

    SInStyleRegionDynamicResult Compute(
        std::size_t frameIndex,
        const cv::Mat &regionLabels,
        const cv::Mat &regionValidMask,
        const cv::Mat &lowResidualMask,
        const cv::Mat &highResidualMask,
        bool collectAudit = false);

private:
    SInStyleRegionDynamicConfig mConfig;
    cv::Mat mPreviousHighResidualMask;
    std::size_t mPreviousFrameIndex = 0;
    bool mbPreviousHighResidualValid = false;
};

} // namespace ORB_SLAM2

#endif // SIN_STYLE_REGION_DYNAMIC_CLASSIFIER_H
