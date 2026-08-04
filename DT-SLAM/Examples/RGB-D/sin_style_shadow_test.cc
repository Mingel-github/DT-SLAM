#include "SInStyleDynamicDetector.h"
#include "SInStyleGradientRegionSplitter.h"
#include "SInStyleInitialRegionClusterer.h"
#include "SInStylePlaneEdgeRegionSplitter.h"
#include "SInStyleRAGRegionMerger.h"

#include <iostream>
#include <limits>
#include <stdexcept>

using ORB_SLAM2::SInStyleDynamicDetector;
using ORB_SLAM2::SInStyleGradientRegionSplitter;
using ORB_SLAM2::SInStyleGradientSplitConfig;
using ORB_SLAM2::SInStyleGradientSplitResult;
using ORB_SLAM2::SInStyleInitialRegionClusterer;
using ORB_SLAM2::SInStyleInitialRegionConfig;
using ORB_SLAM2::SInStyleInitialRegionResult;
using ORB_SLAM2::SInStylePlaneEdgeRegionSplitter;
using ORB_SLAM2::SInStylePlaneEdgeSplitConfig;
using ORB_SLAM2::SInStylePlaneEdgeSplitResult;
using ORB_SLAM2::SInStyleRAGMergeConfig;
using ORB_SLAM2::SInStyleRAGMergeResult;
using ORB_SLAM2::SInStyleRAGRegionMerger;
using ORB_SLAM2::SInStyleShadowResult;

namespace
{

void Require(const bool condition, const char *message)
{
    if(!condition)
        throw std::runtime_error(message);
}

} // namespace

int main()
{
    cv::Mat state = (cv::Mat_<unsigned char>(2,3) <<
        0,125,255,
        125,255,125);
    cv::Mat depth = (cv::Mat_<float>(2,3) <<
        1.0f,1.0f,1.0f,
        1.0f,0.0f,3.0f);
    cv::Mat labels = (cv::Mat_<unsigned char>(2,3) <<
        0,1,1,
        0,2,2);

    const SInStyleShadowResult result =
        SInStyleDynamicDetector::ConvertReferenceState(state,labels,depth);

    Require(result.stats.pixelCount==6,"pixel count mismatch");
    Require(result.stats.depthValidPixels==5,"depth-valid count mismatch");
    Require(result.stats.rawUnknownPixels==1,"raw unknown count mismatch");
    Require(result.stats.rawStaticPixels==3,"raw static count mismatch");
    Require(result.stats.rawDynamicPixels==2,"raw dynamic count mismatch");
    Require(result.stats.validPixels==4,"valid count mismatch");
    Require(result.stats.staticPixels==3,"static count mismatch");
    Require(result.stats.dynamicPixels==1,"dynamic count mismatch");
    Require(result.stats.unknownPixels==2,"unknown count mismatch");
    Require(cv::countNonZero(result.authorDynamicMask)==2,
            "author dynamic mask must preserve raw dynamic pixels");
    Require(result.stats.positiveLabelCount==2,
            "positive label count mismatch");
    Require(result.stats.positiveLabelPixels==4,
            "positive label pixel count mismatch");
    Require(result.stats.depthSupportedPositiveLabelPixels==3,
            "depth-supported label pixel count mismatch");
    Require(result.stats.positiveLabelComponentCount==2,
            "positive label component count mismatch");
    Require(result.stats.authorDynamicPixelsOnPositiveLabels==2,
            "dynamic pixels on positive labels mismatch");
    Require(result.stats.authorDynamicPixelsOnLabelZero==0,
            "dynamic pixels on label zero mismatch");

    cv::Mat labelsWithDynamicZero = labels.clone();
    labelsWithDynamicZero.at<unsigned char>(0,2) = 0;
    const SInStyleShadowResult dynamicOnZero =
        SInStyleDynamicDetector::ConvertReferenceState(
            state,labelsWithDynamicZero,depth);
    Require(dynamicOnZero.stats.authorDynamicPixelsOnPositiveLabels==1,
            "positive-label dynamic split mismatch");
    Require(dynamicOnZero.stats.authorDynamicPixelsOnLabelZero==1,
            "label-zero dynamic split mismatch");

    cv::Mat dynamicOutsideValid;
    cv::bitwise_and(result.dynamicMask,result.unknownMask,dynamicOutsideValid);
    Require(cv::countNonZero(dynamicOutsideValid)==0,
            "dynamic mask must be a subset of valid mask");

    cv::Mat authorDynamicWithoutDepth;
    cv::bitwise_and(result.authorDynamicMask,result.unknownMask,
                    authorDynamicWithoutDepth);
    Require(cv::countNonZero(authorDynamicWithoutDepth)==1,
            "raw author mask must remain independent of depth support");

    cv::Mat validOrUnknown;
    cv::bitwise_or(result.validMask,result.unknownMask,validOrUnknown);
    Require(cv::countNonZero(validOrUnknown)==6,
            "valid and unknown must cover the image");

    bool invalidValueRejected = false;
    try
    {
        cv::Mat invalid = state.clone();
        invalid.at<unsigned char>(0,0) = 1;
        SInStyleDynamicDetector::ConvertReferenceState(invalid,labels,depth);
    }
    catch(const std::invalid_argument &)
    {
        invalidValueRejected = true;
    }
    Require(invalidValueRejected,"invalid state value was not rejected");

    const SInStyleShadowResult withoutLabels =
        SInStyleDynamicDetector::ConvertReferenceState(
            state,cv::Mat(),depth);
    Require(!withoutLabels.stats.labelsAvailable,
            "missing labels must remain explicit");
    Require(withoutLabels.stats.authorDynamicPixelsWithLabelsUnavailable==2,
            "unassigned dynamic count mismatch when labels are unavailable");

    const cv::Mat cameraMatrix = (cv::Mat_<float>(3,3) <<
        100.0f,0.0f,3.5f,
        0.0f,100.0f,3.5f,
        0.0f,0.0f,1.0f);
    SInStyleInitialRegionConfig initialConfig;
    initialConfig.enabled = true;
    initialConfig.clusterPixelDivisor = 64.0;
    initialConfig.maximumDepthMeters = 6.0f;
    initialConfig.maximumIterations = 20;
    initialConfig.epsilon = 1e-3;
    initialConfig.attempts = 1;
    initialConfig.randomSeed = 2025;

    SInStyleInitialRegionClusterer initialClusterer;
    initialClusterer.Configure(initialConfig,cameraMatrix);
    const cv::Mat planeDepth(8,8,CV_32FC1,cv::Scalar(2.0f));
    const SInStyleInitialRegionResult planeRegions =
        initialClusterer.Compute(planeDepth);
    Require(planeRegions.stats.available,
            "single-region initial partition is unavailable");
    Require(!planeRegions.stats.dynamicStateAvailable,
            "initial regions must not expose a dynamic state");
    Require(planeRegions.stats.requestedClusters==1 &&
            planeRegions.stats.producedClusters==1,
            "single-region cluster count mismatch");
    Require(planeRegions.stats.clusteringDepthValidPixels==64 &&
            cv::countNonZero(planeRegions.validMask)==64,
            "single-region valid coverage mismatch");
    cv::Mat planeLabelMismatch;
    cv::compare(planeRegions.labels,1,planeLabelMismatch,cv::CMP_NE);
    Require(cv::countNonZero(planeLabelMismatch)==0,
            "single-region labels must all be one");

    cv::Mat mixedDepth = planeDepth.clone();
    mixedDepth.at<float>(0,0) = 0.0f;
    mixedDepth.at<float>(0,1) = 6.0f;
    mixedDepth.at<float>(0,2) =
        std::numeric_limits<float>::quiet_NaN();
    const SInStyleInitialRegionResult mixedRegions =
        initialClusterer.Compute(mixedDepth);
    Require(mixedRegions.stats.inputDepthValidPixels==62,
            "input depth-valid count mismatch");
    Require(mixedRegions.stats.excludedFarDepthPixels==1,
            "far-depth exclusion count mismatch");
    Require(mixedRegions.stats.clusteringDepthValidPixels==61,
            "clustering depth-valid count mismatch");
    Require(mixedRegions.labels.at<int>(0,0)==-1 &&
            mixedRegions.labels.at<int>(0,1)==-1 &&
            mixedRegions.labels.at<int>(0,2)==-1,
            "invalid/far depth leaked into initial labels");

    initialConfig.clusterPixelDivisor = 16.0;
    initialClusterer.Configure(initialConfig,cameraMatrix);
    cv::Mat steppedDepth(8,8,CV_32FC1);
    for(int row=0; row<steppedDepth.rows; ++row)
    {
        float *values = steppedDepth.ptr<float>(row);
        for(int col=0; col<steppedDepth.cols; ++col)
            values[col] = 1.0f+0.5f*static_cast<float>(col/2);
    }
    const SInStyleInitialRegionResult firstPartition =
        initialClusterer.Compute(steppedDepth);
    initialClusterer.Reset();
    const SInStyleInitialRegionResult secondPartition =
        initialClusterer.Compute(steppedDepth);
    Require(firstPartition.stats.producedClusters==4,
            "four-region cluster count mismatch");
    cv::Mat deterministicMismatch;
    cv::compare(firstPartition.labels,secondPartition.labels,
                deterministicMismatch,cv::CMP_NE);
    Require(cv::countNonZero(deterministicMismatch)==0,
            "initial partition is not deterministic across reset");

    bool invalidInitialConfigRejected = false;
    try
    {
        SInStyleInitialRegionConfig invalidConfig = initialConfig;
        invalidConfig.clusterPixelDivisor = 0.0;
        initialClusterer.Configure(invalidConfig,cameraMatrix);
    }
    catch(const std::invalid_argument &)
    {
        invalidInitialConfigRejected = true;
    }
    Require(invalidInitialConfigRejected,
            "invalid initial-region config was not rejected");

    bool multipleInitializedAttemptsRejected = false;
    try
    {
        SInStyleInitialRegionConfig invalidConfig = initialConfig;
        invalidConfig.coarseToFine = true;
        invalidConfig.attempts = 2;
        initialClusterer.Configure(invalidConfig,cameraMatrix);
    }
    catch(const std::invalid_argument &)
    {
        multipleInitializedAttemptsRejected = true;
    }
    Require(multipleInitializedAttemptsRejected,
            "coarse-to-fine multiple attempts were not rejected");

    bool excessiveClusterCountRejected = false;
    try
    {
        SInStyleInitialRegionConfig excessiveConfig = initialConfig;
        excessiveConfig.clusterPixelDivisor = 0.0001;
        initialClusterer.Configure(excessiveConfig,cameraMatrix);
        initialClusterer.Compute(planeDepth,0);
    }
    catch(const std::invalid_argument &)
    {
        excessiveClusterCountRejected = true;
    }
    Require(excessiveClusterCountRejected,
            "excessive initial-region cluster count was not rejected");

    SInStyleInitialRegionConfig pyramidConfig = initialConfig;
    pyramidConfig.coarseToFine = true;
    pyramidConfig.pyramidLevels = 4;
    pyramidConfig.temporalInitialization = true;
    pyramidConfig.temporalCommitStartInputIndex = 1;
    pyramidConfig.clusterPixelDivisor = 192.0;
    pyramidConfig.maximumIterations = 4;
    pyramidConfig.epsilon = 0.07;
    cv::Mat pyramidDepth(24,32,CV_32FC1);
    for(int row=0; row<pyramidDepth.rows; ++row)
    {
        float *values = pyramidDepth.ptr<float>(row);
        for(int col=0; col<pyramidDepth.cols; ++col)
            values[col] = 1.0f+0.02f*row+0.04f*(col/8);
    }
    cv::Mat pyramidCamera = (cv::Mat_<float>(3,3) <<
        120.0f,0.0f,15.5f,
        0.0f,120.0f,11.5f,
        0.0f,0.0f,1.0f);
    SInStyleInitialRegionClusterer pyramidClusterer;
    pyramidClusterer.Configure(pyramidConfig,pyramidCamera);
    const std::uint64_t rngBefore = cv::theRNG().state;
    const SInStyleInitialRegionResult pyramidFrame0 =
        pyramidClusterer.Compute(pyramidDepth,0);
    Require(cv::theRNG().state==rngBefore,
            "coarse-to-fine clustering polluted OpenCV RNG state");
    Require(pyramidFrame0.stats.coarseToFine &&
            pyramidFrame0.stats.pyramidLevels==4,
            "coarse-to-fine pyramid metadata mismatch");
    Require(pyramidFrame0.stats.initializationSource=="grid" &&
            !pyramidFrame0.stats.temporalPriorCommitted,
            "author-aligned frame zero must use uncommitted grid init");
    Require(pyramidFrame0.stats.levels.size()==4 &&
            pyramidFrame0.stats.levels.front().rows==3 &&
            pyramidFrame0.stats.levels.front().cols==4 &&
            pyramidFrame0.stats.levels.back().rows==24 &&
            pyramidFrame0.stats.levels.back().cols==32,
            "coarse-to-fine level dimensions mismatch");

    cv::Mat previousWithHole = pyramidDepth.clone();
    previousWithHole(cv::Rect(0,0,8,8)).setTo(0.0f);
    previousWithHole.at<float>(10,10) = 6.0f;
    const SInStyleInitialRegionResult pyramidFrame1 =
        pyramidClusterer.Compute(previousWithHole,1);
    Require(pyramidFrame1.stats.initializationSource=="grid" &&
            pyramidFrame1.stats.temporalPriorCommitted,
            "author-aligned frame one must commit grid init");
    Require(pyramidFrame1.labels.at<int>(0,0)==-1 &&
            pyramidFrame1.labels.at<int>(10,10)==-1,
            "coarse-to-fine invalid/far depth leaked into final labels");
    const SInStyleInitialRegionResult pyramidFrame2 =
        pyramidClusterer.Compute(pyramidDepth,2);
    Require(pyramidFrame2.stats.previousPriorSamples>0 &&
            pyramidFrame2.stats.gridFallbackSamples>0 &&
            pyramidFrame2.stats.initializationSource=="mixed",
            "newly valid depth must use deterministic grid fallback");
    Require(pyramidFrame2.stats.previousPriorSamples+
            pyramidFrame2.stats.gridFallbackSamples==
            pyramidFrame2.stats.levels.front().validSamples,
            "coarsest initialization accounting mismatch");

    const SInStyleInitialRegionResult pyramidGap =
        pyramidClusterer.Compute(pyramidDepth,4);
    Require(pyramidGap.stats.initializationSource=="grid",
            "input-index gap must fail safe to grid initialization");
    pyramidClusterer.Reset();
    const SInStyleInitialRegionResult afterReset =
        pyramidClusterer.Compute(pyramidDepth,5);
    SInStyleInitialRegionClusterer freshPyramidClusterer;
    freshPyramidClusterer.Configure(pyramidConfig,pyramidCamera);
    const SInStyleInitialRegionResult freshAfterReset =
        freshPyramidClusterer.Compute(pyramidDepth,5);
    cv::Mat resetMismatch;
    cv::compare(afterReset.labels,freshAfterReset.labels,
                resetMismatch,cv::CMP_NE);
    Require(cv::countNonZero(resetMismatch)==0 &&
            afterReset.stats.initializationSource=="grid",
            "reset did not restore fresh grid behavior");

    SInStyleInitialRegionConfig oddConfig = pyramidConfig;
    oddConfig.clusterPixelDivisor = 713.0;
    oddConfig.temporalInitialization = false;
    cv::Mat oddDepth(23,31,CV_32FC1,cv::Scalar(2.0f));
    cv::Mat oddCamera = (cv::Mat_<float>(3,3) <<
        100.0f,0.0f,15.0f,
        0.0f,100.0f,11.0f,
        0.0f,0.0f,1.0f);
    SInStyleInitialRegionClusterer oddClusterer;
    oddClusterer.Configure(oddConfig,oddCamera);
    const SInStyleInitialRegionResult oddResult =
        oddClusterer.Compute(oddDepth,0);
    Require(oddResult.stats.levels.size()==4 &&
            oddResult.stats.levels.front().rows==2 &&
            oddResult.stats.levels.front().cols==3 &&
            oddResult.stats.levels.back().rows==23 &&
            oddResult.stats.levels.back().cols==31,
            "odd-sized pyramid dimensions mismatch");

    SInStyleGradientSplitConfig gradientConfig;
    gradientConfig.enabled = true;
    gradientConfig.maximumDepthMeters = 6.0f;
    gradientConfig.relativeThreshold = 0.025f;
    gradientConfig.absoluteThresholdMeters = 0.08f;
    gradientConfig.medianRadius = 2;
    gradientConfig.minimumMedianSupport = 5;
    gradientConfig.connectivity = 8;
    gradientConfig.smallComponentAuditPixels = 80;
    SInStyleGradientRegionSplitter gradientSplitter;
    gradientSplitter.Configure(gradientConfig);

    cv::Mat oneInitialRegion(8,8,CV_32SC1,cv::Scalar(1));
    cv::Mat largeStep(8,8,CV_32FC1);
    for(int row=0; row<largeStep.rows; ++row)
    {
        float *values = largeStep.ptr<float>(row);
        for(int col=0; col<largeStep.cols; ++col)
            values[col] = col<4 ? 1.0f : 1.2f;
    }
    const SInStyleGradientSplitResult largeStepSplit =
        gradientSplitter.Compute(largeStep,oneInitialRegion);
    Require(largeStepSplit.stats.available &&
            !largeStepSplit.stats.dynamicStateAvailable,
            "gradient split must expose shadow evidence only");
    Require(largeStepSplit.stats.rawGradientEdgePixels>0 &&
            largeStepSplit.stats.splitComponentCount==2 &&
            largeStepSplit.stats.splitInitialRegionCount==1,
            "0.2m depth jump was not split into two components");
    Require(largeStepSplit.stats.smallComponentCount==2 &&
            largeStepSplit.stats.smallComponentPixels==
                largeStepSplit.stats.splitCorePixels,
            "small split components were deleted or miscounted");

    cv::Mat smallStep = largeStep.clone();
    smallStep(cv::Rect(4,0,4,8)).setTo(1.05f);
    const SInStyleGradientSplitResult smallStepSplit =
        gradientSplitter.Compute(smallStep,oneInitialRegion);
    Require(smallStepSplit.stats.rawGradientEdgePixels==0 &&
            smallStepSplit.stats.splitComponentCount==1,
            "sub-threshold depth jump created an edge");

    cv::Mat invalidHole(9,9,CV_32FC1,cv::Scalar(2.0f));
    cv::Mat invalidHoleLabels(9,9,CV_32SC1,cv::Scalar(1));
    invalidHole.at<float>(4,4) = 0.0f;
    invalidHoleLabels.at<int>(4,4) = -1;
    const SInStyleGradientSplitResult invalidHoleSplit =
        gradientSplitter.Compute(invalidHole,invalidHoleLabels);
    Require(invalidHoleSplit.stats.rawGradientEdgePixels==0,
            "invalid depth hole manufactured a gradient edge");
    Require(invalidHoleSplit.splitCoreLabels.at<int>(4,4)==-1,
            "invalid depth hole did not remain unmeasured");

    cv::Mat farDepth = invalidHole.clone();
    cv::Mat farLabels = invalidHoleLabels.clone();
    farDepth.at<float>(4,5) = 6.0f;
    farLabels.at<int>(4,5) = -1;
    const SInStyleGradientSplitResult farSplit =
        gradientSplitter.Compute(farDepth,farLabels);
    Require(farSplit.splitCoreLabels.at<int>(4,5)==-1 &&
            farSplit.medianValidMask.at<unsigned char>(4,5)==0,
            "depth at the 6m cutoff leaked into split evidence");

    cv::Mat splitSemantics;
    cv::bitwise_and(largeStepSplit.splitBoundaryMask,
                    largeStepSplit.splitValidMask,splitSemantics);
    Require(cv::countNonZero(splitSemantics)==0,
            "split boundary and core masks overlap");
    Require(largeStepSplit.stats.splitBoundaryPixels+
            largeStepSplit.stats.splitCorePixels==
            largeStepSplit.stats.medianValidPixels,
            "split boundary/core accounting failed");

    bool invalidGradientConfigRejected = false;
    try
    {
        SInStyleGradientSplitConfig invalidGradientConfig = gradientConfig;
        invalidGradientConfig.connectivity = 6;
        gradientSplitter.Configure(invalidGradientConfig);
    }
    catch(const std::invalid_argument &)
    {
        invalidGradientConfigRejected = true;
    }
    Require(invalidGradientConfigRejected,
            "invalid gradient connectivity was not rejected");

    SInStylePlaneEdgeSplitConfig planeEdgeConfig;
    planeEdgeConfig.enabled = true;
    planeEdgeConfig.maximumDepthMeters = 6.0f;
    planeEdgeConfig.blockSize = 8;
    planeEdgeConfig.minimumPlanePixels = 64;
    planeEdgeConfig.distanceThresholdMeters = 0.01;
    planeEdgeConfig.endpointRadius = 2;
    planeEdgeConfig.endpointMaximumSupportExclusive = 5;
    planeEdgeConfig.endpointAssociationRadius = 2;
    planeEdgeConfig.minimumEndpointCountExclusive = 1;
    planeEdgeConfig.connectivity = 8;
    SInStylePlaneEdgeRegionSplitter planeEdgeSplitter;
    planeEdgeSplitter.Configure(planeEdgeConfig,pyramidCamera);

    cv::Mat planeEdgeDepth(24,32,CV_32FC1,cv::Scalar(2.0f));
    cv::Mat planeEdgeInitial(24,32,CV_32SC1,cv::Scalar(1));
    cv::Mat planeEdgeLabels(24,32,CV_32SC1,cv::Scalar(0));
    planeEdgeLabels.colRange(16,32).setTo(1);

    SInStylePlaneEdgeSplitConfig disabledPlaneEdgeConfig = planeEdgeConfig;
    disabledPlaneEdgeConfig.enabled = false;
    SInStylePlaneEdgeRegionSplitter disabledPlaneEdgeSplitter;
    disabledPlaneEdgeSplitter.Configure(disabledPlaneEdgeConfig,pyramidCamera);
    const SInStylePlaneEdgeSplitResult disabledPlaneEdgeResult =
        disabledPlaneEdgeSplitter.Compute(
            planeEdgeDepth,cv::Mat(),cv::Mat());
    const SInStylePlaneEdgeSplitResult disabledInjectedPlaneEdgeResult =
        disabledPlaneEdgeSplitter.ComputeFromPlaneLabels(
            planeEdgeDepth,cv::Mat(),cv::Mat(),cv::Mat());
    Require(!disabledPlaneEdgeResult.stats.available &&
            !disabledInjectedPlaneEdgeResult.stats.available,
            "disabled plane-edge splitter validated unavailable inputs");

    cv::Mat endpointGradient(24,32,CV_8UC1,cv::Scalar(0));
    endpointGradient.at<unsigned char>(5,14) = 255;
    endpointGradient.at<unsigned char>(5,15) = 255;
    endpointGradient.at<unsigned char>(18,14) = 255;
    endpointGradient.at<unsigned char>(18,15) = 255;
    const SInStylePlaneEdgeSplitResult supportedPlaneBoundary =
        planeEdgeSplitter.ComputeFromPlaneLabels(
            planeEdgeDepth,planeEdgeInitial,endpointGradient,
            planeEdgeLabels);
    Require(supportedPlaneBoundary.stats.available &&
            supportedPlaneBoundary.stats.opencvPlaneSubstitute &&
            !supportedPlaneBoundary.stats.dynamicStateAvailable,
            "plane-edge substitute must expose shadow evidence only");
    Require(supportedPlaneBoundary.stats.planeCount==2 &&
            supportedPlaneBoundary.stats.rawPlaneBoundaryPixels>0 &&
            supportedPlaneBoundary.stats.gradientEndpointPixels>=2 &&
            supportedPlaneBoundary.stats.retainedPlaneBoundaryPixels>0,
            "endpoint-supported plane transition was not retained");
    Require(supportedPlaneBoundary.stats.combinedComponentCount>=2 &&
            supportedPlaneBoundary.combinedCoreLabels.at<int>(12,15)==0,
            "retained plane boundary did not split the initial region");

    cv::Mat noEndpointGradient(24,32,CV_8UC1,cv::Scalar(0));
    const SInStylePlaneEdgeSplitResult unsupportedPlaneBoundary =
        planeEdgeSplitter.ComputeFromPlaneLabels(
            planeEdgeDepth,planeEdgeInitial,noEndpointGradient,
            planeEdgeLabels);
    Require(unsupportedPlaneBoundary.stats.gradientEndpointPixels==0 &&
            unsupportedPlaneBoundary.stats.retainedPlaneBoundaryPixels==0 &&
            unsupportedPlaneBoundary.stats.combinedComponentCount==1,
            "unsupported plane boundary was retained");

    cv::Mat planeInvalidDepth = planeEdgeDepth.clone();
    cv::Mat planeInvalidInitial = planeEdgeInitial.clone();
    cv::Mat planeInvalidLabels = planeEdgeLabels.clone();
    planeInvalidDepth.at<float>(12,16) = 0.0f;
    planeInvalidInitial.at<int>(12,16) = -1;
    planeInvalidLabels.at<int>(12,16) = -1;
    const SInStylePlaneEdgeSplitResult invalidPlanePixel =
        planeEdgeSplitter.ComputeFromPlaneLabels(
            planeInvalidDepth,planeInvalidInitial,noEndpointGradient,
            planeInvalidLabels);
    Require(invalidPlanePixel.combinedCoreLabels.at<int>(12,16)==-1,
            "invalid plane pixel did not remain unknown");

    const SInStylePlaneEdgeSplitResult planeRepeat =
        planeEdgeSplitter.ComputeFromPlaneLabels(
            planeEdgeDepth,planeEdgeInitial,endpointGradient,
            planeEdgeLabels);
    cv::Mat planeRepeatDifference;
    cv::compare(supportedPlaneBoundary.combinedCoreLabels,
                planeRepeat.combinedCoreLabels,
                planeRepeatDifference,cv::CMP_NE);
    Require(cv::countNonZero(planeRepeatDifference)==0,
            "plane-edge substitute labels are not deterministic");

    // Exercise the installed OpenCV RgbdPlane backend without asserting an
    // implementation-specific plane count on this synthetic image.
    const SInStylePlaneEdgeSplitResult opencvPlaneSmoke =
        planeEdgeSplitter.Compute(
            planeEdgeDepth,planeEdgeInitial,noEndpointGradient);
    Require(opencvPlaneSmoke.stats.available &&
            opencvPlaneSmoke.planeLabels.size()==planeEdgeDepth.size(),
            "OpenCV RgbdPlane substitute smoke failed");

    bool invalidPlaneConfigRejected = false;
    try
    {
        SInStylePlaneEdgeSplitConfig invalidPlaneConfig = planeEdgeConfig;
        invalidPlaneConfig.blockSize = 0;
        planeEdgeSplitter.Configure(invalidPlaneConfig,pyramidCamera);
    }
    catch(const std::invalid_argument &)
    {
        invalidPlaneConfigRejected = true;
    }
    Require(invalidPlaneConfigRejected,
            "invalid plane-edge configuration was not rejected");

    SInStyleRAGMergeConfig ragConfig;
    ragConfig.enabled = true;
    ragConfig.maximumDepthMeters = 6.0f;
    SInStyleRAGRegionMerger ragMerger;
    ragMerger.Configure(ragConfig);

    cv::Mat ragDepth(20,20,CV_32FC1,cv::Scalar(1.0f));
    cv::Mat ragInitial(20,20,CV_32SC1);
    cv::Mat ragSplit(20,20,CV_32SC1);
    cv::Mat noGradient(20,20,CV_8UC1,cv::Scalar(0));
    for(int row=0; row<20; ++row)
    {
        for(int col=0; col<20; ++col)
        {
            ragInitial.at<int>(row,col) = col<10 ? 1 : 2;
            ragSplit.at<int>(row,col) = col<10 ? 1 : 2;
        }
    }
    const SInStyleRAGMergeResult similarNeighbors = ragMerger.Compute(
        ragDepth,ragInitial,ragSplit,noGradient);
    Require(similarNeighbors.stats.available &&
            !similarNeighbors.stats.dynamicStateAvailable &&
            !similarNeighbors.stats.planeRejectionAvailable,
            "RAG must expose region evidence only and missing plane rejection");
    Require(similarNeighbors.stats.outputRegionCount==1 &&
            similarNeighbors.stats.highMiddleMergeCount+
                similarNeighbors.stats.lowScoreMergeCount==1,
            "same-depth fake-boundary neighbors did not merge");
    Require(similarNeighbors.stats.inputCorePixels==
                similarNeighbors.stats.outputCorePixels &&
            similarNeighbors.stats.crossGradientMergeViolationCount==0,
            "RAG merge changed core support or crossed a gradient split");

    cv::Mat sameParent(20,20,CV_32SC1,cv::Scalar(1));
    cv::Mat trueGradient = noGradient.clone();
    trueGradient.col(9).setTo(255);
    trueGradient.col(10).setTo(255);
    const SInStyleRAGMergeResult trueEdgeSeparated = ragMerger.Compute(
        ragDepth,sameParent,ragSplit,trueGradient);
    Require(trueEdgeSeparated.stats.outputRegionCount==2 &&
            trueEdgeSeparated.stats.crossGradientMergeViolationCount==0,
            "components split from one initial region were re-merged");

    cv::Mat differentDepth = ragDepth.clone();
    differentDepth.colRange(10,20).setTo(3.0f);
    const SInStyleRAGMergeResult depthRejected = ragMerger.Compute(
        differentDepth,ragInitial,ragSplit,noGradient);
    Require(depthRejected.stats.outputRegionCount==2 &&
            depthRejected.stats.depthRejectedPairCount==1,
            "strongly different metric-depth regions were not rejected");

    cv::Mat invalidGapDepth = ragDepth.clone();
    cv::Mat invalidGapInitial = ragInitial.clone();
    cv::Mat invalidGapSplit = ragSplit.clone();
    invalidGapDepth.col(9).setTo(0.0f);
    invalidGapInitial.col(9).setTo(-1);
    invalidGapSplit.col(9).setTo(-1);
    invalidGapSplit.colRange(10,20).setTo(2);
    const SInStyleRAGMergeResult invalidGap = ragMerger.Compute(
        invalidGapDepth,invalidGapInitial,invalidGapSplit,noGradient);
    Require(invalidGap.stats.outputRegionCount==2 &&
            invalidGap.mergedLabels.at<int>(5,9)==-1,
            "invalid gap created adjacency or was filled by RAG");

    const SInStyleRAGMergeResult deterministicRepeat = ragMerger.Compute(
        ragDepth,ragInitial,ragSplit,noGradient);
    cv::Mat deterministicDifference;
    cv::compare(similarNeighbors.mergedLabels,
                deterministicRepeat.mergedLabels,
                deterministicDifference,cv::CMP_NE);
    Require(cv::countNonZero(deterministicDifference)==0,
            "RAG labels are not deterministic");

    // The low-score donor touches two high/middle targets. Target label 1
    // has the longer fake boundary and therefore the higher score, despite
    // having the later fixed rank. This guards max-score selection from being
    // replaced by a rank-only tie-break.
    SInStyleRAGMergeConfig unequalCandidateConfig = ragConfig;
    unequalCandidateConfig.highMiddleFraction = 0.66f;
    ragMerger.Configure(unequalCandidateConfig);
    cv::Mat unequalDepth(40,40,CV_32FC1,cv::Scalar(0.0f));
    cv::Mat unequalInitial(40,40,CV_32SC1,cv::Scalar(-1));
    cv::Mat unequalSplit(40,40,CV_32SC1,cv::Scalar(-1));
    cv::Mat unequalGradient(40,40,CV_8UC1,cv::Scalar(0));
    unequalDepth.rowRange(0,15).setTo(1.0f);
    unequalInitial.rowRange(0,15).setTo(1);
    unequalSplit.rowRange(0,15).setTo(1);
    unequalDepth.rowRange(15,20).setTo(1.0f);
    unequalInitial.rowRange(15,20).setTo(3);
    unequalSplit.rowRange(15,20).setTo(3);
    unequalDepth(cv::Rect(0,20,20,1)).setTo(1.0f);
    unequalInitial(cv::Rect(0,20,20,1)).setTo(2);
    unequalSplit(cv::Rect(0,20,20,1)).setTo(2);
    unequalDepth.rowRange(21,40).setTo(1.0f);
    unequalInitial.rowRange(21,40).setTo(2);
    unequalSplit.rowRange(21,40).setTo(2);
    const SInStyleRAGMergeResult unequalCandidate = ragMerger.Compute(
        unequalDepth,unequalInitial,unequalSplit,unequalGradient);
    const int labelA = unequalCandidate.mergedLabels.at<int>(5,5);
    const int labelC = unequalCandidate.mergedLabels.at<int>(17,5);
    const int labelB = unequalCandidate.mergedLabels.at<int>(25,5);
    Require(unequalCandidate.stats.outputRegionCount==2 &&
            labelA==labelC && labelA!=labelB,
            "RAG did not choose the highest-scoring merge target");

    bool invalidRAGConfigRejected = false;
    try
    {
        SInStyleRAGMergeConfig invalidRAGConfig = ragConfig;
        invalidRAGConfig.histogramBins = 1;
        ragMerger.Configure(invalidRAGConfig);
    }
    catch(const std::invalid_argument &)
    {
        invalidRAGConfigRejected = true;
    }
    Require(invalidRAGConfigRejected,
            "invalid RAG configuration was not rejected");

    std::cout << "SIn-style shadow, initial-region, gradient/plane split, and RAG tests passed"
              << std::endl;
    return 0;
}
