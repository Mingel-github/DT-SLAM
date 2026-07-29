#include <cmath>
#include <iostream>
#include <stdexcept>

#include <opencv2/core/core.hpp>

#include "GeometricDynamicDetector.h"

namespace
{

void Require(const bool condition, const char *message)
{
    if(!condition)
        throw std::runtime_error(message);
}

cv::Mat IdentityPose()
{
    return cv::Mat::eye(4,4,CV_32F);
}

void TestIdentityPlane()
{
    const cv::Mat depth(6,8,CV_32FC1,cv::Scalar(2.0f));
    cv::Mat K = cv::Mat::eye(3,3,CV_32F);
    K.at<float>(0,0) = 100.0f;
    K.at<float>(1,1) = 100.0f;
    K.at<float>(0,2) = 3.5f;
    K.at<float>(1,2) = 2.5f;

    const ORB_SLAM2::GeometricWarpResult result =
        ORB_SLAM2::GeometricDynamicDetector::ComputeWarp(
            depth,depth,IdentityPose(),IdentityPose(),K);

    Require(result.stats.validComparisons==depth.total(),
            "identity plane must compare every pixel");
    Require(result.stats.residualMaxAbs<1e-6,
            "identity plane residual must be zero");
}

void TestSignedNearSurface()
{
    const cv::Mat referenceDepth(5,5,CV_32FC1,cv::Scalar(3.0f));
    cv::Mat currentDepth = referenceDepth.clone();
    currentDepth.at<float>(2,2) = 1.0f;

    cv::Mat K = cv::Mat::eye(3,3,CV_32F);
    K.at<float>(0,0) = 100.0f;
    K.at<float>(1,1) = 100.0f;
    K.at<float>(0,2) = 2.0f;
    K.at<float>(1,2) = 2.0f;

    ORB_SLAM2::GeometricWarpResult result =
        ORB_SLAM2::GeometricDynamicDetector::ComputeWarp(
            referenceDepth,currentDepth,IdentityPose(),IdentityPose(),K);
    ORB_SLAM2::GeometricDynamicDetector::ClassifyEvidence(result,0.5f);

    Require(result.validComparisonMask.at<unsigned char>(2,2)==255,
            "near-surface pixel must have valid geometric evidence");
    Require(std::abs(result.signedDepthResidual.at<float>(2,2)-2.0f)<1e-6f,
            "nearer current surface must produce a positive residual");
    Require(result.positiveSeedMask.at<unsigned char>(2,2)==255,
            "positive residual above threshold must become a positive seed");
    Require(result.negativeDiagnosticMask.at<unsigned char>(2,2)==0,
            "positive seed must not also be a negative diagnostic");
}

void TestSignedFarSurface()
{
    const cv::Mat referenceDepth(5,5,CV_32FC1,cv::Scalar(1.0f));
    cv::Mat currentDepth = referenceDepth.clone();
    currentDepth.at<float>(2,2) = 3.0f;

    cv::Mat K = cv::Mat::eye(3,3,CV_32F);
    K.at<float>(0,0) = 100.0f;
    K.at<float>(1,1) = 100.0f;
    K.at<float>(0,2) = 2.0f;
    K.at<float>(1,2) = 2.0f;

    ORB_SLAM2::GeometricWarpResult result =
        ORB_SLAM2::GeometricDynamicDetector::ComputeWarp(
            referenceDepth,currentDepth,IdentityPose(),IdentityPose(),K);
    ORB_SLAM2::GeometricDynamicDetector::ClassifyEvidence(result,0.5f);

    Require(std::abs(result.signedDepthResidual.at<float>(2,2)+2.0f)<1e-6f,
            "farther current surface must produce a negative residual");
    Require(result.negativeDiagnosticMask.at<unsigned char>(2,2)==255,
            "negative residual below threshold must become a diagnostic");
    Require(result.positiveSeedMask.at<unsigned char>(2,2)==0,
            "negative diagnostic must not also be a positive seed");
}

void TestTcwDirection()
{
    cv::Mat referenceDepth = cv::Mat::zeros(5,5,CV_32FC1);
    cv::Mat currentDepth = cv::Mat::zeros(5,5,CV_32FC1);
    referenceDepth.at<float>(2,2) = 3.0f;
    currentDepth.at<float>(2,2) = 2.0f;

    cv::Mat K = cv::Mat::eye(3,3,CV_32F);
    K.at<float>(0,0) = 100.0f;
    K.at<float>(1,1) = 100.0f;
    K.at<float>(0,2) = 2.0f;
    K.at<float>(1,2) = 2.0f;

    cv::Mat TcwCurrent = IdentityPose();
    TcwCurrent.at<float>(2,3) = -1.0f;

    const ORB_SLAM2::GeometricWarpResult result =
        ORB_SLAM2::GeometricDynamicDetector::ComputeWarp(
            referenceDepth,currentDepth,IdentityPose(),TcwCurrent,K);

    Require(result.validComparisonMask.at<unsigned char>(2,2)==255,
            "forward camera translation must project the center point");
    Require(std::abs(result.predictedDepth.at<float>(2,2)-2.0f)<1e-6f,
            "Tcw_current * inverse(Tcw_reference) has the wrong direction");
    Require(std::abs(result.signedDepthResidual.at<float>(2,2))<1e-6f,
            "known camera translation must preserve the synthetic static surface");
}

void TestInvalidDepthIsUnknown()
{
    const cv::Mat referenceDepth(3,3,CV_32FC1,cv::Scalar(2.0f));
    cv::Mat currentDepth = referenceDepth.clone();
    currentDepth.at<float>(1,1) = 0.0f;

    cv::Mat K = cv::Mat::eye(3,3,CV_32F);
    K.at<float>(0,0) = 100.0f;
    K.at<float>(1,1) = 100.0f;
    K.at<float>(0,2) = 1.0f;
    K.at<float>(1,2) = 1.0f;

    ORB_SLAM2::GeometricWarpResult result =
        ORB_SLAM2::GeometricDynamicDetector::ComputeWarp(
            referenceDepth,currentDepth,IdentityPose(),IdentityPose(),K);
    ORB_SLAM2::GeometricDynamicDetector::ClassifyEvidence(result,0.1f);

    Require(result.validComparisonMask.at<unsigned char>(1,1)==0,
            "invalid current depth must remain geometrically unknown");
    Require(result.signedDepthResidual.at<float>(1,1)==0.0f,
            "invalid comparisons must not contain a residual value");
    Require(result.consistentEvidenceMask.at<unsigned char>(1,1)==0 &&
            result.positiveSeedMask.at<unsigned char>(1,1)==0 &&
            result.negativeDiagnosticMask.at<unsigned char>(1,1)==0,
            "unknown pixels must not enter any evidence class");
}

void TestThresholdBoundaryIsConsistent()
{
    ORB_SLAM2::GeometricWarpResult result;
    result.validComparisonMask =
        cv::Mat(1,4,CV_8UC1,cv::Scalar(255));
    result.validComparisonMask.at<unsigned char>(0,3) = 0;
    result.signedDepthResidual =
        cv::Mat::zeros(1,4,CV_32FC1);
    result.signedDepthResidual.at<float>(0,0) = 0.1f;
    result.signedDepthResidual.at<float>(0,1) = -0.1f;
    result.signedDepthResidual.at<float>(0,2) = 0.0f;
    result.signedDepthResidual.at<float>(0,3) = 10.0f;
    result.stats.validComparisons = 3;

    ORB_SLAM2::GeometricDynamicDetector::ClassifyEvidence(result,0.1f);

    Require(result.stats.consistentEvidencePixels==3,
            "residuals exactly on the threshold must remain consistent");
    Require(result.stats.positiveSeedPixels==0 &&
            result.stats.negativeDiagnosticPixels==0,
            "threshold boundary must not enter an inconsistency mask");
    Require(result.stats.consistentEvidencePixels+
            result.stats.positiveSeedPixels+
            result.stats.negativeDiagnosticPixels==
            result.stats.validComparisons,
            "evidence classes must partition every valid comparison");
    Require(result.consistentEvidenceMask.at<unsigned char>(0,3)==0,
            "invalid pixels must stay unknown regardless of residual storage");
}

void TestZBuffer()
{
    cv::Mat referenceDepth = cv::Mat::zeros(1,4,CV_32FC1);
    cv::Mat currentDepth = cv::Mat::zeros(1,4,CV_32FC1);
    referenceDepth.at<float>(0,0) = 1.0f;
    referenceDepth.at<float>(0,1) = 2.0f;
    currentDepth.at<float>(0,2) = 1.0f;

    cv::Mat K = cv::Mat::eye(3,3,CV_32F);
    cv::Mat TcwCurrent = IdentityPose();
    TcwCurrent.at<float>(0,3) = 2.0f;

    const ORB_SLAM2::GeometricWarpResult result =
        ORB_SLAM2::GeometricDynamicDetector::ComputeWarp(
            referenceDepth,currentDepth,IdentityPose(),TcwCurrent,K);

    Require(result.stats.projectedSamples==2,
            "z-buffer test requires two projected samples");
    Require(result.stats.zbufferValidPixels==1,
            "two samples must collide at one target pixel");
    Require(std::abs(result.predictedDepth.at<float>(0,2)-1.0f)<1e-6f,
            "z-buffer must retain the nearest predicted surface");
}

void TestRegionGrowStopsAtDepthBoundary()
{
    cv::Mat currentDepth(4,6,CV_32FC1,cv::Scalar(1.0f));
    currentDepth.colRange(3,6).setTo(2.0f);

    ORB_SLAM2::GeometricWarpResult result;
    result.validComparisonMask =
        cv::Mat(currentDepth.size(),CV_8UC1,cv::Scalar(255));
    result.positiveSeedMask =
        cv::Mat::zeros(currentDepth.size(),CV_8UC1);
    result.negativeDiagnosticMask =
        cv::Mat::zeros(currentDepth.size(),CV_8UC1);
    result.signedDepthResidual =
        cv::Mat::zeros(currentDepth.size(),CV_32FC1);
    result.positiveSeedMask.at<unsigned char>(1,1) = 255;
    result.positiveSeedMask.at<unsigned char>(2,1) = 255;
    result.signedDepthResidual.at<float>(1,1) = 0.2f;
    result.signedDepthResidual.at<float>(2,1) = 0.2f;
    result.stats.positiveSeedPixels = 2;

    ORB_SLAM2::GeometricDynamicDetector::GrowDepthRegions(
        currentDepth,result,0.05f);

    Require(result.stats.depthRegionCount==1,
            "two seeds on one depth surface must form one grown region");
    Require(result.stats.regionCandidatePixels==12,
            "region grow must cover the seeded half-plane");
    Require(result.stats.largestRegionPixels==12,
            "largest region size must match the seeded half-plane");
    Require(result.depthRegions.size()==1 &&
            result.depthRegions[0].positiveSeedPixels==2,
            "region evidence statistics must count both positive seeds");
    Require(std::abs(
                result.depthRegions[0].positiveSeedRatio-
                2.0/12.0)<1e-9,
            "region positive support ratio is incorrect");
    Require(std::abs(
                result.regionPositiveSupport.at<float>(0,0)-
                static_cast<float>(2.0/12.0))<1e-6f,
            "region support image must store the region support ratio");
    Require(std::abs(result.stats.regionGrowthRatio-6.0)<1e-9,
            "region growth ratio must be candidate pixels divided by seeds");
    Require(result.regionCandidateMask.at<unsigned char>(0,2)==255,
            "seeded depth surface must be included");
    Require(result.regionCandidateMask.at<unsigned char>(0,3)==0,
            "region grow must not cross a strong depth discontinuity");
}

void TestRegionGrowStopsAtUnknownBarrier()
{
    const cv::Mat currentDepth(3,5,CV_32FC1,cv::Scalar(1.0f));

    ORB_SLAM2::GeometricWarpResult result;
    result.validComparisonMask =
        cv::Mat(currentDepth.size(),CV_8UC1,cv::Scalar(255));
    result.validComparisonMask.col(2).setTo(0);
    result.positiveSeedMask =
        cv::Mat::zeros(currentDepth.size(),CV_8UC1);
    result.negativeDiagnosticMask =
        cv::Mat::zeros(currentDepth.size(),CV_8UC1);
    result.signedDepthResidual =
        cv::Mat::zeros(currentDepth.size(),CV_32FC1);
    result.positiveSeedMask.at<unsigned char>(1,0) = 255;
    result.signedDepthResidual.at<float>(1,0) = 0.2f;
    result.stats.positiveSeedPixels = 1;

    ORB_SLAM2::GeometricDynamicDetector::GrowDepthRegions(
        currentDepth,result,0.05f);

    Require(result.stats.regionCandidatePixels==6,
            "unknown column must stop region propagation");
    Require(result.regionCandidateMask.at<unsigned char>(1,1)==255,
            "valid pixels before the unknown barrier must grow");
    Require(result.regionCandidateMask.at<unsigned char>(1,2)==0 &&
            result.regionCandidateMask.at<unsigned char>(1,3)==0,
            "unknown and unseeded pixels beyond it must remain outside");
}

void TestMultiReferenceEvidenceCounts()
{
    const cv::Mat currentDepth(3,4,CV_32FC1,cv::Scalar(2.0f));
    cv::Mat referenceDepthA = currentDepth.clone();
    cv::Mat referenceDepthB = currentDepth.clone();
    referenceDepthA.at<float>(1,1) = 3.0f;
    referenceDepthB.at<float>(1,1) = 3.0f;
    referenceDepthB.at<float>(1,2) = 1.0f;

    cv::Mat K = cv::Mat::eye(3,3,CV_32F);
    K.at<float>(0,0) = 100.0f;
    K.at<float>(1,1) = 100.0f;
    K.at<float>(0,2) = 1.5f;
    K.at<float>(1,2) = 1.0f;

    std::vector<ORB_SLAM2::GeometricReferenceFrame> references(2);
    references[0].depthMeters = referenceDepthA;
    references[0].Tcw = IdentityPose();
    references[1].depthMeters = referenceDepthB;
    references[1].Tcw = IdentityPose();

    const ORB_SLAM2::GeometricMultiReferenceResult result =
        ORB_SLAM2::GeometricDynamicDetector::ComputeMultiReferenceEvidence(
            references,currentDepth,IdentityPose(),K,0.5f);

    Require(result.comparisonCount.at<unsigned char>(1,1)==2,
            "both references must contribute at the shared valid pixel");
    Require(result.positiveCount.at<unsigned char>(1,1)==2,
            "shared nearer current surface must receive two positive votes");
    Require(result.negativeCount.at<unsigned char>(1,2)==1,
            "one farther current surface must receive one negative vote");
    Require(result.consistentCount.at<unsigned char>(1,2)==1,
            "the other reference must remain consistent at the same pixel");
    Require(result.stats.referenceCount==2 &&
            result.stats.totalComparisons==2*currentDepth.total(),
            "multi-reference statistics must count every valid comparison");
    Require(result.stats.totalPositiveVotes==2 &&
            result.stats.totalNegativeVotes==1,
            "multi-reference evidence totals are incorrect");
    Require(result.stats.totalPositiveVotes+
            result.stats.totalNegativeVotes+
            result.stats.totalConsistentVotes==
            result.stats.totalComparisons,
            "multi-reference evidence classes must partition comparisons");
    Require(result.perReference.size()==2 &&
            result.perReference[0].warp.validComparisons==
                currentDepth.total() &&
            result.perReference[1].warp.validComparisons==
                currentDepth.total(),
            "per-reference diagnostics must preserve each warp result");
}

void TestCachedReferenceSelection()
{
    std::vector<ORB_SLAM2::GeometricReferenceFrame> cached(3);
    cached[0].frameId = 10;
    cached[1].frameId = 20;
    cached[2].frameId = 30;
    cached[2].featureDepthPixels.push_back(cv::Point2i(3,4));
    cached[2].gridDepthPixels.push_back(cv::Point2i(2,3));
    const std::vector<long unsigned int> candidates =
        {30,99,20,20,10};

    const ORB_SLAM2::GeometricReferenceSelectionResult result =
        ORB_SLAM2::GeometricDynamicDetector::SelectCachedReferences(
            cached,candidates,2);

    Require(result.stats.candidateCount==4,
            "duplicate candidate frame ids must be counted once");
    Require(result.stats.cachedReferenceMatchCount==3,
            "selection diagnostics must count all cached candidate matches");
    Require(result.stats.selectedReferenceCount==2 &&
            result.references.size()==2,
            "selection must respect the configured maximum");
    Require(result.references[0].frameId==30 &&
            result.references[1].frameId==20,
            "selection must preserve the caller's candidate priority");
    Require(result.references[0].featureDepthPixels.size()==1 &&
            result.references[0].featureDepthPixels[0]==cv::Point2i(3,4),
            "selection must preserve cached ORB-depth samples");
    Require(result.references[0].gridDepthPixels.size()==1 &&
            result.references[0].gridDepthPixels[0]==cv::Point2i(2,3),
            "selection must preserve cached grid-depth samples");
}

void TestOrbDepthSamplingEvidence()
{
    const cv::Mat currentDepth(3,4,CV_32FC1,cv::Scalar(2.0f));
    cv::Mat referenceDepth = currentDepth.clone();
    referenceDepth.at<float>(1,1) = 3.0f;
    referenceDepth.at<float>(1,2) = 1.0f;

    cv::Mat K = cv::Mat::eye(3,3,CV_32F);
    K.at<float>(0,0) = 100.0f;
    K.at<float>(1,1) = 100.0f;
    K.at<float>(0,2) = 1.5f;
    K.at<float>(1,2) = 1.0f;

    ORB_SLAM2::GeometricReferenceFrame reference;
    reference.depthMeters = referenceDepth;
    reference.Tcw = IdentityPose();
    reference.featureDepthPixels.push_back(cv::Point2i(1,1));
    reference.featureDepthPixels.push_back(cv::Point2i(2,1));
    std::vector<ORB_SLAM2::GeometricReferenceFrame> references(1,reference);

    const ORB_SLAM2::GeometricMultiReferenceResult result =
        ORB_SLAM2::GeometricDynamicDetector::ComputeMultiReferenceEvidence(
            references,currentDepth,IdentityPose(),K,0.5f,
            ORB_SLAM2::GeometricReferenceSamplingPolicy::OrbDepth);

    Require(result.stats.totalComparisons==2 &&
            result.stats.pixelsWithComparison==2,
            "ORB-depth sampling must compare only projected sampled pixels");
    Require(result.comparisonCount.at<unsigned char>(1,1)==1 &&
            result.positiveCount.at<unsigned char>(1,1)==1,
            "sampled nearer current surface must produce positive evidence");
    Require(result.comparisonCount.at<unsigned char>(1,2)==1 &&
            result.negativeCount.at<unsigned char>(1,2)==1,
            "sampled farther current surface must produce negative evidence");
    Require(result.comparisonCount.at<unsigned char>(0,0)==0,
            "unsampled reference pixels must remain unknown");
    Require(result.perReference.size()==1 &&
            result.perReference[0].warp.referenceValidPixels==2 &&
            result.perReference[0].warp.validComparisons==2,
            "ORB-depth per-reference statistics are incorrect");
}

void TestGridDepthSamplingEvidence()
{
    const cv::Mat currentDepth(4,4,CV_32FC1,cv::Scalar(2.0f));
    cv::Mat referenceDepth = currentDepth.clone();
    referenceDepth.at<float>(0,0) = 3.0f;
    referenceDepth.at<float>(2,2) = 1.0f;

    cv::Mat K = cv::Mat::eye(3,3,CV_32F);
    K.at<float>(0,0) = 100.0f;
    K.at<float>(1,1) = 100.0f;
    K.at<float>(0,2) = 1.5f;
    K.at<float>(1,2) = 1.5f;

    ORB_SLAM2::GeometricReferenceFrame reference;
    reference.depthMeters = referenceDepth;
    reference.Tcw = IdentityPose();
    reference.gridDepthPixels.push_back(cv::Point2i(0,0));
    reference.gridDepthPixels.push_back(cv::Point2i(2,0));
    reference.gridDepthPixels.push_back(cv::Point2i(0,2));
    reference.gridDepthPixels.push_back(cv::Point2i(2,2));
    std::vector<ORB_SLAM2::GeometricReferenceFrame> references(1,reference);

    const ORB_SLAM2::GeometricMultiReferenceResult result =
        ORB_SLAM2::GeometricDynamicDetector::ComputeMultiReferenceEvidence(
            references,currentDepth,IdentityPose(),K,0.5f,
            ORB_SLAM2::GeometricReferenceSamplingPolicy::GridDepth);

    Require(result.stats.totalComparisons==4 &&
            result.stats.pixelsWithComparison==4,
            "grid-depth sampling must compare only cached grid pixels");
    Require(result.positiveCount.at<unsigned char>(0,0)==1,
            "grid-depth positive evidence has the wrong sign");
    Require(result.negativeCount.at<unsigned char>(2,2)==1,
            "grid-depth negative evidence has the wrong sign");
    Require(result.consistentCount.at<unsigned char>(0,2)==1 &&
            result.consistentCount.at<unsigned char>(2,0)==1,
            "grid-depth consistent evidence is incorrect");
    Require(result.comparisonCount.at<unsigned char>(1,1)==0,
            "non-grid pixels must remain unknown");
}

void TestDepthBoundaryPartitionSplitsStep()
{
    cv::Mat depth(5,8,CV_32FC1,cv::Scalar(1.0f));
    depth.colRange(4,8).setTo(2.0f);

    const ORB_SLAM2::GeometricRegionPartitionResult result =
        ORB_SLAM2::GeometricDynamicDetector::
            PartitionDepthByDiscontinuity(depth,0.025f,0.08f);

    Require(result.stats.validDepthPixels==40,
            "region partition valid-depth count is incorrect");
    Require(result.stats.boundaryPixels==10,
            "depth step must mark both sides of the discontinuity");
    Require(result.stats.regionCount==2,
            "depth step must split two non-boundary regions");
    Require(result.stats.largestRegionPixels==15 &&
            result.stats.topFiveRegionPixels==30,
            "depth-step region sizes are incorrect");
    Require(result.labels.at<int>(2,3)==-2 &&
            result.labels.at<int>(2,4)==-2,
            "depth discontinuity pixels must keep the boundary label");
    Require(result.labels.at<int>(2,1)>=0 &&
            result.labels.at<int>(2,6)>=0 &&
            result.labels.at<int>(2,1)!=
                result.labels.at<int>(2,6),
            "opposite sides of a depth step must have distinct labels");
}

void TestDepthBoundaryPartitionPreservesUnknown()
{
    cv::Mat depth(3,5,CV_32FC1,cv::Scalar(1.0f));
    depth.col(2).setTo(0.0f);

    const ORB_SLAM2::GeometricRegionPartitionResult result =
        ORB_SLAM2::GeometricDynamicDetector::
            PartitionDepthByDiscontinuity(depth,0.025f,0.08f);

    Require(result.stats.validDepthPixels==12 &&
            result.stats.boundaryPixels==0,
            "invalid-depth barrier statistics are incorrect");
    Require(result.stats.regionCount==2,
            "invalid depth must separate connected regions");
    Require(result.labels.at<int>(1,2)==-1,
            "invalid depth must remain unknown rather than static");
    Require(result.labels.at<int>(1,0)>=0 &&
            result.labels.at<int>(1,4)>=0 &&
            result.labels.at<int>(1,0)!=
                result.labels.at<int>(1,4),
            "invalid-depth barrier must split region labels");
}

void TestDepthBoundaryPartitionKeepsPlane()
{
    const cv::Mat depth(4,6,CV_32FC1,cv::Scalar(2.0f));
    const ORB_SLAM2::GeometricRegionPartitionResult result =
        ORB_SLAM2::GeometricDynamicDetector::
            PartitionDepthByDiscontinuity(depth,0.025f,0.08f);

    Require(result.stats.boundaryPixels==0 &&
            result.stats.regionCount==1 &&
            result.stats.largestRegionPixels==24,
            "uniform plane must remain one diagnostic region");
    Require(result.stats.largestRegionValidRatio==1.0,
            "uniform-plane largest-region ratio is incorrect");
}

void TestPyramidDepthAndEvidence()
{
    const cv::Mat depth = (cv::Mat_<float>(4,4) <<
        1.00f,1.02f,2.00f,2.03f,
        1.04f,1.06f,2.04f,2.06f,
        0.00f,1.00f,3.00f,3.02f,
        1.00f,1.00f,3.04f,4.00f);

    const cv::Mat pyramid =
        ORB_SLAM2::GeometricDynamicDetector::
            DownsampleDepthBoundaryAware(
                depth,2,0.025f,0.08f);
    Require(pyramid.rows==2 && pyramid.cols==2,
            "2x depth pyramid has the wrong size");
    Require(std::abs(
                pyramid.at<float>(0,0)-1.03f)<1e-6f &&
            std::abs(
                pyramid.at<float>(0,1)-2.0325f)<1e-6f &&
            pyramid.at<float>(1,0)==0.0f &&
            std::abs(
                pyramid.at<float>(1,1)-3.02f)<1e-6f,
            "boundary-aware block averaging is incorrect");

    cv::Mat K = cv::Mat::eye(3,3,CV_32F);
    K.at<float>(0,0) = 100.0f;
    K.at<float>(1,1) = 120.0f;
    K.at<float>(0,2) = 2.0f;
    K.at<float>(1,2) = 3.0f;
    const cv::Mat scaledK =
        ORB_SLAM2::GeometricDynamicDetector::
            ScaleCameraMatrix(K,2);
    Require(scaledK.at<float>(0,0)==50.0f &&
            scaledK.at<float>(1,1)==60.0f &&
            scaledK.at<float>(0,2)==1.0f &&
            scaledK.at<float>(1,2)==1.5f,
            "pyramid camera intrinsics are incorrect");

    ORB_SLAM2::GeometricReferenceFrame reference;
    reference.depthMeters = depth;
    reference.pyramidDepthMeters = pyramid;
    reference.Tcw = IdentityPose();
    reference.frameId = 1;
    std::vector<ORB_SLAM2::GeometricReferenceFrame> references(
        1,reference);
    const ORB_SLAM2::GeometricMultiReferenceResult evidence =
        ORB_SLAM2::GeometricDynamicDetector::
            ComputePyramidMultiReferenceEvidence(
                references,depth,IdentityPose(),K,
                0.10f,2,0.025f,0.08f);

    Require(evidence.comparisonCount.size()==depth.size() &&
            evidence.positiveCount.size()==depth.size(),
            "expanded pyramid evidence has the wrong size");
    Require(evidence.stats.pixelsWithComparison==12 &&
            evidence.stats.totalComparisons==12 &&
            evidence.stats.pixelsWithPositiveEvidence==0 &&
            evidence.stats.totalPositiveVotes==0 &&
            evidence.stats.totalNegativeVotes==0 &&
            evidence.stats.totalConsistentVotes==12,
            "identity pyramid evidence counts are incorrect");
}

void TestRegionEvidenceAggregation()
{
    ORB_SLAM2::GeometricRegionPartitionResult partition;
    partition.labels = (cv::Mat_<int>(2,4) <<
        0,0,-2,1,
        0,0,-1,1);
    partition.regionSizes.push_back(4);
    partition.regionSizes.push_back(2);

    ORB_SLAM2::GeometricMultiReferenceResult evidence;
    evidence.comparisonCount = cv::Mat::zeros(2,4,CV_8UC1);
    evidence.positiveCount = cv::Mat::zeros(2,4,CV_8UC1);
    evidence.negativeCount = cv::Mat::zeros(2,4,CV_8UC1);
    evidence.consistentCount = cv::Mat::zeros(2,4,CV_8UC1);

    evidence.comparisonCount.at<unsigned char>(0,0) = 2;
    evidence.positiveCount.at<unsigned char>(0,0) = 1;
    evidence.consistentCount.at<unsigned char>(0,0) = 1;
    evidence.comparisonCount.at<unsigned char>(0,1) = 1;
    evidence.consistentCount.at<unsigned char>(0,1) = 1;
    evidence.comparisonCount.at<unsigned char>(1,0) = 1;
    evidence.negativeCount.at<unsigned char>(1,0) = 1;
    evidence.comparisonCount.at<unsigned char>(0,2) = 1;
    evidence.positiveCount.at<unsigned char>(0,2) = 1;
    evidence.comparisonCount.at<unsigned char>(1,2) = 1;
    evidence.positiveCount.at<unsigned char>(1,2) = 1;
    evidence.comparisonCount.at<unsigned char>(0,3) = 1;
    evidence.positiveCount.at<unsigned char>(0,3) = 1;

    cv::Mat semantic = cv::Mat::zeros(2,4,CV_8UC1);
    semantic.at<unsigned char>(0,0) = 255;
    semantic.at<unsigned char>(0,3) = 255;
    semantic.at<unsigned char>(1,3) = 255;

    const ORB_SLAM2::GeometricRegionEvidenceAggregationResult result =
        ORB_SLAM2::GeometricDynamicDetector::
            AggregateMultiReferenceEvidenceByRegion(
                partition,evidence,semantic);

    Require(result.regions.size()==2 &&
            result.stats.regionPixels==6 &&
            result.stats.comparisonPixels==4 &&
            result.stats.comparisonVotes==5,
            "region evidence aggregate totals are incorrect");
    const ORB_SLAM2::GeometricRegionEvidenceStats &first =
        result.regions[0];
    Require(first.regionPixels==4 &&
            first.semanticProxyPixels==1 &&
            first.semanticComparisonPixels==1 &&
            first.semanticPositivePresencePixels==1 &&
            first.comparisonPixels==3 &&
            first.positivePresencePixels==1 &&
            first.negativePresencePixels==1 &&
            first.consistentPresencePixels==2,
            "first-region evidence presence is incorrect");
    Require(first.comparisonVotes==4 &&
            first.positiveVotes==1 &&
            first.negativeVotes==1 &&
            first.consistentVotes==2,
            "first-region vote sums are incorrect");
    Require(std::abs(first.comparisonCoverage-0.75)<1e-9 &&
            std::abs(first.positiveVoteRatio-0.25)<1e-9,
            "first-region evidence ratios are incorrect");
    const ORB_SLAM2::GeometricRegionEvidenceStats &second =
        result.regions[1];
    Require(second.regionPixels==2 &&
            second.semanticProxyPixels==2 &&
            second.semanticComparisonPixels==1 &&
            second.semanticPositivePresencePixels==1 &&
            second.comparisonPixels==1 &&
            second.positivePresencePixels==1,
            "second-region evidence aggregation is incorrect");
    Require(result.stats.regionsWithComparison==2 &&
            result.stats.regionsWithPositiveEvidence==2,
            "region evidence aggregate region counts are incorrect");
}

} // namespace

int main()
{
    try
    {
        TestIdentityPlane();
        TestSignedNearSurface();
        TestSignedFarSurface();
        TestTcwDirection();
        TestInvalidDepthIsUnknown();
        TestThresholdBoundaryIsConsistent();
        TestZBuffer();
        TestRegionGrowStopsAtDepthBoundary();
        TestRegionGrowStopsAtUnknownBarrier();
        TestMultiReferenceEvidenceCounts();
        TestCachedReferenceSelection();
        TestOrbDepthSamplingEvidence();
        TestGridDepthSamplingEvidence();
        TestDepthBoundaryPartitionSplitsStep();
        TestDepthBoundaryPartitionPreservesUnknown();
        TestDepthBoundaryPartitionKeepsPlane();
        TestPyramidDepthAndEvidence();
        TestRegionEvidenceAggregation();
    }
    catch(const std::exception &error)
    {
        std::cerr << "[Geometry G0/G2-1/G2-2R/G2-2S/G2-2G/G2-3R0/G2-3R1/G2-3R3 Test] FAIL: "
                  << error.what() << std::endl;
        return 1;
    }

    std::cout << "[Geometry G0/G2-1/G2-2R/G2-2S/G2-2G/G2-3R0/G2-3R1/G2-3R3 Test] PASS"
              << std::endl;
    return 0;
}
