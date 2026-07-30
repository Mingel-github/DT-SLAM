#include <cmath>
#include <iostream>
#include <sstream>
#include <stdexcept>

#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "GeometricDynamicDetector.h"
#include "RGBDInputRectifier.h"

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

cv::FileStorage RectificationSettings(
    const double cameraK1 = 0.0,
    const bool enabled = true)
{
    std::ostringstream yaml;
    yaml << "%YAML:1.0\n"
         << "RGBD.InputRectification.Enable: " << (enabled ? 1 : 0) << "\n"
         << "RGBD.InputRectification.fx: 542.822841\n"
         << "RGBD.InputRectification.fy: 542.576870\n"
         << "RGBD.InputRectification.cx: 315.593520\n"
         << "RGBD.InputRectification.cy: 237.756098\n"
         << "RGBD.InputRectification.k1: 0.039903\n"
         << "RGBD.InputRectification.k2: -0.099343\n"
         << "RGBD.InputRectification.p1: -0.000730\n"
         << "RGBD.InputRectification.p2: -0.000144\n"
         << "RGBD.InputRectification.k3: 0.000000\n"
         << "Camera.fx: 542.822841\n"
         << "Camera.fy: 542.576870\n"
         << "Camera.cx: 315.593520\n"
         << "Camera.cy: 237.756098\n"
         << "Camera.k1: " << cameraK1 << "\n"
         << "Camera.k2: 0.0\n"
         << "Camera.p1: 0.0\n"
         << "Camera.p2: 0.0\n"
         << "Camera.k3: 0.0\n";
    return cv::FileStorage(
        yaml.str(),cv::FileStorage::READ | cv::FileStorage::MEMORY);
}

void TestRGBDRectificationDisabledBypass()
{
    cv::FileStorage settings = RectificationSettings(0.0,false);
    ORB_SLAM2::RGBDInputRectifier rectifier;
    rectifier.Configure(settings);

    cv::Mat rgb(4,6,CV_8UC3,cv::Scalar(1,2,3));
    cv::Mat depth(4,6,CV_16UC1,cv::Scalar(1000));
    cv::Mat outputRGB;
    cv::Mat outputDepth;
    rectifier.RectifyRGBD(rgb,depth,outputRGB,outputDepth);

    Require(!rectifier.IsEnabled(),
            "RGB-D input rectification must be disabled by default");
    Require(outputRGB.data==rgb.data && outputDepth.data==depth.data,
            "disabled RGB-D rectification must bypass input without resampling");
}

void TestRGBDRectificationDepthUsesNearestNeighbor()
{
    cv::FileStorage settings = RectificationSettings();
    ORB_SLAM2::RGBDInputRectifier rectifier;
    rectifier.Configure(settings);

    cv::Mat rgb(480,640,CV_8UC3,cv::Scalar(20,40,60));
    cv::Mat depth(480,640,CV_16UC1);
    for(int row=0; row<depth.rows; ++row)
    {
        unsigned short *depthRow = depth.ptr<unsigned short>(row);
        for(int col=0; col<depth.cols; ++col)
            depthRow[col] = col<320 ? 1000 : 2000;
    }

    cv::Mat outputRGB;
    cv::Mat outputDepth;
    rectifier.RectifyRGBD(rgb,depth,outputRGB,outputDepth);

    Require(rectifier.IsEnabled() &&
            rectifier.DomainName()=="undistorted_pinhole",
            "enabled RGB-D rectification must declare the pinhole domain");
    Require(outputRGB.size()==rgb.size() && outputRGB.type()==rgb.type() &&
            outputDepth.size()==depth.size() &&
            outputDepth.type()==depth.type(),
            "RGB-D rectification must preserve size and image types");
    for(int row=0; row<outputDepth.rows; ++row)
    {
        const unsigned short *depthRow =
            outputDepth.ptr<unsigned short>(row);
        for(int col=0; col<outputDepth.cols; ++col)
        {
            Require(depthRow[col]==0 ||
                    depthRow[col]==1000 ||
                    depthRow[col]==2000,
                    "depth rectification introduced an interpolated depth value");
        }
    }
}

void TestRGBDRectificationRejectsDoubleUndistortion()
{
    cv::FileStorage settings = RectificationSettings(0.01,true);
    ORB_SLAM2::RGBDInputRectifier rectifier;
    bool rejected = false;
    try
    {
        rectifier.Configure(settings);
    }
    catch(const std::invalid_argument &)
    {
        rejected = true;
    }
    Require(rejected,
            "rectified RGB-D input must reject non-zero tracking distortion");
}

void TestBonnCalibrationRoundTrip()
{
    const cv::Mat K = (cv::Mat_<double>(3,3) <<
        542.822841,0.0,315.593520,
        0.0,542.576870,237.756098,
        0.0,0.0,1.0);
    const cv::Mat distortion = (cv::Mat_<double>(5,1) <<
        0.039903,-0.099343,-0.000730,-0.000144,0.0);

    std::vector<cv::Point2d> rawPoints;
    for(int row=0; row<=48; ++row)
    {
        for(int col=0; col<=64; ++col)
        {
            rawPoints.push_back(cv::Point2d(
                639.0*col/64.0,479.0*row/48.0));
        }
    }
    std::vector<cv::Point2d> rectifiedPoints;
    cv::undistortPoints(
        rawPoints,rectifiedPoints,K,distortion,cv::Mat(),K);

    double maxRoundTripError = 0.0;
    for(std::size_t index=0; index<rectifiedPoints.size(); ++index)
    {
        const double x =
            (rectifiedPoints[index].x-K.at<double>(0,2))/
            K.at<double>(0,0);
        const double y =
            (rectifiedPoints[index].y-K.at<double>(1,2))/
            K.at<double>(1,1);
        const double r2 = x*x+y*y;
        const double radial =
            1.0+distortion.at<double>(0)*r2+
            distortion.at<double>(1)*r2*r2+
            distortion.at<double>(4)*r2*r2*r2;
        const double xd =
            x*radial+
            2.0*distortion.at<double>(2)*x*y+
            distortion.at<double>(3)*(r2+2.0*x*x);
        const double yd =
            y*radial+
            distortion.at<double>(2)*(r2+2.0*y*y)+
            2.0*distortion.at<double>(3)*x*y;
        const cv::Point2d reconstructed(
            K.at<double>(0,0)*xd+K.at<double>(0,2),
            K.at<double>(1,1)*yd+K.at<double>(1,2));
        maxRoundTripError = std::max(
            maxRoundTripError,
            cv::norm(reconstructed-rawPoints[index]));
    }
    Require(maxRoundTripError<=0.05,
            "Bonn raw/undistorted calibration round-trip exceeds 0.05 px");
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
    Require(evidence.nativeScale==2 &&
            evidence.nativeDepthMeters.size()==pyramid.size() &&
            evidence.nativeComparisonCount.size()==pyramid.size() &&
            evidence.nativePositiveCount.size()==pyramid.size(),
            "native pyramid evidence state has the wrong domain");
    Require(cv::sum(evidence.nativeComparisonCount)[0]==3.0 &&
            cv::sum(evidence.nativeConsistentCount)[0]==3.0,
            "native pyramid cells must be counted once");
    Require(evidence.stats.pixelsWithComparison==12 &&
            evidence.stats.totalComparisons==12 &&
            evidence.stats.pixelsWithPositiveEvidence==0 &&
            evidence.stats.totalPositiveVotes==0 &&
            evidence.stats.totalNegativeVotes==0 &&
            evidence.stats.totalConsistentVotes==12,
            "identity pyramid evidence counts are incorrect");
}

void TestLowResolutionRegionUsesNativeCells()
{
    const cv::Mat depth(4,4,CV_32FC1,cv::Scalar(2.0f));
    cv::Mat K = cv::Mat::eye(3,3,CV_32F);
    K.at<float>(0,0) = 100.0f;
    K.at<float>(1,1) = 100.0f;
    K.at<float>(0,2) = 1.5f;
    K.at<float>(1,2) = 1.5f;

    ORB_SLAM2::GeometricReferenceFrame reference;
    reference.depthMeters = depth;
    reference.Tcw = IdentityPose();
    std::vector<ORB_SLAM2::GeometricReferenceFrame> references(
        1,reference);
    const ORB_SLAM2::GeometricMultiReferenceResult evidence =
        ORB_SLAM2::GeometricDynamicDetector::
            ComputePyramidMultiReferenceEvidence(
                references,depth,IdentityPose(),K,
                0.10f,2,0.025f,0.08f);

    const ORB_SLAM2::GeometricRegionPartitionResult partition =
        ORB_SLAM2::GeometricDynamicDetector::
            PartitionDepthByDiscontinuity(
                evidence.nativeDepthMeters,0.025f,0.08f);
    ORB_SLAM2::GeometricMultiReferenceResult nativeEvidence;
    nativeEvidence.comparisonCount =
        evidence.nativeComparisonCount;
    nativeEvidence.positiveCount =
        evidence.nativePositiveCount;
    nativeEvidence.negativeCount =
        evidence.nativeNegativeCount;
    nativeEvidence.consistentCount =
        evidence.nativeConsistentCount;

    cv::Mat fullMask = cv::Mat::zeros(4,4,CV_8UC1);
    fullMask.at<unsigned char>(1,1) = 255;
    fullMask.at<unsigned char>(3,2) = 255;
    const cv::Mat nativeMask =
        ORB_SLAM2::GeometricDynamicDetector::
            DownsampleMaskAny(fullMask,2);
    const ORB_SLAM2::GeometricRegionEvidenceAggregationResult
        aggregation =
            ORB_SLAM2::GeometricDynamicDetector::
                AggregateMultiReferenceEvidenceByRegion(
                    partition,nativeEvidence,nativeMask);

    Require(partition.stats.regionCount==1 &&
            partition.regionSizes[0]==4,
            "uniform half-resolution depth must remain one region");
    Require(nativeMask.rows==2 && nativeMask.cols==2 &&
            cv::countNonZero(nativeMask)==2,
            "semantic proxy must use block-any half-cell projection");
    Require(aggregation.stats.regionPixels==4 &&
            aggregation.stats.comparisonPixels==4 &&
            aggregation.stats.comparisonVotes==4,
            "G2-3R4 aggregation must count four native cells, not "
            "sixteen expanded pixels");
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
                partition,evidence,semantic,true);

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
    Require(first.singleReferenceComparisonPixels==2 &&
            first.multiReferenceComparisonPixels==1 &&
            first.singleReferencePositivePresencePixels==0 &&
            first.multiReferencePositivePresencePixels==1 &&
            first.unanimousPositivePixels==0,
            "first-region reference-support diagnostics are incorrect");
    Require(first.boundaryWithinOnePixel.regionPixels==2 &&
            first.boundaryWithinTwoPixels.regionPixels==4 &&
            first.boundaryWithinTwoPixels.positiveVotes==1 &&
            first.invalidWithinOnePixel.regionPixels==2 &&
            first.invalidWithinTwoPixels.regionPixels==4,
            "first-region boundary/invalid risk bands are incorrect");
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
    Require(second.singleReferenceComparisonPixels==1 &&
            second.singleReferencePositivePresencePixels==1 &&
            second.unanimousPositivePixels==1 &&
            second.boundaryWithinOnePixel.regionPixels==2 &&
            second.boundaryWithinOnePixel.positiveVotes==1 &&
            second.invalidWithinOnePixel.regionPixels==2 &&
            second.invalidWithinOnePixel.positiveVotes==1,
            "second-region risk diagnostics are incorrect");
    Require(result.stats.regionsWithComparison==2 &&
            result.stats.regionsWithPositiveEvidence==2,
            "region evidence aggregate region counts are incorrect");
}

void TestFeatureEvidenceUsesNativePyramidCells()
{
    ORB_SLAM2::GeometricMultiReferenceResult evidence;
    evidence.comparisonCount = cv::Mat::zeros(4,4,CV_8UC1);
    evidence.positiveCount = cv::Mat::zeros(4,4,CV_8UC1);
    evidence.negativeCount = cv::Mat::zeros(4,4,CV_8UC1);
    evidence.consistentCount = cv::Mat::zeros(4,4,CV_8UC1);
    evidence.nativeScale = 2;
    evidence.nativeComparisonCount =
        (cv::Mat_<unsigned char>(2,2) << 2,1,0,3);
    evidence.nativePositiveCount =
        (cv::Mat_<unsigned char>(2,2) << 1,0,0,2);
    evidence.nativeNegativeCount =
        (cv::Mat_<unsigned char>(2,2) << 0,1,0,0);
    evidence.nativeConsistentCount =
        (cv::Mat_<unsigned char>(2,2) << 1,0,0,1);

    std::vector<cv::Point2f> features;
    features.push_back(cv::Point2f(0.5f,0.5f));
    features.push_back(cv::Point2f(1.9f,1.2f));
    features.push_back(cv::Point2f(2.1f,0.1f));
    features.push_back(cv::Point2f(3.0f,3.0f));
    features.push_back(cv::Point2f(-1.0f,0.0f));
    features.push_back(cv::Point2f(8.0f,8.0f));
    features.push_back(cv::Point2f(-0.2f,0.5f));

    const std::vector<ORB_SLAM2::GeometricFeatureEvidenceSample>
        samples =
            ORB_SLAM2::GeometricDynamicDetector::
                SampleMultiReferenceEvidenceAtFeatures(
                    evidence,features);
    Require(samples.size()==features.size(),
            "feature evidence must retain every requested feature");
    Require(samples[0].nativeScale==2 &&
            samples[0].nativeU==0 &&
            samples[0].nativeV==0 &&
            samples[0].comparisonCount==2 &&
            samples[0].positiveCount==1 &&
            samples[0].consistentCount==1,
            "first feature did not read the expected native cell");
    Require(samples[1].nativeU==0 &&
            samples[1].nativeV==0 &&
            samples[1].comparisonCount==samples[0].comparisonCount,
            "features in one expanded 2x2 block must share one native cell");
    Require(samples[2].nativeU==1 &&
            samples[2].nativeV==0 &&
            samples[2].comparisonCount==1 &&
            samples[2].negativeCount==1,
            "feature-to-native-cell mapping is incorrect");
    Require(samples[3].nativeU==1 &&
            samples[3].nativeV==1 &&
            samples[3].comparisonCount==3 &&
            samples[3].positiveCount==2 &&
            samples[3].consistentCount==1,
            "bottom-right native evidence is incorrect");
    Require(samples[4].nativeU==-1 &&
            samples[4].nativeV==-1 &&
            samples[4].comparisonCount==0 &&
            samples[5].nativeU==-1 &&
            samples[5].nativeV==-1 &&
            samples[5].comparisonCount==0 &&
            samples[6].nativeU==-1 &&
            samples[6].nativeV==-1 &&
            samples[6].comparisonCount==0,
            "out-of-domain features must remain no-evidence");
}

cv::Mat MakeSparseFlowTexture()
{
    cv::Mat image(96,128,CV_8UC1);
    cv::RNG random(12345);
    random.fill(image,cv::RNG::UNIFORM,0,255);
    cv::GaussianBlur(image,image,cv::Size(3,3),0.6);
    return image;
}

cv::Mat ShiftImage(
    const cv::Mat &image,
    const float deltaX,
    const float deltaY)
{
    cv::Mat shifted;
    const cv::Mat transform =
        (cv::Mat_<double>(2,3) <<
            1.0,0.0,deltaX,0.0,1.0,deltaY);
    cv::warpAffine(
        image,shifted,transform,image.size(),
        cv::INTER_LINEAR,cv::BORDER_REFLECT101);
    return shifted;
}

cv::Mat SparseFlowCameraMatrix()
{
    cv::Mat K = cv::Mat::eye(3,3,CV_32F);
    K.at<float>(0,0) = 100.0f;
    K.at<float>(1,1) = 100.0f;
    K.at<float>(0,2) = 64.0f;
    K.at<float>(1,2) = 48.0f;
    return K;
}

void TestSparseEgoFlowIdentityAndLkDirection()
{
    const cv::Mat reference = MakeSparseFlowTexture();
    const cv::Mat current = reference.clone();
    const cv::Mat depth(
        reference.size(),CV_32FC1,cv::Scalar(2.0f));
    const std::vector<cv::Point2f> features = {
        cv::Point2f(30.0f,30.0f),
        cv::Point2f(64.0f,48.0f),
        cv::Point2f(90.0f,60.0f)};
    const ORB_SLAM2::GeometricSparseFlowResult result =
        ORB_SLAM2::GeometricDynamicDetector::
            ComputeSparseEgoFlow(
                current,reference,depth,features,
                IdentityPose(),IdentityPose(),
                SparseFlowCameraMatrix());

    Require(result.samples.size()==features.size(),
            "sparse flow must retain every feature");
    for(std::size_t index=0;
        index<result.samples.size(); ++index)
    {
        const ORB_SLAM2::GeometricSparseFlowSample &sample =
            result.samples[index];
        Require(
            sample.evidenceState==
                ORB_SLAM2::GeometricSparseFlowEvidenceState::
                    Measured,
            "identity sparse flow must be measured");
        Require(sample.forwardBackwardErrorPixels<0.05f,
                "identity LK forward-backward error is too large");
        Require(sample.slamResidualMagnitudePixels<0.05f,
                "identity ego-flow residual must be zero");
    }
}

void TestSparseEgoFlowCameraAndIndependentMotion()
{
    const cv::Mat reference = MakeSparseFlowTexture();
    const cv::Mat current = ShiftImage(reference,2.0f,0.0f);
    const cv::Mat depth(
        reference.size(),CV_32FC1,cv::Scalar(2.0f));
    const std::vector<cv::Point2f> features(
        1,cv::Point2f(66.0f,48.0f));
    cv::Mat matchingCameraPose = IdentityPose();
    matchingCameraPose.at<float>(0,3) = 0.04f;

    const ORB_SLAM2::GeometricSparseFlowResult staticResult =
        ORB_SLAM2::GeometricDynamicDetector::
            ComputeSparseEgoFlow(
                current,reference,depth,features,
                IdentityPose(),matchingCameraPose,
                SparseFlowCameraMatrix());
    Require(
        staticResult.samples[0].evidenceState==
            ORB_SLAM2::GeometricSparseFlowEvidenceState::
                Measured &&
        staticResult.samples[0].slamResidualMagnitudePixels<0.2f,
        "camera-induced image shift must have near-zero residual");

    const ORB_SLAM2::GeometricSparseFlowResult independentResult =
        ORB_SLAM2::GeometricDynamicDetector::
            ComputeSparseEgoFlow(
                current,reference,depth,features,
                IdentityPose(),IdentityPose(),
                SparseFlowCameraMatrix(),
                IdentityPose(),matchingCameraPose);
    Require(
        std::fabs(
            independentResult.samples[0].
                slamResidualPixels.x-2.0f)<0.2f,
        "independent pixel displacement has wrong residual sign");
    Require(
        independentResult.samples[0].
            groundTruthProjectionValid &&
        independentResult.samples[0].
            groundTruthResidualMagnitudePixels<0.2f,
        "GT and SLAM sparse-flow pose branches are not independent");
}

void TestSparseEgoFlowInvalidEvidence()
{
    const cv::Mat texture = MakeSparseFlowTexture();
    cv::Mat invalidDepth(
        texture.size(),CV_32FC1,cv::Scalar(0.0f));
    const std::vector<cv::Point2f> feature(
        1,cv::Point2f(64.0f,48.0f));
    ORB_SLAM2::GeometricSparseFlowResult result =
        ORB_SLAM2::GeometricDynamicDetector::
            ComputeSparseEgoFlow(
                texture,texture,invalidDepth,feature,
                IdentityPose(),IdentityPose(),
                SparseFlowCameraMatrix());
    Require(
        result.samples[0].evidenceState==
            ORB_SLAM2::GeometricSparseFlowEvidenceState::
                DepthInvalid,
        "invalid reference depth must remain no-evidence");

    cv::Mat validDepth(
        texture.size(),CV_32FC1,cv::Scalar(2.0f));
    cv::Mat behindCameraPose = IdentityPose();
    behindCameraPose.at<float>(2,3) = -3.0f;
    result =
        ORB_SLAM2::GeometricDynamicDetector::
            ComputeSparseEgoFlow(
                texture,texture,validDepth,feature,
                IdentityPose(),behindCameraPose,
                SparseFlowCameraMatrix());
    Require(
        result.samples[0].evidenceState==
            ORB_SLAM2::GeometricSparseFlowEvidenceState::
                ProjectionInvalid,
        "behind-camera projection must remain no-evidence");

    const cv::Mat blank =
        cv::Mat::zeros(texture.size(),CV_8UC1);
    result =
        ORB_SLAM2::GeometricDynamicDetector::
            ComputeSparseEgoFlow(
                blank,blank,validDepth,feature,
                IdentityPose(),IdentityPose(),
                SparseFlowCameraMatrix());
    Require(
        result.samples[0].evidenceState==
            ORB_SLAM2::GeometricSparseFlowEvidenceState::
                LkInvalid,
        "LK failure must not produce a sparse-flow residual");
}

void TestSparseEgoFlowDepthRiskDiagnostics()
{
    const cv::Mat texture = MakeSparseFlowTexture();
    const std::vector<cv::Point2f> feature(
        1,cv::Point2f(64.0f,48.0f));

    cv::Mat stepDepth(
        texture.size(),CV_32FC1,cv::Scalar(2.0f));
    stepDepth.colRange(66,stepDepth.cols).setTo(4.0f);
    ORB_SLAM2::GeometricSparseFlowResult result =
        ORB_SLAM2::GeometricDynamicDetector::
            ComputeSparseEgoFlow(
                texture,texture,stepDepth,feature,
                IdentityPose(),IdentityPose(),
                SparseFlowCameraMatrix());
    Require(
        result.samples[0].evidenceState==
            ORB_SLAM2::GeometricSparseFlowEvidenceState::Measured &&
        result.samples[0].
            referenceDepthBoundaryWithinOnePixel &&
        result.samples[0].
            referenceDepthBoundaryWithinTwoPixels,
        "depth step must be recorded as nearby boundary risk");

    cv::Mat invalidNeighborDepth(
        texture.size(),CV_32FC1,cv::Scalar(2.0f));
    invalidNeighborDepth.at<float>(48,66) = 0.0f;
    result =
        ORB_SLAM2::GeometricDynamicDetector::
            ComputeSparseEgoFlow(
                texture,texture,invalidNeighborDepth,feature,
                IdentityPose(),IdentityPose(),
                SparseFlowCameraMatrix());
    Require(
        !result.samples[0].
            referenceInvalidDepthWithinOnePixel &&
        result.samples[0].
            referenceInvalidDepthWithinTwoPixels,
        "invalid-depth risk bands must preserve one/two-pixel distance");
    Require(
        !result.samples[0].
            referenceDepthBoundaryWithinTwoPixels,
        "invalid depth must not be reinterpreted as a depth boundary");
}

ORB_SLAM2::GeometricSparseFlowResult MakeRigiditySparseFlow(
    const std::vector<cv::Point2f> &referencePixels,
    const std::vector<cv::Point2f> &currentPixels,
    const float referenceDepthMeters = 2.0f)
{
    Require(referencePixels.size()==currentPixels.size(),
            "rigidity synthetic pixel vectors must match");
    ORB_SLAM2::GeometricSparseFlowResult result;
    result.samples.resize(currentPixels.size());
    for(std::size_t index=0; index<currentPixels.size(); ++index)
    {
        ORB_SLAM2::GeometricSparseFlowSample &sample =
            result.samples[index];
        sample.featureIndex = index;
        sample.referencePixel = referencePixels[index];
        sample.currentPixel = currentPixels[index];
        sample.forwardBackPixel = currentPixels[index];
        sample.forwardBackwardErrorPixels = 0.0f;
        sample.referenceDepthMeters = referenceDepthMeters;
        sample.referenceDepthValid = true;
        sample.backwardLkValid = true;
        sample.forwardLkValid = true;
        sample.slamProjectionValid = true;
        sample.evidenceState =
            ORB_SLAM2::GeometricSparseFlowEvidenceState::Measured;
    }
    return result;
}

void TestLocalRigidityRigidTranslation()
{
    const std::vector<cv::Point2f> reference = {
        cv::Point2f(30.0f,25.0f),
        cv::Point2f(90.0f,25.0f),
        cv::Point2f(90.0f,70.0f),
        cv::Point2f(30.0f,70.0f),
        cv::Point2f(60.0f,48.0f)};
    std::vector<cv::Point2f> current = reference;
    for(std::size_t index=0; index<current.size(); ++index)
        current[index].x += 2.0f;
    const cv::Mat depth(
        cv::Size(128,96),CV_32FC1,cv::Scalar(2.0f));
    const ORB_SLAM2::GeometricRigidityResult result =
        ORB_SLAM2::GeometricDynamicDetector::
            ComputeLocalRigidity(
                depth,depth,SparseFlowCameraMatrix(),
                MakeRigiditySparseFlow(reference,current));

    Require(result.stats.eligibleNodeCount==reference.size(),
            "rigid graph must retain all valid nodes");
    Require(result.stats.validEdgeCount>=5,
            "rigid graph must contain Delaunay edges");
    for(std::size_t index=0; index<result.edges.size(); ++index)
    {
        Require(result.edges[index].absoluteStrainMeters<1e-5f,
                "rigid image translation must preserve 3-D edge lengths");
        Require(
            result.edges[index].uncertaintyNormalizedStrain<1e-3f,
            "rigid image translation must have near-zero normalized strain");
    }
}

void TestLocalRigidityIndependentNode()
{
    const std::vector<cv::Point2f> reference = {
        cv::Point2f(30.0f,25.0f),
        cv::Point2f(90.0f,25.0f),
        cv::Point2f(90.0f,70.0f),
        cv::Point2f(30.0f,70.0f),
        cv::Point2f(60.0f,48.0f)};
    std::vector<cv::Point2f> current = reference;
    for(std::size_t index=0; index<current.size(); ++index)
        current[index].x += 2.0f;
    current[4].x += 8.0f;
    const cv::Mat depth(
        cv::Size(128,96),CV_32FC1,cv::Scalar(2.0f));
    const ORB_SLAM2::GeometricRigidityResult result =
        ORB_SLAM2::GeometricDynamicDetector::
            ComputeLocalRigidity(
                depth,depth,SparseFlowCameraMatrix(),
                MakeRigiditySparseFlow(reference,current));

    bool incidentEdgeChanged = false;
    bool backgroundEdgePreserved = false;
    for(std::size_t index=0; index<result.edges.size(); ++index)
    {
        const ORB_SLAM2::GeometricRigidityEdgeSample &edge =
            result.edges[index];
        if(edge.featureIndexA==4 || edge.featureIndexB==4)
        {
            incidentEdgeChanged =
                incidentEdgeChanged ||
                edge.absoluteStrainMeters>0.01f;
        }
        else
        {
            backgroundEdgePreserved =
                backgroundEdgePreserved ||
                edge.absoluteStrainMeters<1e-5f;
        }
    }
    Require(incidentEdgeChanged,
            "independent node must change at least one incident edge");
    Require(backgroundEdgePreserved,
            "independent node must not change every background edge");
    Require(
        result.nodes[4].incidentAbsoluteStrainMedianMeters>0.01f,
        "independent node must have elevated local strain");
    Require(
        result.nodes[4].
            incidentUncertaintyNormalizedStrainMedian>0.0f,
        "independent node must have positive uncertainty-normalized strain");
}

void TestLocalRigidityInvalidAndDuplicateNodes()
{
    const std::vector<cv::Point2f> reference = {
        cv::Point2f(30.0f,25.0f),
        cv::Point2f(90.0f,25.0f),
        cv::Point2f(90.0f,70.0f),
        cv::Point2f(30.0f,70.0f),
        cv::Point2f(30.0f,25.0f)};
    std::vector<cv::Point2f> current = reference;
    current[4] = cv::Point2f(30.00005f,25.00005f);
    cv::Mat depth(
        cv::Size(128,96),CV_32FC1,cv::Scalar(2.0f));
    depth.at<float>(70,90) = 0.0f;
    const ORB_SLAM2::GeometricRigidityResult result =
        ORB_SLAM2::GeometricDynamicDetector::
            ComputeLocalRigidity(
                depth,depth,SparseFlowCameraMatrix(),
                MakeRigiditySparseFlow(reference,current));

    Require(result.stats.currentDepthInvalidCount==1,
            "invalid current depth must remain no rigidity evidence");
    Require(result.stats.duplicateImagePointCount==1,
            "numerically near-duplicate image point must be rejected explicitly");
    for(std::size_t index=0; index<result.edges.size(); ++index)
    {
        Require(
            result.edges[index].featureIndexA!=
                result.edges[index].featureIndexB,
            "rigidity graph must not contain self edges");
    }
}

void TestLocalRigidityDepthUncertaintyScaling()
{
    const std::vector<cv::Point2f> pixels = {
        cv::Point2f(30.0f,25.0f),
        cv::Point2f(90.0f,25.0f),
        cv::Point2f(90.0f,70.0f),
        cv::Point2f(30.0f,70.0f),
        cv::Point2f(60.0f,48.0f)};
    const cv::Mat nearDepth(
        cv::Size(128,96),CV_32FC1,cv::Scalar(2.0f));
    const cv::Mat farDepth(
        cv::Size(128,96),CV_32FC1,cv::Scalar(4.0f));
    const ORB_SLAM2::GeometricRigidityResult nearResult =
        ORB_SLAM2::GeometricDynamicDetector::
            ComputeLocalRigidity(
                nearDepth,nearDepth,SparseFlowCameraMatrix(),
                MakeRigiditySparseFlow(pixels,pixels,2.0f));
    const ORB_SLAM2::GeometricRigidityResult farResult =
        ORB_SLAM2::GeometricDynamicDetector::
            ComputeLocalRigidity(
                farDepth,farDepth,SparseFlowCameraMatrix(),
                MakeRigiditySparseFlow(pixels,pixels,4.0f));
    Require(
        farResult.nodes[0].referenceDepthUncertaintyStdMeters>
            3.5f*nearResult.nodes[0].
                referenceDepthUncertaintyStdMeters,
        "depth-square model must increase uncertainty with range");
}

void TestLocalRigidityDepthMixtureUncertainty()
{
    const std::vector<cv::Point2f> pixels = {
        cv::Point2f(30.0f,25.0f),
        cv::Point2f(90.0f,25.0f),
        cv::Point2f(90.0f,70.0f),
        cv::Point2f(30.0f,70.0f),
        cv::Point2f(60.0f,48.0f)};
    const cv::Mat referenceDepth(
        cv::Size(128,96),CV_32FC1,cv::Scalar(2.0f));
    cv::Mat currentDepth = referenceDepth.clone();
    currentDepth.colRange(61,currentDepth.cols).setTo(4.0f);
    const ORB_SLAM2::GeometricRigidityResult result =
        ORB_SLAM2::GeometricDynamicDetector::
            ComputeLocalRigidity(
                referenceDepth,currentDepth,
                SparseFlowCameraMatrix(),
                MakeRigiditySparseFlow(pixels,pixels,2.0f));
    Require(
        result.nodes[4].currentDepthUncertaintyStdMeters>
            10.0f*result.nodes[0].
                currentDepthUncertaintyStdMeters,
        "3x3 depth mixture must expose discontinuity uncertainty");
    Require(
        result.nodes[4].currentDepthNeighborhoodValidWeight==1.0f,
        "valid mixed-depth neighborhood must retain full valid weight");
}

} // namespace

int main()
{
    try
    {
        TestIdentityPlane();
        TestRGBDRectificationDisabledBypass();
        TestRGBDRectificationDepthUsesNearestNeighbor();
        TestRGBDRectificationRejectsDoubleUndistortion();
        TestBonnCalibrationRoundTrip();
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
        TestLowResolutionRegionUsesNativeCells();
        TestRegionEvidenceAggregation();
        TestFeatureEvidenceUsesNativePyramidCells();
        TestSparseEgoFlowIdentityAndLkDirection();
        TestSparseEgoFlowCameraAndIndependentMotion();
        TestSparseEgoFlowInvalidEvidence();
        TestSparseEgoFlowDepthRiskDiagnostics();
        TestLocalRigidityRigidTranslation();
        TestLocalRigidityIndependentNode();
        TestLocalRigidityInvalidAndDuplicateNodes();
        TestLocalRigidityDepthUncertaintyScaling();
        TestLocalRigidityDepthMixtureUncertainty();
    }
    catch(const std::exception &error)
    {
        std::cerr << "[Geometry G0/G2-1/G2-2R/G2-2S/G2-2G/G2-3R0/G2-3R1/G2-3R3/G2-3R4/G2-4A/G2-4B/G2-4F0/G2-4F1/G2-4F2B/G2-4F3/G2-4F3U Test] FAIL: "
                  << error.what() << std::endl;
        return 1;
    }

    std::cout << "[Geometry G0/G2-1/G2-2R/G2-2S/G2-2G/G2-3R0/G2-3R1/G2-3R3/G2-3R4/G2-4A/G2-4B/G2-4F0/G2-4F1/G2-4F2B/G2-4F3/G2-4F3U Test] PASS"
              << std::endl;
    return 0;
}
