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

    const ORB_SLAM2::GeometricWarpResult result =
        ORB_SLAM2::GeometricDynamicDetector::ComputeWarp(
            referenceDepth,currentDepth,IdentityPose(),IdentityPose(),K);

    Require(result.validComparisonMask.at<unsigned char>(2,2)==255,
            "near-surface pixel must have valid geometric evidence");
    Require(std::abs(result.signedDepthResidual.at<float>(2,2)-2.0f)<1e-6f,
            "nearer current surface must produce a positive residual");
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

    const ORB_SLAM2::GeometricWarpResult result =
        ORB_SLAM2::GeometricDynamicDetector::ComputeWarp(
            referenceDepth,currentDepth,IdentityPose(),IdentityPose(),K);

    Require(result.validComparisonMask.at<unsigned char>(1,1)==0,
            "invalid current depth must remain geometrically unknown");
    Require(result.signedDepthResidual.at<float>(1,1)==0.0f,
            "invalid comparisons must not contain a residual value");
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

} // namespace

int main()
{
    try
    {
        TestIdentityPlane();
        TestSignedNearSurface();
        TestTcwDirection();
        TestInvalidDepthIsUnknown();
        TestZBuffer();
    }
    catch(const std::exception &error)
    {
        std::cerr << "[Geometry G0-1 Test] FAIL: "
                  << error.what() << std::endl;
        return 1;
    }

    std::cout << "[Geometry G0-1 Test] PASS" << std::endl;
    return 0;
}
