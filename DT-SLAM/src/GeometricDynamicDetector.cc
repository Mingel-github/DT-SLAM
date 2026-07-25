/**
* This file is part of DT-SLAM.
*
* G0-1 extracts single-reference RGB-D geometric inconsistency evidence.
* It does not classify dynamic pixels or modify SLAM state.
*/

#include "GeometricDynamicDetector.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <stdexcept>
#include <string>

namespace ORB_SLAM2
{

namespace
{

cv::Mat AsFloatMatrix(const cv::Mat &matrix)
{
    cv::Mat floatMatrix;
    if(matrix.type()==CV_32F)
        floatMatrix = matrix;
    else
        matrix.convertTo(floatMatrix,CV_32F);
    return floatMatrix;
}

void ValidateDepth(const cv::Mat &depth, const char *name)
{
    if(depth.empty() || depth.type()!=CV_32FC1)
        throw std::invalid_argument(std::string(name)+" must be a non-empty CV_32FC1 image");
}

void ValidatePose(const cv::Mat &pose, const char *name)
{
    if(pose.empty() || pose.rows!=4 || pose.cols!=4 || pose.channels()!=1)
        throw std::invalid_argument(std::string(name)+" must be a non-empty 4x4 matrix");
}

void ValidateCameraMatrix(const cv::Mat &K)
{
    if(K.empty() || K.rows!=3 || K.cols!=3 || K.channels()!=1)
        throw std::invalid_argument("K must be a non-empty 3x3 matrix");
}

bool IsValidDepth(const float depth)
{
    return std::isfinite(depth) && depth>0.0f;
}

} // namespace

GeometricDynamicDetector::GeometricDynamicDetector()
    : mnReferenceFrameId(0), mbHasReference(false)
{
}

void GeometricDynamicDetector::SetCameraMatrix(const cv::Mat &K)
{
    ValidateCameraMatrix(K);
    mK = AsFloatMatrix(K).clone();
}

void GeometricDynamicDetector::UpdateReference(const cv::Mat &depthMeters,
                                               const cv::Mat &Tcw,
                                               const long unsigned int frameId)
{
    ValidateDepth(depthMeters,"reference depth");
    ValidatePose(Tcw,"reference Tcw");
    if(mK.empty())
        throw std::logic_error("camera matrix must be set before updating the geometry reference");

    mReferenceDepthMeters = depthMeters.clone();
    mTcwReference = AsFloatMatrix(Tcw).clone();
    mnReferenceFrameId = frameId;
    mbHasReference = true;
}

void GeometricDynamicDetector::ResetReference()
{
    mReferenceDepthMeters.release();
    mTcwReference.release();
    mnReferenceFrameId = 0;
    mbHasReference = false;
}

bool GeometricDynamicDetector::HasReference() const
{
    return mbHasReference;
}

long unsigned int GeometricDynamicDetector::ReferenceFrameId() const
{
    return mnReferenceFrameId;
}

bool GeometricDynamicDetector::Compute(const cv::Mat &currentDepthMeters,
                                       const cv::Mat &TcwCurrent,
                                       GeometricWarpResult &result) const
{
    if(!mbHasReference)
        return false;

    result = ComputeWarp(mReferenceDepthMeters,currentDepthMeters,
                         mTcwReference,TcwCurrent,mK);
    return true;
}

GeometricWarpResult GeometricDynamicDetector::ComputeWarp(
    const cv::Mat &referenceDepthMeters,
    const cv::Mat &currentDepthMeters,
    const cv::Mat &TcwReference,
    const cv::Mat &TcwCurrent,
    const cv::Mat &K)
{
    const std::chrono::steady_clock::time_point totalStart =
        std::chrono::steady_clock::now();

    ValidateDepth(referenceDepthMeters,"reference depth");
    ValidateDepth(currentDepthMeters,"current depth");
    ValidatePose(TcwReference,"reference Tcw");
    ValidatePose(TcwCurrent,"current Tcw");
    ValidateCameraMatrix(K);

    if(referenceDepthMeters.size()!=currentDepthMeters.size())
        throw std::invalid_argument("reference and current depth images must have the same size");

    const cv::Mat Kf = AsFloatMatrix(K);
    const cv::Mat TcwReferenceFloat = AsFloatMatrix(TcwReference);
    const cv::Mat TcwCurrentFloat = AsFloatMatrix(TcwCurrent);
    const cv::Mat TCurrentReference =
        TcwCurrentFloat*TcwReferenceFloat.inv();

    const float fx = Kf.at<float>(0,0);
    const float fy = Kf.at<float>(1,1);
    const float cx = Kf.at<float>(0,2);
    const float cy = Kf.at<float>(1,2);
    if(!std::isfinite(fx) || !std::isfinite(fy) ||
       !std::isfinite(cx) || !std::isfinite(cy) ||
       fx<=0.0f || fy<=0.0f)
    {
        throw std::invalid_argument("K contains invalid pinhole intrinsics");
    }

    const float r00 = TCurrentReference.at<float>(0,0);
    const float r01 = TCurrentReference.at<float>(0,1);
    const float r02 = TCurrentReference.at<float>(0,2);
    const float tx = TCurrentReference.at<float>(0,3);
    const float r10 = TCurrentReference.at<float>(1,0);
    const float r11 = TCurrentReference.at<float>(1,1);
    const float r12 = TCurrentReference.at<float>(1,2);
    const float ty = TCurrentReference.at<float>(1,3);
    const float r20 = TCurrentReference.at<float>(2,0);
    const float r21 = TCurrentReference.at<float>(2,1);
    const float r22 = TCurrentReference.at<float>(2,2);
    const float tz = TCurrentReference.at<float>(2,3);

    GeometricWarpResult result;
    result.predictedDepth =
        cv::Mat::zeros(currentDepthMeters.size(),CV_32FC1);

    const std::chrono::steady_clock::time_point warpStart =
        std::chrono::steady_clock::now();

    for(int v=0; v<referenceDepthMeters.rows; ++v)
    {
        const float *referenceRow = referenceDepthMeters.ptr<float>(v);
        for(int u=0; u<referenceDepthMeters.cols; ++u)
        {
            const float depth = referenceRow[u];
            if(!IsValidDepth(depth))
                continue;

            ++result.stats.referenceValidPixels;

            const float xReference = (static_cast<float>(u)-cx)*depth/fx;
            const float yReference = (static_cast<float>(v)-cy)*depth/fy;

            const float xCurrent =
                r00*xReference+r01*yReference+r02*depth+tx;
            const float yCurrent =
                r10*xReference+r11*yReference+r12*depth+ty;
            const float zCurrent =
                r20*xReference+r21*yReference+r22*depth+tz;

            if(!IsValidDepth(zCurrent))
                continue;

            const float projectedU = fx*xCurrent/zCurrent+cx;
            const float projectedV = fy*yCurrent/zCurrent+cy;
            if(!std::isfinite(projectedU) || !std::isfinite(projectedV))
                continue;
            if(projectedU<0.0f ||
               projectedU>static_cast<float>(result.predictedDepth.cols-1) ||
               projectedV<0.0f ||
               projectedV>static_cast<float>(result.predictedDepth.rows-1))
            {
                continue;
            }

            const int targetU = cvRound(projectedU);
            const int targetV = cvRound(projectedV);
            if(targetU<0 || targetU>=result.predictedDepth.cols ||
               targetV<0 || targetV>=result.predictedDepth.rows)
            {
                continue;
            }

            ++result.stats.projectedSamples;

            float &predicted =
                result.predictedDepth.at<float>(targetV,targetU);
            if(!IsValidDepth(predicted) || zCurrent<predicted)
                predicted = zCurrent;
        }
    }

    const std::chrono::steady_clock::time_point warpEnd =
        std::chrono::steady_clock::now();

    result.validComparisonMask =
        cv::Mat::zeros(currentDepthMeters.size(),CV_8UC1);
    result.signedDepthResidual =
        cv::Mat::zeros(currentDepthMeters.size(),CV_32FC1);

    double residualSum = 0.0;
    double residualAbsSum = 0.0;
    double residualMaxAbs = 0.0;

    for(int v=0; v<currentDepthMeters.rows; ++v)
    {
        const float *predictedRow = result.predictedDepth.ptr<float>(v);
        const float *currentRow = currentDepthMeters.ptr<float>(v);
        float *residualRow = result.signedDepthResidual.ptr<float>(v);
        unsigned char *validRow = result.validComparisonMask.ptr<unsigned char>(v);

        for(int u=0; u<currentDepthMeters.cols; ++u)
        {
            const float predicted = predictedRow[u];
            const float current = currentRow[u];

            if(IsValidDepth(predicted))
                ++result.stats.zbufferValidPixels;
            if(IsValidDepth(current))
                ++result.stats.currentValidPixels;
            if(!IsValidDepth(predicted) || !IsValidDepth(current))
                continue;

            const float residual = predicted-current;
            residualRow[u] = residual;
            validRow[u] = 255;
            ++result.stats.validComparisons;

            residualSum += residual;
            const double residualAbs = std::abs(static_cast<double>(residual));
            residualAbsSum += residualAbs;
            residualMaxAbs = std::max(residualMaxAbs,residualAbs);
        }
    }

    const std::chrono::steady_clock::time_point residualEnd =
        std::chrono::steady_clock::now();

    const double pixelCount =
        static_cast<double>(currentDepthMeters.total());
    if(pixelCount>0.0)
    {
        result.stats.predictionCoverageRatio =
            static_cast<double>(result.stats.zbufferValidPixels)/pixelCount;
        result.stats.comparisonCoverageRatio =
            static_cast<double>(result.stats.validComparisons)/pixelCount;
    }

    if(result.stats.validComparisons>0)
    {
        const double comparisonCount =
            static_cast<double>(result.stats.validComparisons);
        result.stats.residualMean = residualSum/comparisonCount;
        result.stats.residualMeanAbs = residualAbsSum/comparisonCount;
        result.stats.residualMaxAbs = residualMaxAbs;
    }

    result.stats.warpMs =
        std::chrono::duration<double,std::milli>(warpEnd-warpStart).count();
    result.stats.residualMs =
        std::chrono::duration<double,std::milli>(residualEnd-warpEnd).count();
    result.stats.totalMs =
        std::chrono::duration<double,std::milli>(residualEnd-totalStart).count();

    return result;
}

} // namespace ORB_SLAM2
