/**
* This file is part of DT-SLAM.
*
* G0-1 computes single-reference RGB-D residuals, G0-2 classifies
* diagnostic evidence, and G0-3 forms depth-connected shadow candidates.
* None of these stages modifies SLAM state.
*/

#include "GeometricDynamicDetector.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <deque>
#include <stdexcept>
#include <string>
#include <vector>

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
    : mResidualThresholdMeters(0.10f),
      mRegionDepthThresholdMeters(0.05f),
      mnReferenceFrameId(0), mReferenceTimestampSeconds(0.0),
      mbHasReference(false),
      mbRegionGrowEnabled(false)
{
}

void GeometricDynamicDetector::SetCameraMatrix(const cv::Mat &K)
{
    ValidateCameraMatrix(K);
    mK = AsFloatMatrix(K).clone();
}

void GeometricDynamicDetector::SetResidualThresholdMeters(
    const float thresholdMeters)
{
    if(!std::isfinite(thresholdMeters) || thresholdMeters<=0.0f)
        throw std::invalid_argument("geometry residual threshold must be finite and positive");
    mResidualThresholdMeters = thresholdMeters;
}

float GeometricDynamicDetector::ResidualThresholdMeters() const
{
    return mResidualThresholdMeters;
}

void GeometricDynamicDetector::SetRegionGrowEnabled(const bool enabled)
{
    mbRegionGrowEnabled = enabled;
}

bool GeometricDynamicDetector::RegionGrowEnabled() const
{
    return mbRegionGrowEnabled;
}

void GeometricDynamicDetector::SetRegionDepthThresholdMeters(
    const float thresholdMeters)
{
    if(!std::isfinite(thresholdMeters) || thresholdMeters<=0.0f)
        throw std::invalid_argument(
            "geometry region depth threshold must be finite and positive");
    mRegionDepthThresholdMeters = thresholdMeters;
}

float GeometricDynamicDetector::RegionDepthThresholdMeters() const
{
    return mRegionDepthThresholdMeters;
}

void GeometricDynamicDetector::UpdateReference(const cv::Mat &depthMeters,
                                               const cv::Mat &Tcw,
                                               const long unsigned int frameId,
                                               const double timestampSeconds)
{
    ValidateDepth(depthMeters,"reference depth");
    ValidatePose(Tcw,"reference Tcw");
    if(!std::isfinite(timestampSeconds))
        throw std::invalid_argument("reference timestamp must be finite");
    if(mK.empty())
        throw std::logic_error("camera matrix must be set before updating the geometry reference");

    mReferenceDepthMeters = depthMeters.clone();
    mTcwReference = AsFloatMatrix(Tcw).clone();
    mnReferenceFrameId = frameId;
    mReferenceTimestampSeconds = timestampSeconds;
    mbHasReference = true;
}

void GeometricDynamicDetector::ResetReference()
{
    mReferenceDepthMeters.release();
    mTcwReference.release();
    mnReferenceFrameId = 0;
    mReferenceTimestampSeconds = 0.0;
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

double GeometricDynamicDetector::ReferenceTimestampSeconds() const
{
    return mReferenceTimestampSeconds;
}

bool GeometricDynamicDetector::Compute(const cv::Mat &currentDepthMeters,
                                       const cv::Mat &TcwCurrent,
                                       GeometricWarpResult &result) const
{
    if(!mbHasReference)
        return false;

    result = ComputeWarp(mReferenceDepthMeters,currentDepthMeters,
                         mTcwReference,TcwCurrent,mK);
    ClassifyEvidence(result,mResidualThresholdMeters);
    if(mbRegionGrowEnabled)
        GrowDepthRegions(currentDepthMeters,result,mRegionDepthThresholdMeters);
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

void GeometricDynamicDetector::ClassifyEvidence(
    GeometricWarpResult &result,
    const float thresholdMeters)
{
    if(!std::isfinite(thresholdMeters) || thresholdMeters<=0.0f)
        throw std::invalid_argument("geometry residual threshold must be finite and positive");
    if(result.validComparisonMask.empty() ||
       result.validComparisonMask.type()!=CV_8UC1)
    {
        throw std::invalid_argument("validComparisonMask must be a non-empty CV_8UC1 image");
    }
    if(result.signedDepthResidual.empty() ||
       result.signedDepthResidual.type()!=CV_32FC1 ||
       result.signedDepthResidual.size()!=result.validComparisonMask.size())
    {
        throw std::invalid_argument(
            "signedDepthResidual must be CV_32FC1 and match validComparisonMask");
    }

    const std::chrono::steady_clock::time_point evidenceStart =
        std::chrono::steady_clock::now();

    result.consistentEvidenceMask =
        cv::Mat::zeros(result.validComparisonMask.size(),CV_8UC1);
    result.positiveSeedMask =
        cv::Mat::zeros(result.validComparisonMask.size(),CV_8UC1);
    result.negativeDiagnosticMask =
        cv::Mat::zeros(result.validComparisonMask.size(),CV_8UC1);

    result.stats.consistentEvidencePixels = 0;
    result.stats.positiveSeedPixels = 0;
    result.stats.negativeDiagnosticPixels = 0;
    result.stats.consistentEvidenceRatio = 0.0;
    result.stats.positiveSeedRatio = 0.0;
    result.stats.negativeDiagnosticRatio = 0.0;

    std::size_t evidenceValidPixels = 0;
    for(int v=0; v<result.validComparisonMask.rows; ++v)
    {
        const unsigned char *validRow =
            result.validComparisonMask.ptr<unsigned char>(v);
        const float *residualRow =
            result.signedDepthResidual.ptr<float>(v);
        unsigned char *consistentRow =
            result.consistentEvidenceMask.ptr<unsigned char>(v);
        unsigned char *positiveRow =
            result.positiveSeedMask.ptr<unsigned char>(v);
        unsigned char *negativeRow =
            result.negativeDiagnosticMask.ptr<unsigned char>(v);

        for(int u=0; u<result.validComparisonMask.cols; ++u)
        {
            if(validRow[u]==0)
                continue;

            const float residual = residualRow[u];
            if(!std::isfinite(residual))
                throw std::logic_error("valid geometric comparison contains a non-finite residual");

            ++evidenceValidPixels;
            if(residual>thresholdMeters)
            {
                positiveRow[u] = 255;
                ++result.stats.positiveSeedPixels;
            }
            else if(residual<-thresholdMeters)
            {
                negativeRow[u] = 255;
                ++result.stats.negativeDiagnosticPixels;
            }
            else
            {
                consistentRow[u] = 255;
                ++result.stats.consistentEvidencePixels;
            }
        }
    }

    result.stats.validComparisons = evidenceValidPixels;
    if(evidenceValidPixels>0)
    {
        const double validCount = static_cast<double>(evidenceValidPixels);
        result.stats.consistentEvidenceRatio =
            static_cast<double>(result.stats.consistentEvidencePixels)/validCount;
        result.stats.positiveSeedRatio =
            static_cast<double>(result.stats.positiveSeedPixels)/validCount;
        result.stats.negativeDiagnosticRatio =
            static_cast<double>(result.stats.negativeDiagnosticPixels)/validCount;
    }

    const std::chrono::steady_clock::time_point evidenceEnd =
        std::chrono::steady_clock::now();
    const double previousEvidenceMs = result.stats.evidenceMs;
    result.stats.evidenceMs =
        std::chrono::duration<double,std::milli>(
            evidenceEnd-evidenceStart).count();
    result.stats.totalMs += result.stats.evidenceMs-previousEvidenceMs;
}

void GeometricDynamicDetector::GrowDepthRegions(
    const cv::Mat &currentDepthMeters,
    GeometricWarpResult &result,
    const float depthThresholdMeters)
{
    ValidateDepth(currentDepthMeters,"current depth");
    if(!std::isfinite(depthThresholdMeters) || depthThresholdMeters<=0.0f)
    {
        throw std::invalid_argument(
            "geometry region depth threshold must be finite and positive");
    }
    if(result.validComparisonMask.empty() ||
       result.validComparisonMask.type()!=CV_8UC1 ||
       result.validComparisonMask.size()!=currentDepthMeters.size())
    {
        throw std::invalid_argument(
            "validComparisonMask must be CV_8UC1 and match current depth");
    }
    if(result.positiveSeedMask.empty() ||
       result.positiveSeedMask.type()!=CV_8UC1 ||
       result.positiveSeedMask.size()!=currentDepthMeters.size())
    {
        throw std::invalid_argument(
            "positiveSeedMask must be CV_8UC1 and match current depth");
    }
    if(result.negativeDiagnosticMask.empty() ||
       result.negativeDiagnosticMask.type()!=CV_8UC1 ||
       result.negativeDiagnosticMask.size()!=currentDepthMeters.size())
    {
        throw std::invalid_argument(
            "negativeDiagnosticMask must be CV_8UC1 and match current depth");
    }
    if(result.signedDepthResidual.empty() ||
       result.signedDepthResidual.type()!=CV_32FC1 ||
       result.signedDepthResidual.size()!=currentDepthMeters.size())
    {
        throw std::invalid_argument(
            "signedDepthResidual must be CV_32FC1 and match current depth");
    }

    const std::chrono::steady_clock::time_point regionStart =
        std::chrono::steady_clock::now();

    result.regionCandidateMask =
        cv::Mat::zeros(currentDepthMeters.size(),CV_8UC1);
    result.regionPositiveSupport =
        cv::Mat::zeros(currentDepthMeters.size(),CV_32FC1);
    result.depthRegions.clear();
    cv::Mat visited =
        cv::Mat::zeros(currentDepthMeters.size(),CV_8UC1);

    result.stats.depthRegionCount = 0;
    result.stats.regionCandidatePixels = 0;
    result.stats.largestRegionPixels = 0;
    result.stats.regionGrowthRatio = 0.0;

    const int neighborU[4] = {-1,1,0,0};
    const int neighborV[4] = {0,0,-1,1};
    std::deque<cv::Point> frontier;
    std::vector<cv::Point> regionPixels;
    std::vector<float> regionResiduals;

    for(int seedV=0; seedV<currentDepthMeters.rows; ++seedV)
    {
        const unsigned char *positiveRow =
            result.positiveSeedMask.ptr<unsigned char>(seedV);
        for(int seedU=0; seedU<currentDepthMeters.cols; ++seedU)
        {
            if(positiveRow[seedU]==0 ||
               visited.at<unsigned char>(seedV,seedU)!=0)
            {
                continue;
            }
            if(result.validComparisonMask.at<unsigned char>(seedV,seedU)==0 ||
               !IsValidDepth(currentDepthMeters.at<float>(seedV,seedU)))
            {
                continue;
            }

            frontier.clear();
            regionPixels.clear();
            regionResiduals.clear();
            frontier.push_back(cv::Point(seedU,seedV));
            visited.at<unsigned char>(seedV,seedU) = 255;

            while(!frontier.empty())
            {
                const cv::Point pixel = frontier.front();
                frontier.pop_front();
                regionPixels.push_back(pixel);

                const float pixelDepth =
                    currentDepthMeters.at<float>(pixel.y,pixel.x);
                for(int neighborIndex=0; neighborIndex<4; ++neighborIndex)
                {
                    const int u = pixel.x+neighborU[neighborIndex];
                    const int v = pixel.y+neighborV[neighborIndex];
                    if(u<0 || u>=currentDepthMeters.cols ||
                       v<0 || v>=currentDepthMeters.rows ||
                       visited.at<unsigned char>(v,u)!=0 ||
                       result.validComparisonMask.at<unsigned char>(v,u)==0)
                    {
                        continue;
                    }

                    const float neighborDepth =
                        currentDepthMeters.at<float>(v,u);
                    if(!IsValidDepth(neighborDepth) ||
                       std::abs(neighborDepth-pixelDepth)>depthThresholdMeters)
                    {
                        continue;
                    }

                    visited.at<unsigned char>(v,u) = 255;
                    frontier.push_back(cv::Point(u,v));
                }
            }

            ++result.stats.depthRegionCount;
            result.stats.largestRegionPixels =
                std::max(result.stats.largestRegionPixels,regionPixels.size());
            result.stats.regionCandidatePixels += regionPixels.size();

            GeometricDepthRegionStats regionStats;
            regionStats.pixels = regionPixels.size();
            for(std::size_t index=0; index<regionPixels.size(); ++index)
            {
                const cv::Point &pixel = regionPixels[index];
                result.regionCandidateMask.at<unsigned char>(
                    pixel.y,pixel.x) = 255;

                if(result.positiveSeedMask.at<unsigned char>(
                       pixel.y,pixel.x)!=0)
                {
                    ++regionStats.positiveSeedPixels;
                }
                if(result.negativeDiagnosticMask.at<unsigned char>(
                       pixel.y,pixel.x)!=0)
                {
                    ++regionStats.negativeDiagnosticPixels;
                }
                regionResiduals.push_back(
                    result.signedDepthResidual.at<float>(
                        pixel.y,pixel.x));
            }

            if(regionStats.pixels>0)
            {
                const double regionPixelCount =
                    static_cast<double>(regionStats.pixels);
                regionStats.positiveSeedRatio =
                    static_cast<double>(regionStats.positiveSeedPixels)/
                    regionPixelCount;
                regionStats.negativeDiagnosticRatio =
                    static_cast<double>(regionStats.negativeDiagnosticPixels)/
                    regionPixelCount;

                const std::size_t medianIndex =
                    regionResiduals.size()/2;
                std::nth_element(
                    regionResiduals.begin(),
                    regionResiduals.begin()+medianIndex,
                    regionResiduals.end());
                regionStats.signedResidualMedian =
                    regionResiduals[medianIndex];

                for(std::size_t index=0; index<regionPixels.size(); ++index)
                {
                    const cv::Point &pixel = regionPixels[index];
                    result.regionPositiveSupport.at<float>(
                        pixel.y,pixel.x) =
                        static_cast<float>(regionStats.positiveSeedRatio);
                }
            }
            result.depthRegions.push_back(regionStats);
        }
    }

    if(result.stats.positiveSeedPixels>0)
    {
        result.stats.regionGrowthRatio =
            static_cast<double>(result.stats.regionCandidatePixels)/
            static_cast<double>(result.stats.positiveSeedPixels);
    }

    const std::chrono::steady_clock::time_point regionEnd =
        std::chrono::steady_clock::now();
    const double previousRegionMs = result.stats.regionGrowMs;
    result.stats.regionGrowMs =
        std::chrono::duration<double,std::milli>(
            regionEnd-regionStart).count();
    result.stats.totalMs += result.stats.regionGrowMs-previousRegionMs;
}

} // namespace ORB_SLAM2
