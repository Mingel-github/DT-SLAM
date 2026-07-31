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
#include <limits>
#include <map>
#include <set>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/tracking.hpp>

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

cv::Point3f BackProjectPoint(
    const cv::Point2f &pixel,
    const float depthMeters,
    const cv::Mat &K)
{
    const float fx = K.at<float>(0,0);
    const float fy = K.at<float>(1,1);
    const float cx = K.at<float>(0,2);
    const float cy = K.at<float>(1,2);
    return cv::Point3f(
        (pixel.x-cx)*depthMeters/fx,
        (pixel.y-cy)*depthMeters/fy,
        depthMeters);
}

struct AxialDepthUncertainty
{
    float meanMeters = 0.0f;
    float standardDeviationMeters = 0.0f;
    float validWeight = 0.0f;
    bool valid = false;
};

AxialDepthUncertainty ComputeAxialDepthUncertainty(
    const cv::Mat &depthMeters,
    const cv::Point2f &pixel,
    const float axialDepthNoiseCoefficientPerMeter)
{
    static const int weights[3][3] = {
        {1,2,1},
        {2,4,2},
        {1,2,1}
    };
    const int centerU = cvRound(pixel.x);
    const int centerV = cvRound(pixel.y);
    AxialDepthUncertainty uncertainty;
    if(centerU<0 || centerV<0 ||
       centerU>=depthMeters.cols || centerV>=depthMeters.rows ||
       !IsValidDepth(depthMeters.at<float>(centerV,centerU)))
    {
        return uncertainty;
    }

    double validWeight = 0.0;
    double weightedMean = 0.0;
    double weightedSecondMoment = 0.0;
    for(int offsetV=-1; offsetV<=1; ++offsetV)
    {
        for(int offsetU=-1; offsetU<=1; ++offsetU)
        {
            const int u = centerU+offsetU;
            const int v = centerV+offsetV;
            if(u<0 || v<0 || u>=depthMeters.cols ||
               v>=depthMeters.rows)
            {
                continue;
            }
            const float depth = depthMeters.at<float>(v,u);
            if(!IsValidDepth(depth))
                continue;
            const double weight =
                static_cast<double>(weights[offsetV+1][offsetU+1]);
            const double standardDeviation =
                static_cast<double>(
                    axialDepthNoiseCoefficientPerMeter)*
                static_cast<double>(depth)*
                static_cast<double>(depth);
            validWeight += weight;
            weightedMean += weight*static_cast<double>(depth);
            weightedSecondMoment +=
                weight*(standardDeviation*standardDeviation+
                        static_cast<double>(depth)*
                        static_cast<double>(depth));
        }
    }
    if(validWeight<=0.0)
        return uncertainty;

    const double mean = weightedMean/validWeight;
    const double variance = std::max(
        0.0,weightedSecondMoment/validWeight-mean*mean);
    uncertainty.meanMeters = static_cast<float>(mean);
    uncertainty.standardDeviationMeters =
        static_cast<float>(std::sqrt(variance));
    uncertainty.validWeight =
        static_cast<float>(validWeight/16.0);
    uncertainty.valid =
        std::isfinite(uncertainty.standardDeviationMeters) &&
        uncertainty.standardDeviationMeters>0.0f;
    return uncertainty;
}

float EdgeLengthAxialVariance(
    const cv::Point3f &firstPoint,
    const cv::Point3f &secondPoint,
    const cv::Point2f &firstPixel,
    const cv::Point2f &secondPixel,
    const float firstDepthStandardDeviationMeters,
    const float secondDepthStandardDeviationMeters,
    const cv::Mat &K)
{
    const cv::Point3f edge = firstPoint-secondPoint;
    const float distance = cv::norm(edge);
    if(!std::isfinite(distance) || distance<=0.0f)
        return std::numeric_limits<float>::quiet_NaN();
    const cv::Point3f direction = edge*(1.0f/distance);
    const float fx = K.at<float>(0,0);
    const float fy = K.at<float>(1,1);
    const float cx = K.at<float>(0,2);
    const float cy = K.at<float>(1,2);
    const cv::Point3f firstRay(
        (firstPixel.x-cx)/fx,(firstPixel.y-cy)/fy,1.0f);
    const cv::Point3f secondRay(
        (secondPixel.x-cx)/fx,(secondPixel.y-cy)/fy,1.0f);
    const float firstJacobian = direction.dot(firstRay);
    const float secondJacobian = -direction.dot(secondRay);
    const float variance =
        firstJacobian*firstJacobian*
            firstDepthStandardDeviationMeters*
            firstDepthStandardDeviationMeters+
        secondJacobian*secondJacobian*
            secondDepthStandardDeviationMeters*
            secondDepthStandardDeviationMeters;
    return std::isfinite(variance) && variance>=0.0f
        ? variance : std::numeric_limits<float>::quiet_NaN();
}

void ValidateGrayImage(const cv::Mat &image, const char *name)
{
    if(image.empty() || image.type()!=CV_8UC1)
    {
        throw std::invalid_argument(
            std::string(name)+" must be a non-empty CV_8UC1 image");
    }
}

bool ProjectSparseFlowPoint(
    const cv::Point2f &referencePixel,
    const float referenceDepthMeters,
    const cv::Mat &relativePose,
    const cv::Mat &K,
    cv::Point2f &currentPixel)
{
    if(!IsValidDepth(referenceDepthMeters))
        return false;

    const float fx = K.at<float>(0,0);
    const float fy = K.at<float>(1,1);
    const float cx = K.at<float>(0,2);
    const float cy = K.at<float>(1,2);
    const float x =
        (referencePixel.x-cx)*referenceDepthMeters/fx;
    const float y =
        (referencePixel.y-cy)*referenceDepthMeters/fy;

    const float transformedX =
        relativePose.at<float>(0,0)*x+
        relativePose.at<float>(0,1)*y+
        relativePose.at<float>(0,2)*referenceDepthMeters+
        relativePose.at<float>(0,3);
    const float transformedY =
        relativePose.at<float>(1,0)*x+
        relativePose.at<float>(1,1)*y+
        relativePose.at<float>(1,2)*referenceDepthMeters+
        relativePose.at<float>(1,3);
    const float transformedZ =
        relativePose.at<float>(2,0)*x+
        relativePose.at<float>(2,1)*y+
        relativePose.at<float>(2,2)*referenceDepthMeters+
        relativePose.at<float>(2,3);
    if(!std::isfinite(transformedX) ||
       !std::isfinite(transformedY) ||
       !std::isfinite(transformedZ) ||
       transformedZ<=0.0f)
    {
        return false;
    }

    currentPixel.x = fx*transformedX/transformedZ+cx;
    currentPixel.y = fy*transformedY/transformedZ+cy;
    return std::isfinite(currentPixel.x) &&
           std::isfinite(currentPixel.y);
}

double Percentile(
    std::vector<float> values,
    const double probability)
{
    if(values.empty())
        return 0.0;
    std::sort(values.begin(),values.end());
    const double position =
        probability*static_cast<double>(values.size()-1);
    const std::size_t lower =
        static_cast<std::size_t>(std::floor(position));
    const std::size_t upper =
        static_cast<std::size_t>(std::ceil(position));
    if(lower==upper)
        return values[lower];
    const double fraction =
        position-static_cast<double>(lower);
    return
        static_cast<double>(values[lower])*(1.0-fraction)+
        static_cast<double>(values[upper])*fraction;
}

GeometricWarpStats AccumulateSampledReferenceEvidence(
    const GeometricReferenceFrame &reference,
    const std::vector<cv::Point2i> &samplePixels,
    const cv::Mat &currentDepthMeters,
    const cv::Mat &TcwCurrent,
    const cv::Mat &K,
    const float residualThresholdMeters,
    cv::Mat &comparisonCount,
    cv::Mat &positiveCount,
    cv::Mat &negativeCount,
    cv::Mat &consistentCount)
{
    const std::chrono::steady_clock::time_point totalStart =
        std::chrono::steady_clock::now();
    const cv::Mat Kf = AsFloatMatrix(K);
    const cv::Mat TCurrentReference =
        AsFloatMatrix(TcwCurrent)*AsFloatMatrix(reference.Tcw).inv();

    const float fx = Kf.at<float>(0,0);
    const float fy = Kf.at<float>(1,1);
    const float cx = Kf.at<float>(0,2);
    const float cy = Kf.at<float>(1,2);
    if(!std::isfinite(fx) || !std::isfinite(fy) ||
       !std::isfinite(cx) || !std::isfinite(cy) ||
       fx<=0.0f || fy<=0.0f)
    {
        throw std::invalid_argument(
            "K contains invalid pinhole intrinsics");
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

    GeometricWarpStats stats;
    const bool useContiguousZBuffer = samplePixels.size()>=4096;
    std::unordered_map<int,float> predictedDepthByPixel;
    std::vector<float> contiguousPredictedDepth;
    std::vector<int> touchedTargetPixels;
    if(useContiguousZBuffer)
    {
        contiguousPredictedDepth.assign(
            currentDepthMeters.total(),
            std::numeric_limits<float>::infinity());
        touchedTargetPixels.reserve(samplePixels.size());
    }
    else
    {
        predictedDepthByPixel.reserve(
            samplePixels.size()*2+1);
    }

    const std::chrono::steady_clock::time_point warpStart =
        std::chrono::steady_clock::now();
    for(std::size_t sampleIndex=0;
        sampleIndex<samplePixels.size(); ++sampleIndex)
    {
        const cv::Point2i &pixel = samplePixels[sampleIndex];
        if(pixel.x<0 || pixel.x>=reference.depthMeters.cols ||
           pixel.y<0 || pixel.y>=reference.depthMeters.rows)
        {
            continue;
        }

        const float depth =
            reference.depthMeters.at<float>(pixel.y,pixel.x);
        if(!IsValidDepth(depth))
            continue;
        ++stats.referenceValidPixels;

        const float xReference =
            (static_cast<float>(pixel.x)-cx)*depth/fx;
        const float yReference =
            (static_cast<float>(pixel.y)-cy)*depth/fy;
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
        if(!std::isfinite(projectedU) ||
           !std::isfinite(projectedV) ||
           projectedU<0.0f ||
           projectedU>
               static_cast<float>(currentDepthMeters.cols-1) ||
           projectedV<0.0f ||
           projectedV>
               static_cast<float>(currentDepthMeters.rows-1))
        {
            continue;
        }

        const int targetU = cvRound(projectedU);
        const int targetV = cvRound(projectedV);
        if(targetU<0 || targetU>=currentDepthMeters.cols ||
           targetV<0 || targetV>=currentDepthMeters.rows)
        {
            continue;
        }
        ++stats.projectedSamples;

        const int targetIndex =
            targetV*currentDepthMeters.cols+targetU;
        if(useContiguousZBuffer)
        {
            float &existing =
                contiguousPredictedDepth[
                    static_cast<std::size_t>(targetIndex)];
            if(!std::isfinite(existing))
            {
                existing = zCurrent;
                touchedTargetPixels.push_back(targetIndex);
            }
            else if(zCurrent<existing)
            {
                existing = zCurrent;
            }
        }
        else
        {
            const std::unordered_map<int,float>::iterator existing =
                predictedDepthByPixel.find(targetIndex);
            if(existing==predictedDepthByPixel.end())
            {
                predictedDepthByPixel.insert(
                    std::make_pair(targetIndex,zCurrent));
            }
            else if(zCurrent<existing->second)
            {
                existing->second = zCurrent;
            }
        }
    }
    const std::chrono::steady_clock::time_point warpEnd =
        std::chrono::steady_clock::now();

    stats.zbufferValidPixels =
        useContiguousZBuffer
            ? touchedTargetPixels.size()
            : predictedDepthByPixel.size();
    double residualSum = 0.0;
    double residualAbsSum = 0.0;
    double residualMaxAbs = 0.0;
    const auto classifyPrediction =
        [&](const int targetIndex,const float predictedDepth)
        {
        const int targetV = targetIndex/currentDepthMeters.cols;
        const int targetU = targetIndex%currentDepthMeters.cols;
        const float current =
            currentDepthMeters.at<float>(targetV,targetU);
        if(!IsValidDepth(current))
            return;

        ++stats.currentValidPixels;
        ++stats.validComparisons;
        ++comparisonCount.at<unsigned char>(targetV,targetU);
        const float residual = predictedDepth-current;
        residualSum += residual;
        const double residualAbs =
            std::abs(static_cast<double>(residual));
        residualAbsSum += residualAbs;
        residualMaxAbs =
            std::max(residualMaxAbs,residualAbs);

        if(residual>residualThresholdMeters)
        {
            ++positiveCount.at<unsigned char>(targetV,targetU);
            ++stats.positiveSeedPixels;
        }
        else if(residual<-residualThresholdMeters)
        {
            ++negativeCount.at<unsigned char>(targetV,targetU);
            ++stats.negativeDiagnosticPixels;
        }
        else
        {
            ++consistentCount.at<unsigned char>(targetV,targetU);
            ++stats.consistentEvidencePixels;
        }
        };
    if(useContiguousZBuffer)
    {
        for(std::size_t touchedIndex=0;
            touchedIndex<touchedTargetPixels.size(); ++touchedIndex)
        {
            const int targetIndex =
                touchedTargetPixels[touchedIndex];
            classifyPrediction(
                targetIndex,
                contiguousPredictedDepth[
                    static_cast<std::size_t>(targetIndex)]);
        }
    }
    else
    {
        for(std::unordered_map<int,float>::const_iterator
                prediction=predictedDepthByPixel.begin();
            prediction!=predictedDepthByPixel.end(); ++prediction)
        {
            classifyPrediction(prediction->first,prediction->second);
        }
    }
    const std::chrono::steady_clock::time_point evidenceEnd =
        std::chrono::steady_clock::now();

    const double imagePixels =
        static_cast<double>(currentDepthMeters.total());
    if(imagePixels>0.0)
    {
        stats.predictionCoverageRatio =
            static_cast<double>(stats.zbufferValidPixels)/
            imagePixels;
        stats.comparisonCoverageRatio =
            static_cast<double>(stats.validComparisons)/
            imagePixels;
    }
    if(stats.validComparisons>0)
    {
        const double validComparisons =
            static_cast<double>(stats.validComparisons);
        stats.consistentEvidenceRatio =
            static_cast<double>(
                stats.consistentEvidencePixels)/
            validComparisons;
        stats.positiveSeedRatio =
            static_cast<double>(stats.positiveSeedPixels)/
            validComparisons;
        stats.negativeDiagnosticRatio =
            static_cast<double>(
                stats.negativeDiagnosticPixels)/
            validComparisons;
        stats.residualMean = residualSum/validComparisons;
        stats.residualMeanAbs =
            residualAbsSum/validComparisons;
        stats.residualMaxAbs = residualMaxAbs;
    }
    stats.warpMs =
        std::chrono::duration<double,std::milli>(
            warpEnd-warpStart).count();
    stats.evidenceMs =
        std::chrono::duration<double,std::milli>(
            evidenceEnd-warpEnd).count();
    stats.totalMs =
        std::chrono::duration<double,std::milli>(
            evidenceEnd-totalStart).count();
    return stats;
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

GeometricMultiReferenceResult
GeometricDynamicDetector::ComputeMultiReferenceEvidence(
    const std::vector<GeometricReferenceFrame> &references,
    const cv::Mat &currentDepthMeters,
    const cv::Mat &TcwCurrent,
    const cv::Mat &K,
    const float residualThresholdMeters,
    const GeometricReferenceSamplingPolicy samplingPolicy)
{
    const std::chrono::steady_clock::time_point totalStart =
        std::chrono::steady_clock::now();

    ValidateDepth(currentDepthMeters,"current depth");
    ValidatePose(TcwCurrent,"current Tcw");
    ValidateCameraMatrix(K);
    if(references.empty())
        throw std::invalid_argument("multi-reference evidence requires at least one reference");
    if(references.size()>255)
        throw std::invalid_argument("multi-reference vote counts support at most 255 references");
    if(!std::isfinite(residualThresholdMeters) ||
       residualThresholdMeters<=0.0f)
    {
        throw std::invalid_argument(
            "multi-reference residual threshold must be finite and positive");
    }
    if(samplingPolicy==GeometricReferenceSamplingPolicy::PyramidDense)
    {
        throw std::invalid_argument(
            "pyramid dense evidence requires "
            "ComputePyramidMultiReferenceEvidence");
    }

    GeometricMultiReferenceResult result;
    result.comparisonCount =
        cv::Mat::zeros(currentDepthMeters.size(),CV_8UC1);
    result.positiveCount =
        cv::Mat::zeros(currentDepthMeters.size(),CV_8UC1);
    result.negativeCount =
        cv::Mat::zeros(currentDepthMeters.size(),CV_8UC1);
    result.consistentCount =
        cv::Mat::zeros(currentDepthMeters.size(),CV_8UC1);
    result.stats.referenceCount = references.size();

    double warpAndEvidenceMs = 0.0;
    const std::chrono::steady_clock::time_point aggregateStart =
        std::chrono::steady_clock::now();
    for(std::size_t referenceIndex=0;
        referenceIndex<references.size(); ++referenceIndex)
    {
        const GeometricReferenceFrame &reference =
            references[referenceIndex];
        ValidateDepth(reference.depthMeters,"multi-reference depth");
        ValidatePose(reference.Tcw,"multi-reference Tcw");
        if(reference.depthMeters.size()!=currentDepthMeters.size())
        {
            throw std::invalid_argument(
                "multi-reference and current depth images must have the same size");
        }

        GeometricPerReferenceStats perReference;
        perReference.frameId = reference.frameId;
        if(samplingPolicy==GeometricReferenceSamplingPolicy::OrbDepth ||
           samplingPolicy==GeometricReferenceSamplingPolicy::GridDepth)
        {
            const std::vector<cv::Point2i> &samplePixels =
                samplingPolicy==GeometricReferenceSamplingPolicy::OrbDepth
                    ? reference.featureDepthPixels
                    : reference.gridDepthPixels;
            perReference.warp =
                AccumulateSampledReferenceEvidence(
                    reference,samplePixels,currentDepthMeters,TcwCurrent,K,
                    residualThresholdMeters,result.comparisonCount,
                    result.positiveCount,result.negativeCount,
                    result.consistentCount);
        }
        else
        {
            GeometricWarpResult single = ComputeWarp(
                reference.depthMeters,currentDepthMeters,
                reference.Tcw,TcwCurrent,K);
            ClassifyEvidence(single,residualThresholdMeters);
            perReference.warp = single.stats;

            for(int v=0; v<currentDepthMeters.rows; ++v)
            {
                const unsigned char *validRow =
                    single.validComparisonMask.ptr<unsigned char>(v);
                const unsigned char *positiveRow =
                    single.positiveSeedMask.ptr<unsigned char>(v);
                const unsigned char *negativeRow =
                    single.negativeDiagnosticMask.ptr<unsigned char>(v);
                const unsigned char *consistentRow =
                    single.consistentEvidenceMask.ptr<unsigned char>(v);
                unsigned char *comparisonCountRow =
                    result.comparisonCount.ptr<unsigned char>(v);
                unsigned char *positiveCountRow =
                    result.positiveCount.ptr<unsigned char>(v);
                unsigned char *negativeCountRow =
                    result.negativeCount.ptr<unsigned char>(v);
                unsigned char *consistentCountRow =
                    result.consistentCount.ptr<unsigned char>(v);

                for(int u=0; u<currentDepthMeters.cols; ++u)
                {
                    if(validRow[u]==0)
                        continue;

                    ++comparisonCountRow[u];
                    if(positiveRow[u]!=0)
                        ++positiveCountRow[u];
                    else if(negativeRow[u]!=0)
                        ++negativeCountRow[u];
                    else if(consistentRow[u]!=0)
                        ++consistentCountRow[u];
                    else
                        throw std::logic_error(
                            "valid comparison is missing a geometric evidence class");
                }
            }
        }
        warpAndEvidenceMs += perReference.warp.totalMs;
        result.perReference.push_back(perReference);
    }

    const std::chrono::steady_clock::time_point aggregateEnd =
        std::chrono::steady_clock::now();
    for(int v=0; v<currentDepthMeters.rows; ++v)
    {
        const unsigned char *comparisonRow =
            result.comparisonCount.ptr<unsigned char>(v);
        const unsigned char *positiveRow =
            result.positiveCount.ptr<unsigned char>(v);
        const unsigned char *negativeRow =
            result.negativeCount.ptr<unsigned char>(v);
        const unsigned char *consistentRow =
            result.consistentCount.ptr<unsigned char>(v);
        for(int u=0; u<currentDepthMeters.cols; ++u)
        {
            const std::size_t comparisons = comparisonRow[u];
            const std::size_t positives = positiveRow[u];
            const std::size_t negatives = negativeRow[u];
            const std::size_t consistent = consistentRow[u];
            if(positives+negatives+consistent!=comparisons)
            {
                throw std::logic_error(
                    "multi-reference evidence counts do not partition comparisons");
            }
            if(comparisons>0)
                ++result.stats.pixelsWithComparison;
            if(positives>0)
                ++result.stats.pixelsWithPositiveEvidence;
            result.stats.totalComparisons += comparisons;
            result.stats.totalPositiveVotes += positives;
            result.stats.totalNegativeVotes += negatives;
            result.stats.totalConsistentVotes += consistent;
        }
    }

    const std::chrono::steady_clock::time_point totalEnd =
        std::chrono::steady_clock::now();
    result.stats.warpAndEvidenceMs = warpAndEvidenceMs;
    result.stats.aggregateMs =
        std::chrono::duration<double,std::milli>(
            aggregateEnd-aggregateStart).count()-warpAndEvidenceMs;
    result.stats.aggregateMs = std::max(0.0,result.stats.aggregateMs);
    result.stats.totalMs =
        std::chrono::duration<double,std::milli>(
            totalEnd-totalStart).count();
    return result;
}

cv::Mat GeometricDynamicDetector::DownsampleDepthBoundaryAware(
    const cv::Mat &depthMeters,
    const int scale,
    const float relativeThreshold,
    const float absoluteThresholdMeters)
{
    ValidateDepth(depthMeters,"pyramid depth");
    if(scale<2)
        throw std::invalid_argument("depth pyramid scale must be at least 2");
    if(!std::isfinite(relativeThreshold) ||
       relativeThreshold<0.0f ||
       !std::isfinite(absoluteThresholdMeters) ||
       absoluteThresholdMeters<=0.0f)
    {
        throw std::invalid_argument(
            "depth pyramid thresholds must be finite and non-negative");
    }

    const int outputRows =
        (depthMeters.rows+scale-1)/scale;
    const int outputCols =
        (depthMeters.cols+scale-1)/scale;
    cv::Mat output =
        cv::Mat::zeros(outputRows,outputCols,CV_32FC1);

    for(int outputV=0; outputV<outputRows; ++outputV)
    {
        const int anchorV = outputV*scale;
        float *outputRow = output.ptr<float>(outputV);
        for(int outputU=0; outputU<outputCols; ++outputU)
        {
            const int anchorU = outputU*scale;
            const float anchor =
                depthMeters.at<float>(anchorV,anchorU);
            if(!IsValidDepth(anchor))
                continue;

            const float threshold =
                std::max(relativeThreshold*anchor,
                         absoluteThresholdMeters);
            double sum = 0.0;
            std::size_t count = 0;
            const int endV =
                std::min(anchorV+scale,depthMeters.rows);
            const int endU =
                std::min(anchorU+scale,depthMeters.cols);
            for(int v=anchorV; v<endV; ++v)
            {
                const float *depthRow =
                    depthMeters.ptr<float>(v);
                for(int u=anchorU; u<endU; ++u)
                {
                    const float depth = depthRow[u];
                    if(IsValidDepth(depth) &&
                       std::fabs(depth-anchor)<=threshold)
                    {
                        sum += depth;
                        ++count;
                    }
                }
            }
            if(count>0)
            {
                outputRow[outputU] =
                    static_cast<float>(
                        sum/static_cast<double>(count));
            }
        }
    }
    return output;
}

cv::Mat GeometricDynamicDetector::ScaleCameraMatrix(
    const cv::Mat &K,
    const int scale)
{
    ValidateCameraMatrix(K);
    if(scale<2)
        throw std::invalid_argument("camera pyramid scale must be at least 2");

    cv::Mat scaled = AsFloatMatrix(K).clone();
    scaled.at<float>(0,0) /= static_cast<float>(scale);
    scaled.at<float>(1,1) /= static_cast<float>(scale);
    scaled.at<float>(0,2) /= static_cast<float>(scale);
    scaled.at<float>(1,2) /= static_cast<float>(scale);
    return scaled;
}

cv::Mat GeometricDynamicDetector::DownsampleMaskAny(
    const cv::Mat &mask,
    const int scale)
{
    if(mask.empty() || mask.type()!=CV_8UC1)
    {
        throw std::invalid_argument(
            "mask pyramid requires a non-empty CV_8UC1 mask");
    }
    if(scale<2)
        throw std::invalid_argument("mask pyramid scale must be at least 2");

    const int outputRows = (mask.rows+scale-1)/scale;
    const int outputCols = (mask.cols+scale-1)/scale;
    cv::Mat output = cv::Mat::zeros(
        outputRows,outputCols,CV_8UC1);
    for(int outputV=0; outputV<outputRows; ++outputV)
    {
        unsigned char *outputRow =
            output.ptr<unsigned char>(outputV);
        const int beginV = outputV*scale;
        const int endV = std::min(beginV+scale,mask.rows);
        for(int outputU=0; outputU<outputCols; ++outputU)
        {
            const int beginU = outputU*scale;
            const int endU = std::min(beginU+scale,mask.cols);
            bool any = false;
            for(int v=beginV; v<endV && !any; ++v)
            {
                const unsigned char *maskRow =
                    mask.ptr<unsigned char>(v);
                for(int u=beginU; u<endU; ++u)
                {
                    if(maskRow[u]!=0)
                    {
                        any = true;
                        break;
                    }
                }
            }
            if(any)
                outputRow[outputU] = 255;
        }
    }
    return output;
}

GeometricMultiReferenceResult
GeometricDynamicDetector::ComputePyramidMultiReferenceEvidence(
    const std::vector<GeometricReferenceFrame> &references,
    const cv::Mat &currentDepthMeters,
    const cv::Mat &TcwCurrent,
    const cv::Mat &K,
    const float residualThresholdMeters,
    const int scale,
    const float relativeThreshold,
    const float absoluteThresholdMeters)
{
    const std::chrono::steady_clock::time_point totalStart =
        std::chrono::steady_clock::now();

    ValidateDepth(currentDepthMeters,"current pyramid depth");
    ValidatePose(TcwCurrent,"current pyramid Tcw");
    ValidateCameraMatrix(K);
    if(references.empty())
    {
        throw std::invalid_argument(
            "pyramid evidence requires at least one reference");
    }

    const std::chrono::steady_clock::time_point preprocessStart =
        std::chrono::steady_clock::now();
    const cv::Mat currentPyramid =
        DownsampleDepthBoundaryAware(
            currentDepthMeters,scale,relativeThreshold,
            absoluteThresholdMeters);
    const cv::Mat pyramidK =
        ScaleCameraMatrix(K,scale);

    std::vector<GeometricReferenceFrame> pyramidReferences;
    pyramidReferences.reserve(references.size());
    for(std::size_t index=0; index<references.size(); ++index)
    {
        const GeometricReferenceFrame &reference =
            references[index];
        ValidateDepth(
            reference.depthMeters,
            "full-resolution pyramid reference depth");
        if(reference.depthMeters.size()!=
           currentDepthMeters.size())
        {
            throw std::invalid_argument(
                "pyramid reference and current depth sizes differ");
        }

        GeometricReferenceFrame pyramidReference = reference;
        if(reference.pyramidDepthMeters.empty())
        {
            pyramidReference.depthMeters =
                DownsampleDepthBoundaryAware(
                    reference.depthMeters,scale,
                    relativeThreshold,
                    absoluteThresholdMeters);
        }
        else
        {
            ValidateDepth(
                reference.pyramidDepthMeters,
                "cached pyramid reference depth");
            if(reference.pyramidDepthMeters.size()!=
               currentPyramid.size())
            {
                throw std::invalid_argument(
                    "cached pyramid reference has the wrong size");
            }
            pyramidReference.depthMeters =
                reference.pyramidDepthMeters;
        }
        pyramidReferences.push_back(pyramidReference);
    }
    const std::chrono::steady_clock::time_point preprocessEnd =
        std::chrono::steady_clock::now();

    const GeometricMultiReferenceResult pyramid =
        ComputeMultiReferenceEvidence(
            pyramidReferences,currentPyramid,TcwCurrent,
            pyramidK,residualThresholdMeters,
            GeometricReferenceSamplingPolicy::Dense);

    const std::chrono::steady_clock::time_point expandStart =
        std::chrono::steady_clock::now();
    GeometricMultiReferenceResult result;
    cv::resize(
        pyramid.comparisonCount,result.comparisonCount,
        currentDepthMeters.size(),0.0,0.0,cv::INTER_NEAREST);
    cv::resize(
        pyramid.positiveCount,result.positiveCount,
        currentDepthMeters.size(),0.0,0.0,cv::INTER_NEAREST);
    cv::resize(
        pyramid.negativeCount,result.negativeCount,
        currentDepthMeters.size(),0.0,0.0,cv::INTER_NEAREST);
    cv::resize(
        pyramid.consistentCount,result.consistentCount,
        currentDepthMeters.size(),0.0,0.0,cv::INTER_NEAREST);
    result.nativeDepthMeters = currentPyramid;
    result.nativeComparisonCount = pyramid.comparisonCount;
    result.nativePositiveCount = pyramid.positiveCount;
    result.nativeNegativeCount = pyramid.negativeCount;
    result.nativeConsistentCount = pyramid.consistentCount;
    result.nativeScale = scale;
    result.perReference = pyramid.perReference;
    result.stats.referenceCount = references.size();

    for(int v=0; v<result.comparisonCount.rows; ++v)
    {
        const unsigned char *comparisonRow =
            result.comparisonCount.ptr<unsigned char>(v);
        const unsigned char *positiveRow =
            result.positiveCount.ptr<unsigned char>(v);
        const unsigned char *negativeRow =
            result.negativeCount.ptr<unsigned char>(v);
        const unsigned char *consistentRow =
            result.consistentCount.ptr<unsigned char>(v);
        for(int u=0; u<result.comparisonCount.cols; ++u)
        {
            const std::size_t comparisons = comparisonRow[u];
            const std::size_t positives = positiveRow[u];
            const std::size_t negatives = negativeRow[u];
            const std::size_t consistent = consistentRow[u];
            if(positives+negatives+consistent!=comparisons)
            {
                throw std::logic_error(
                    "expanded pyramid evidence violates vote conservation");
            }
            if(comparisons>0)
                ++result.stats.pixelsWithComparison;
            if(positives>0)
                ++result.stats.pixelsWithPositiveEvidence;
            result.stats.totalComparisons += comparisons;
            result.stats.totalPositiveVotes += positives;
            result.stats.totalNegativeVotes += negatives;
            result.stats.totalConsistentVotes += consistent;
        }
    }
    const std::chrono::steady_clock::time_point expandEnd =
        std::chrono::steady_clock::now();

    result.stats.warpAndEvidenceMs =
        pyramid.stats.warpAndEvidenceMs;
    result.stats.aggregateMs = pyramid.stats.aggregateMs;
    result.stats.preprocessMs =
        std::chrono::duration<double,std::milli>(
            preprocessEnd-preprocessStart).count();
    result.stats.expandMs =
        std::chrono::duration<double,std::milli>(
            expandEnd-expandStart).count();
    result.stats.totalMs =
        std::chrono::duration<double,std::milli>(
            expandEnd-totalStart).count();
    return result;
}

std::vector<GeometricFeatureEvidenceSample>
GeometricDynamicDetector::SampleMultiReferenceEvidenceAtFeatures(
    const GeometricMultiReferenceResult &evidence,
    const std::vector<cv::Point2f> &featurePixels)
{
    const bool hasNativeEvidence =
        !evidence.nativeComparisonCount.empty() ||
        !evidence.nativePositiveCount.empty() ||
        !evidence.nativeNegativeCount.empty() ||
        !evidence.nativeConsistentCount.empty();
    const cv::Mat *comparison = &evidence.comparisonCount;
    const cv::Mat *positive = &evidence.positiveCount;
    const cv::Mat *negative = &evidence.negativeCount;
    const cv::Mat *consistent = &evidence.consistentCount;
    int scale = 1;

    if(hasNativeEvidence)
    {
        if(evidence.nativeComparisonCount.empty() ||
           evidence.nativePositiveCount.empty() ||
           evidence.nativeNegativeCount.empty() ||
           evidence.nativeConsistentCount.empty() ||
           evidence.nativeScale<1)
        {
            throw std::invalid_argument(
                "native feature evidence must provide four count images "
                "and a positive scale");
        }
        comparison = &evidence.nativeComparisonCount;
        positive = &evidence.nativePositiveCount;
        negative = &evidence.nativeNegativeCount;
        consistent = &evidence.nativeConsistentCount;
        scale = evidence.nativeScale;
    }

    if(comparison->empty() || positive->empty() ||
       negative->empty() || consistent->empty())
    {
        throw std::invalid_argument(
            "feature evidence sampling requires four count images");
    }
    if(comparison->type()!=CV_8UC1 ||
       positive->type()!=CV_8UC1 ||
       negative->type()!=CV_8UC1 ||
       consistent->type()!=CV_8UC1 ||
       positive->size()!=comparison->size() ||
       negative->size()!=comparison->size() ||
       consistent->size()!=comparison->size())
    {
        throw std::invalid_argument(
            "feature evidence count images must be same-size CV_8UC1");
    }

    std::vector<GeometricFeatureEvidenceSample> samples;
    samples.reserve(featurePixels.size());
    for(std::size_t index=0; index<featurePixels.size(); ++index)
    {
        GeometricFeatureEvidenceSample sample;
        sample.featureIndex = index;
        sample.imageU =
            static_cast<int>(std::floor(featurePixels[index].x));
        sample.imageV =
            static_cast<int>(std::floor(featurePixels[index].y));
        sample.nativeScale = scale;
        sample.nativeU = sample.imageU/scale;
        sample.nativeV = sample.imageV/scale;

        if(sample.imageU<0 || sample.imageV<0 ||
           sample.nativeU<0 || sample.nativeV<0 ||
           sample.nativeU>=comparison->cols ||
           sample.nativeV>=comparison->rows)
        {
            sample.nativeU = -1;
            sample.nativeV = -1;
            samples.push_back(sample);
            continue;
        }

        sample.comparisonCount =
            comparison->at<unsigned char>(
                sample.nativeV,sample.nativeU);
        sample.positiveCount =
            positive->at<unsigned char>(
                sample.nativeV,sample.nativeU);
        sample.negativeCount =
            negative->at<unsigned char>(
                sample.nativeV,sample.nativeU);
        sample.consistentCount =
            consistent->at<unsigned char>(
                sample.nativeV,sample.nativeU);
        if(static_cast<unsigned int>(sample.positiveCount)+
               static_cast<unsigned int>(sample.negativeCount)+
               static_cast<unsigned int>(sample.consistentCount)!=
           static_cast<unsigned int>(sample.comparisonCount))
        {
            throw std::logic_error(
                "feature evidence votes do not partition comparisons");
        }
        samples.push_back(sample);
    }
    return samples;
}

GeometricSparseFlowResult
GeometricDynamicDetector::ComputeSparseEgoFlow(
    const cv::Mat &currentGray,
    const cv::Mat &referenceGray,
    const cv::Mat &referenceDepthMeters,
    const std::vector<cv::Point2f> &currentFeaturePixels,
    const cv::Mat &TcwReference,
    const cv::Mat &TcwCurrent,
    const cv::Mat &K,
    const cv::Mat &TcwGroundTruthReference,
    const cv::Mat &TcwGroundTruthCurrent,
    const float depthBoundaryRelativeThreshold,
    const float depthBoundaryAbsoluteThresholdMeters)
{
    ValidateGrayImage(currentGray,"current gray image");
    ValidateGrayImage(referenceGray,"reference gray image");
    ValidateDepth(referenceDepthMeters,"sparse-flow reference depth");
    ValidatePose(TcwReference,"sparse-flow reference Tcw");
    ValidatePose(TcwCurrent,"sparse-flow current Tcw");
    ValidateCameraMatrix(K);
    if(currentGray.size()!=referenceGray.size() ||
       currentGray.size()!=referenceDepthMeters.size())
    {
        throw std::invalid_argument(
            "sparse-flow gray/depth images must have identical dimensions");
    }
    if(!cv::checkRange(TcwReference) ||
       !cv::checkRange(TcwCurrent) ||
       !cv::checkRange(K))
    {
        throw std::invalid_argument(
            "sparse-flow poses and K must contain finite values");
    }
    if(!std::isfinite(depthBoundaryRelativeThreshold) ||
       depthBoundaryRelativeThreshold<0.0f ||
       !std::isfinite(depthBoundaryAbsoluteThresholdMeters) ||
       depthBoundaryAbsoluteThresholdMeters<=0.0f)
    {
        throw std::invalid_argument(
            "sparse-flow depth-boundary thresholds must be finite, "
            "with a non-negative relative and positive absolute value");
    }

    const bool hasGroundTruth =
        !TcwGroundTruthReference.empty() ||
        !TcwGroundTruthCurrent.empty();
    if(hasGroundTruth)
    {
        if(TcwGroundTruthReference.empty() ||
           TcwGroundTruthCurrent.empty())
        {
            throw std::invalid_argument(
                "both sparse-flow ground-truth poses must be provided");
        }
        ValidatePose(
            TcwGroundTruthReference,
            "sparse-flow ground-truth reference Tcw");
        ValidatePose(
            TcwGroundTruthCurrent,
            "sparse-flow ground-truth current Tcw");
        if(!cv::checkRange(TcwGroundTruthReference) ||
           !cv::checkRange(TcwGroundTruthCurrent))
        {
            throw std::invalid_argument(
                "sparse-flow ground-truth poses must be finite");
        }
    }

    const std::chrono::steady_clock::time_point totalStart =
        std::chrono::steady_clock::now();
    GeometricSparseFlowResult result;
    result.stats.featureCount = currentFeaturePixels.size();
    result.samples.resize(currentFeaturePixels.size());
    if(currentFeaturePixels.empty())
    {
        result.stats.totalMs =
            std::chrono::duration<double,std::milli>(
                std::chrono::steady_clock::now()-totalStart).count();
        return result;
    }

    std::vector<cv::Point2f> referencePixels;
    std::vector<cv::Point2f> forwardBackPixels;
    std::vector<unsigned char> backwardStatus;
    std::vector<unsigned char> forwardStatus;
    std::vector<float> backwardErrors;
    std::vector<float> forwardErrors;
    const cv::Size lkWindow(21,21);
    const int maximumLevel = 3;
    const cv::TermCriteria termination(
        cv::TermCriteria::COUNT | cv::TermCriteria::EPS,
        30,0.01);
    const double minimumEigenvalueThreshold = 1e-4;

    const std::chrono::steady_clock::time_point backwardStart =
        std::chrono::steady_clock::now();
    cv::calcOpticalFlowPyrLK(
        currentGray,referenceGray,currentFeaturePixels,
        referencePixels,backwardStatus,backwardErrors,
        lkWindow,maximumLevel,termination,0,
        minimumEigenvalueThreshold);
    result.stats.backwardLkMs =
        std::chrono::duration<double,std::milli>(
            std::chrono::steady_clock::now()-backwardStart).count();

    const std::chrono::steady_clock::time_point forwardStart =
        std::chrono::steady_clock::now();
    cv::calcOpticalFlowPyrLK(
        referenceGray,currentGray,referencePixels,
        forwardBackPixels,forwardStatus,forwardErrors,
        lkWindow,maximumLevel,termination,0,
        minimumEigenvalueThreshold);
    result.stats.forwardLkMs =
        std::chrono::duration<double,std::milli>(
            std::chrono::steady_clock::now()-forwardStart).count();

    const cv::Mat camera = AsFloatMatrix(K);
    const cv::Mat slamRelativePose =
        AsFloatMatrix(TcwCurrent)*
        AsFloatMatrix(TcwReference).inv();
    cv::Mat groundTruthRelativePose;
    if(hasGroundTruth)
    {
        groundTruthRelativePose =
            AsFloatMatrix(TcwGroundTruthCurrent)*
            AsFloatMatrix(TcwGroundTruthReference).inv();
    }

    std::vector<float> slamResidualMagnitudes;
    std::vector<float> groundTruthResidualMagnitudes;
    const std::chrono::steady_clock::time_point projectionStart =
        std::chrono::steady_clock::now();
    for(std::size_t index=0;
        index<currentFeaturePixels.size(); ++index)
    {
        GeometricSparseFlowSample &sample = result.samples[index];
        sample.featureIndex = index;
        sample.currentPixel = currentFeaturePixels[index];
        sample.referencePixel = referencePixels[index];
        sample.forwardBackPixel = forwardBackPixels[index];
        sample.backwardLkValid = backwardStatus[index]!=0;
        sample.forwardLkValid = forwardStatus[index]!=0;
        sample.backwardLkError = backwardErrors[index];
        sample.forwardLkError = forwardErrors[index];
        sample.forwardBackwardErrorPixels =
            cv::norm(sample.forwardBackPixel-sample.currentPixel);
        sample.groundTruthPoseAvailable = hasGroundTruth;
        if(sample.backwardLkValid)
            ++result.stats.backwardLkValidCount;
        if(sample.forwardLkValid)
            ++result.stats.forwardLkValidCount;
        if(!sample.backwardLkValid || !sample.forwardLkValid)
        {
            sample.evidenceState =
                GeometricSparseFlowEvidenceState::LkInvalid;
            continue;
        }

        const int referenceU = cvRound(sample.referencePixel.x);
        const int referenceV = cvRound(sample.referencePixel.y);
        if(referenceU<0 || referenceV<0 ||
           referenceU>=referenceDepthMeters.cols ||
           referenceV>=referenceDepthMeters.rows)
        {
            sample.evidenceState =
                GeometricSparseFlowEvidenceState::DepthInvalid;
            continue;
        }
        sample.referenceDepthMeters =
            referenceDepthMeters.at<float>(
                referenceV,referenceU);
        sample.referenceDepthValid =
            IsValidDepth(sample.referenceDepthMeters);
        if(!sample.referenceDepthValid)
        {
            sample.evidenceState =
                GeometricSparseFlowEvidenceState::DepthInvalid;
            continue;
        }
        ++result.stats.referenceDepthValidCount;

        for(int offsetV=-2; offsetV<=2; ++offsetV)
        {
            for(int offsetU=-2; offsetU<=2; ++offsetU)
            {
                const int neighborU = referenceU+offsetU;
                const int neighborV = referenceV+offsetV;
                if(neighborU<0 ||
                   neighborU>=referenceDepthMeters.cols ||
                   neighborV<0 ||
                   neighborV>=referenceDepthMeters.rows)
                {
                    continue;
                }
                const int distance =
                    std::max(std::abs(offsetU),std::abs(offsetV));
                const float neighborDepth =
                    referenceDepthMeters.at<float>(
                        neighborV,neighborU);
                if(!IsValidDepth(neighborDepth))
                {
                    if(distance<=1)
                    {
                        sample.referenceInvalidDepthWithinOnePixel =
                            true;
                    }
                    sample.referenceInvalidDepthWithinTwoPixels =
                        true;
                    continue;
                }

                bool isBoundary = false;
                static const int du[4] = {-1,1,0,0};
                static const int dv[4] = {0,0,-1,1};
                const float discontinuityThreshold =
                    std::max(
                        depthBoundaryRelativeThreshold*neighborDepth,
                        depthBoundaryAbsoluteThresholdMeters);
                for(int direction=0; direction<4; ++direction)
                {
                    const int adjacentU =
                        neighborU+du[direction];
                    const int adjacentV =
                        neighborV+dv[direction];
                    if(adjacentU<0 ||
                       adjacentU>=referenceDepthMeters.cols ||
                       adjacentV<0 ||
                       adjacentV>=referenceDepthMeters.rows)
                    {
                        continue;
                    }
                    const float adjacentDepth =
                        referenceDepthMeters.at<float>(
                            adjacentV,adjacentU);
                    if(IsValidDepth(adjacentDepth) &&
                       std::abs(adjacentDepth-neighborDepth)>
                           discontinuityThreshold)
                    {
                        isBoundary = true;
                        break;
                    }
                }
                if(isBoundary)
                {
                    if(distance<=1)
                    {
                        sample.referenceDepthBoundaryWithinOnePixel =
                            true;
                    }
                    sample.referenceDepthBoundaryWithinTwoPixels =
                        true;
                }
            }
        }

        sample.slamProjectionValid =
            ProjectSparseFlowPoint(
                sample.referencePixel,
                sample.referenceDepthMeters,
                slamRelativePose,camera,
                sample.slamEgoPixel);
        if(sample.slamProjectionValid)
        {
            sample.slamProjectionValid =
                sample.slamEgoPixel.x>=0.0f &&
                sample.slamEgoPixel.y>=0.0f &&
                sample.slamEgoPixel.x<
                    static_cast<float>(currentGray.cols) &&
                sample.slamEgoPixel.y<
                    static_cast<float>(currentGray.rows);
        }
        if(!sample.slamProjectionValid)
        {
            sample.evidenceState =
                GeometricSparseFlowEvidenceState::ProjectionInvalid;
            continue;
        }

        sample.slamResidualPixels =
            sample.currentPixel-sample.slamEgoPixel;
        sample.slamResidualMagnitudePixels =
            cv::norm(sample.slamResidualPixels);
        sample.evidenceState =
            GeometricSparseFlowEvidenceState::Measured;
        ++result.stats.slamResidualValidCount;
        slamResidualMagnitudes.push_back(
            sample.slamResidualMagnitudePixels);

        if(hasGroundTruth)
        {
            sample.groundTruthProjectionValid =
                ProjectSparseFlowPoint(
                    sample.referencePixel,
                    sample.referenceDepthMeters,
                    groundTruthRelativePose,camera,
                    sample.groundTruthEgoPixel);
            if(sample.groundTruthProjectionValid)
            {
                sample.groundTruthProjectionValid =
                    sample.groundTruthEgoPixel.x>=0.0f &&
                    sample.groundTruthEgoPixel.y>=0.0f &&
                    sample.groundTruthEgoPixel.x<
                        static_cast<float>(currentGray.cols) &&
                    sample.groundTruthEgoPixel.y<
                        static_cast<float>(currentGray.rows);
            }
            if(sample.groundTruthProjectionValid)
            {
                sample.groundTruthResidualPixels =
                    sample.currentPixel-
                    sample.groundTruthEgoPixel;
                sample.groundTruthResidualMagnitudePixels =
                    cv::norm(
                        sample.groundTruthResidualPixels);
                ++result.stats.groundTruthResidualValidCount;
                groundTruthResidualMagnitudes.push_back(
                    sample.groundTruthResidualMagnitudePixels);
            }
        }
    }
    result.stats.depthAndProjectionMs =
        std::chrono::duration<double,std::milli>(
            std::chrono::steady_clock::now()-projectionStart).count();

    result.stats.slamResidualMedianPixels =
        Percentile(slamResidualMagnitudes,0.5);
    result.stats.slamResidualP90Pixels =
        Percentile(slamResidualMagnitudes,0.9);
    result.stats.slamResidualP95Pixels =
        Percentile(slamResidualMagnitudes,0.95);
    result.stats.groundTruthResidualMedianPixels =
        Percentile(groundTruthResidualMagnitudes,0.5);
    result.stats.groundTruthResidualP90Pixels =
        Percentile(groundTruthResidualMagnitudes,0.9);
    result.stats.groundTruthResidualP95Pixels =
        Percentile(groundTruthResidualMagnitudes,0.95);
    result.stats.totalMs =
        std::chrono::duration<double,std::milli>(
            std::chrono::steady_clock::now()-totalStart).count();
    return result;
}

GeometricSparseFlowFilterResult
GeometricDynamicDetector::SelectSparseFlowHighResidualCandidates(
    const GeometricSparseFlowResult &sparseFlow,
    const std::vector<unsigned char> &semanticDynamic,
    const float qThreshold,
    const float maximumForwardBackwardErrorPixels,
    const std::size_t minimumScaleSupport,
    const float scaleFactor,
    const float scaleFloorPixels)
{
    if(!std::isfinite(qThreshold) || qThreshold<=0.0f ||
       !std::isfinite(maximumForwardBackwardErrorPixels) ||
       maximumForwardBackwardErrorPixels<0.0f ||
       minimumScaleSupport==0 ||
       !std::isfinite(scaleFactor) || scaleFactor<=0.0f ||
       !std::isfinite(scaleFloorPixels) || scaleFloorPixels<=0.0f)
    {
        throw std::invalid_argument(
            "sparse-flow filter parameters must be finite and positive");
    }
    if(!semanticDynamic.empty() &&
       semanticDynamic.size()!=sparseFlow.samples.size())
    {
        throw std::invalid_argument(
            "sparse-flow semantic flags must be empty or match samples");
    }

    GeometricSparseFlowFilterResult result;
    result.candidateMask.assign(sparseFlow.samples.size(),0);
    std::vector<float> scaleResiduals;
    scaleResiduals.reserve(sparseFlow.samples.size());
    std::vector<unsigned char> qualityEligible(
        sparseFlow.samples.size(),0);
    for(std::size_t index=0;
        index<sparseFlow.samples.size(); ++index)
    {
        const GeometricSparseFlowSample &sample =
            sparseFlow.samples[index];
        if(sample.featureIndex!=index)
        {
            throw std::invalid_argument(
                "sparse-flow sample indices must be contiguous");
        }
        const bool eligible =
            sample.evidenceState==
                GeometricSparseFlowEvidenceState::Measured &&
            std::isfinite(sample.forwardBackwardErrorPixels) &&
            sample.forwardBackwardErrorPixels<=
                maximumForwardBackwardErrorPixels &&
            std::isfinite(sample.slamResidualMagnitudePixels) &&
            sample.slamResidualMagnitudePixels>=0.0f;
        if(!eligible)
            continue;
        qualityEligible[index] = 1;
        ++result.qualityEligibleFeatureCount;
        scaleResiduals.push_back(
            sample.slamResidualMagnitudePixels);
    }

    result.scaleSupport = scaleResiduals.size();
    if(result.scaleSupport<minimumScaleSupport)
        return result;

    result.frameScalePixels = std::max(
        scaleFloorPixels,
        scaleFactor*static_cast<float>(
            Percentile(scaleResiduals,0.5)));
    result.scaleValid =
        std::isfinite(result.frameScalePixels) &&
        result.frameScalePixels>0.0f;
    if(!result.scaleValid)
        return result;

    for(std::size_t index=0;
        index<sparseFlow.samples.size(); ++index)
    {
        if(!qualityEligible[index] ||
           (!semanticDynamic.empty() &&
            semanticDynamic[index]!=0))
        {
            continue;
        }
        const float q =
            sparseFlow.samples[index].
                slamResidualMagnitudePixels/
            result.frameScalePixels;
        if(std::isfinite(q) && q>=qThreshold)
        {
            result.candidateMask[index] = 1;
            ++result.candidateFeatureCount;
        }
    }
    return result;
}

const char *GeometricDynamicDetector::SparseFlowEvidenceStateName(
    const GeometricSparseFlowEvidenceState state)
{
    switch(state)
    {
    case GeometricSparseFlowEvidenceState::Measured:
        return "measured";
    case GeometricSparseFlowEvidenceState::LkInvalid:
        return "lk_invalid";
    case GeometricSparseFlowEvidenceState::DepthInvalid:
        return "depth_invalid";
    case GeometricSparseFlowEvidenceState::ProjectionInvalid:
        return "projection_invalid";
    case GeometricSparseFlowEvidenceState::DomainInvalid:
        return "domain_invalid";
    case GeometricSparseFlowEvidenceState::ReferenceUnavailable:
        return "reference_unavailable";
    }
    return "reference_unavailable";
}

GeometricRigidityResult
GeometricDynamicDetector::ComputeLocalRigidity(
    const cv::Mat &referenceDepthMeters,
    const cv::Mat &currentDepthMeters,
    const cv::Mat &K,
    const GeometricSparseFlowResult &sparseFlow,
    const std::vector<unsigned char> &semanticDynamic,
    const float maximumForwardBackwardErrorPixels,
    const float relativeDenominatorFloorMeters,
    const float axialDepthNoiseCoefficientPerMeter,
    const float uncertaintyDenominatorFloorMeters)
{
    ValidateDepth(referenceDepthMeters,"rigidity reference depth");
    ValidateDepth(currentDepthMeters,"rigidity current depth");
    if(referenceDepthMeters.size()!=currentDepthMeters.size())
    {
        throw std::invalid_argument(
            "rigidity reference/current depth sizes must match");
    }
    ValidateCameraMatrix(K);
    if(!cv::checkRange(K))
        throw std::invalid_argument("rigidity K must contain finite values");
    if(!semanticDynamic.empty() &&
       semanticDynamic.size()!=sparseFlow.samples.size())
    {
        throw std::invalid_argument(
            "rigidity semantic flags must match sparse-flow samples");
    }
    if(!std::isfinite(maximumForwardBackwardErrorPixels) ||
       maximumForwardBackwardErrorPixels<0.0f ||
       !std::isfinite(relativeDenominatorFloorMeters) ||
       relativeDenominatorFloorMeters<=0.0f ||
       !std::isfinite(axialDepthNoiseCoefficientPerMeter) ||
       axialDepthNoiseCoefficientPerMeter<=0.0f ||
       !std::isfinite(uncertaintyDenominatorFloorMeters) ||
       uncertaintyDenominatorFloorMeters<=0.0f)
    {
        throw std::invalid_argument(
            "rigidity numeric safeguards must be finite and positive");
    }

    const std::chrono::steady_clock::time_point totalStart =
        std::chrono::steady_clock::now();
    GeometricRigidityResult result;
    result.stats.axialDepthNoiseCoefficientPerMeter =
        axialDepthNoiseCoefficientPerMeter;
    result.stats.uncertaintyDenominatorFloorMeters =
        uncertaintyDenominatorFloorMeters;
    result.stats.inputFeatureCount = sparseFlow.samples.size();
    result.nodes.resize(sparseFlow.samples.size());
    const cv::Mat camera = AsFloatMatrix(K);

    std::vector<std::size_t> graphCandidates;
    graphCandidates.reserve(sparseFlow.samples.size());
    for(std::size_t index=0;
        index<sparseFlow.samples.size(); ++index)
    {
        const GeometricSparseFlowSample &flow =
            sparseFlow.samples[index];
        GeometricRigidityNodeSample &node = result.nodes[index];
        node.featureIndex = flow.featureIndex;
        node.currentPixel = flow.currentPixel;
        node.referencePixel = flow.referencePixel;
        node.forwardBackwardErrorPixels =
            flow.forwardBackwardErrorPixels;
        node.flowResidualMagnitudePixels =
            flow.slamResidualMagnitudePixels;

        if(flow.evidenceState!=
               GeometricSparseFlowEvidenceState::Measured ||
           !flow.referenceDepthValid)
        {
            node.state =
                GeometricRigidityNodeState::SparseFlowInvalid;
            continue;
        }
        ++result.stats.sparseFlowMeasuredCount;
        if(flow.forwardBackwardErrorPixels>
               maximumForwardBackwardErrorPixels)
        {
            node.state =
                GeometricRigidityNodeState::
                    ForwardBackwardRejected;
            ++result.stats.forwardBackwardRejectedCount;
            continue;
        }
        if(!semanticDynamic.empty() &&
           semanticDynamic[index]!=0)
        {
            node.state =
                GeometricRigidityNodeState::SemanticExcluded;
            ++result.stats.semanticExcludedCount;
            continue;
        }

        const int currentU = cvRound(flow.currentPixel.x);
        const int currentV = cvRound(flow.currentPixel.y);
        if(currentU<0 || currentV<0 ||
           currentU>=currentDepthMeters.cols ||
           currentV>=currentDepthMeters.rows)
        {
            node.state =
                GeometricRigidityNodeState::OutsideImage;
            ++result.stats.outsideImageCount;
            continue;
        }
        const float currentDepth =
            currentDepthMeters.at<float>(currentV,currentU);
        if(!IsValidDepth(currentDepth))
        {
            node.state =
                GeometricRigidityNodeState::
                    CurrentDepthInvalid;
            ++result.stats.currentDepthInvalidCount;
            continue;
        }

        node.referencePointMeters =
            BackProjectPoint(
                flow.referencePixel,
                flow.referenceDepthMeters,camera);
        node.currentPointMeters =
            BackProjectPoint(
                flow.currentPixel,currentDepth,camera);
        const AxialDepthUncertainty referenceUncertainty =
            ComputeAxialDepthUncertainty(
                referenceDepthMeters,flow.referencePixel,
                axialDepthNoiseCoefficientPerMeter);
        const AxialDepthUncertainty currentUncertainty =
            ComputeAxialDepthUncertainty(
                currentDepthMeters,flow.currentPixel,
                axialDepthNoiseCoefficientPerMeter);
        if(!referenceUncertainty.valid || !currentUncertainty.valid)
        {
            node.state =
                GeometricRigidityNodeState::UncertaintyInvalid;
            ++result.stats.uncertaintyInvalidCount;
            continue;
        }
        node.referenceDepthUncertaintyStdMeters =
            referenceUncertainty.standardDeviationMeters;
        node.currentDepthUncertaintyStdMeters =
            currentUncertainty.standardDeviationMeters;
        node.referenceDepthNeighborhoodValidWeight =
            referenceUncertainty.validWeight;
        node.currentDepthNeighborhoodValidWeight =
            currentUncertainty.validWeight;
        node.state = GeometricRigidityNodeState::Measured;
        graphCandidates.push_back(index);
    }

    const std::chrono::steady_clock::time_point graphStart =
        std::chrono::steady_clock::now();
    cv::Subdiv2D subdivision(
        cv::Rect(
            0,0,currentDepthMeters.cols,currentDepthMeters.rows));
    std::map<int,std::size_t> featureByVertex;
    // ORB pyramid coordinates can differ only by floating-point round-off
    // (observed around 1e-5 px). Keeping both creates nearly zero-length
    // Delaunay edges. This is a numerical de-duplication tolerance, not a
    // motion or rigidity threshold.
    const float duplicatePointTolerancePixels = 1e-3f;
    std::map<
        std::pair<int,int>,
        std::vector<std::size_t> > acceptedCoordinateCells;
    for(std::size_t candidateIndex=0;
        candidateIndex<graphCandidates.size(); ++candidateIndex)
    {
        const std::size_t sampleIndex =
            graphCandidates[candidateIndex];
        GeometricRigidityNodeSample &node =
            result.nodes[sampleIndex];
        const int cellX = static_cast<int>(
            std::floor(
                node.currentPixel.x/
                duplicatePointTolerancePixels));
        const int cellY = static_cast<int>(
            std::floor(
                node.currentPixel.y/
                duplicatePointTolerancePixels));
        bool nearDuplicate = false;
        for(int offsetY=-1;
            offsetY<=1 && !nearDuplicate; ++offsetY)
        {
            for(int offsetX=-1;
                offsetX<=1 && !nearDuplicate; ++offsetX)
            {
                const std::map<
                    std::pair<int,int>,
                    std::vector<std::size_t> >::const_iterator
                    cellIt =
                        acceptedCoordinateCells.find(
                            std::make_pair(
                                cellX+offsetX,
                                cellY+offsetY));
                if(cellIt==acceptedCoordinateCells.end())
                    continue;
                for(std::size_t nearbyIndex=0;
                    nearbyIndex<cellIt->second.size();
                    ++nearbyIndex)
                {
                    if(cv::norm(
                           node.currentPixel-
                           result.nodes[
                               cellIt->second[nearbyIndex]].
                                   currentPixel)<=
                       duplicatePointTolerancePixels)
                    {
                        nearDuplicate = true;
                        break;
                    }
                }
            }
        }
        if(nearDuplicate)
        {
            node.state =
                GeometricRigidityNodeState::
                    DuplicateImagePoint;
            ++result.stats.duplicateImagePointCount;
            continue;
        }
        int vertexId = -1;
        try
        {
            vertexId = subdivision.insert(node.currentPixel);
        }
        catch(const cv::Exception &)
        {
            node.state =
                GeometricRigidityNodeState::OutsideImage;
            ++result.stats.outsideImageCount;
            continue;
        }
        const std::pair<std::map<int,std::size_t>::iterator,bool>
            inserted =
                featureByVertex.insert(
                    std::make_pair(vertexId,sampleIndex));
        if(!inserted.second)
        {
            node.state =
                GeometricRigidityNodeState::
                    DuplicateImagePoint;
            ++result.stats.duplicateImagePointCount;
            continue;
        }
        acceptedCoordinateCells[
            std::make_pair(cellX,cellY)].push_back(
                sampleIndex);
        ++result.stats.eligibleNodeCount;
    }

    std::set<std::pair<std::size_t,std::size_t> > uniqueEdges;
    std::vector<int> leadingEdges;
    subdivision.getLeadingEdgeList(leadingEdges);
    for(std::size_t triangleIndex=0;
        triangleIndex<leadingEdges.size(); ++triangleIndex)
    {
        int edge = leadingEdges[triangleIndex];
        for(int side=0; side<3; ++side)
        {
            const int origin = subdivision.edgeOrg(edge);
            const int destination = subdivision.edgeDst(edge);
            const std::map<int,std::size_t>::const_iterator
                originIt = featureByVertex.find(origin);
            const std::map<int,std::size_t>::const_iterator
                destinationIt = featureByVertex.find(destination);
            if(originIt!=featureByVertex.end() &&
               destinationIt!=featureByVertex.end() &&
               originIt->second!=destinationIt->second)
            {
                const std::size_t first =
                    std::min(originIt->second,destinationIt->second);
                const std::size_t second =
                    std::max(originIt->second,destinationIt->second);
                uniqueEdges.insert(std::make_pair(first,second));
            }
            edge = subdivision.getEdge(
                edge,cv::Subdiv2D::NEXT_AROUND_LEFT);
        }
    }
    result.stats.graphMs =
        std::chrono::duration<double,std::milli>(
            std::chrono::steady_clock::now()-graphStart).count();

    const std::chrono::steady_clock::time_point metricStart =
        std::chrono::steady_clock::now();
    std::vector<std::vector<float> > incidentAbsolute(
        result.nodes.size());
    std::vector<std::vector<float> > incidentRelative(
        result.nodes.size());
    std::vector<std::vector<float> > incidentUncertaintyNormalized(
        result.nodes.size());
    result.edges.reserve(uniqueEdges.size());
    for(std::set<std::pair<std::size_t,std::size_t> >::
            const_iterator edgeIt=uniqueEdges.begin();
        edgeIt!=uniqueEdges.end(); ++edgeIt)
    {
        const GeometricRigidityNodeSample &first =
            result.nodes[edgeIt->first];
        const GeometricRigidityNodeSample &second =
            result.nodes[edgeIt->second];
        const float referenceDistance =
            cv::norm(
                first.referencePointMeters-
                second.referencePointMeters);
        const float currentDistance =
            cv::norm(
                first.currentPointMeters-
                second.currentPointMeters);
        if(!std::isfinite(referenceDistance) ||
           !std::isfinite(currentDistance))
        {
            continue;
        }
        GeometricRigidityEdgeSample edge;
        edge.featureIndexA = first.featureIndex;
        edge.featureIndexB = second.featureIndex;
        edge.referenceDistanceMeters = referenceDistance;
        edge.currentDistanceMeters = currentDistance;
        edge.absoluteStrainMeters =
            std::abs(currentDistance-referenceDistance);
        edge.relativeStrain =
            edge.absoluteStrainMeters/
            std::max(
                relativeDenominatorFloorMeters,
                0.5f*(currentDistance+referenceDistance));
        const float referenceDistanceVariance =
            EdgeLengthAxialVariance(
                first.referencePointMeters,
                second.referencePointMeters,
                first.referencePixel,second.referencePixel,
                first.referenceDepthUncertaintyStdMeters,
                second.referenceDepthUncertaintyStdMeters,
                camera);
        const float currentDistanceVariance =
            EdgeLengthAxialVariance(
                first.currentPointMeters,
                second.currentPointMeters,
                first.currentPixel,second.currentPixel,
                first.currentDepthUncertaintyStdMeters,
                second.currentDepthUncertaintyStdMeters,
                camera);
        const float deltaLengthVariance =
            referenceDistanceVariance+currentDistanceVariance;
        if(!std::isfinite(deltaLengthVariance) ||
           deltaLengthVariance<0.0f)
        {
            continue;
        }
        edge.deltaLengthUncertaintyStdMeters =
            std::sqrt(deltaLengthVariance);
        const float uncertaintyDenominator =
            std::max(
                uncertaintyDenominatorFloorMeters,
                edge.deltaLengthUncertaintyStdMeters);
        if(edge.deltaLengthUncertaintyStdMeters<
               uncertaintyDenominatorFloorMeters)
        {
            ++result.stats.uncertaintyFloorUseCount;
        }
        edge.uncertaintyNormalizedStrain =
            edge.absoluteStrainMeters/uncertaintyDenominator;
        edge.flowResidualMagnitudePixelsA =
            first.flowResidualMagnitudePixels;
        edge.flowResidualMagnitudePixelsB =
            second.flowResidualMagnitudePixels;
        edge.forwardBackwardErrorPixelsA =
            first.forwardBackwardErrorPixels;
        edge.forwardBackwardErrorPixelsB =
            second.forwardBackwardErrorPixels;
        result.edges.push_back(edge);
        incidentAbsolute[edgeIt->first].push_back(
            edge.absoluteStrainMeters);
        incidentAbsolute[edgeIt->second].push_back(
            edge.absoluteStrainMeters);
        incidentRelative[edgeIt->first].push_back(
            edge.relativeStrain);
        incidentRelative[edgeIt->second].push_back(
            edge.relativeStrain);
        incidentUncertaintyNormalized[edgeIt->first].push_back(
            edge.uncertaintyNormalizedStrain);
        incidentUncertaintyNormalized[edgeIt->second].push_back(
            edge.uncertaintyNormalizedStrain);
        ++result.stats.uncertaintyNormalizedEdgeCount;
    }
    result.stats.validEdgeCount = result.edges.size();
    for(std::size_t index=0; index<result.nodes.size(); ++index)
    {
        GeometricRigidityNodeSample &node = result.nodes[index];
        if(node.state!=GeometricRigidityNodeState::Measured)
            continue;
        node.validNeighborCount = incidentAbsolute[index].size();
        if(node.validNeighborCount==0)
        {
            node.state =
                GeometricRigidityNodeState::NoGraphEdge;
            continue;
        }
        ++result.stats.nodeWithEdgeCount;
        node.incidentAbsoluteStrainMedianMeters =
            static_cast<float>(
                Percentile(incidentAbsolute[index],0.5));
        node.incidentAbsoluteStrainP90Meters =
            static_cast<float>(
                Percentile(incidentAbsolute[index],0.9));
        node.incidentRelativeStrainMedian =
            static_cast<float>(
                Percentile(incidentRelative[index],0.5));
        node.incidentRelativeStrainP90 =
            static_cast<float>(
                Percentile(incidentRelative[index],0.9));
        node.incidentUncertaintyNormalizedStrainMedian =
            static_cast<float>(
                Percentile(
                    incidentUncertaintyNormalized[index],0.5));
        node.incidentUncertaintyNormalizedStrainP90 =
            static_cast<float>(
                Percentile(
                    incidentUncertaintyNormalized[index],0.9));
    }
    result.stats.metricMs =
        std::chrono::duration<double,std::milli>(
            std::chrono::steady_clock::now()-metricStart).count();
    result.stats.totalMs =
        std::chrono::duration<double,std::milli>(
            std::chrono::steady_clock::now()-totalStart).count();
    return result;
}

const char *GeometricDynamicDetector::RigidityNodeStateName(
    const GeometricRigidityNodeState state)
{
    switch(state)
    {
    case GeometricRigidityNodeState::Measured:
        return "measured";
    case GeometricRigidityNodeState::SparseFlowInvalid:
        return "sparse_flow_invalid";
    case GeometricRigidityNodeState::ForwardBackwardRejected:
        return "forward_backward_rejected";
    case GeometricRigidityNodeState::SemanticExcluded:
        return "semantic_excluded";
    case GeometricRigidityNodeState::CurrentDepthInvalid:
        return "current_depth_invalid";
    case GeometricRigidityNodeState::UncertaintyInvalid:
        return "uncertainty_invalid";
    case GeometricRigidityNodeState::OutsideImage:
        return "outside_image";
    case GeometricRigidityNodeState::DuplicateImagePoint:
        return "duplicate_image_point";
    case GeometricRigidityNodeState::NoGraphEdge:
        return "no_graph_edge";
    }
    return "sparse_flow_invalid";
}

GeometricReferenceSelectionResult
GeometricDynamicDetector::SelectCachedReferences(
    const std::vector<GeometricReferenceFrame> &cachedReferences,
    const std::vector<long unsigned int> &orderedCandidateFrameIds,
    const std::size_t maximumReferences)
{
    if(maximumReferences==0)
    {
        throw std::invalid_argument(
            "reference selection requires a positive maximum");
    }

    GeometricReferenceSelectionResult result;
    std::vector<long unsigned int> uniqueCandidateIds;
    uniqueCandidateIds.reserve(orderedCandidateFrameIds.size());
    for(std::size_t candidateIndex=0;
        candidateIndex<orderedCandidateFrameIds.size(); ++candidateIndex)
    {
        const long unsigned int candidateId =
            orderedCandidateFrameIds[candidateIndex];
        if(std::find(uniqueCandidateIds.begin(),uniqueCandidateIds.end(),
                     candidateId)!=uniqueCandidateIds.end())
        {
            continue;
        }
        uniqueCandidateIds.push_back(candidateId);
    }
    result.stats.candidateCount = uniqueCandidateIds.size();

    for(std::size_t candidateIndex=0;
        candidateIndex<uniqueCandidateIds.size(); ++candidateIndex)
    {
        const long unsigned int candidateId =
            uniqueCandidateIds[candidateIndex];
        const GeometricReferenceFrame *matchedReference = NULL;
        for(std::size_t cacheIndex=0;
            cacheIndex<cachedReferences.size(); ++cacheIndex)
        {
            if(cachedReferences[cacheIndex].frameId==candidateId)
            {
                matchedReference = &cachedReferences[cacheIndex];
                break;
            }
        }
        if(!matchedReference)
            continue;

        ++result.stats.cachedReferenceMatchCount;
        if(result.references.size()<maximumReferences)
            result.references.push_back(*matchedReference);
    }
    result.stats.selectedReferenceCount = result.references.size();
    return result;
}

GeometricRegionPartitionResult
GeometricDynamicDetector::PartitionDepthByDiscontinuity(
    const cv::Mat &depthMeters,
    const float relativeThreshold,
    const float absoluteThresholdMeters,
    const std::size_t smallRegionMaximumPixels)
{
    ValidateDepth(depthMeters,"region-partition depth");
    if(!std::isfinite(relativeThreshold) ||
       relativeThreshold<0.0f)
    {
        throw std::invalid_argument(
            "region relative threshold must be finite and non-negative");
    }
    if(!std::isfinite(absoluteThresholdMeters) ||
       absoluteThresholdMeters<=0.0f)
    {
        throw std::invalid_argument(
            "region absolute threshold must be finite and positive");
    }

    const std::chrono::steady_clock::time_point totalStart =
        std::chrono::steady_clock::now();
    GeometricRegionPartitionResult result;
    result.boundaryMask = cv::Mat::zeros(
        depthMeters.size(),CV_8UC1);
    result.labels = cv::Mat(
        depthMeters.size(),CV_32SC1,cv::Scalar(-1));

    static const int du[4] = {-1,1,0,0};
    static const int dv[4] = {0,0,-1,1};
    for(int v=0; v<depthMeters.rows; ++v)
    {
        const float *depthRow = depthMeters.ptr<float>(v);
        unsigned char *boundaryRow =
            result.boundaryMask.ptr<unsigned char>(v);
        for(int u=0; u<depthMeters.cols; ++u)
        {
            const float centerDepth = depthRow[u];
            if(!IsValidDepth(centerDepth))
                continue;
            ++result.stats.validDepthPixels;

            const float discontinuityThreshold =
                std::max(relativeThreshold*centerDepth,
                         absoluteThresholdMeters);
            for(int direction=0; direction<4; ++direction)
            {
                const int neighborU = u+du[direction];
                const int neighborV = v+dv[direction];
                if(neighborU<0 || neighborU>=depthMeters.cols ||
                   neighborV<0 || neighborV>=depthMeters.rows)
                {
                    continue;
                }
                const float neighborDepth =
                    depthMeters.at<float>(neighborV,neighborU);
                if(IsValidDepth(neighborDepth) &&
                   std::abs(neighborDepth-centerDepth)>
                       discontinuityThreshold)
                {
                    boundaryRow[u] = 255;
                    ++result.stats.boundaryPixels;
                    break;
                }
            }
        }
    }

    std::deque<cv::Point2i> frontier;
    int nextLabel = 0;
    for(int v=0; v<depthMeters.rows; ++v)
    {
        int *labelRow = result.labels.ptr<int>(v);
        const float *depthRow = depthMeters.ptr<float>(v);
        const unsigned char *boundaryRow =
            result.boundaryMask.ptr<unsigned char>(v);
        for(int u=0; u<depthMeters.cols; ++u)
        {
            if(!IsValidDepth(depthRow[u]))
                continue;
            if(boundaryRow[u]!=0)
            {
                labelRow[u] = -2;
                continue;
            }
            if(labelRow[u]>=0)
                continue;

            std::size_t regionPixels = 0;
            labelRow[u] = nextLabel;
            frontier.push_back(cv::Point2i(u,v));
            while(!frontier.empty())
            {
                const cv::Point2i pixel = frontier.front();
                frontier.pop_front();
                ++regionPixels;

                for(int direction=0; direction<4; ++direction)
                {
                    const int neighborU = pixel.x+du[direction];
                    const int neighborV = pixel.y+dv[direction];
                    if(neighborU<0 ||
                       neighborU>=depthMeters.cols ||
                       neighborV<0 ||
                       neighborV>=depthMeters.rows)
                    {
                        continue;
                    }
                    if(result.labels.at<int>(
                           neighborV,neighborU)!=-1 ||
                       result.boundaryMask.at<unsigned char>(
                           neighborV,neighborU)!=0 ||
                       !IsValidDepth(depthMeters.at<float>(
                           neighborV,neighborU)))
                    {
                        continue;
                    }
                    result.labels.at<int>(
                        neighborV,neighborU) = nextLabel;
                    frontier.push_back(
                        cv::Point2i(neighborU,neighborV));
                }
            }

            result.regionSizes.push_back(regionPixels);
            result.stats.assignedRegionPixels += regionPixels;
            if(regionPixels==1)
                ++result.stats.singletonRegionCount;
            if(regionPixels<=smallRegionMaximumPixels)
                ++result.stats.smallRegionCount;
            ++nextLabel;
        }
    }

    result.stats.regionCount = result.regionSizes.size();
    std::vector<std::size_t> sortedSizes = result.regionSizes;
    std::sort(sortedSizes.begin(),sortedSizes.end(),
              std::greater<std::size_t>());
    if(!sortedSizes.empty())
        result.stats.largestRegionPixels = sortedSizes.front();
    const std::size_t topCount =
        std::min<std::size_t>(5,sortedSizes.size());
    for(std::size_t index=0; index<topCount; ++index)
        result.stats.topFiveRegionPixels += sortedSizes[index];

    if(result.stats.validDepthPixels>0)
    {
        const double valid =
            static_cast<double>(result.stats.validDepthPixels);
        result.stats.boundaryValidRatio =
            static_cast<double>(result.stats.boundaryPixels)/valid;
        result.stats.assignedValidRatio =
            static_cast<double>(
                result.stats.assignedRegionPixels)/valid;
        result.stats.largestRegionValidRatio =
            static_cast<double>(
                result.stats.largestRegionPixels)/valid;
        result.stats.topFiveRegionValidRatio =
            static_cast<double>(
                result.stats.topFiveRegionPixels)/valid;
    }
    result.stats.totalMs =
        std::chrono::duration<double,std::milli>(
            std::chrono::steady_clock::now()-totalStart).count();
    return result;
}

GeometricRegionEvidenceAggregationResult
GeometricDynamicDetector::AggregateMultiReferenceEvidenceByRegion(
    const GeometricRegionPartitionResult &partition,
    const GeometricMultiReferenceResult &evidence,
    const cv::Mat &semanticProxyMask,
    const bool collectRiskDiagnostics)
{
    if(partition.labels.empty() ||
       partition.labels.type()!=CV_32SC1)
    {
        throw std::invalid_argument(
            "region evidence aggregation requires CV_32SC1 labels");
    }
    const cv::Size size = partition.labels.size();
    const cv::Mat *countImages[4] = {
        &evidence.comparisonCount,
        &evidence.positiveCount,
        &evidence.negativeCount,
        &evidence.consistentCount
    };
    for(int imageIndex=0; imageIndex<4; ++imageIndex)
    {
        if(countImages[imageIndex]->empty() ||
           countImages[imageIndex]->type()!=CV_8UC1 ||
           countImages[imageIndex]->size()!=size)
        {
            throw std::invalid_argument(
                "region evidence aggregation requires same-size CV_8UC1 count images");
        }
    }
    if(!semanticProxyMask.empty() &&
       (semanticProxyMask.type()!=CV_8UC1 ||
        semanticProxyMask.size()!=size))
    {
        throw std::invalid_argument(
            "semantic proxy must be empty or a same-size CV_8UC1 mask");
    }

    const std::chrono::steady_clock::time_point totalStart =
        std::chrono::steady_clock::now();
    GeometricRegionEvidenceAggregationResult result;
    result.regions.resize(partition.regionSizes.size());
    for(std::size_t regionIndex=0;
        regionIndex<result.regions.size(); ++regionIndex)
    {
        result.regions[regionIndex].regionLabel =
            static_cast<int>(regionIndex);
    }

    cv::Mat boundaryWithinOne;
    cv::Mat boundaryWithinTwo;
    cv::Mat invalidWithinOne;
    cv::Mat invalidWithinTwo;
    if(collectRiskDiagnostics)
    {
        cv::Mat boundaryMask;
        cv::compare(partition.labels,-2,boundaryMask,cv::CMP_EQ);
        cv::Mat invalidMask;
        cv::compare(partition.labels,-1,invalidMask,cv::CMP_EQ);
        cv::dilate(
            boundaryMask,boundaryWithinOne,
            cv::getStructuringElement(
                cv::MORPH_RECT,cv::Size(3,3)));
        cv::dilate(
            boundaryMask,boundaryWithinTwo,
            cv::getStructuringElement(
                cv::MORPH_RECT,cv::Size(5,5)));
        cv::dilate(
            invalidMask,invalidWithinOne,
            cv::getStructuringElement(
                cv::MORPH_RECT,cv::Size(3,3)));
        cv::dilate(
            invalidMask,invalidWithinTwo,
            cv::getStructuringElement(
                cv::MORPH_RECT,cv::Size(5,5)));
    }

    const bool hasSemanticProxy = !semanticProxyMask.empty();
    for(int v=0; v<size.height; ++v)
    {
        const int *labelRow = partition.labels.ptr<int>(v);
        const unsigned char *comparisonRow =
            evidence.comparisonCount.ptr<unsigned char>(v);
        const unsigned char *positiveRow =
            evidence.positiveCount.ptr<unsigned char>(v);
        const unsigned char *negativeRow =
            evidence.negativeCount.ptr<unsigned char>(v);
        const unsigned char *consistentRow =
            evidence.consistentCount.ptr<unsigned char>(v);
        const unsigned char *semanticRow =
            hasSemanticProxy
                ? semanticProxyMask.ptr<unsigned char>(v)
                : static_cast<const unsigned char*>(NULL);
        const unsigned char *boundaryOneRow =
            collectRiskDiagnostics
                ? boundaryWithinOne.ptr<unsigned char>(v) : NULL;
        const unsigned char *boundaryTwoRow =
            collectRiskDiagnostics
                ? boundaryWithinTwo.ptr<unsigned char>(v) : NULL;
        const unsigned char *invalidOneRow =
            collectRiskDiagnostics
                ? invalidWithinOne.ptr<unsigned char>(v) : NULL;
        const unsigned char *invalidTwoRow =
            collectRiskDiagnostics
                ? invalidWithinTwo.ptr<unsigned char>(v) : NULL;
        for(int u=0; u<size.width; ++u)
        {
            const int label = labelRow[u];
            if(label<0)
                continue;
            if(static_cast<std::size_t>(label)>=
               result.regions.size())
            {
                throw std::logic_error(
                    "region label is outside regionSizes");
            }

            const std::size_t comparisons = comparisonRow[u];
            const std::size_t positives = positiveRow[u];
            const std::size_t negatives = negativeRow[u];
            const std::size_t consistent = consistentRow[u];
            if(positives+negatives+consistent!=comparisons)
            {
                throw std::logic_error(
                    "region evidence votes do not partition comparisons");
            }

            GeometricRegionEvidenceStats &region =
                result.regions[static_cast<std::size_t>(label)];
            ++region.regionPixels;
            ++result.stats.regionPixels;
            GeometricRegionRiskBandStats *riskBands[4] = {
                boundaryOneRow && boundaryOneRow[u]!=0
                    ? &region.boundaryWithinOnePixel : NULL,
                boundaryTwoRow && boundaryTwoRow[u]!=0
                    ? &region.boundaryWithinTwoPixels : NULL,
                invalidOneRow && invalidOneRow[u]!=0
                    ? &region.invalidWithinOnePixel : NULL,
                invalidTwoRow && invalidTwoRow[u]!=0
                    ? &region.invalidWithinTwoPixels : NULL
            };
            for(int riskIndex=0;
                collectRiskDiagnostics && riskIndex<4;
                ++riskIndex)
            {
                if(riskBands[riskIndex])
                    ++riskBands[riskIndex]->regionPixels;
            }
            const bool isSemanticProxy =
                semanticRow && semanticRow[u]!=0;
            if(isSemanticProxy)
                ++region.semanticProxyPixels;
            if(comparisons==0)
                continue;

            ++region.comparisonPixels;
            ++result.stats.comparisonPixels;
            if(collectRiskDiagnostics)
            {
                if(comparisons==1)
                    ++region.singleReferenceComparisonPixels;
                else
                    ++region.multiReferenceComparisonPixels;
            }
            if(isSemanticProxy)
                ++region.semanticComparisonPixels;
            region.comparisonVotes += comparisons;
            region.positiveVotes += positives;
            region.negativeVotes += negatives;
            region.consistentVotes += consistent;
            result.stats.comparisonVotes += comparisons;
            for(int riskIndex=0;
                collectRiskDiagnostics && riskIndex<4;
                ++riskIndex)
            {
                if(!riskBands[riskIndex])
                    continue;
                ++riskBands[riskIndex]->comparisonPixels;
                riskBands[riskIndex]->comparisonVotes += comparisons;
                riskBands[riskIndex]->positiveVotes += positives;
            }
            if(positives>0)
            {
                ++region.positivePresencePixels;
                if(collectRiskDiagnostics && comparisons==1)
                {
                    ++region.singleReferencePositivePresencePixels;
                }
                else if(collectRiskDiagnostics)
                {
                    ++region.multiReferencePositivePresencePixels;
                }
                if(collectRiskDiagnostics && positives==comparisons)
                    ++region.unanimousPositivePixels;
                for(int riskIndex=0;
                    collectRiskDiagnostics && riskIndex<4;
                    ++riskIndex)
                {
                    if(riskBands[riskIndex])
                    {
                        ++riskBands[riskIndex]->
                            positivePresencePixels;
                    }
                }
                if(isSemanticProxy)
                    ++region.semanticPositivePresencePixels;
            }
            if(negatives>0)
            {
                ++region.negativePresencePixels;
                if(isSemanticProxy)
                    ++region.semanticNegativePresencePixels;
            }
            if(consistent>0)
            {
                ++region.consistentPresencePixels;
                if(isSemanticProxy)
                    ++region.semanticConsistentPresencePixels;
            }
        }
    }

    result.stats.regionCount = result.regions.size();
    for(std::size_t regionIndex=0;
        regionIndex<result.regions.size(); ++regionIndex)
    {
        GeometricRegionEvidenceStats &region =
            result.regions[regionIndex];
        if(region.regionPixels!=partition.regionSizes[regionIndex])
        {
            throw std::logic_error(
                "region evidence pixels disagree with partition sizes");
        }
        if(region.comparisonPixels>0)
            ++result.stats.regionsWithComparison;
        if(region.positivePresencePixels>0)
            ++result.stats.regionsWithPositiveEvidence;
        if(region.regionPixels>0)
        {
            const double pixels =
                static_cast<double>(region.regionPixels);
            region.semanticProxyRegionRatio =
                static_cast<double>(
                    region.semanticProxyPixels)/pixels;
            region.comparisonCoverage =
                static_cast<double>(
                    region.comparisonPixels)/pixels;
        }
        if(region.semanticProxyPixels>0)
        {
            region.semanticComparisonCoverage =
                static_cast<double>(
                    region.semanticComparisonPixels)/
                static_cast<double>(
                    region.semanticProxyPixels);
        }
        if(region.semanticComparisonPixels>0)
        {
            region.semanticPositiveComparedPixelRatio =
                static_cast<double>(
                    region.semanticPositivePresencePixels)/
                static_cast<double>(
                    region.semanticComparisonPixels);
        }
        if(region.comparisonPixels>0)
        {
            const double compared =
                static_cast<double>(region.comparisonPixels);
            region.positiveComparedPixelRatio =
                static_cast<double>(
                    region.positivePresencePixels)/compared;
            region.negativeComparedPixelRatio =
                static_cast<double>(
                    region.negativePresencePixels)/compared;
            region.consistentComparedPixelRatio =
                static_cast<double>(
                    region.consistentPresencePixels)/compared;
        }
        if(region.comparisonVotes>0)
        {
            const double votes =
                static_cast<double>(region.comparisonVotes);
            region.positiveVoteRatio =
                static_cast<double>(region.positiveVotes)/votes;
            region.negativeVoteRatio =
                static_cast<double>(region.negativeVotes)/votes;
            region.consistentVoteRatio =
                static_cast<double>(region.consistentVotes)/votes;
        }
    }
    result.stats.totalMs =
        std::chrono::duration<double,std::milli>(
            std::chrono::steady_clock::now()-totalStart).count();
    return result;
}

} // namespace ORB_SLAM2
