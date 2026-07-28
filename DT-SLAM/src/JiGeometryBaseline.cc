#include "JiGeometryBaseline.h"

#include <opencv2/core/core.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace ORB_SLAM2
{

namespace
{

double ElapsedMilliseconds(
    const std::chrono::steady_clock::time_point &start,
    const std::chrono::steady_clock::time_point &end)
{
    return std::chrono::duration_cast<
        std::chrono::duration<double,std::milli> >(end-start).count();
}

}

JiGeometryBaseline::JiGeometryBaseline():
    mClusterCount(24),
    mMaxIterations(20),
    mEpsilon(1e-3),
    mAttempts(1),
    mRandomSeed(2021)
{
}

void JiGeometryBaseline::SetCameraMatrix(const cv::Mat &cameraMatrix)
{
    if(cameraMatrix.rows!=3 || cameraMatrix.cols!=3 ||
       cameraMatrix.channels()!=1 || !cv::checkRange(cameraMatrix))
    {
        throw std::invalid_argument(
            "JiGeometryBaseline camera matrix must be finite 3x3");
    }

    cv::Mat cameraMatrixFloat;
    cameraMatrix.convertTo(cameraMatrixFloat,CV_32F);
    const float fx = cameraMatrixFloat.at<float>(0,0);
    const float fy = cameraMatrixFloat.at<float>(1,1);
    if(!std::isfinite(fx) || !std::isfinite(fy) ||
       fx<=0.0f || fy<=0.0f)
    {
        throw std::invalid_argument(
            "JiGeometryBaseline focal lengths must be positive");
    }
    mCameraMatrix = cameraMatrixFloat;
}

void JiGeometryBaseline::SetClusterCount(const int clusterCount)
{
    if(clusterCount<=0 || clusterCount>std::numeric_limits<short>::max())
        throw std::invalid_argument(
            "JiGeometryBaseline cluster count must be in [1,32767]");
    mClusterCount = clusterCount;
}

void JiGeometryBaseline::SetKMeansCriteria(
    const int maxIterations,
    const double epsilon)
{
    if(maxIterations<=0 || !std::isfinite(epsilon) || epsilon<0.0)
        throw std::invalid_argument(
            "JiGeometryBaseline K-means criteria are invalid");
    mMaxIterations = maxIterations;
    mEpsilon = epsilon;
}

void JiGeometryBaseline::SetKMeansAttempts(const int attempts)
{
    if(attempts<=0)
        throw std::invalid_argument(
            "JiGeometryBaseline K-means attempts must be positive");
    mAttempts = attempts;
}

void JiGeometryBaseline::SetRandomSeed(const std::uint64_t randomSeed)
{
    mRandomSeed = randomSeed;
}

int JiGeometryBaseline::ClusterCount() const
{
    return mClusterCount;
}

int JiGeometryBaseline::MaxIterations() const
{
    return mMaxIterations;
}

double JiGeometryBaseline::Epsilon() const
{
    return mEpsilon;
}

int JiGeometryBaseline::Attempts() const
{
    return mAttempts;
}

std::uint64_t JiGeometryBaseline::RandomSeed() const
{
    return mRandomSeed;
}

bool JiGeometryBaseline::ComputeDepthClusters(
    const cv::Mat &depthMeters,
    JiDepthClusteringResult &result) const
{
    result = JiDepthClusteringResult();
    if(depthMeters.empty() || depthMeters.type()!=CV_32FC1 ||
       mCameraMatrix.empty())
    {
        return false;
    }

    const std::chrono::steady_clock::time_point totalStart =
        std::chrono::steady_clock::now();
    result.stats.imagePixels =
        static_cast<std::size_t>(depthMeters.rows)*
        static_cast<std::size_t>(depthMeters.cols);
    result.stats.requestedClusters = mClusterCount;

    std::size_t validDepthPixels = 0;
    for(int v=0; v<depthMeters.rows; ++v)
    {
        const float *depthRow = depthMeters.ptr<float>(v);
        for(int u=0; u<depthMeters.cols; ++u)
        {
            const float depth = depthRow[u];
            if(std::isfinite(depth) && depth>0.0f)
                ++validDepthPixels;
        }
    }
    result.stats.validDepthPixels = validDepthPixels;
    if(validDepthPixels<static_cast<std::size_t>(mClusterCount) ||
       validDepthPixels>static_cast<std::size_t>(
           std::numeric_limits<int>::max()))
    {
        return false;
    }

    cv::Mat samples(
        static_cast<int>(validDepthPixels),3,CV_32F);
    std::vector<cv::Point> samplePixels;
    samplePixels.reserve(validDepthPixels);

    const float fx = mCameraMatrix.at<float>(0,0);
    const float fy = mCameraMatrix.at<float>(1,1);
    const float cx = mCameraMatrix.at<float>(0,2);
    const float cy = mCameraMatrix.at<float>(1,2);
    int sampleIndex = 0;
    for(int v=0; v<depthMeters.rows; ++v)
    {
        const float *depthRow = depthMeters.ptr<float>(v);
        for(int u=0; u<depthMeters.cols; ++u)
        {
            const float depth = depthRow[u];
            if(!std::isfinite(depth) || depth<=0.0f)
                continue;

            float *sample = samples.ptr<float>(sampleIndex);
            sample[0] = (static_cast<float>(u)-cx)*depth/fx;
            sample[1] = (static_cast<float>(v)-cy)*depth/fy;
            sample[2] = depth;
            samplePixels.push_back(cv::Point(u,v));
            ++sampleIndex;
        }
    }
    const std::chrono::steady_clock::time_point prepareEnd =
        std::chrono::steady_clock::now();

    cv::Mat labels;
    cv::Mat centers;
    const std::uint64_t savedRandomState = cv::theRNG().state;
    cv::theRNG().state = mRandomSeed;
    double compactness = 0.0;
    try
    {
        compactness = cv::kmeans(
            samples,mClusterCount,labels,
            cv::TermCriteria(
                cv::TermCriteria::COUNT+cv::TermCriteria::EPS,
                mMaxIterations,mEpsilon),
            mAttempts,cv::KMEANS_PP_CENTERS,centers);
    }
    catch(...)
    {
        cv::theRNG().state = savedRandomState;
        throw;
    }
    cv::theRNG().state = savedRandomState;
    const std::chrono::steady_clock::time_point kmeansEnd =
        std::chrono::steady_clock::now();

    result.labelImage = cv::Mat(
        depthMeters.size(),CV_16SC1,cv::Scalar(-1));
    result.clusters.resize(mClusterCount);
    for(int clusterId=0; clusterId<mClusterCount; ++clusterId)
    {
        result.clusters[clusterId].id = clusterId;
        result.clusters[clusterId].centroid = cv::Vec3f(
            centers.at<float>(clusterId,0),
            centers.at<float>(clusterId,1),
            centers.at<float>(clusterId,2));
    }

    for(int index=0; index<labels.rows; ++index)
    {
        const int clusterId = labels.at<int>(index,0);
        if(clusterId<0 || clusterId>=mClusterCount)
            throw std::runtime_error(
                "JiGeometryBaseline K-means returned an invalid label");
        const cv::Point &pixel = samplePixels[index];
        result.labelImage.at<short>(pixel.y,pixel.x) =
            static_cast<short>(clusterId);
        ++result.clusters[clusterId].pixels;
    }

    result.stats.producedClusters = mClusterCount;
    result.stats.compactness = compactness;
    result.stats.smallestClusterPixels = validDepthPixels;
    for(int clusterId=0; clusterId<mClusterCount; ++clusterId)
    {
        result.stats.smallestClusterPixels = std::min(
            result.stats.smallestClusterPixels,
            result.clusters[clusterId].pixels);
        result.stats.largestClusterPixels = std::max(
            result.stats.largestClusterPixels,
            result.clusters[clusterId].pixels);
    }
    const std::chrono::steady_clock::time_point labelEnd =
        std::chrono::steady_clock::now();

    result.stats.prepareMs =
        ElapsedMilliseconds(totalStart,prepareEnd);
    result.stats.kmeansMs =
        ElapsedMilliseconds(prepareEnd,kmeansEnd);
    result.stats.labelMs =
        ElapsedMilliseconds(kmeansEnd,labelEnd);
    result.stats.totalMs =
        ElapsedMilliseconds(totalStart,labelEnd);
    return true;
}

bool JiGeometryBaseline::ComputeClusterReprojectionStats(
    const JiDepthClusteringResult &clustering,
    const std::vector<cv::Point2f> &rawFeaturePixels,
    const std::vector<JiReprojectionObservation> &observations,
    const cv::Mat &initialTcw,
    const cv::Mat &projectionCameraMatrix,
    std::vector<JiClusterReprojectionStats> &clusterStats,
    JiReprojectionFrameStats &frameStats) const
{
    clusterStats.clear();
    frameStats = JiReprojectionFrameStats();
    if(clustering.labelImage.empty() ||
       clustering.labelImage.type()!=CV_16SC1 ||
       clustering.clusters.empty() ||
       projectionCameraMatrix.rows!=3 ||
       projectionCameraMatrix.cols!=3 ||
       projectionCameraMatrix.channels()!=1 ||
       !cv::checkRange(projectionCameraMatrix))
    {
        return false;
    }

    cv::Mat projectionK;
    projectionCameraMatrix.convertTo(projectionK,CV_32F);
    const float fx = projectionK.at<float>(0,0);
    const float fy = projectionK.at<float>(1,1);
    const float cx = projectionK.at<float>(0,2);
    const float cy = projectionK.at<float>(1,2);
    if(!std::isfinite(fx) || !std::isfinite(fy) ||
       !std::isfinite(cx) || !std::isfinite(cy) ||
       fx<=0.0f || fy<=0.0f)
    {
        return false;
    }

    cv::Mat pose;
    if(!initialTcw.empty())
    {
        if(initialTcw.rows!=4 || initialTcw.cols!=4 ||
           initialTcw.channels()!=1 || !cv::checkRange(initialTcw))
        {
            return false;
        }
        initialTcw.convertTo(pose,CV_32F);
        frameStats.initialPoseAvailable = true;
    }

    const std::chrono::steady_clock::time_point totalStart =
        std::chrono::steady_clock::now();
    const int clusterCount =
        static_cast<int>(clustering.clusters.size());
    clusterStats.resize(clusterCount);
    std::vector<std::vector<double> > errorPixels(clusterCount);
    for(int clusterId=0; clusterId<clusterCount; ++clusterId)
        clusterStats[clusterId].clusterId = clusterId;

    frameStats.featureCount = rawFeaturePixels.size();
    for(std::size_t index=0; index<rawFeaturePixels.size(); ++index)
    {
        const cv::Point2f &pixel = rawFeaturePixels[index];
        if(!std::isfinite(pixel.x) || !std::isfinite(pixel.y) ||
           pixel.x<0.0f || pixel.y<0.0f ||
           pixel.x>=clustering.labelImage.cols ||
           pixel.y>=clustering.labelImage.rows)
        {
            continue;
        }
        const int u = static_cast<int>(pixel.x);
        const int v = static_cast<int>(pixel.y);
        const int clusterId =
            clustering.labelImage.at<short>(v,u);
        if(clusterId<0 || clusterId>=clusterCount)
            continue;
        ++clusterStats[clusterId].featureCount;
        ++frameStats.featuresAssignedToClusters;
    }
    const std::chrono::steady_clock::time_point assignmentEnd =
        std::chrono::steady_clock::now();

    frameStats.matchedObservations = observations.size();
    for(std::size_t index=0; index<observations.size(); ++index)
    {
        const JiReprojectionObservation &observation =
            observations[index];
        const cv::Point2f &rawPixel = observation.rawPixel;
        if(!std::isfinite(rawPixel.x) || !std::isfinite(rawPixel.y) ||
           rawPixel.x<0.0f || rawPixel.y<0.0f ||
           rawPixel.x>=clustering.labelImage.cols ||
           rawPixel.y>=clustering.labelImage.rows)
        {
            continue;
        }
        const int u = static_cast<int>(rawPixel.x);
        const int v = static_cast<int>(rawPixel.y);
        const int clusterId =
            clustering.labelImage.at<short>(v,u);
        if(clusterId<0 || clusterId>=clusterCount)
            continue;

        JiClusterReprojectionStats &stats =
            clusterStats[clusterId];
        ++stats.matchedMapSupport;
        ++frameStats.matchesAssignedToClusters;
        if(observation.optimizerOutlier)
        {
            ++stats.optimizerOutlierSupport;
            ++frameStats.optimizerOutlierObservations;
        }

        if(pose.empty() ||
           !std::isfinite(observation.observedPinholePixel.x) ||
           !std::isfinite(observation.observedPinholePixel.y) ||
           !std::isfinite(observation.worldPoint.x) ||
           !std::isfinite(observation.worldPoint.y) ||
           !std::isfinite(observation.worldPoint.z))
        {
            ++stats.invalidProjectionCount;
            continue;
        }

        const float x =
            pose.at<float>(0,0)*observation.worldPoint.x+
            pose.at<float>(0,1)*observation.worldPoint.y+
            pose.at<float>(0,2)*observation.worldPoint.z+
            pose.at<float>(0,3);
        const float y =
            pose.at<float>(1,0)*observation.worldPoint.x+
            pose.at<float>(1,1)*observation.worldPoint.y+
            pose.at<float>(1,2)*observation.worldPoint.z+
            pose.at<float>(1,3);
        const float z =
            pose.at<float>(2,0)*observation.worldPoint.x+
            pose.at<float>(2,1)*observation.worldPoint.y+
            pose.at<float>(2,2)*observation.worldPoint.z+
            pose.at<float>(2,3);
        if(!std::isfinite(x) || !std::isfinite(y) ||
           !std::isfinite(z) || z<=0.0f)
        {
            ++stats.invalidProjectionCount;
            continue;
        }

        const double predictedU =
            static_cast<double>(fx)*x/z+cx;
        const double predictedV =
            static_cast<double>(fy)*y/z+cy;
        const double deltaU =
            observation.observedPinholePixel.x-predictedU;
        const double deltaV =
            observation.observedPinholePixel.y-predictedV;
        const double squaredError =
            deltaU*deltaU+deltaV*deltaV;
        if(!std::isfinite(squaredError))
        {
            ++stats.invalidProjectionCount;
            continue;
        }

        errorPixels[clusterId].push_back(std::sqrt(squaredError));
        stats.meanSquaredErrorPixels2 += squaredError;
        ++stats.validReprojectionSupport;
        ++frameStats.validReprojections;
    }
    const std::chrono::steady_clock::time_point reprojectionEnd =
        std::chrono::steady_clock::now();

    for(int clusterId=0; clusterId<clusterCount; ++clusterId)
    {
        JiClusterReprojectionStats &stats =
            clusterStats[clusterId];
        std::vector<double> &errors = errorPixels[clusterId];
        if(errors.empty())
        {
            ++frameStats.clustersWithoutEvidence;
            continue;
        }

        stats.hasGeometryEvidence = true;
        stats.meanSquaredErrorPixels2 /=
            static_cast<double>(errors.size());
        double errorSum = 0.0;
        for(std::size_t index=0; index<errors.size(); ++index)
            errorSum += errors[index];
        stats.meanErrorPixels =
            errorSum/static_cast<double>(errors.size());

        std::sort(errors.begin(),errors.end());
        const std::size_t middle = errors.size()/2;
        if(errors.size()%2==0)
        {
            stats.medianErrorPixels =
                0.5*(errors[middle-1]+errors[middle]);
        }
        else
        {
            stats.medianErrorPixels = errors[middle];
        }
        const std::size_t p90Index =
            static_cast<std::size_t>(
                std::ceil(0.9*static_cast<double>(errors.size())))-1;
        stats.p90ErrorPixels = errors[p90Index];
        stats.maximumErrorPixels = errors.back();
    }
    const std::chrono::steady_clock::time_point aggregateEnd =
        std::chrono::steady_clock::now();

    frameStats.featureAssignmentMs =
        ElapsedMilliseconds(totalStart,assignmentEnd);
    frameStats.reprojectionMs =
        ElapsedMilliseconds(assignmentEnd,reprojectionEnd);
    frameStats.aggregateMs =
        ElapsedMilliseconds(reprojectionEnd,aggregateEnd);
    frameStats.totalMs =
        ElapsedMilliseconds(totalStart,aggregateEnd);
    return true;
}

} // namespace ORB_SLAM2
