#include "SInStyleInitialRegionClusterer.h"

#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>

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

bool IsClusteringDepthValid(const float depth, const float maximumDepth)
{
    return std::isfinite(depth) && depth>0.0f && depth<maximumDepth;
}

int GridLabel(const int row, const int col,
              const int rows, const int cols,
              const int clusterCount)
{
    const double aspect = static_cast<double>(rows)/
                          static_cast<double>(std::max(1,cols));
    const int gridRows = std::max(
        1,static_cast<int>(std::lround(
            std::sqrt(static_cast<double>(clusterCount)*aspect))));
    const int gridCols = std::max(
        1,static_cast<int>(std::ceil(
            static_cast<double>(clusterCount)/gridRows)));
    const int gridRow = std::min(
        gridRows-1,row*gridRows/std::max(1,rows));
    const int gridCol = std::min(
        gridCols-1,col*gridCols/std::max(1,cols));
    return std::min(clusterCount-1,gridRow*gridCols+gridCol);
}

void EnsureEveryClusterHasASample(
    cv::Mat &initialLabels,
    std::vector<unsigned char> &usedPrior,
    const int clusterCount)
{
    std::vector<int> counts(clusterCount,0);
    for(int index=0; index<initialLabels.rows; ++index)
        ++counts[initialLabels.at<int>(index,0)];

    for(int missing=0; missing<clusterCount; ++missing)
    {
        if(counts[missing]>0)
            continue;
        int donorIndex = -1;
        for(int index=0; index<initialLabels.rows; ++index)
        {
            const int donor = initialLabels.at<int>(index,0);
            if(counts[donor]>1)
            {
                donorIndex = index;
                --counts[donor];
                break;
            }
        }
        if(donorIndex<0)
            throw std::runtime_error(
                "SIn initial-region labels cannot populate every cluster");
        initialLabels.at<int>(donorIndex,0) = missing;
        usedPrior[donorIndex] = 0;
        ++counts[missing];
    }
}

struct InitializedLevelResult
{
    cv::Mat labels;
    SInStyleInitialRegionLevelStats stats;
};

InitializedLevelResult ClusterInitializedLevel(
    const cv::Mat &depthMeters,
    const cv::Mat &initialLabelImage,
    const cv::Mat &fullCameraMatrix,
    const cv::Size &fullSize,
    const SInStyleInitialRegionConfig &config,
    const int clusterCount,
    const int level)
{
    InitializedLevelResult result;
    result.stats.level = level;
    result.stats.rows = depthMeters.rows;
    result.stats.cols = depthMeters.cols;

    const std::chrono::steady_clock::time_point prepareStart =
        std::chrono::steady_clock::now();
    std::size_t validSamples = 0;
    for(int row=0; row<depthMeters.rows; ++row)
    {
        const float *depth = depthMeters.ptr<float>(row);
        for(int col=0; col<depthMeters.cols; ++col)
        {
            if(IsClusteringDepthValid(
                   depth[col],config.maximumDepthMeters))
            {
                ++validSamples;
            }
        }
    }
    result.stats.validSamples = validSamples;
    if(validSamples<static_cast<std::size_t>(clusterCount) ||
       validSamples>static_cast<std::size_t>(
           std::numeric_limits<int>::max()))
    {
        return result;
    }

    cv::Mat samples(static_cast<int>(validSamples),3,CV_32F);
    cv::Mat initialLabels(static_cast<int>(validSamples),1,CV_32S);
    std::vector<cv::Point> samplePixels;
    std::vector<unsigned char> usedPrior(validSamples,0);
    samplePixels.reserve(validSamples);

    const float scaleX = static_cast<float>(depthMeters.cols)/fullSize.width;
    const float scaleY = static_cast<float>(depthMeters.rows)/fullSize.height;
    const float fx = fullCameraMatrix.at<float>(0,0)*scaleX;
    const float fy = fullCameraMatrix.at<float>(1,1)*scaleY;
    const float cx = fullCameraMatrix.at<float>(0,2)*scaleX;
    const float cy = fullCameraMatrix.at<float>(1,2)*scaleY;
    int sampleIndex = 0;
    for(int row=0; row<depthMeters.rows; ++row)
    {
        const float *depth = depthMeters.ptr<float>(row);
        for(int col=0; col<depthMeters.cols; ++col)
        {
            const float value = depth[col];
            if(!IsClusteringDepthValid(value,config.maximumDepthMeters))
                continue;
            float *sample = samples.ptr<float>(sampleIndex);
            sample[0] = (static_cast<float>(col)-cx)*value/fx;
            sample[1] = (static_cast<float>(row)-cy)*value/fy;
            sample[2] = value;

            int label = -1;
            if(!initialLabelImage.empty())
            {
                const int candidate =
                    initialLabelImage.at<int>(row,col);
                if(candidate>0 && candidate<=clusterCount)
                {
                    label = candidate-1;
                    usedPrior[sampleIndex] = 1;
                }
            }
            if(label<0)
                label = GridLabel(
                    row,col,depthMeters.rows,depthMeters.cols,clusterCount);
            initialLabels.at<int>(sampleIndex,0) = label;
            samplePixels.push_back(cv::Point(col,row));
            ++sampleIndex;
        }
    }
    EnsureEveryClusterHasASample(
        initialLabels,usedPrior,clusterCount);
    result.stats.priorInitializedSamples = static_cast<std::size_t>(
        std::count(usedPrior.begin(),usedPrior.end(),1));
    result.stats.gridFallbackSamples =
        validSamples-result.stats.priorInitializedSamples;
    const std::chrono::steady_clock::time_point prepareEnd =
        std::chrono::steady_clock::now();

    cv::Mat centers;
    const std::uint64_t savedRandomState = cv::theRNG().state;
    cv::theRNG().state = config.randomSeed+static_cast<std::uint64_t>(level);
    const std::chrono::steady_clock::time_point kmeansStart =
        std::chrono::steady_clock::now();
    try
    {
        result.stats.compactness = cv::kmeans(
            samples,clusterCount,initialLabels,
            cv::TermCriteria(
                cv::TermCriteria::COUNT+cv::TermCriteria::EPS,
                config.maximumIterations,config.epsilon),
            config.attempts,cv::KMEANS_USE_INITIAL_LABELS,centers);
    }
    catch(...)
    {
        cv::theRNG().state = savedRandomState;
        throw;
    }
    cv::theRNG().state = savedRandomState;
    const std::chrono::steady_clock::time_point kmeansEnd =
        std::chrono::steady_clock::now();

    const std::chrono::steady_clock::time_point labelStart =
        std::chrono::steady_clock::now();
    result.labels = cv::Mat(
        depthMeters.size(),CV_32SC1,cv::Scalar(-1));
    for(int index=0; index<initialLabels.rows; ++index)
    {
        const int label = initialLabels.at<int>(index,0);
        if(label<0 || label>=clusterCount)
            throw std::runtime_error(
                "SIn initialized K-means returned an invalid label");
        const cv::Point &pixel = samplePixels[index];
        result.labels.at<int>(pixel.y,pixel.x) = label+1;
    }
    const std::chrono::steady_clock::time_point labelEnd =
        std::chrono::steady_clock::now();
    result.stats.prepareMs = ElapsedMilliseconds(prepareStart,prepareEnd);
    result.stats.kmeansMs = ElapsedMilliseconds(kmeansStart,kmeansEnd);
    result.stats.labelMs = ElapsedMilliseconds(labelStart,labelEnd);
    return result;
}

void PopulateFinalRegionStats(
    const cv::Mat &labels,
    const int clusterCount,
    SInStyleInitialRegionStats &stats)
{
    std::vector<std::size_t> areas(clusterCount,0);
    for(int row=0; row<labels.rows; ++row)
    {
        const int *values = labels.ptr<int>(row);
        for(int col=0; col<labels.cols; ++col)
        {
            if(values[col]>0 && values[col]<=clusterCount)
                ++areas[values[col]-1];
        }
    }
    stats.producedClusters = static_cast<int>(
        std::count_if(areas.begin(),areas.end(),
            [](const std::size_t area) { return area>0; }));
    stats.smallestRegionPixels = *std::min_element(areas.begin(),areas.end());
    stats.largestRegionPixels = *std::max_element(areas.begin(),areas.end());
}

} // namespace

SInStyleInitialRegionClusterer::SInStyleInitialRegionClusterer()
{
}

void SInStyleInitialRegionClusterer::Configure(
    const SInStyleInitialRegionConfig &config,
    const cv::Mat &cameraMatrix)
{
    if(config.enabled)
    {
        if(!std::isfinite(config.clusterPixelDivisor) ||
           config.clusterPixelDivisor<=0.0)
            throw std::invalid_argument(
                "SIn initial-region cluster pixel divisor must be positive");
        if(!std::isfinite(config.maximumDepthMeters) ||
           config.maximumDepthMeters<=0.0f)
            throw std::invalid_argument(
                "SIn initial-region maximum depth must be positive");
        if(config.maximumIterations<=0 ||
           !std::isfinite(config.epsilon) || config.epsilon<0.0 ||
           config.attempts<=0 || config.pyramidLevels<=0)
            throw std::invalid_argument(
                "SIn initial-region K-means settings are invalid");
        if(config.coarseToFine && config.attempts!=1)
            throw std::invalid_argument(
                "SIn coarse-to-fine initial regions require one K-means attempt");
        if(cameraMatrix.rows!=3 || cameraMatrix.cols!=3 ||
           cameraMatrix.channels()!=1 || !cv::checkRange(cameraMatrix))
            throw std::invalid_argument(
                "SIn initial-region camera matrix must be finite 3x3");
        cameraMatrix.convertTo(mCameraMatrix,CV_32F);
        if(mCameraMatrix.at<float>(0,0)<=0.0f ||
           mCameraMatrix.at<float>(1,1)<=0.0f)
            throw std::invalid_argument(
                "SIn initial-region focal lengths must be positive");
    }
    else
    {
        mCameraMatrix.release();
    }
    mConfig = config;
    Reset();
}

void SInStyleInitialRegionClusterer::Reset()
{
    mPreviousInitialLabels.release();
    mPreviousInputIndex = 0;
    mbHasPreviousInitialLabels = false;
}

SInStyleInitialRegionResult SInStyleInitialRegionClusterer::Compute(
    const cv::Mat &depthMeters,
    const long unsigned int inputIndex)
{
    SInStyleInitialRegionResult result;
    result.stats.enabled = mConfig.enabled;
    result.stats.coarseToFine = mConfig.coarseToFine;
    result.stats.pyramidLevels = mConfig.coarseToFine ?
        mConfig.pyramidLevels : 1;
    if(!mConfig.enabled)
        return result;
    if(depthMeters.empty() || depthMeters.type()!=CV_32FC1 ||
       mCameraMatrix.empty())
        throw std::invalid_argument(
            "SIn initial regions require CV_32FC1 depth and camera intrinsics");

    const std::chrono::steady_clock::time_point totalStart =
        std::chrono::steady_clock::now();
    result.stats.imagePixels = depthMeters.total();
    for(int row=0; row<depthMeters.rows; ++row)
    {
        const float *values = depthMeters.ptr<float>(row);
        for(int col=0; col<depthMeters.cols; ++col)
        {
            const float depth = values[col];
            if(std::isfinite(depth) && depth>0.0f)
            {
                ++result.stats.inputDepthValidPixels;
                if(depth<mConfig.maximumDepthMeters)
                    ++result.stats.clusteringDepthValidPixels;
                else
                    ++result.stats.excludedFarDepthPixels;
            }
        }
    }

    const double requested =
        static_cast<double>(result.stats.imagePixels)/
        mConfig.clusterPixelDivisor;
    if(!std::isfinite(requested) ||
       requested>static_cast<double>(
           std::numeric_limits<short>::max()))
    {
        throw std::invalid_argument(
            "SIn initial-region cluster count exceeds the supported label domain");
    }
    const int clusterCount = std::max(
        1,static_cast<int>(std::lround(requested)));
    result.stats.requestedClusters = clusterCount;
    if(result.stats.clusteringDepthValidPixels<
       static_cast<std::size_t>(clusterCount))
    {
        result.labels = cv::Mat(
            depthMeters.size(),CV_32SC1,cv::Scalar(-1));
        result.validMask = cv::Mat(
            depthMeters.size(),CV_8UC1,cv::Scalar(0));
        result.stats.totalMs = ElapsedMilliseconds(
            totalStart,std::chrono::steady_clock::now());
        return result;
    }

    if(!mConfig.coarseToFine)
    {
        cv::Mat clusteringDepth = depthMeters.clone();
        for(int row=0; row<clusteringDepth.rows; ++row)
        {
            float *values = clusteringDepth.ptr<float>(row);
            for(int col=0; col<clusteringDepth.cols; ++col)
            {
                if(!IsClusteringDepthValid(
                       values[col],mConfig.maximumDepthMeters))
                {
                    values[col] = 0.0f;
                }
            }
        }

        JiGeometryBaseline clusterer;
        clusterer.SetCameraMatrix(mCameraMatrix);
        clusterer.SetClusterCount(clusterCount);
        clusterer.SetKMeansCriteria(
            mConfig.maximumIterations,mConfig.epsilon);
        clusterer.SetKMeansAttempts(mConfig.attempts);
        clusterer.SetRandomSeed(mConfig.randomSeed);
        JiDepthClusteringResult clustering;
        if(!clusterer.ComputeDepthClusters(clusteringDepth,clustering))
            throw std::runtime_error("SIn initial 3D K-means failed");

        const std::chrono::steady_clock::time_point conversionStart =
            std::chrono::steady_clock::now();
        result.labels = cv::Mat(
            depthMeters.size(),CV_32SC1,cv::Scalar(-1));
        result.validMask = cv::Mat(
            depthMeters.size(),CV_8UC1,cv::Scalar(0));
        for(int row=0; row<clustering.labelImage.rows; ++row)
        {
            const short *source = clustering.labelImage.ptr<short>(row);
            int *labels = result.labels.ptr<int>(row);
            unsigned char *valid = result.validMask.ptr<unsigned char>(row);
            for(int col=0; col<clustering.labelImage.cols; ++col)
            {
                if(source[col]<0)
                    continue;
                labels[col] = static_cast<int>(source[col])+1;
                valid[col] = 255;
            }
        }
        const std::chrono::steady_clock::time_point conversionEnd =
            std::chrono::steady_clock::now();
        result.stats.available = true;
        result.stats.producedClusters = clustering.stats.producedClusters;
        result.stats.smallestRegionPixels =
            clustering.stats.smallestClusterPixels;
        result.stats.largestRegionPixels =
            clustering.stats.largestClusterPixels;
        result.stats.compactness = clustering.stats.compactness;
        result.stats.prepareMs = clustering.stats.prepareMs;
        result.stats.kmeansMs = clustering.stats.kmeansMs;
        result.stats.labelConversionMs = ElapsedMilliseconds(
            conversionStart,conversionEnd);
        result.stats.initializationSource = "from_scratch";
        result.stats.totalMs = ElapsedMilliseconds(totalStart,conversionEnd);
        return result;
    }

    std::vector<cv::Mat> depthPyramid;
    depthPyramid.push_back(depthMeters);
    for(int level=1; level<mConfig.pyramidLevels; ++level)
    {
        const int cols = std::max(1,depthPyramid.back().cols/2);
        const int rows = std::max(1,depthPyramid.back().rows/2);
        if(cols==depthPyramid.back().cols && rows==depthPyramid.back().rows)
            break;
        cv::Mat next;
        cv::resize(depthPyramid.back(),next,cv::Size(cols,rows),
                   0.0,0.0,cv::INTER_NEAREST);
        depthPyramid.push_back(next);
    }

    int coarsestLevel = static_cast<int>(depthPyramid.size())-1;
    while(coarsestLevel>0)
    {
        std::size_t valid = 0;
        for(int row=0; row<depthPyramid[coarsestLevel].rows; ++row)
        {
            const float *values =
                depthPyramid[coarsestLevel].ptr<float>(row);
            for(int col=0; col<depthPyramid[coarsestLevel].cols; ++col)
            {
                if(IsClusteringDepthValid(
                       values[col],mConfig.maximumDepthMeters))
                    ++valid;
            }
        }
        if(valid>=static_cast<std::size_t>(clusterCount))
            break;
        --coarsestLevel;
    }
    result.stats.pyramidLevels = coarsestLevel+1;

    const bool previousIsEligible =
        mConfig.temporalInitialization &&
        mbHasPreviousInitialLabels &&
        inputIndex==mPreviousInputIndex+1 &&
        inputIndex>mConfig.temporalCommitStartInputIndex &&
        mPreviousInitialLabels.size()==depthMeters.size();

    cv::Mat initialization;
    if(previousIsEligible)
    {
        cv::resize(mPreviousInitialLabels,initialization,
                   depthPyramid[coarsestLevel].size(),
                   0.0,0.0,cv::INTER_NEAREST);
    }

    cv::Mat currentLabels;
    for(int level=coarsestLevel; level>=0; --level)
    {
        if(level<coarsestLevel)
        {
            cv::resize(currentLabels,initialization,
                       depthPyramid[level].size(),
                       0.0,0.0,cv::INTER_NEAREST);
        }
        InitializedLevelResult levelResult = ClusterInitializedLevel(
            depthPyramid[level],initialization,mCameraMatrix,
            depthMeters.size(),mConfig,clusterCount,level);
        if(levelResult.labels.empty())
            throw std::runtime_error(
                "SIn coarse-to-fine level has insufficient valid depth");
        if(level==coarsestLevel)
        {
            result.stats.previousPriorSamples =
                previousIsEligible ?
                levelResult.stats.priorInitializedSamples : 0;
            result.stats.gridFallbackSamples = previousIsEligible ?
                levelResult.stats.gridFallbackSamples :
                levelResult.stats.validSamples;
        }
        result.stats.prepareMs += levelResult.stats.prepareMs;
        result.stats.kmeansMs += levelResult.stats.kmeansMs;
        result.stats.labelConversionMs += levelResult.stats.labelMs;
        result.stats.levels.push_back(levelResult.stats);
        currentLabels = levelResult.labels;
        initialization = currentLabels;
    }

    result.labels = currentLabels;
    cv::compare(result.labels,0,result.validMask,cv::CMP_GT);
    result.stats.available = true;
    result.stats.compactness = result.stats.levels.back().compactness;
    PopulateFinalRegionStats(
        result.labels,clusterCount,result.stats);
    const std::size_t coarsestSamples =
        result.stats.previousPriorSamples+result.stats.gridFallbackSamples;
    if(coarsestSamples>0)
    {
        result.stats.previousPriorCoverage =
            static_cast<double>(result.stats.previousPriorSamples)/
            static_cast<double>(coarsestSamples);
    }
    if(result.stats.previousPriorSamples==0)
        result.stats.initializationSource = "grid";
    else if(result.stats.gridFallbackSamples==0)
        result.stats.initializationSource = "previous";
    else
        result.stats.initializationSource = "mixed";

    if(mConfig.temporalInitialization &&
       inputIndex>=mConfig.temporalCommitStartInputIndex)
    {
        mPreviousInitialLabels = result.labels.clone();
        mPreviousInputIndex = inputIndex;
        mbHasPreviousInitialLabels = true;
        result.stats.temporalPriorCommitted = true;
    }
    result.stats.totalMs = ElapsedMilliseconds(
        totalStart,std::chrono::steady_clock::now());
    return result;
}

} // namespace ORB_SLAM2
