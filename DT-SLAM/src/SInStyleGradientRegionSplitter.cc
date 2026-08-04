#include "SInStyleGradientRegionSplitter.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <deque>
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

bool IsDepthValid(const float depth, const float maximumDepth)
{
    return std::isfinite(depth) && depth>0.0f && depth<maximumDepth;
}

} // namespace

SInStyleGradientRegionSplitter::SInStyleGradientRegionSplitter()
{
}

void SInStyleGradientRegionSplitter::Configure(
    const SInStyleGradientSplitConfig &config)
{
    if(config.enabled)
    {
        if(!std::isfinite(config.maximumDepthMeters) ||
           config.maximumDepthMeters<=0.0f ||
           !std::isfinite(config.relativeThreshold) ||
           config.relativeThreshold<0.0f ||
           !std::isfinite(config.absoluteThresholdMeters) ||
           config.absoluteThresholdMeters<0.0f ||
           config.medianRadius<0 || config.minimumMedianSupport<=0 ||
           (config.connectivity!=4 && config.connectivity!=8))
        {
            throw std::invalid_argument(
                "SIn gradient-depth split configuration is invalid");
        }
        const long long diameter =
            2LL*static_cast<long long>(config.medianRadius)+1LL;
        const long long maximumSupport = diameter*diameter;
        if(maximumSupport>
               static_cast<long long>(std::numeric_limits<unsigned short>::max()) ||
           config.minimumMedianSupport>maximumSupport)
        {
            throw std::invalid_argument(
                "SIn gradient-depth median support is outside its label domain");
        }
    }
    mConfig = config;
}

SInStyleGradientSplitResult SInStyleGradientRegionSplitter::Compute(
    const cv::Mat &depthMeters,
    const cv::Mat &initialLabels) const
{
    SInStyleGradientSplitResult result;
    result.stats.enabled = mConfig.enabled;
    if(!mConfig.enabled)
        return result;
    if(depthMeters.empty() || depthMeters.type()!=CV_32FC1 ||
       initialLabels.empty() || initialLabels.type()!=CV_32SC1 ||
       depthMeters.size()!=initialLabels.size())
    {
        throw std::invalid_argument(
            "SIn gradient split requires aligned CV_32FC1 depth and CV_32SC1 labels");
    }

    const std::chrono::steady_clock::time_point totalStart =
        std::chrono::steady_clock::now();
    result.stats.imagePixels = depthMeters.total();
    result.filteredDepth = cv::Mat(
        depthMeters.size(),CV_32FC1,cv::Scalar(0.0f));
    result.medianSupport = cv::Mat(
        depthMeters.size(),CV_16UC1,cv::Scalar(0));
    result.medianValidMask = cv::Mat(
        depthMeters.size(),CV_8UC1,cv::Scalar(0));
    result.insufficientSupportMask = cv::Mat(
        depthMeters.size(),CV_8UC1,cv::Scalar(0));
    result.rawGradientEdgeMask = cv::Mat(
        depthMeters.size(),CV_8UC1,cv::Scalar(0));
    result.splitBoundaryMask = cv::Mat(
        depthMeters.size(),CV_8UC1,cv::Scalar(0));
    result.splitCoreLabels = cv::Mat(
        depthMeters.size(),CV_32SC1,cv::Scalar(-1));
    result.splitValidMask = cv::Mat(
        depthMeters.size(),CV_8UC1,cv::Scalar(0));

    int maximumInitialLabel = 0;
    const std::chrono::steady_clock::time_point medianStart =
        std::chrono::steady_clock::now();
    std::vector<float> samples;
    std::vector<unsigned char> initialLabelPresent(1,0);
    const int diameter = 2*mConfig.medianRadius+1;
    samples.reserve(static_cast<std::size_t>(diameter*diameter));
    for(int row=0; row<depthMeters.rows; ++row)
    {
        const float *depth = depthMeters.ptr<float>(row);
        const int *labels = initialLabels.ptr<int>(row);
        float *filtered = result.filteredDepth.ptr<float>(row);
        unsigned short *support = result.medianSupport.ptr<unsigned short>(row);
        unsigned char *medianValid =
            result.medianValidMask.ptr<unsigned char>(row);
        unsigned char *insufficient =
            result.insufficientSupportMask.ptr<unsigned char>(row);
        for(int col=0; col<depthMeters.cols; ++col)
        {
            const bool depthValid =
                IsDepthValid(depth[col],mConfig.maximumDepthMeters);
            if(depthValid)
                ++result.stats.inputDepthValidPixels;
            if(labels[col]>0)
            {
                ++result.stats.initialRegionPixels;
                maximumInitialLabel = std::max(maximumInitialLabel,labels[col]);
                if(initialLabelPresent.size()<=
                   static_cast<std::size_t>(labels[col]))
                {
                    initialLabelPresent.resize(
                        static_cast<std::size_t>(labels[col])+1,0);
                }
                if(initialLabelPresent[labels[col]]==0)
                {
                    initialLabelPresent[labels[col]] = 1;
                    ++result.stats.initialRegionCount;
                }
            }
            if(!depthValid || labels[col]<=0)
                continue;

            samples.clear();
            const int firstRow = std::max(0,row-mConfig.medianRadius);
            const int lastRow = std::min(
                depthMeters.rows-1,row+mConfig.medianRadius);
            const int firstCol = std::max(0,col-mConfig.medianRadius);
            const int lastCol = std::min(
                depthMeters.cols-1,col+mConfig.medianRadius);
            for(int neighborRow=firstRow;
                neighborRow<=lastRow; ++neighborRow)
            {
                const float *neighborDepth =
                    depthMeters.ptr<float>(neighborRow);
                for(int neighborCol=firstCol;
                    neighborCol<=lastCol; ++neighborCol)
                {
                    const float value = neighborDepth[neighborCol];
                    if(IsDepthValid(value,mConfig.maximumDepthMeters))
                        samples.push_back(value);
                }
            }
            support[col] = static_cast<unsigned short>(samples.size());
            if(samples.size()<
               static_cast<std::size_t>(mConfig.minimumMedianSupport))
            {
                insufficient[col] = 255;
                ++result.stats.insufficientSupportPixels;
                continue;
            }
            const std::size_t middle = samples.size()/2;
            std::nth_element(
                samples.begin(),samples.begin()+middle,samples.end());
            float median = samples[middle];
            if(samples.size()%2==0)
            {
                const float lower = *std::max_element(
                    samples.begin(),samples.begin()+middle);
                median = 0.5f*(lower+median);
            }
            filtered[col] = median;
            medianValid[col] = 255;
            ++result.stats.medianValidPixels;
        }
    }
    const std::chrono::steady_clock::time_point medianEnd =
        std::chrono::steady_clock::now();

    const std::chrono::steady_clock::time_point edgeStart =
        std::chrono::steady_clock::now();
    for(int row=0; row<depthMeters.rows; ++row)
    {
        const float *filtered = result.filteredDepth.ptr<float>(row);
        const unsigned char *medianValid =
            result.medianValidMask.ptr<unsigned char>(row);
        unsigned char *edge =
            result.rawGradientEdgeMask.ptr<unsigned char>(row);
        for(int col=0; col<depthMeters.cols; ++col)
        {
            if(medianValid[col]==0)
                continue;
            float maximumDifference = 0.0f;
            const int firstRow = std::max(0,row-mConfig.medianRadius);
            const int lastRow = std::min(
                depthMeters.rows-1,row+mConfig.medianRadius);
            const int firstCol = std::max(0,col-mConfig.medianRadius);
            const int lastCol = std::min(
                depthMeters.cols-1,col+mConfig.medianRadius);
            for(int neighborRow=firstRow;
                neighborRow<=lastRow; ++neighborRow)
            {
                const float *neighborFiltered =
                    result.filteredDepth.ptr<float>(neighborRow);
                const unsigned char *neighborValid =
                    result.medianValidMask.ptr<unsigned char>(neighborRow);
                for(int neighborCol=firstCol;
                    neighborCol<=lastCol; ++neighborCol)
                {
                    if(neighborValid[neighborCol]==0)
                        continue;
                    maximumDifference = std::max(
                        maximumDifference,
                        std::fabs(filtered[col]-neighborFiltered[neighborCol]));
                }
            }
            const float threshold = std::max(
                mConfig.relativeThreshold*filtered[col],
                mConfig.absoluteThresholdMeters);
            if(maximumDifference>threshold)
            {
                edge[col] = 255;
                ++result.stats.rawGradientEdgePixels;
            }
        }
    }
    const std::chrono::steady_clock::time_point edgeEnd =
        std::chrono::steady_clock::now();

    std::vector<int> fragments(
        static_cast<std::size_t>(maximumInitialLabel)+1,0);
    std::vector<unsigned char> visited(depthMeters.total(),0);
    const int rowOffsets8[8] = {-1,-1,-1,0,0,1,1,1};
    const int colOffsets8[8] = {-1,0,1,-1,1,-1,0,1};
    const int rowOffsets4[4] = {-1,0,0,1};
    const int colOffsets4[4] = {0,-1,1,0};
    const int neighborCount = mConfig.connectivity==8 ? 8 : 4;
    const int *rowOffsets =
        mConfig.connectivity==8 ? rowOffsets8 : rowOffsets4;
    const int *colOffsets =
        mConfig.connectivity==8 ? colOffsets8 : colOffsets4;
    const std::chrono::steady_clock::time_point componentStart =
        std::chrono::steady_clock::now();
    int componentLabel = 0;
    std::deque<cv::Point> frontier;
    std::vector<std::size_t> componentAreas;
    for(int row=0; row<depthMeters.rows; ++row)
    {
        for(int col=0; col<depthMeters.cols; ++col)
        {
            const std::size_t linear =
                static_cast<std::size_t>(row)*depthMeters.cols+col;
            const int initialLabel = initialLabels.at<int>(row,col);
            const bool medianValid =
                result.medianValidMask.at<unsigned char>(row,col)!=0;
            const bool isEdge =
                result.rawGradientEdgeMask.at<unsigned char>(row,col)!=0;
            if(initialLabel<=0 || !medianValid)
                continue;
            if(isEdge)
            {
                result.splitCoreLabels.at<int>(row,col) = 0;
                result.splitBoundaryMask.at<unsigned char>(row,col) = 255;
                ++result.stats.splitBoundaryPixels;
                continue;
            }
            if(visited[linear])
                continue;

            ++componentLabel;
            ++fragments[initialLabel];
            std::size_t componentArea = 0;
            visited[linear] = 1;
            frontier.clear();
            frontier.push_back(cv::Point(col,row));
            while(!frontier.empty())
            {
                const cv::Point current = frontier.front();
                frontier.pop_front();
                result.splitCoreLabels.at<int>(current.y,current.x) =
                    componentLabel;
                result.splitValidMask.at<unsigned char>(current.y,current.x) =
                    255;
                ++componentArea;
                for(int neighbor=0; neighbor<neighborCount; ++neighbor)
                {
                    const int neighborRow = current.y+rowOffsets[neighbor];
                    const int neighborCol = current.x+colOffsets[neighbor];
                    if(neighborRow<0 || neighborRow>=depthMeters.rows ||
                       neighborCol<0 || neighborCol>=depthMeters.cols)
                        continue;
                    const std::size_t neighborLinear =
                        static_cast<std::size_t>(neighborRow)*depthMeters.cols+
                        neighborCol;
                    if(visited[neighborLinear] ||
                       initialLabels.at<int>(neighborRow,neighborCol)!=initialLabel ||
                       result.medianValidMask.at<unsigned char>(
                           neighborRow,neighborCol)==0 ||
                       result.rawGradientEdgeMask.at<unsigned char>(
                           neighborRow,neighborCol)!=0)
                    {
                        continue;
                    }
                    visited[neighborLinear] = 1;
                    frontier.push_back(cv::Point(neighborCol,neighborRow));
                }
            }
            componentAreas.push_back(componentArea);
        }
    }
    result.stats.splitComponentCount = componentLabel;
    result.stats.splitCorePixels = static_cast<std::size_t>(
        cv::countNonZero(result.splitValidMask));
    std::vector<int> nonzeroFragments;
    for(int initialLabel=1;
        initialLabel<=maximumInitialLabel; ++initialLabel)
    {
        if(initialLabel>=static_cast<int>(initialLabelPresent.size()) ||
           initialLabelPresent[initialLabel]==0)
            continue;
        if(fragments[initialLabel]>1)
            ++result.stats.splitInitialRegionCount;
        if(fragments[initialLabel]==0)
            ++result.stats.fullyConsumedInitialRegionCount;
        else
            nonzeroFragments.push_back(fragments[initialLabel]);
        result.stats.maximumFragmentation = std::max(
            result.stats.maximumFragmentation,fragments[initialLabel]);
    }
    if(!nonzeroFragments.empty())
    {
        const std::size_t middle = nonzeroFragments.size()/2;
        std::nth_element(nonzeroFragments.begin(),
                         nonzeroFragments.begin()+middle,
                         nonzeroFragments.end());
        result.stats.medianFragmentation = nonzeroFragments[middle];
        if(nonzeroFragments.size()%2==0)
        {
            const int lower = *std::max_element(
                nonzeroFragments.begin(),nonzeroFragments.begin()+middle);
            result.stats.medianFragmentation =
                0.5*static_cast<double>(lower+nonzeroFragments[middle]);
        }
    }
    for(std::size_t index=0; index<componentAreas.size(); ++index)
    {
        if(componentAreas[index]<mConfig.smallComponentAuditPixels)
        {
            ++result.stats.smallComponentCount;
            result.stats.smallComponentPixels += componentAreas[index];
        }
    }
    const std::chrono::steady_clock::time_point componentEnd =
        std::chrono::steady_clock::now();

    result.stats.available = true;
    result.stats.medianFilterMs = ElapsedMilliseconds(medianStart,medianEnd);
    result.stats.gradientEdgeMs = ElapsedMilliseconds(edgeStart,edgeEnd);
    result.stats.connectedComponentsMs =
        ElapsedMilliseconds(componentStart,componentEnd);
    result.stats.totalMs = ElapsedMilliseconds(
        totalStart,std::chrono::steady_clock::now());
    return result;
}

} // namespace ORB_SLAM2
