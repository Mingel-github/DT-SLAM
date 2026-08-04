#include "SInStylePlaneEdgeRegionSplitter.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <deque>
#include <limits>
#include <set>
#include <stdexcept>
#include <vector>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/rgbd/depth.hpp>

namespace ORB_SLAM2
{

namespace
{

double Milliseconds(
    const std::chrono::steady_clock::time_point &start,
    const std::chrono::steady_clock::time_point &end)
{
    return std::chrono::duration_cast<
        std::chrono::duration<double,std::milli> >(end-start).count();
}

bool IsValidDepth(const float value, const float maximumDepthMeters)
{
    return std::isfinite(value) && value>0.0f &&
           value<maximumDepthMeters;
}

void ValidateImageInputs(const cv::Mat &depthMeters,
                         const cv::Mat &initialLabels,
                         const cv::Mat &rawGradientEdgeMask)
{
    if(depthMeters.empty() || depthMeters.type()!=CV_32FC1)
        throw std::invalid_argument(
            "SIn plane-edge split requires CV_32FC1 metric depth");
    if(initialLabels.empty() || initialLabels.type()!=CV_32SC1 ||
       initialLabels.size()!=depthMeters.size())
    {
        throw std::invalid_argument(
            "SIn plane-edge split requires aligned CV_32SC1 initial labels");
    }
    if(rawGradientEdgeMask.empty() ||
       rawGradientEdgeMask.type()!=CV_8UC1 ||
       rawGradientEdgeMask.size()!=depthMeters.size())
    {
        throw std::invalid_argument(
            "SIn plane-edge split requires aligned CV_8UC1 gradient edge");
    }
}

cv::Mat BuildDiskKernel(const int radius)
{
    const int size = 2*radius+1;
    return cv::getStructuringElement(
        cv::MORPH_ELLIPSE,cv::Size(size,size));
}

} // namespace

SInStylePlaneEdgeRegionSplitter::SInStylePlaneEdgeRegionSplitter()
{
}

void SInStylePlaneEdgeRegionSplitter::Configure(
    const SInStylePlaneEdgeSplitConfig &config,
    const cv::Mat &cameraMatrix)
{
    if(config.maximumDepthMeters<=0.0f ||
       !std::isfinite(config.maximumDepthMeters))
        throw std::invalid_argument(
            "SIn plane-edge maximum depth must be finite and positive");
    if(config.blockSize<=0 || config.minimumPlanePixels<=0 ||
       config.distanceThresholdMeters<=0.0 ||
       !std::isfinite(config.distanceThresholdMeters))
        throw std::invalid_argument("invalid OpenCV RgbdPlane parameters");
    if(config.sensorErrorA<0.0 || config.sensorErrorB<0.0 ||
       config.sensorErrorC<0.0 ||
       !std::isfinite(config.sensorErrorA) ||
       !std::isfinite(config.sensorErrorB) ||
       !std::isfinite(config.sensorErrorC))
        throw std::invalid_argument("invalid RGB-D sensor error model");
    if(config.endpointRadius<0 ||
       config.endpointMaximumSupportExclusive<=0 ||
       config.endpointAssociationRadius<0 ||
       config.minimumEndpointCountExclusive<0 ||
       (config.connectivity!=4 && config.connectivity!=8))
        throw std::invalid_argument("invalid SIn plane-edge topology config");
    if(cameraMatrix.empty() ||
       (cameraMatrix.type()!=CV_32FC1 && cameraMatrix.type()!=CV_64FC1) ||
       cameraMatrix.rows!=3 || cameraMatrix.cols!=3)
        throw std::invalid_argument(
            "SIn plane-edge split requires a 3x3 camera matrix");

    mConfig = config;
    cameraMatrix.convertTo(mCameraMatrix,CV_32FC1);
}

SInStylePlaneEdgeSplitResult SInStylePlaneEdgeRegionSplitter::Compute(
    const cv::Mat &depthMeters,
    const cv::Mat &initialLabels,
    const cv::Mat &rawGradientEdgeMask) const
{
    if(!mConfig.enabled)
        return SInStylePlaneEdgeSplitResult();
    ValidateImageInputs(depthMeters,initialLabels,rawGradientEdgeMask);
    if(mCameraMatrix.empty())
        throw std::logic_error("SIn plane-edge splitter is not configured");

    const std::chrono::steady_clock::time_point start =
        std::chrono::steady_clock::now();
    cv::Mat sanitizedDepth = depthMeters.clone();
    for(int row=0; row<sanitizedDepth.rows; ++row)
    {
        float *values = sanitizedDepth.ptr<float>(row);
        for(int col=0; col<sanitizedDepth.cols; ++col)
        {
            if(!IsValidDepth(values[col],mConfig.maximumDepthMeters))
                values[col] = std::numeric_limits<float>::quiet_NaN();
        }
    }

    cv::Mat points3d;
    cv::rgbd::depthTo3d(sanitizedDepth,mCameraMatrix,points3d);
    cv::Ptr<cv::rgbd::RgbdPlane> planeDetector =
        cv::rgbd::RgbdPlane::create(
            cv::rgbd::RgbdPlane::RGBD_PLANE_METHOD_DEFAULT,
            mConfig.blockSize,mConfig.minimumPlanePixels,
            mConfig.distanceThresholdMeters,mConfig.sensorErrorA,
            mConfig.sensorErrorB,mConfig.sensorErrorC);
    cv::Mat labels8;
    cv::Mat coefficients;
    (*planeDetector)(points3d,labels8,coefficients);

    cv::Mat planeLabels(depthMeters.size(),CV_32SC1,cv::Scalar(-1));
    if(!labels8.empty())
    {
        if(labels8.type()!=CV_8UC1 || labels8.size()!=depthMeters.size())
            throw std::runtime_error("OpenCV RgbdPlane returned invalid labels");
        for(int row=0; row<labels8.rows; ++row)
        {
            const unsigned char *source = labels8.ptr<unsigned char>(row);
            int *destination = planeLabels.ptr<int>(row);
            for(int col=0; col<labels8.cols; ++col)
            {
                if(source[col]!=255 &&
                   IsValidDepth(depthMeters.at<float>(row,col),
                                mConfig.maximumDepthMeters))
                    destination[col] = static_cast<int>(source[col]);
            }
        }
    }
    const std::chrono::steady_clock::time_point planeEnd =
        std::chrono::steady_clock::now();

    SInStylePlaneEdgeSplitResult result = ComputeFromPlaneLabels(
        depthMeters,initialLabels,rawGradientEdgeMask,planeLabels);
    result.stats.planeExtractionMs = Milliseconds(start,planeEnd);
    result.stats.totalMs += result.stats.planeExtractionMs;
    return result;
}

SInStylePlaneEdgeSplitResult
SInStylePlaneEdgeRegionSplitter::ComputeFromPlaneLabels(
    const cv::Mat &depthMeters,
    const cv::Mat &initialLabels,
    const cv::Mat &rawGradientEdgeMask,
    const cv::Mat &planeLabels) const
{
    if(!mConfig.enabled)
        return SInStylePlaneEdgeSplitResult();
    ValidateImageInputs(depthMeters,initialLabels,rawGradientEdgeMask);
    if(planeLabels.empty() || planeLabels.type()!=CV_32SC1 ||
       planeLabels.size()!=depthMeters.size())
        throw std::invalid_argument(
            "SIn plane-edge split requires aligned CV_32SC1 plane labels");
    const std::chrono::steady_clock::time_point start =
        std::chrono::steady_clock::now();
    SInStylePlaneEdgeSplitResult result;
    result.planeLabels = planeLabels.clone();
    result.rawPlaneBoundaryMask = cv::Mat(
        depthMeters.size(),CV_8UC1,cv::Scalar(0));
    result.gradientEndpointMask = cv::Mat(
        depthMeters.size(),CV_8UC1,cv::Scalar(0));
    result.planeCandidateBoundaryMask = cv::Mat(
        depthMeters.size(),CV_8UC1,cv::Scalar(0));
    result.retainedPlaneBoundaryMask = cv::Mat(
        depthMeters.size(),CV_8UC1,cv::Scalar(0));
    result.combinedEdgeMask = rawGradientEdgeMask.clone();
    result.combinedCoreLabels = cv::Mat(
        depthMeters.size(),CV_32SC1,cv::Scalar(-1));
    result.combinedValidMask = cv::Mat(
        depthMeters.size(),CV_8UC1,cv::Scalar(0));
    result.stats.enabled = true;
    result.stats.available = true;
    result.stats.dynamicStateAvailable = false;
    result.stats.imagePixels = depthMeters.total();

    std::set<int> planeIds;
    std::set<int> initialIds;
    for(int row=0; row<depthMeters.rows; ++row)
    {
        for(int col=0; col<depthMeters.cols; ++col)
        {
            if(IsValidDepth(depthMeters.at<float>(row,col),
                            mConfig.maximumDepthMeters))
                ++result.stats.inputDepthValidPixels;
            const int initialLabel = initialLabels.at<int>(row,col);
            if(initialLabel>0)
            {
                ++result.stats.initialRegionPixels;
                initialIds.insert(initialLabel);
            }
            const int planeLabel = planeLabels.at<int>(row,col);
            if(planeLabel>=0)
            {
                ++result.stats.planePixels;
                planeIds.insert(planeLabel);
            }
        }
    }
    result.stats.planeCount = static_cast<int>(planeIds.size());
    result.stats.initialRegionCount = static_cast<int>(initialIds.size());

    const int neighborRows[2] = {0,1};
    const int neighborCols[2] = {1,0};
    for(int row=0; row<depthMeters.rows; ++row)
    {
        for(int col=0; col<depthMeters.cols; ++col)
        {
            const int currentPlane = planeLabels.at<int>(row,col);
            for(int neighbor=0; neighbor<2; ++neighbor)
            {
                const int nextRow = row+neighborRows[neighbor];
                const int nextCol = col+neighborCols[neighbor];
                if(nextRow>=depthMeters.rows || nextCol>=depthMeters.cols)
                    continue;
                if(!IsValidDepth(depthMeters.at<float>(row,col),
                                 mConfig.maximumDepthMeters) ||
                   !IsValidDepth(depthMeters.at<float>(nextRow,nextCol),
                                 mConfig.maximumDepthMeters))
                    continue;
                const int nextPlane = planeLabels.at<int>(nextRow,nextCol);
                if(currentPlane==nextPlane ||
                   (currentPlane<0 && nextPlane<0))
                    continue;
                result.rawPlaneBoundaryMask.at<unsigned char>(row,col) = 255;
                result.rawPlaneBoundaryMask.at<unsigned char>(
                    nextRow,nextCol) = 255;
            }
        }
    }
    result.stats.rawPlaneBoundaryPixels = static_cast<std::size_t>(
        cv::countNonZero(result.rawPlaneBoundaryMask));
    cv::Mat gradientOverlap;
    cv::bitwise_and(result.rawPlaneBoundaryMask,rawGradientEdgeMask,
                    gradientOverlap);
    result.stats.gradientOverlapPixels = static_cast<std::size_t>(
        cv::countNonZero(gradientOverlap));
    cv::Mat inverseGradient;
    cv::bitwise_not(rawGradientEdgeMask,inverseGradient);
    cv::bitwise_and(result.rawPlaneBoundaryMask,inverseGradient,
                    result.planeCandidateBoundaryMask);
    result.stats.planeCandidateBoundaryPixels = static_cast<std::size_t>(
        cv::countNonZero(result.planeCandidateBoundaryMask));
    const std::chrono::steady_clock::time_point boundaryEnd =
        std::chrono::steady_clock::now();
    result.stats.boundaryBuildMs = Milliseconds(start,boundaryEnd);

    const cv::Mat endpointKernel = BuildDiskKernel(mConfig.endpointRadius);
    for(int row=0; row<rawGradientEdgeMask.rows; ++row)
    {
        for(int col=0; col<rawGradientEdgeMask.cols; ++col)
        {
            if(rawGradientEdgeMask.at<unsigned char>(row,col)==0)
                continue;
            int support = 0;
            for(int kernelRow=0; kernelRow<endpointKernel.rows; ++kernelRow)
            {
                const int imageRow = row+kernelRow-mConfig.endpointRadius;
                if(imageRow<0 || imageRow>=rawGradientEdgeMask.rows)
                    continue;
                const unsigned char *kernelValues =
                    endpointKernel.ptr<unsigned char>(kernelRow);
                for(int kernelCol=0; kernelCol<endpointKernel.cols;
                    ++kernelCol)
                {
                    if(kernelValues[kernelCol]==0)
                        continue;
                    const int imageCol =
                        col+kernelCol-mConfig.endpointRadius;
                    if(imageCol<0 || imageCol>=rawGradientEdgeMask.cols)
                        continue;
                    if(rawGradientEdgeMask.at<unsigned char>(
                           imageRow,imageCol)!=0)
                        ++support;
                }
            }
            if(support<mConfig.endpointMaximumSupportExclusive)
                result.gradientEndpointMask.at<unsigned char>(row,col) = 255;
        }
    }
    result.stats.gradientEndpointPixels = static_cast<std::size_t>(
        cv::countNonZero(result.gradientEndpointMask));

    cv::Mat segmentLabels;
    const int segmentCountWithBackground = cv::connectedComponents(
        result.planeCandidateBoundaryMask,segmentLabels,
        mConfig.connectivity,CV_32S);
    const int segmentCount = std::max(0,segmentCountWithBackground-1);
    result.stats.planeBoundarySegmentCount = segmentCount;
    std::vector<int> endpointSupport(
        static_cast<std::size_t>(segmentCount)+1,0);
    const cv::Mat associationKernel =
        BuildDiskKernel(mConfig.endpointAssociationRadius);
    for(int row=0; row<result.gradientEndpointMask.rows; ++row)
    {
        for(int col=0; col<result.gradientEndpointMask.cols; ++col)
        {
            if(result.gradientEndpointMask.at<unsigned char>(row,col)==0)
                continue;
            std::set<int> touchedSegments;
            for(int kernelRow=0; kernelRow<associationKernel.rows;
                ++kernelRow)
            {
                const int imageRow =
                    row+kernelRow-mConfig.endpointAssociationRadius;
                if(imageRow<0 || imageRow>=segmentLabels.rows)
                    continue;
                const unsigned char *kernelValues =
                    associationKernel.ptr<unsigned char>(kernelRow);
                for(int kernelCol=0; kernelCol<associationKernel.cols;
                    ++kernelCol)
                {
                    if(kernelValues[kernelCol]==0)
                        continue;
                    const int imageCol =
                        col+kernelCol-mConfig.endpointAssociationRadius;
                    if(imageCol<0 || imageCol>=segmentLabels.cols)
                        continue;
                    const int segment =
                        segmentLabels.at<int>(imageRow,imageCol);
                    if(segment>0)
                        touchedSegments.insert(segment);
                }
            }
            for(std::set<int>::const_iterator iterator=
                    touchedSegments.begin();
                iterator!=touchedSegments.end(); ++iterator)
            {
                ++endpointSupport[*iterator];
            }
        }
    }
    std::vector<unsigned char> retain(
        static_cast<std::size_t>(segmentCount)+1,0);
    for(int segment=1; segment<=segmentCount; ++segment)
    {
        if(endpointSupport[segment]>
           mConfig.minimumEndpointCountExclusive)
        {
            retain[segment] = 1;
            ++result.stats.retainedPlaneBoundarySegmentCount;
        }
        else
        {
            ++result.stats.unsupportedPlaneBoundarySegmentCount;
        }
    }
    for(int row=0; row<segmentLabels.rows; ++row)
    {
        const int *labels = segmentLabels.ptr<int>(row);
        unsigned char *retained =
            result.retainedPlaneBoundaryMask.ptr<unsigned char>(row);
        for(int col=0; col<segmentLabels.cols; ++col)
        {
            if(labels[col]>0 && retain[labels[col]])
                retained[col] = 255;
        }
    }
    result.stats.retainedPlaneBoundaryPixels = static_cast<std::size_t>(
        cv::countNonZero(result.retainedPlaneBoundaryMask));
    cv::bitwise_or(rawGradientEdgeMask,result.retainedPlaneBoundaryMask,
                   result.combinedEdgeMask);
    result.stats.combinedEdgePixels = static_cast<std::size_t>(
        cv::countNonZero(result.combinedEdgeMask));
    const std::chrono::steady_clock::time_point endpointEnd =
        std::chrono::steady_clock::now();
    result.stats.endpointFilterMs =
        Milliseconds(boundaryEnd,endpointEnd);

    int maximumInitialLabel = 0;
    for(std::set<int>::const_iterator iterator=initialIds.begin();
        iterator!=initialIds.end(); ++iterator)
        maximumInitialLabel = std::max(maximumInitialLabel,*iterator);
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
    int componentLabel = 0;
    std::deque<cv::Point> frontier;
    for(int row=0; row<depthMeters.rows; ++row)
    {
        for(int col=0; col<depthMeters.cols; ++col)
        {
            const std::size_t linear =
                static_cast<std::size_t>(row)*depthMeters.cols+col;
            const int initialLabel = initialLabels.at<int>(row,col);
            if(initialLabel<=0 ||
               !IsValidDepth(depthMeters.at<float>(row,col),
                             mConfig.maximumDepthMeters))
                continue;
            if(result.combinedEdgeMask.at<unsigned char>(row,col)!=0)
            {
                result.combinedCoreLabels.at<int>(row,col) = 0;
                continue;
            }
            if(visited[linear])
                continue;
            ++componentLabel;
            ++fragments[initialLabel];
            visited[linear] = 1;
            frontier.clear();
            frontier.push_back(cv::Point(col,row));
            while(!frontier.empty())
            {
                const cv::Point current = frontier.front();
                frontier.pop_front();
                result.combinedCoreLabels.at<int>(current.y,current.x) =
                    componentLabel;
                result.combinedValidMask.at<unsigned char>(
                    current.y,current.x) = 255;
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
                       initialLabels.at<int>(neighborRow,neighborCol)!=
                           initialLabel ||
                       !IsValidDepth(depthMeters.at<float>(
                           neighborRow,neighborCol),
                           mConfig.maximumDepthMeters) ||
                       result.combinedEdgeMask.at<unsigned char>(
                           neighborRow,neighborCol)!=0)
                        continue;
                    visited[neighborLinear] = 1;
                    frontier.push_back(cv::Point(neighborCol,neighborRow));
                }
            }
        }
    }
    result.stats.combinedComponentCount = componentLabel;
    result.stats.combinedCorePixels = static_cast<std::size_t>(
        cv::countNonZero(result.combinedValidMask));
    for(std::set<int>::const_iterator iterator=initialIds.begin();
        iterator!=initialIds.end(); ++iterator)
    {
        const int fragmentCount = fragments[*iterator];
        if(fragmentCount>1)
            ++result.stats.splitInitialRegionCount;
        if(fragmentCount==0)
            ++result.stats.fullyConsumedInitialRegionCount;
        result.stats.maximumFragmentation = std::max(
            result.stats.maximumFragmentation,fragmentCount);
    }
    const std::chrono::steady_clock::time_point componentEnd =
        std::chrono::steady_clock::now();
    result.stats.connectedComponentsMs =
        Milliseconds(endpointEnd,componentEnd);
    result.stats.totalMs = Milliseconds(start,componentEnd);
    return result;
}

} // namespace ORB_SLAM2
