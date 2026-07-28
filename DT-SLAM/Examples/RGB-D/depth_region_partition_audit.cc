/**
* G2-3R0 standalone depth-region topology audit.
*
* This executable does not start SLAM and makes no dynamic/static decision.
*/

#include "GeometricDynamicDetector.h"

#include <algorithm>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <numeric>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <opencv2/imgcodecs/imgcodecs.hpp>
#include <opencv2/imgproc/imgproc.hpp>

namespace
{

struct Association
{
    double timestamp = 0.0;
    std::string rgbPath;
    std::string depthPath;
};

std::vector<Association> LoadAssociations(const std::string &path)
{
    std::ifstream input(path.c_str());
    if(!input.is_open())
        throw std::runtime_error("cannot open association file: "+path);

    std::vector<Association> associations;
    std::string line;
    while(std::getline(input,line))
    {
        if(line.empty() || line[0]=='#')
            continue;
        std::istringstream stream(line);
        double rgbTimestamp = 0.0;
        Association association;
        if(!(stream >> rgbTimestamp >> association.rgbPath >>
             association.timestamp >> association.depthPath))
        {
            throw std::runtime_error(
                "invalid association line: "+line);
        }
        associations.push_back(association);
    }
    return associations;
}

std::string JoinPath(const std::string &root,
                     const std::string &relative)
{
    if(!relative.empty() && relative[0]=='/')
        return relative;
    if(root.empty() || root[root.size()-1]=='/')
        return root+relative;
    return root+"/"+relative;
}

double Quantile(std::vector<double> values, const double quantile)
{
    if(values.empty())
        return 0.0;
    std::sort(values.begin(),values.end());
    const double position =
        quantile*static_cast<double>(values.size()-1);
    const std::size_t lower =
        static_cast<std::size_t>(position);
    const std::size_t upper =
        std::min(lower+1,values.size()-1);
    const double fraction = position-static_cast<double>(lower);
    return values[lower]*(1.0-fraction)+values[upper]*fraction;
}

double Mean(const std::vector<double> &values)
{
    if(values.empty())
        return 0.0;
    return std::accumulate(values.begin(),values.end(),0.0)/
           static_cast<double>(values.size());
}

cv::Vec3b RegionColor(const int label)
{
    const unsigned int value =
        static_cast<unsigned int>(label+1)*2654435761u;
    return cv::Vec3b(
        static_cast<unsigned char>(64+(value&127u)),
        static_cast<unsigned char>(64+((value>>8)&127u)),
        static_cast<unsigned char>(64+((value>>16)&127u)));
}

void SaveDebugOverlay(
    const std::string &datasetRoot,
    const Association &association,
    const std::size_t frameIndex,
    const ORB_SLAM2::GeometricRegionPartitionResult &result,
    const std::string &debugDirectory)
{
    cv::Mat rgb = cv::imread(
        JoinPath(datasetRoot,association.rgbPath),
        cv::IMREAD_COLOR);
    if(rgb.empty() || rgb.size()!=result.labels.size())
        return;

    cv::Mat colors(rgb.size(),CV_8UC3,cv::Scalar(0,0,0));
    for(int v=0; v<result.labels.rows; ++v)
    {
        const int *labels = result.labels.ptr<int>(v);
        cv::Vec3b *colorRow = colors.ptr<cv::Vec3b>(v);
        for(int u=0; u<result.labels.cols; ++u)
        {
            if(labels[u]>=0)
                colorRow[u] = RegionColor(labels[u]);
            else if(labels[u]==-2)
                colorRow[u] = cv::Vec3b(255,255,255);
        }
    }

    cv::Mat overlay;
    cv::addWeighted(rgb,0.55,colors,0.45,0.0,overlay);
    overlay.setTo(
        cv::Scalar(0,0,255),result.boundaryMask);
    std::ostringstream path;
    path << debugDirectory << "/frame_"
         << std::setw(6) << std::setfill('0') << frameIndex
         << "_region_overlay.png";
    cv::imwrite(path.str(),overlay);
}

} // namespace

int main(int argc, char **argv)
{
    if(argc<4 || argc>9)
    {
        std::cerr
            << "Usage: " << argv[0]
            << " dataset_root associations.txt output.csv"
            << " [max_frames=0] [relative_threshold=0.025]"
            << " [absolute_threshold_m=0.08]"
            << " [debug_dir] [debug_every=30]"
            << std::endl;
        return 1;
    }

    try
    {
        const std::string datasetRoot = argv[1];
        const std::string associationPath = argv[2];
        const std::string outputPath = argv[3];
        const std::size_t maximumFrames =
            argc>=5
                ? static_cast<std::size_t>(
                    std::max(0,std::atoi(argv[4])))
                : 0;
        const float relativeThreshold =
            argc>=6 ? std::atof(argv[5]) : 0.025f;
        const float absoluteThresholdMeters =
            argc>=7 ? std::atof(argv[6]) : 0.08f;
        const std::string debugDirectory =
            argc>=8 ? argv[7] : "";
        const std::size_t debugEvery =
            argc>=9
                ? static_cast<std::size_t>(
                    std::max(1,std::atoi(argv[8])))
                : 30;

        const std::vector<Association> associations =
            LoadAssociations(associationPath);
        const std::size_t frameCount =
            maximumFrames==0
                ? associations.size()
                : std::min(maximumFrames,associations.size());
        if(frameCount==0)
            throw std::runtime_error("association file has no frames");

        std::ofstream output(outputPath.c_str());
        if(!output.is_open())
            throw std::runtime_error("cannot open output CSV: "+outputPath);
        output
            << "frame_index,timestamp,depth_path,valid_depth_pixels,"
            << "boundary_pixels,assigned_region_pixels,region_count,"
            << "largest_region_pixels,top_five_region_pixels,"
            << "singleton_region_count,small_region_count,"
            << "boundary_valid_ratio,assigned_valid_ratio,"
            << "largest_region_valid_ratio,top_five_region_valid_ratio,"
            << "partition_ms\n";

        std::vector<double> regionCounts;
        std::vector<double> boundaryRatios;
        std::vector<double> largestRatios;
        std::vector<double> topFiveRatios;
        std::vector<double> runtimes;
        for(std::size_t frameIndex=0;
            frameIndex<frameCount; ++frameIndex)
        {
            const Association &association =
                associations[frameIndex];
            const std::string depthPath =
                JoinPath(datasetRoot,association.depthPath);
            const cv::Mat rawDepth =
                cv::imread(depthPath,cv::IMREAD_UNCHANGED);
            if(rawDepth.empty())
                throw std::runtime_error(
                    "cannot read depth image: "+depthPath);

            cv::Mat depthMeters;
            rawDepth.convertTo(depthMeters,CV_32FC1,1.0/5000.0);
            const ORB_SLAM2::GeometricRegionPartitionResult result =
                ORB_SLAM2::GeometricDynamicDetector::
                    PartitionDepthByDiscontinuity(
                        depthMeters,relativeThreshold,
                        absoluteThresholdMeters);
            const ORB_SLAM2::GeometricRegionPartitionStats &stats =
                result.stats;
            output << frameIndex << ','
                   << std::setprecision(15)
                   << association.timestamp << ','
                   << association.depthPath << ','
                   << stats.validDepthPixels << ','
                   << stats.boundaryPixels << ','
                   << stats.assignedRegionPixels << ','
                   << stats.regionCount << ','
                   << stats.largestRegionPixels << ','
                   << stats.topFiveRegionPixels << ','
                   << stats.singletonRegionCount << ','
                   << stats.smallRegionCount << ','
                   << std::setprecision(9)
                   << stats.boundaryValidRatio << ','
                   << stats.assignedValidRatio << ','
                   << stats.largestRegionValidRatio << ','
                   << stats.topFiveRegionValidRatio << ','
                   << stats.totalMs << '\n';

            regionCounts.push_back(stats.regionCount);
            boundaryRatios.push_back(stats.boundaryValidRatio);
            largestRatios.push_back(
                stats.largestRegionValidRatio);
            topFiveRatios.push_back(
                stats.topFiveRegionValidRatio);
            runtimes.push_back(stats.totalMs);

            if(!debugDirectory.empty() &&
               (frameIndex==0 ||
                frameIndex%debugEvery==0 ||
                frameIndex+1==frameCount))
            {
                SaveDebugOverlay(
                    datasetRoot,association,frameIndex,
                    result,debugDirectory);
            }
        }

        std::cout << "[Geometry G2-3R0] frames=" << frameCount
                  << " tau_rel=" << relativeThreshold
                  << " tau_abs_m=" << absoluteThresholdMeters
                  << " regions_mean=" << Mean(regionCounts)
                  << " regions_p50=" << Quantile(regionCounts,0.50)
                  << " boundary_ratio_mean=" << Mean(boundaryRatios)
                  << " largest_ratio_mean=" << Mean(largestRatios)
                  << " largest_ratio_p50="
                  << Quantile(largestRatios,0.50)
                  << " largest_ratio_p95="
                  << Quantile(largestRatios,0.95)
                  << " top5_ratio_mean=" << Mean(topFiveRatios)
                  << " partition_ms_mean=" << Mean(runtimes)
                  << " partition_ms_p50=" << Quantile(runtimes,0.50)
                  << " partition_ms_p95=" << Quantile(runtimes,0.95)
                  << " csv=" << outputPath
                  << std::endl;
    }
    catch(const std::exception &error)
    {
        std::cerr << "[Geometry G2-3R0] FAIL: "
                  << error.what() << std::endl;
        return 1;
    }
    return 0;
}
