/**
 * G2-4E development-only target-box/region partition audit.
 *
 * Reconstructs the exact C++ depth partition for independent RGB-only coarse
 * bboxes. It emits no motion score or SLAM state change.
 */

#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs.hpp>

#include "GeometricDynamicDetector.h"
#include "RGBDInputRectifier.h"

namespace
{

struct Association
{
    double rgbTimestamp = 0.0;
    std::string rgbPath;
    double depthTimestamp = 0.0;
    std::string depthPath;
};

struct Preannotation
{
    int sourceFrame = -1;
    std::string visibility;
    bool hasBox = false;
    cv::Rect box;
    std::string annotationSource;
    std::string reviewStatus;
};

std::string JoinPath(const std::string &root, const std::string &relative)
{
    if(!relative.empty() && relative[0]=='/')
        return relative;
    if(root.empty() || root[root.size()-1]=='/')
        return root+relative;
    return root+"/"+relative;
}

bool PathExists(const std::string &path)
{
    std::ifstream stream(path.c_str());
    return stream.good();
}

std::vector<std::string> ParseCSVLine(const std::string &line)
{
    std::vector<std::string> fields;
    std::string field;
    bool quoted = false;
    for(std::size_t index=0; index<line.size(); ++index)
    {
        const char value = line[index];
        if(value=='"')
        {
            if(quoted && index+1<line.size() && line[index+1]=='"')
            {
                field.push_back('"');
                ++index;
            }
            else
            {
                quoted = !quoted;
            }
        }
        else if(value==',' && !quoted)
        {
            fields.push_back(field);
            field.clear();
        }
        else
        {
            field.push_back(value);
        }
    }
    if(quoted)
        throw std::runtime_error("unterminated quoted CSV field");
    fields.push_back(field);
    return fields;
}

const std::string &RequiredField(
    const std::vector<std::string> &row,
    const std::map<std::string,std::size_t> &columns,
    const std::string &name)
{
    const std::map<std::string,std::size_t>::const_iterator found =
        columns.find(name);
    if(found==columns.end())
        throw std::runtime_error("missing CSV column: "+name);
    if(found->second>=row.size())
        throw std::runtime_error("CSV row shorter than header");
    return row[found->second];
}

std::vector<Association> LoadAssociations(const std::string &path)
{
    std::ifstream input(path.c_str());
    if(!input)
        throw std::runtime_error("cannot open association file: "+path);

    std::vector<Association> associations;
    std::string line;
    int lineNumber = 0;
    while(std::getline(input,line))
    {
        ++lineNumber;
        const std::string::size_type first =
            line.find_first_not_of(" \t\r");
        if(first==std::string::npos || line[first]=='#')
            continue;
        std::istringstream stream(line);
        Association association;
        if(!(stream >> association.rgbTimestamp >> association.rgbPath >>
             association.depthTimestamp >> association.depthPath))
        {
            throw std::runtime_error(
                "invalid association line "+std::to_string(lineNumber));
        }
        associations.push_back(association);
    }
    if(associations.empty())
        throw std::runtime_error("association file has no data rows");
    return associations;
}

std::vector<Preannotation> LoadPreannotations(
    const std::string &path,
    const std::string &exportName)
{
    std::ifstream input(path.c_str());
    if(!input)
        throw std::runtime_error("cannot open preannotation CSV: "+path);

    std::string line;
    if(!std::getline(input,line))
        throw std::runtime_error("preannotation CSV is empty");
    if(!line.empty() && line[line.size()-1]=='\r')
        line.erase(line.size()-1);
    const std::vector<std::string> header = ParseCSVLine(line);
    std::map<std::string,std::size_t> columns;
    for(std::size_t index=0; index<header.size(); ++index)
        columns[header[index]] = index;

    std::vector<Preannotation> annotations;
    int lineNumber = 1;
    while(std::getline(input,line))
    {
        ++lineNumber;
        if(!line.empty() && line[line.size()-1]=='\r')
            line.erase(line.size()-1);
        if(line.empty())
            continue;
        const std::vector<std::string> row = ParseCSVLine(line);
        if(RequiredField(row,columns,"export_name")!=exportName)
            continue;

        try
        {
            Preannotation annotation;
            annotation.sourceFrame = std::stoi(
                RequiredField(row,columns,"source_frame"));
            annotation.visibility =
                RequiredField(row,columns,"visibility");
            annotation.annotationSource =
                RequiredField(row,columns,"annotation_source");
            annotation.reviewStatus =
                RequiredField(row,columns,"review_status");
            annotation.hasBox = annotation.visibility!="absent";
            if(annotation.hasBox)
            {
                annotation.box = cv::Rect(
                    std::stoi(RequiredField(row,columns,"x")),
                    std::stoi(RequiredField(row,columns,"y")),
                    std::stoi(RequiredField(row,columns,"width")),
                    std::stoi(RequiredField(row,columns,"height")));
            }
            annotations.push_back(annotation);
        }
        catch(const std::exception &error)
        {
            throw std::runtime_error(
                "preannotation line "+std::to_string(lineNumber)+": "+
                error.what());
        }
    }
    if(annotations.empty())
        throw std::runtime_error(
            "no preannotations for export_name="+exportName);

    std::map<int,int> counts;
    for(const Preannotation &annotation : annotations)
        ++counts[annotation.sourceFrame];
    for(const std::pair<const int,int> &entry : counts)
    {
        if(entry.second!=1)
            throw std::runtime_error(
                "duplicate source_frame="+std::to_string(entry.first));
    }
    return annotations;
}

double ReadRequiredDouble(
    const cv::FileStorage &settings,
    const std::string &name)
{
    const cv::FileNode node = settings[name];
    if(node.empty())
        throw std::runtime_error("settings missing "+name);
    return static_cast<double>(node);
}

} // namespace

int main(int argc, char **argv)
{
    if(argc!=9)
    {
        std::cerr
            << "Usage: box_region_partition_audit settings.yaml dataset_root "
            << "associations.txt preannotations.csv export_name "
            << "frame_output.csv intersection_output.csv expected_count"
            << std::endl;
        return EXIT_FAILURE;
    }

    const std::string settingsPath = argv[1];
    const std::string datasetRoot = argv[2];
    const std::string associationPath = argv[3];
    const std::string preannotationPath = argv[4];
    const std::string exportName = argv[5];
    const std::string frameOutputPath = argv[6];
    const std::string intersectionOutputPath = argv[7];
    const int expectedCount = std::atoi(argv[8]);

    try
    {
        if(expectedCount<=0)
            throw std::invalid_argument("expected_count must be positive");
        if(PathExists(frameOutputPath) ||
           PathExists(intersectionOutputPath))
        {
            throw std::runtime_error(
                "refusing to overwrite an existing output CSV");
        }

        const std::vector<Association> associations =
            LoadAssociations(associationPath);
        const std::vector<Preannotation> annotations =
            LoadPreannotations(preannotationPath,exportName);
        if(static_cast<int>(annotations.size())!=expectedCount)
        {
            throw std::runtime_error(
                "preannotation count mismatch: expected "+
                std::to_string(expectedCount)+" got "+
                std::to_string(annotations.size()));
        }

        cv::FileStorage settings(settingsPath,cv::FileStorage::READ);
        if(!settings.isOpened())
            throw std::runtime_error("cannot open settings: "+settingsPath);
        ORB_SLAM2::RGBDInputRectifier rectifier;
        rectifier.Configure(settings);
        const double depthMapFactor =
            ReadRequiredDouble(settings,"DepthMapFactor");
        if(depthMapFactor<=0.0)
            throw std::runtime_error("DepthMapFactor must be positive");
        const float relativeThreshold = static_cast<float>(
            ReadRequiredDouble(
                settings,"Geometry.RegionPartitionRelativeThreshold"));
        const float absoluteThresholdMeters = static_cast<float>(
            ReadRequiredDouble(
                settings,"Geometry.RegionPartitionAbsoluteThresholdM"));

        std::ofstream frameOutput(frameOutputPath.c_str());
        std::ofstream intersectionOutput(intersectionOutputPath.c_str());
        if(!frameOutput || !intersectionOutput)
            throw std::runtime_error("cannot create output CSV");

        frameOutput
            << "export_name,source_frame,rgb_timestamp,depth_timestamp,"
            << "visibility,has_box,bbox_x,bbox_y,bbox_width,bbox_height,"
            << "bbox_area,bbox_invalid_pixels,bbox_boundary_pixels,"
            << "bbox_assigned_region_pixels,bbox_intersecting_region_count,"
            << "valid_depth_pixels,boundary_pixels,assigned_region_pixels,"
            << "partition_region_count,largest_region_pixels,"
            << "top_five_region_pixels,largest_region_valid_ratio,"
            << "top_five_region_valid_ratio,domain_signature,"
            << "annotation_source,review_status\n";
        intersectionOutput
            << "export_name,source_frame,rgb_timestamp,depth_timestamp,"
            << "visibility,region_label,bbox_intersection_pixels,bbox_area,"
            << "bbox_coverage,region_pixels,region_coverage\n";
        frameOutput << std::setprecision(15);
        intersectionOutput << std::setprecision(15);

        for(const Preannotation &annotation : annotations)
        {
            if(annotation.sourceFrame<0 ||
               annotation.sourceFrame>=
                   static_cast<int>(associations.size()))
            {
                throw std::runtime_error(
                    "source_frame outside association range: "+
                    std::to_string(annotation.sourceFrame));
            }
            const Association &association =
                associations[annotation.sourceFrame];
            const cv::Mat rawRGB = cv::imread(
                JoinPath(datasetRoot,association.rgbPath),
                cv::IMREAD_UNCHANGED);
            const cv::Mat rawDepth = cv::imread(
                JoinPath(datasetRoot,association.depthPath),
                cv::IMREAD_UNCHANGED);
            if(rawRGB.empty() || rawDepth.empty())
            {
                throw std::runtime_error(
                    "cannot read RGB-D for frame "+
                    std::to_string(annotation.sourceFrame));
            }

            cv::Mat rectifiedRGB;
            cv::Mat rectifiedDepth;
            rectifier.RectifyRGBD(
                rawRGB,rawDepth,rectifiedRGB,rectifiedDepth);
            cv::Mat depthMeters;
            rectifiedDepth.convertTo(
                depthMeters,CV_32FC1,1.0/depthMapFactor);

            const ORB_SLAM2::GeometricRegionPartitionResult partition =
                ORB_SLAM2::GeometricDynamicDetector::
                    PartitionDepthByDiscontinuity(
                        depthMeters,relativeThreshold,
                        absoluteThresholdMeters);

            int invalidPixels = 0;
            int boundaryPixels = 0;
            int assignedPixels = 0;
            std::map<int,int> intersections;
            int boxArea = 0;
            if(annotation.hasBox)
            {
                const cv::Rect imageBounds(
                    0,0,depthMeters.cols,depthMeters.rows);
                if(annotation.box.area()<=0 ||
                   (annotation.box & imageBounds)!=annotation.box)
                {
                    throw std::runtime_error(
                        "bbox outside rectified image at frame "+
                        std::to_string(annotation.sourceFrame));
                }
                boxArea = annotation.box.area();
                for(int v=annotation.box.y;
                    v<annotation.box.y+annotation.box.height; ++v)
                {
                    const int *labels = partition.labels.ptr<int>(v);
                    for(int u=annotation.box.x;
                        u<annotation.box.x+annotation.box.width; ++u)
                    {
                        if(labels[u]==-1)
                            ++invalidPixels;
                        else if(labels[u]==-2)
                            ++boundaryPixels;
                        else
                        {
                            ++assignedPixels;
                            ++intersections[labels[u]];
                        }
                    }
                }
            }

            const ORB_SLAM2::GeometricRegionPartitionStats &stats =
                partition.stats;
            frameOutput
                << exportName << ","
                << annotation.sourceFrame << ","
                << association.rgbTimestamp << ","
                << association.depthTimestamp << ","
                << annotation.visibility << ","
                << (annotation.hasBox ? 1 : 0) << ",";
            if(annotation.hasBox)
            {
                frameOutput
                    << annotation.box.x << ","
                    << annotation.box.y << ","
                    << annotation.box.width << ","
                    << annotation.box.height << ",";
            }
            else
            {
                frameOutput << ",,,,";
            }
            frameOutput
                << boxArea << ","
                << invalidPixels << ","
                << boundaryPixels << ","
                << assignedPixels << ","
                << intersections.size() << ","
                << stats.validDepthPixels << ","
                << stats.boundaryPixels << ","
                << stats.assignedRegionPixels << ","
                << stats.regionCount << ","
                << stats.largestRegionPixels << ","
                << stats.topFiveRegionPixels << ","
                << stats.largestRegionValidRatio << ","
                << stats.topFiveRegionValidRatio << ",\""
                << rectifier.DomainSignature() << "\","
                << annotation.annotationSource << ","
                << annotation.reviewStatus << "\n";

            for(const std::pair<const int,int> &entry : intersections)
            {
                const int label = entry.first;
                if(label<0 ||
                   label>=static_cast<int>(partition.regionSizes.size()))
                {
                    throw std::logic_error("invalid partition label");
                }
                const std::size_t regionPixels =
                    partition.regionSizes[label];
                intersectionOutput
                    << exportName << ","
                    << annotation.sourceFrame << ","
                    << association.rgbTimestamp << ","
                    << association.depthTimestamp << ","
                    << annotation.visibility << ","
                    << label << ","
                    << entry.second << ","
                    << boxArea << ","
                    << static_cast<double>(entry.second)/
                           static_cast<double>(boxArea) << ","
                    << regionPixels << ","
                    << static_cast<double>(entry.second)/
                           static_cast<double>(regionPixels) << "\n";
            }
        }

        std::cout
            << "[G2-4E] export_name=" << exportName
            << " candidates=" << annotations.size()
            << " dynamic_decision=none"
            << " direct_slam_state_mutation=none" << std::endl;
    }
    catch(const std::exception &error)
    {
        std::cerr << "[G2-4E] ERROR: " << error.what() << std::endl;
        return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
}
