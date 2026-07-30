#include <cerrno>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <sys/stat.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include "RGBDInputRectifier.h"
#include "YOLOSegment.h"

namespace
{

struct Candidate
{
    std::string selectionRole;
    int selectionRank;
    int sourceFrame;
    std::string rgbTimestamp;
    std::string rgbRelative;
    std::string depthTimestamp;
    std::string depthRelative;
    std::string poseTimestampSource;
    std::string poseTimestamp;
};

std::string JoinPath(const std::string &left, const std::string &right)
{
    if(left.empty())
        return right;
    if(left[left.size()-1]=='/')
        return left+right;
    return left+"/"+right;
}

bool PathExists(const std::string &path)
{
    struct stat info;
    return stat(path.c_str(),&info)==0;
}

void CreateDirectory(const std::string &path)
{
    if(mkdir(path.c_str(),0755)!=0)
        throw std::runtime_error(
            "Failed to create directory "+path+": errno="+
            std::to_string(errno));
}

std::vector<std::string> ParseCSVLine(const std::string &line)
{
    std::vector<std::string> fields;
    std::string field;
    bool quoted = false;
    for(size_t i=0;i<line.size();++i)
    {
        const char value = line[i];
        if(value=='"')
        {
            if(quoted && i+1<line.size() && line[i+1]=='"')
            {
                field.push_back('"');
                ++i;
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
        throw std::runtime_error("Unterminated quoted CSV field");
    fields.push_back(field);
    return fields;
}

std::string CSVField(const std::string &value)
{
    if(value.find_first_of(",\"\n\r")==std::string::npos)
        return value;
    std::string escaped = "\"";
    for(const char character : value)
    {
        if(character=='"')
            escaped += "\"\"";
        else
            escaped.push_back(character);
    }
    escaped.push_back('"');
    return escaped;
}

const std::string &RequiredField(
    const std::vector<std::string> &row,
    const std::map<std::string,size_t> &columns,
    const std::string &name)
{
    const std::map<std::string,size_t>::const_iterator found =
        columns.find(name);
    if(found==columns.end())
        throw std::runtime_error("Missing required CSV column: "+name);
    if(found->second>=row.size())
        throw std::runtime_error("CSV row is shorter than its header");
    return row[found->second];
}

std::vector<Candidate> LoadCandidates(const std::string &filename)
{
    std::ifstream input(filename.c_str());
    if(!input)
        throw std::runtime_error("Failed to open candidate CSV: "+filename);

    std::string line;
    if(!std::getline(input,line))
        throw std::runtime_error("Candidate CSV is empty: "+filename);
    if(!line.empty() && line[line.size()-1]=='\r')
        line.erase(line.size()-1);

    const std::vector<std::string> header = ParseCSVLine(line);
    std::map<std::string,size_t> columns;
    for(size_t index=0;index<header.size();++index)
        columns[header[index]] = index;

    std::vector<Candidate> candidates;
    int lineNumber = 1;
    while(std::getline(input,line))
    {
        ++lineNumber;
        if(!line.empty() && line[line.size()-1]=='\r')
            line.erase(line.size()-1);
        if(line.empty())
            continue;

        try
        {
            const std::vector<std::string> row = ParseCSVLine(line);
            Candidate candidate;
            candidate.selectionRole =
                RequiredField(row,columns,"selection_role");
            candidate.selectionRank = std::stoi(
                RequiredField(row,columns,"selection_rank"));
            candidate.sourceFrame = std::stoi(
                RequiredField(row,columns,"frame"));
            candidate.rgbTimestamp =
                RequiredField(row,columns,"rgb_timestamp");
            candidate.rgbRelative =
                RequiredField(row,columns,"rgb_relative");
            candidate.depthTimestamp =
                RequiredField(row,columns,"depth_timestamp");
            candidate.depthRelative =
                RequiredField(row,columns,"depth_relative");
            candidate.poseTimestampSource =
                RequiredField(row,columns,"pose_timestamp_source");
            candidate.poseTimestamp =
                RequiredField(row,columns,"pose_timestamp");
            candidates.push_back(candidate);
        }
        catch(const std::exception &error)
        {
            throw std::runtime_error(
                "Candidate CSV line "+std::to_string(lineNumber)+": "+
                error.what());
        }
    }

    if(candidates.empty())
        throw std::runtime_error("Candidate CSV has no data rows");
    return candidates;
}

std::string FrameStem(const int sourceFrame)
{
    std::ostringstream stream;
    stream << "frame_" << std::setfill('0') << std::setw(6)
           << sourceFrame;
    return stream.str();
}

cv::Mat MakeOverlay(
    const cv::Mat &rgb,
    const cv::Mat &mask,
    const std::vector<ORB_SLAM2::Detection> &detections)
{
    cv::Mat colored = rgb.clone();
    colored.setTo(cv::Scalar(0,0,255),mask);
    cv::Mat overlay;
    cv::addWeighted(rgb,0.65,colored,0.35,0.0,overlay);

    for(size_t index=0;index<detections.size();++index)
    {
        const ORB_SLAM2::Detection &detection = detections[index];
        cv::rectangle(overlay,detection.box,cv::Scalar(0,255,0),2);
        std::ostringstream label;
        label << "person " << std::fixed << std::setprecision(2)
              << detection.confidence;
        const int labelY = std::max(15,detection.box.y-4);
        cv::putText(
            overlay,label.str(),cv::Point(detection.box.x,labelY),
            cv::FONT_HERSHEY_SIMPLEX,0.45,cv::Scalar(0,255,0),1,
            cv::LINE_AA);
    }
    return overlay;
}

void RequireWrite(const std::string &filename, const cv::Mat &image)
{
    if(!cv::imwrite(filename,image))
        throw std::runtime_error("Failed to write image: "+filename);
}

} // namespace

int main(int argc, char **argv)
{
    if(argc!=6)
    {
        std::cerr
            << "Usage: semantic_review_export settings.yaml model.onnx "
            << "dataset_root selected_frames.csv output_dir" << std::endl;
        return EXIT_FAILURE;
    }

    const std::string settingsPath = argv[1];
    const std::string modelPath = argv[2];
    const std::string datasetRoot = argv[3];
    const std::string candidatesPath = argv[4];
    const std::string outputDirectory = argv[5];
    const char *manifestOnlyEnvironment =
        std::getenv("DT_SLAM_SEMANTIC_REVIEW_MANIFEST_ONLY");
    const bool manifestOnly =
        manifestOnlyEnvironment &&
        std::string(manifestOnlyEnvironment)=="1";

    try
    {
        if(PathExists(outputDirectory))
            throw std::runtime_error(
                "Refusing to overwrite existing output directory: "+
                outputDirectory);
        CreateDirectory(outputDirectory);
        const std::string rgbDirectory = JoinPath(outputDirectory,"rgb");
        const std::string maskDirectory = JoinPath(outputDirectory,"mask");
        const std::string overlayDirectory =
            JoinPath(outputDirectory,"overlay");
        if(!manifestOnly)
        {
            CreateDirectory(rgbDirectory);
            CreateDirectory(maskDirectory);
            CreateDirectory(overlayDirectory);
        }

        const std::vector<Candidate> candidates =
            LoadCandidates(candidatesPath);

        cv::FileStorage settings(settingsPath,cv::FileStorage::READ);
        if(!settings.isOpened())
            throw std::runtime_error("Failed to open settings: "+settingsPath);
        ORB_SLAM2::RGBDInputRectifier rectifier;
        rectifier.Configure(settings);

        std::ofstream manifest(
            JoinPath(outputDirectory,"manifest.csv").c_str());
        std::ofstream detectionsOutput(
            JoinPath(outputDirectory,"detections.csv").c_str());
        if(!manifest || !detectionsOutput)
            throw std::runtime_error("Failed to create output CSV files");

        manifest
            << "semantic_seq,returned_mask_seq,selection_role,selection_rank,"
            << "source_frame,rgb_timestamp,rgb_relative,depth_timestamp,"
            << "depth_relative,pose_timestamp_source,pose_timestamp,"
            << "domain_signature,width,height,mask_type,mask_nonzero_pixels,"
            << "mask_ratio,mask_intermediate_pixels,mask_intermediate_ratio,"
            << "mask_min,mask_max,detection_count,rgb_output,mask_output,"
            << "overlay_output\n";
        detectionsOutput
            << "semantic_seq,source_frame,detection_index,x,y,width,height,"
            << "confidence\n";

        ORB_SLAM2::YOLOSegment yolo(modelPath,0.5f,0.45f);
        yolo.Start();

        size_t totalMaskPixels = 0;
        size_t totalIntermediatePixels = 0;
        size_t totalDetections = 0;
        for(size_t index=0;index<candidates.size();++index)
        {
            const Candidate &candidate = candidates[index];
            const std::string sourceRGB =
                JoinPath(datasetRoot,candidate.rgbRelative);
            const cv::Mat rawRGB =
                cv::imread(sourceRGB,cv::IMREAD_UNCHANGED);
            if(rawRGB.empty())
                throw std::runtime_error("Failed to read RGB: "+sourceRGB);

            cv::Mat rectifiedRGB;
            rectifier.RectifyRGB(rawRGB,rectifiedRGB);
            if(rectifiedRGB.empty())
                throw std::runtime_error("Rectification returned empty RGB");

            const int semanticSequence = static_cast<int>(index);
            yolo.PushFrame(rectifiedRGB,semanticSequence);
            cv::Mat mask;
            if(!yolo.WaitForMask(semanticSequence,mask))
                throw std::runtime_error(
                    "Timed out waiting for semantic sequence "+
                    std::to_string(semanticSequence));
            const int returnedSequence = yolo.GetMaskSeq();
            if(returnedSequence!=semanticSequence)
                throw std::runtime_error(
                    "Semantic sequence mismatch: requested "+
                    std::to_string(semanticSequence)+" received "+
                    std::to_string(returnedSequence));
            const std::vector<ORB_SLAM2::Detection> detections =
                yolo.GetDetections();

            if(mask.type()!=CV_8UC1 || mask.size()!=rectifiedRGB.size())
                throw std::runtime_error(
                    "Unexpected semantic mask type or size at sequence "+
                    std::to_string(semanticSequence));
            double minimum = 0.0;
            double maximum = 0.0;
            cv::minMaxLoc(mask,&minimum,&maximum);
            if(minimum<0.0 || maximum>255.0)
                throw std::runtime_error("Semantic mask range is invalid");
            cv::Mat intermediate;
            cv::compare(mask,0,intermediate,cv::CMP_NE);
            cv::Mat notMaximum;
            cv::compare(mask,255,notMaximum,cv::CMP_NE);
            cv::bitwise_and(intermediate,notMaximum,intermediate);

            const int nonzero = cv::countNonZero(mask);
            const int intermediateCount = cv::countNonZero(intermediate);
            if(detections.empty() && nonzero!=0)
                throw std::runtime_error(
                    "Non-empty mask without a person detection");

            for(size_t detectionIndex=0;
                detectionIndex<detections.size();++detectionIndex)
            {
                const ORB_SLAM2::Detection &detection =
                    detections[detectionIndex];
                const cv::Rect imageBounds(
                    0,0,rectifiedRGB.cols,rectifiedRGB.rows);
                if((detection.box & imageBounds)!=detection.box ||
                   detection.box.area()<=0)
                {
                    throw std::runtime_error(
                        "Detection box lies outside the rectified image");
                }
                detectionsOutput
                    << semanticSequence << ","
                    << candidate.sourceFrame << ","
                    << detectionIndex << ","
                    << detection.box.x << ","
                    << detection.box.y << ","
                    << detection.box.width << ","
                    << detection.box.height << ","
                    << std::setprecision(9) << detection.confidence << "\n";
            }

            const std::string stem = FrameStem(candidate.sourceFrame);
            const std::string rgbRelativeOutput =
                manifestOnly ? "" : "rgb/"+stem+".png";
            const std::string maskRelativeOutput =
                manifestOnly ? "" : "mask/"+stem+".png";
            const std::string overlayRelativeOutput =
                manifestOnly ? "" : "overlay/"+stem+".png";
            if(!manifestOnly)
            {
                RequireWrite(
                    JoinPath(outputDirectory,rgbRelativeOutput),
                    rectifiedRGB);
                RequireWrite(
                    JoinPath(outputDirectory,maskRelativeOutput),mask);
                RequireWrite(
                    JoinPath(outputDirectory,overlayRelativeOutput),
                    MakeOverlay(rectifiedRGB,mask,detections));
            }

            const double maskRatio =
                static_cast<double>(nonzero)/
                static_cast<double>(mask.total());
            const double intermediateRatio =
                static_cast<double>(intermediateCount)/
                static_cast<double>(mask.total());
            manifest
                << semanticSequence << ","
                << returnedSequence << ","
                << CSVField(candidate.selectionRole) << ","
                << candidate.selectionRank << ","
                << candidate.sourceFrame << ","
                << CSVField(candidate.rgbTimestamp) << ","
                << CSVField(candidate.rgbRelative) << ","
                << CSVField(candidate.depthTimestamp) << ","
                << CSVField(candidate.depthRelative) << ","
                << CSVField(candidate.poseTimestampSource) << ","
                << CSVField(candidate.poseTimestamp) << ","
                << CSVField(rectifier.DomainSignature()) << ","
                << rectifiedRGB.cols << ","
                << rectifiedRGB.rows << ","
                << "CV_8UC1" << ","
                << nonzero << ","
                << std::setprecision(12) << maskRatio << ","
                << intermediateCount << ","
                << std::setprecision(12) << intermediateRatio << ","
                << minimum << ","
                << maximum << ","
                << detections.size() << ","
                << rgbRelativeOutput << ","
                << maskRelativeOutput << ","
                << overlayRelativeOutput << "\n";

            totalMaskPixels += static_cast<size_t>(nonzero);
            totalIntermediatePixels +=
                static_cast<size_t>(intermediateCount);
            totalDetections += detections.size();
            std::cout
                << "[G2-4D1] " << index+1 << "/" << candidates.size()
                << " source_frame=" << candidate.sourceFrame
                << " detections=" << detections.size()
                << " mask_pixels=" << nonzero
                << " intermediate_pixels=" << intermediateCount
                << std::endl;
        }

        yolo.Stop();
        std::ofstream summary(
            JoinPath(outputDirectory,"summary.txt").c_str());
        if(!summary)
            throw std::runtime_error("Failed to create summary.txt");
        summary
            << "candidate_count=" << candidates.size() << "\n"
            << "mask_frame_count=" << candidates.size() << "\n"
            << "total_mask_nonzero_pixels=" << totalMaskPixels << "\n"
            << "total_mask_intermediate_pixels="
            << totalIntermediatePixels << "\n"
            << "total_person_detections=" << totalDetections << "\n"
            << "mask_semantics=final_person_union_after_7x7_dilation\n"
            << "mask_filter_rule=nonzero_is_filtered\n"
            << "manifest_only=" << (manifestOnly ? 1 : 0) << "\n"
            << "dynamic_decision=none\n"
            << "direct_slam_state_mutation=none\n";
        std::cout
            << "[G2-4D1] Export complete: " << outputDirectory
            << std::endl;
    }
    catch(const std::exception &error)
    {
        std::cerr << "[G2-4D1] ERROR: " << error.what() << std::endl;
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
