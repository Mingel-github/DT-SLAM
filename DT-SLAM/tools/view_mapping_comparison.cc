#include <algorithm>
#include <array>
#include <chrono>
#include <iostream>
#include <limits>
#include <string>
#include <thread>
#include <vector>

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

namespace {

struct Entry {
  std::string path;
  std::string title;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud{
      new pcl::PointCloud<pcl::PointXYZRGB>};
  int viewport = 0;
};

}  // namespace

int main(int argc, char** argv) {
  std::cerr << "Starting mapping comparison viewer\n";
  const bool s3_support_mode =
      argc == 6 && std::string(argv[1]) == "--s3-support";
  const bool aws_s3_support_mode =
      argc == 6 && std::string(argv[1]) == "--aws-s3-support";
  const bool paired_mode =
      argc == 4 && std::string(argv[1]) == "--paired";
  if (argc != 6 && !paired_mode) {
    std::cerr << "Usage: " << argv[0]
              << " raw.pcd stable3.pcd stable8.pcd octomap.pcd s3_octomap.pcd\n"
              << "   or: " << argv[0]
              << " --s3-support s3_raw.pcd s3_stable3.pcd s3_stable8.pcd"
                 " s3_octomap.pcd\n"
              << "   or: " << argv[0]
              << " --aws-s3-support raw.pcd s3_raw.pcd s3_stable8.pcd"
                 " s3_octomap.pcd\n"
              << "   or: " << argv[0]
              << " --paired unfiltered.pcd s3_filtered.pcd\n";
    return 2;
  }

  std::vector<Entry> entries;
  if (paired_mode) {
    entries = {
        {argv[2], "A  Same-pose RGB-D accumulation | no dynamic mask"},
        {argv[3], "B  Same-pose RGB-D accumulation | S3 mask applied"},
    };
  } else if (aws_s3_support_mode) {
    entries = {
        {argv[2], "A  No dynamic mask | ordinary accumulation | ghost 100%"},
        {argv[3], "B  S3 mask + accumulation | ghost 96.35%"},
        {argv[4], "C  S3 mask + >=8 frames | ghost 8.38%"},
        {argv[5], "D  S3 mask + OctoMap | ghost 36.31%"},
    };
  } else if (s3_support_mode) {
    entries = {
        {argv[2], "A  S3 mask + accumulation | ghost 75.17%"},
        {argv[3], "B  S3 mask + >=3 frames | ghost 14.71%"},
        {argv[4], "C  S3 mask + >=8 frames | ghost 0%"},
        {argv[5], "D  S3 mask + OctoMap | ghost 0%"},
    };
  } else {
    entries = {
        {argv[1], "A  Raw accumulation | ghost 100%"},
        {argv[2], "B  >=3-frame support | ghost 26.65%"},
        {argv[3], "C  >=8-frame support | ghost 6.69%"},
        {argv[4], "D  OctoMap, no mask | ghost 2.39%"},
        {argv[5], "E  S3 mask + OctoMap | ghost 0%"},
    };
  }

  float min_x = std::numeric_limits<float>::max();
  float min_y = std::numeric_limits<float>::max();
  float min_z = std::numeric_limits<float>::max();
  float max_x = std::numeric_limits<float>::lowest();
  float max_y = std::numeric_limits<float>::lowest();
  float max_z = std::numeric_limits<float>::lowest();

  for (auto& entry : entries) {
    std::cerr << "Loading: " << entry.path << '\n';
    if (pcl::io::loadPCDFile(entry.path, *entry.cloud) != 0 ||
        entry.cloud->empty()) {
      std::cerr << "Failed to load non-empty cloud: " << entry.path << '\n';
      return 1;
    }
    std::cerr << "Loaded points: " << entry.cloud->size() << '\n';
    for (const auto& point : *entry.cloud) {
      if (!pcl::isFinite(point)) {
        continue;
      }
      min_x = std::min(min_x, point.x);
      min_y = std::min(min_y, point.y);
      min_z = std::min(min_z, point.z);
      max_x = std::max(max_x, point.x);
      max_y = std::max(max_y, point.y);
      max_z = std::max(max_z, point.z);
    }
  }

  auto viewer = std::make_shared<pcl::visualization::PCLVisualizer>(
      paired_mode
          ? "DT-SLAM same-pose point-cloud comparison"
          : "DT-SLAM mapping comparison: positive support vs free-space update");
  std::cerr << "Viewer created\n";
  viewer->setSize(1800, 1000);
  std::cerr << "Viewer size set\n";

  const std::vector<std::array<double, 4>> bounds = paired_mode
      ? std::vector<std::array<double, 4>>{
            {{0.0, 0.0, 0.5, 1.0}}, {{0.5, 0.0, 1.0, 1.0}}}
      : (s3_support_mode || aws_s3_support_mode)
      ? std::vector<std::array<double, 4>>{
            {{0.0, 0.5, 0.5, 1.0}}, {{0.5, 0.5, 1.0, 1.0}},
            {{0.0, 0.0, 0.5, 0.5}}, {{0.5, 0.0, 1.0, 0.5}}}
      : std::vector<std::array<double, 4>>{
            {{0.0, 0.5, 1.0 / 3.0, 1.0}},
            {{1.0 / 3.0, 0.5, 2.0 / 3.0, 1.0}},
            {{2.0 / 3.0, 0.5, 1.0, 1.0}},
            {{0.0, 0.0, 0.5, 0.5}}, {{0.5, 0.0, 1.0, 0.5}}};

  for (std::size_t i = 0; i < entries.size(); ++i) {
    std::cerr << "Creating viewport " << i << '\n';
    const auto& b = bounds[i];
    viewer->createViewPort(b[0], b[1], b[2], b[3], entries[i].viewport);
    viewer->setBackgroundColor(0.035, 0.035, 0.045, entries[i].viewport);
    const std::string id = "cloud_" + std::to_string(i);
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(
        entries[i].cloud);
    viewer->addPointCloud(entries[i].cloud, rgb, id, entries[i].viewport);
    viewer->setPointCloudRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2.0, id,
        entries[i].viewport);
    viewer->addText(entries[i].title, 12, 16, 18, 1.0, 0.9, 0.2,
                    "title_" + std::to_string(i), entries[i].viewport);
    viewer->addText(std::to_string(entries[i].cloud->size()) + " occupied points",
                    12, 40, 14, 0.85, 0.85, 0.85,
                    "count_" + std::to_string(i), entries[i].viewport);
    std::cerr << "Viewport ready " << i << '\n';
  }

  const double cx = 0.5 * (min_x + max_x);
  const double cy = 0.5 * (min_y + max_y);
  const double cz = 0.5 * (min_z + max_z);
  const double span = std::max({static_cast<double>(max_x - min_x),
                                static_cast<double>(max_y - min_y),
                                static_cast<double>(max_z - min_z), 1.0});
  for (const auto& entry : entries) {
    viewer->setCameraPosition(cx - 1.25 * span, cy - 1.25 * span,
                              cz + 0.85 * span, cx, cy, cz, 0.0, 0.0, 1.0,
                              entry.viewport);
  }
  std::cerr << "Cameras set; entering event loop\n";

  viewer->spin();
  return 0;
}
