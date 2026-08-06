#include <octomap/ColorOcTree.h>
#include <octomap/Pointcloud.h>

#include <opencv2/imgcodecs.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace fs = std::filesystem;

struct Pose {
  double timestamp = 0.0;
  cv::Matx33d rotation = cv::Matx33d::eye();
  cv::Vec3d translation{0.0, 0.0, 0.0};
};

struct Association {
  double timestamp = 0.0;
  fs::path rgb;
  fs::path depth;
};

struct Options {
  fs::path dataset_root;
  fs::path associations;
  fs::path map_trajectory;
  fs::path reference_camera_trajectory;
  fs::path box_trajectory;
  fs::path mask_directory;
  fs::path output_directory;
  std::string mask_pattern = "frame_{index:06d}_dynamic_depth_mask.png";
  double fx = 0.0;
  double fy = 0.0;
  double cx = 0.0;
  double cy = 0.0;
  double depth_factor = 5000.0;
  double maximum_depth = 6.0;
  double resolution = 0.10;
  double box_half_extent = 0.32;
  double maximum_pose_difference = 0.02;
  int dynamic_threshold = 1;
  int pixel_stride = 8;
  int frame_step = 2;
  int stable_minimum_frames = 3;
  bool self_test = false;
};

struct VoxelStats {
  int hit_frames = 0;
  int masked_hit_frames = 0;
  int box_hit_frames = 0;
  int nonbox_hit_frames = 0;
  int last_box_hit_frame = -1;
  int maximum_nonbox_free_frame = -1;
  std::uint64_t red_sum = 0;
  std::uint64_t green_sum = 0;
  std::uint64_t blue_sum = 0;
  std::uint64_t color_count = 0;
};

struct ModeMetrics {
  std::string name;
  std::size_t occupied_leaf_nodes = 0;
  std::size_t ghost_candidate_voxels = 0;
  std::size_t ghost_retained_voxels = 0;
  double ghost_retained_fraction = 0.0;
  std::size_t persistent_nonbox_reference_voxels = 0;
  std::size_t persistent_nonbox_retained_voxels = 0;
  double persistent_nonbox_retained_fraction = 0.0;
  std::size_t oracle_free_reference_voxels = 0;
  std::size_t oracle_free_conflicts = 0;
  double oracle_free_conflict_fraction = 0.0;
};

using Key = octomap::OcTreeKey;
using KeySet = std::unordered_set<Key, Key::KeyHash>;
using StatsMap = std::unordered_map<Key, VoxelStats, Key::KeyHash>;

std::string Require(const std::unordered_map<std::string, std::string>& args,
                    const std::string& name) {
  const auto found = args.find(name);
  if (found == args.end()) throw std::runtime_error("missing argument: " + name);
  return found->second;
}

Options ParseOptions(int argc, char** argv) {
  Options options;
  std::unordered_map<std::string, std::string> values;
  for (int index = 1; index < argc; ++index) {
    const std::string argument(argv[index]);
    if (argument == "--self-test") {
      options.self_test = true;
      continue;
    }
    if (argument.rfind("--", 0) != 0 || index + 1 >= argc)
      throw std::runtime_error("expected --name value arguments");
    values[argument] = argv[++index];
  }
  if (options.self_test) return options;
  options.dataset_root = Require(values, "--dataset-root");
  options.associations = Require(values, "--associations");
  options.map_trajectory = Require(values, "--map-trajectory");
  options.reference_camera_trajectory =
      Require(values, "--reference-camera-trajectory");
  options.box_trajectory = Require(values, "--box-trajectory");
  options.mask_directory = Require(values, "--mask-directory");
  options.output_directory = Require(values, "--output-directory");
  if (values.count("--mask-pattern")) options.mask_pattern = values.at("--mask-pattern");
  options.fx = std::stod(Require(values, "--fx"));
  options.fy = std::stod(Require(values, "--fy"));
  options.cx = std::stod(Require(values, "--cx"));
  options.cy = std::stod(Require(values, "--cy"));
  if (values.count("--depth-factor"))
    options.depth_factor = std::stod(values.at("--depth-factor"));
  if (values.count("--maximum-depth"))
    options.maximum_depth = std::stod(values.at("--maximum-depth"));
  if (values.count("--resolution"))
    options.resolution = std::stod(values.at("--resolution"));
  if (values.count("--box-half-extent"))
    options.box_half_extent = std::stod(values.at("--box-half-extent"));
  if (values.count("--maximum-pose-difference"))
    options.maximum_pose_difference =
        std::stod(values.at("--maximum-pose-difference"));
  if (values.count("--dynamic-threshold"))
    options.dynamic_threshold = std::stoi(values.at("--dynamic-threshold"));
  if (values.count("--pixel-stride"))
    options.pixel_stride = std::stoi(values.at("--pixel-stride"));
  if (values.count("--frame-step"))
    options.frame_step = std::stoi(values.at("--frame-step"));
  if (values.count("--stable-minimum-frames"))
    options.stable_minimum_frames =
        std::stoi(values.at("--stable-minimum-frames"));
  if (options.fx <= 0.0 || options.fy <= 0.0 ||
      options.depth_factor <= 0.0 || options.resolution <= 0.0 ||
      options.pixel_stride < 1 || options.frame_step < 1 ||
      options.stable_minimum_frames < 1 || options.dynamic_threshold < 0 ||
      options.dynamic_threshold > 255)
    throw std::runtime_error("invalid numeric option");
  return options;
}

cv::Matx33d QuaternionRotation(double x, double y, double z, double w) {
  const double norm = std::sqrt(x*x + y*y + z*z + w*w);
  if (!(norm > 0.0)) throw std::runtime_error("zero quaternion");
  x /= norm; y /= norm; z /= norm; w /= norm;
  return cv::Matx33d(
      1.0-2.0*(y*y+z*z), 2.0*(x*y-z*w), 2.0*(x*z+y*w),
      2.0*(x*y+z*w), 1.0-2.0*(x*x+z*z), 2.0*(y*z-x*w),
      2.0*(x*z-y*w), 2.0*(y*z+x*w), 1.0-2.0*(x*x+y*y));
}

std::vector<Pose> ReadPoses(const fs::path& path) {
  std::ifstream stream(path);
  if (!stream) throw std::runtime_error("cannot open trajectory: " + path.string());
  std::vector<Pose> poses;
  std::string line;
  while (std::getline(stream, line)) {
    if (line.empty() || line.front() == '#') continue;
    std::istringstream input(line);
    Pose pose;
    double qx, qy, qz, qw;
    if (!(input >> pose.timestamp >> pose.translation[0] >> pose.translation[1]
          >> pose.translation[2] >> qx >> qy >> qz >> qw))
      throw std::runtime_error("invalid trajectory row");
    pose.rotation = QuaternionRotation(qx, qy, qz, qw);
    poses.push_back(pose);
  }
  return poses;
}

std::vector<Association> ReadAssociations(const fs::path& path) {
  std::ifstream stream(path);
  if (!stream) throw std::runtime_error("cannot open associations");
  std::vector<Association> rows;
  std::string line;
  while (std::getline(stream, line)) {
    if (line.empty() || line.front() == '#') continue;
    std::istringstream input(line);
    Association row;
    double depth_timestamp;
    if (!(input >> row.timestamp >> row.rgb >> depth_timestamp >> row.depth))
      throw std::runtime_error("invalid association row");
    rows.push_back(row);
  }
  return rows;
}

const Pose* NearestPose(const std::vector<Pose>& poses, double timestamp,
                        double maximum_difference) {
  if (poses.empty()) return nullptr;
  const Pose* best = &poses.front();
  double difference = std::abs(best->timestamp - timestamp);
  for (const Pose& pose : poses) {
    const double candidate = std::abs(pose.timestamp - timestamp);
    if (candidate < difference) {
      best = &pose;
      difference = candidate;
    }
  }
  return difference <= maximum_difference ? best : nullptr;
}

std::string FormatMaskName(const std::string& pattern, int index) {
  const std::string token = "{index:06d}";
  const std::size_t position = pattern.find(token);
  if (position == std::string::npos)
    throw std::runtime_error("mask pattern must contain {index:06d}");
  std::ostringstream value;
  value << std::setw(6) << std::setfill('0') << index;
  std::string result = pattern;
  result.replace(position, token.size(), value.str());
  return result;
}

cv::Vec3d Transform(const Pose& pose, const cv::Vec3d& point) {
  return pose.rotation * point + pose.translation;
}

bool IsBoxPoint(const cv::Vec3d& camera_point, const Pose& camera_pose,
                const Pose& box_pose, double half_extent) {
  const cv::Vec3d world = Transform(camera_pose, camera_point);
  const cv::Vec3d relative = box_pose.rotation.t() * (world-box_pose.translation);
  return std::abs(relative[0]) <= half_extent &&
         std::abs(relative[1]) <= half_extent &&
         std::abs(relative[2]) <= half_extent;
}

octomap::point3d ToPoint(const cv::Vec3d& point) {
  return octomap::point3d(static_cast<float>(point[0]),
                          static_cast<float>(point[1]),
                          static_cast<float>(point[2]));
}

bool Occupied(const octomap::ColorOcTree& tree, const Key& key) {
  const auto* node = tree.search(key);
  return node != nullptr && tree.isNodeOccupied(node);
}

bool FreeKnown(const octomap::ColorOcTree& tree, const Key& key) {
  const auto* node = tree.search(key);
  return node != nullptr && !tree.isNodeOccupied(node);
}

void IntegrateColors(octomap::ColorOcTree& tree,
                     const std::vector<std::pair<Key, cv::Vec3b>>& colors) {
  for (const auto& item : colors) {
    tree.averageNodeColor(item.first, item.second[2], item.second[1], item.second[0]);
  }
}

std::size_t OccupiedLeafCount(const octomap::ColorOcTree& tree) {
  std::size_t count = 0;
  for (auto iterator = tree.begin_leafs(), end = tree.end_leafs();
       iterator != end; ++iterator)
    if (tree.isNodeOccupied(*iterator)) ++count;
  return count;
}

void WritePlyHeader(std::ofstream& stream, std::size_t count) {
  stream << "ply\nformat ascii 1.0\nelement vertex " << count
         << "\nproperty float x\nproperty float y\nproperty float z\n"
         << "property uchar red\nproperty uchar green\nproperty uchar blue\n"
         << "end_header\n";
}

void WriteStatsPly(const fs::path& path, const StatsMap& stats,
                   const octomap::ColorOcTree& coordinate_tree,
                   int minimum_hit_frames,
                   bool use_masked_hits = false) {
  std::size_t count = 0;
  for (const auto& item : stats)
    if ((use_masked_hits ? item.second.masked_hit_frames
                         : item.second.hit_frames) >= minimum_hit_frames)
      ++count;
  std::ofstream stream(path);
  if (!stream) throw std::runtime_error("cannot create PLY");
  WritePlyHeader(stream, count);
  stream << std::fixed << std::setprecision(4);
  for (const auto& item : stats) {
    const VoxelStats& value = item.second;
    const int hit_frames =
        use_masked_hits ? value.masked_hit_frames : value.hit_frames;
    if (hit_frames < minimum_hit_frames) continue;
    const octomap::point3d point = coordinate_tree.keyToCoord(item.first);
    const std::uint64_t divisor = std::max<std::uint64_t>(1, value.color_count);
    stream << point.x() << ' ' << point.y() << ' ' << point.z() << ' '
           << value.red_sum/divisor << ' ' << value.green_sum/divisor << ' '
           << value.blue_sum/divisor << '\n';
  }
}

void WriteTreePly(const fs::path& path, const octomap::ColorOcTree& tree) {
  const std::size_t count = OccupiedLeafCount(tree);
  std::ofstream stream(path);
  if (!stream) throw std::runtime_error("cannot create tree PLY");
  WritePlyHeader(stream, count);
  stream << std::fixed << std::setprecision(4);
  for (auto iterator = tree.begin_leafs(), end = tree.end_leafs();
       iterator != end; ++iterator) {
    if (!tree.isNodeOccupied(*iterator)) continue;
    const auto color = iterator->getColor();
    stream << iterator.getX() << ' ' << iterator.getY() << ' '
           << iterator.getZ() << ' ' << static_cast<int>(color.r) << ' '
           << static_cast<int>(color.g) << ' ' << static_cast<int>(color.b)
           << '\n';
  }
}

template <typename OccupiedFunction>
ModeMetrics EvaluateMode(const std::string& name, std::size_t leaf_count,
                         const KeySet& ghost_candidates,
                         const KeySet& persistent_nonbox_reference,
                         const KeySet& oracle_free_reference,
                         OccupiedFunction occupied) {
  ModeMetrics metrics;
  metrics.name = name;
  metrics.occupied_leaf_nodes = leaf_count;
  metrics.ghost_candidate_voxels = ghost_candidates.size();
  metrics.persistent_nonbox_reference_voxels =
      persistent_nonbox_reference.size();
  metrics.oracle_free_reference_voxels = oracle_free_reference.size();
  for (const Key& key : ghost_candidates)
    if (occupied(key)) ++metrics.ghost_retained_voxels;
  for (const Key& key : persistent_nonbox_reference)
    if (occupied(key)) ++metrics.persistent_nonbox_retained_voxels;
  for (const Key& key : oracle_free_reference)
    if (occupied(key)) ++metrics.oracle_free_conflicts;
  if (!ghost_candidates.empty())
    metrics.ghost_retained_fraction =
        static_cast<double>(metrics.ghost_retained_voxels) /
        ghost_candidates.size();
  if (!persistent_nonbox_reference.empty())
    metrics.persistent_nonbox_retained_fraction =
        static_cast<double>(metrics.persistent_nonbox_retained_voxels) /
        persistent_nonbox_reference.size();
  if (!oracle_free_reference.empty())
    metrics.oracle_free_conflict_fraction =
        static_cast<double>(metrics.oracle_free_conflicts) /
        oracle_free_reference.size();
  return metrics;
}

void RunSelfTest() {
  octomap::ColorOcTree tree(0.1);
  const octomap::point3d origin(0.0f, 0.0f, 0.0f);
  octomap::Pointcloud hit;
  hit.push_back(1.0f, 0.0f, 0.0f);
  tree.insertPointCloud(hit, origin, -1.0, false, true);
  Key key;
  if (!tree.coordToKeyChecked(octomap::point3d(1.0f, 0.0f, 0.0f), key) ||
      !Occupied(tree, key))
    throw std::runtime_error("self-test hit did not occupy endpoint");
  octomap::Pointcloud farther;
  farther.push_back(2.0f, 0.0f, 0.0f);
  for (int index = 0; index < 4; ++index)
    tree.insertPointCloud(farther, origin, -1.0, false, true);
  if (Occupied(tree, key))
    throw std::runtime_error("self-test misses did not clear old endpoint");
  std::cout << "self-test passed: p_hit=" << tree.getProbHit()
            << " p_miss=" << tree.getProbMiss()
            << " occupancy=" << tree.getOccupancyThres()
            << " clamp_min=" << tree.getClampingThresMin()
            << " clamp_max=" << tree.getClampingThresMax() << '\n';
}

int main(int argc, char** argv) {
  try {
    const Options options = ParseOptions(argc, argv);
    if (options.self_test) {
      RunSelfTest();
      return 0;
    }
    fs::create_directories(options.output_directory);
    const auto associations = ReadAssociations(options.associations);
    const auto map_poses = ReadPoses(options.map_trajectory);
    const auto reference_camera_poses =
        ReadPoses(options.reference_camera_trajectory);
    const auto box_poses = ReadPoses(options.box_trajectory);

    octomap::ColorOcTree all_tree(options.resolution);
    octomap::ColorOcTree masked_tree(options.resolution);
    octomap::ColorOcTree oracle_tree(options.resolution);
    StatsMap stats;
    int processed_frames = 0;
    int missing_masks = 0;
    int missing_poses = 0;
    const auto begin = std::chrono::steady_clock::now();

    for (int frame = 0; frame < static_cast<int>(associations.size()); ++frame) {
      if (frame % options.frame_step != 0) continue;
      const fs::path mask_path = options.mask_directory /
          FormatMaskName(options.mask_pattern, frame);
      if (!fs::is_regular_file(mask_path)) {
        ++missing_masks;
        continue;
      }
      const Association& row = associations[frame];
      const Pose* map_pose = NearestPose(
          map_poses, row.timestamp, options.maximum_pose_difference);
      const Pose* reference_pose = NearestPose(
          reference_camera_poses, row.timestamp,
          options.maximum_pose_difference);
      const Pose* box_pose = NearestPose(
          box_poses, row.timestamp, options.maximum_pose_difference);
      if (!map_pose || !reference_pose || !box_pose) {
        ++missing_poses;
        continue;
      }
      const cv::Mat color = cv::imread(
          (options.dataset_root / row.rgb).string(), cv::IMREAD_COLOR);
      const cv::Mat depth = cv::imread(
          (options.dataset_root / row.depth).string(), cv::IMREAD_UNCHANGED);
      const cv::Mat mask = cv::imread(mask_path.string(), cv::IMREAD_GRAYSCALE);
      if (color.empty() || depth.empty() || mask.empty() ||
          color.size() != depth.size() || mask.size() != depth.size())
        throw std::runtime_error("invalid RGB/depth/mask input");

      octomap::Pointcloud all_cloud, masked_cloud, oracle_cloud;
      std::vector<std::pair<Key, cv::Vec3b>> all_colors;
      std::vector<std::pair<Key, cv::Vec3b>> masked_colors;
      std::vector<std::pair<Key, cv::Vec3b>> oracle_colors;
      KeySet frame_all, frame_masked, frame_box, frame_nonbox;
      for (int v = 0; v < depth.rows; v += options.pixel_stride) {
        for (int u = 0; u < depth.cols; u += options.pixel_stride) {
          double z = 0.0;
          if (depth.type() == CV_16U)
            z = depth.at<std::uint16_t>(v, u) / options.depth_factor;
          else if (depth.type() == CV_32F)
            z = depth.at<float>(v, u);
          else
            throw std::runtime_error("unsupported depth type");
          if (!std::isfinite(z) || z <= 0.0 ||
              (options.maximum_depth > 0.0 && z > options.maximum_depth))
            continue;
          const cv::Vec3d camera_point(
              (u-options.cx)*z/options.fx,
              (v-options.cy)*z/options.fy, z);
          const cv::Vec3d world = Transform(*map_pose, camera_point);
          Key key;
          if (!all_tree.coordToKeyChecked(ToPoint(world), key)) continue;
          const bool box = IsBoxPoint(
              camera_point, *reference_pose, *box_pose,
              options.box_half_extent);
          const bool dynamic = mask.at<std::uint8_t>(v, u) >=
                               options.dynamic_threshold;
          const cv::Vec3b bgr = color.at<cv::Vec3b>(v, u);
          all_cloud.push_back(ToPoint(world));
          all_colors.emplace_back(key, bgr);
          frame_all.insert(key);
          if (box) frame_box.insert(key); else frame_nonbox.insert(key);
          VoxelStats& value = stats[key];
          value.red_sum += bgr[2];
          value.green_sum += bgr[1];
          value.blue_sum += bgr[0];
          ++value.color_count;
          if (!dynamic) {
            masked_cloud.push_back(ToPoint(world));
            masked_colors.emplace_back(key, bgr);
            frame_masked.insert(key);
          }
          if (!box) {
            oracle_cloud.push_back(ToPoint(world));
            oracle_colors.emplace_back(key, bgr);
          }
        }
      }
      for (const Key& key : frame_all) ++stats[key].hit_frames;
      for (const Key& key : frame_masked) ++stats[key].masked_hit_frames;
      for (const Key& key : frame_box) {
        ++stats[key].box_hit_frames;
        stats[key].last_box_hit_frame = frame;
      }
      for (const Key& key : frame_nonbox) ++stats[key].nonbox_hit_frames;
      const octomap::point3d origin = ToPoint(map_pose->translation);
      // Depth has already been filtered in the optical-axis z domain above.
      // Do not apply a second Euclidean ray-length truncation in OctoMap.
      all_tree.insertPointCloud(all_cloud, origin, -1.0, false, true);
      masked_tree.insertPointCloud(masked_cloud, origin, -1.0, false, true);
      oracle_tree.insertPointCloud(oracle_cloud, origin, -1.0, false, true);
      IntegrateColors(all_tree, all_colors);
      IntegrateColors(masked_tree, masked_colors);
      IntegrateColors(oracle_tree, oracle_colors);
      ++processed_frames;
      if (processed_frames % 50 == 0)
        std::cout << "first pass frames=" << processed_frames << '\n';
    }
    all_tree.updateInnerOccupancy();
    masked_tree.updateInnerOccupancy();
    oracle_tree.updateInnerOccupancy();

    KeySet ghost_candidates;
    KeySet persistent_nonbox_reference;
    for (const auto& item : stats) {
      if (item.second.box_hit_frames > 0 &&
          item.second.nonbox_hit_frames == 0)
        ghost_candidates.insert(item.first);
      if (item.second.nonbox_hit_frames >= options.stable_minimum_frames &&
          item.second.box_hit_frames == 0)
        persistent_nonbox_reference.insert(item.first);
    }

    // Second pass: measure whether a later non-box ray traverses each
    // box-exclusive endpoint voxel. This is a causal clearing opportunity,
    // not merely a final free/occupied correlation.
    int ray_frames = 0;
    for (int frame = 0; frame < static_cast<int>(associations.size()); ++frame) {
      if (frame % options.frame_step != 0) continue;
      const fs::path mask_path = options.mask_directory /
          FormatMaskName(options.mask_pattern, frame);
      if (!fs::is_regular_file(mask_path)) continue;
      const Association& row = associations[frame];
      const Pose* map_pose = NearestPose(
          map_poses, row.timestamp, options.maximum_pose_difference);
      const Pose* reference_pose = NearestPose(
          reference_camera_poses, row.timestamp,
          options.maximum_pose_difference);
      const Pose* box_pose = NearestPose(
          box_poses, row.timestamp, options.maximum_pose_difference);
      if (!map_pose || !reference_pose || !box_pose) continue;
      const cv::Mat depth = cv::imread(
          (options.dataset_root / row.depth).string(), cv::IMREAD_UNCHANGED);
      if (depth.empty()) throw std::runtime_error("invalid depth in ray pass");
      const octomap::point3d origin = ToPoint(map_pose->translation);
      octomap::KeyRay ray;
      for (int v = 0; v < depth.rows; v += options.pixel_stride) {
        for (int u = 0; u < depth.cols; u += options.pixel_stride) {
          double z = 0.0;
          if (depth.type() == CV_16U)
            z = depth.at<std::uint16_t>(v, u) / options.depth_factor;
          else if (depth.type() == CV_32F)
            z = depth.at<float>(v, u);
          if (!std::isfinite(z) || z <= 0.0 ||
              (options.maximum_depth > 0.0 && z > options.maximum_depth))
            continue;
          const cv::Vec3d camera_point(
              (u-options.cx)*z/options.fx,
              (v-options.cy)*z/options.fy, z);
          if (IsBoxPoint(camera_point, *reference_pose, *box_pose,
                         options.box_half_extent))
            continue;
          const cv::Vec3d world = Transform(*map_pose, camera_point);
          if (!all_tree.computeRayKeys(origin, ToPoint(world), ray)) continue;
          for (const Key& key : ray) {
            auto found = stats.find(key);
            if (found != stats.end() && found->second.box_hit_frames > 0)
              found->second.maximum_nonbox_free_frame = std::max(
                  found->second.maximum_nonbox_free_frame, frame);
          }
        }
      }
      ++ray_frames;
      if (ray_frames % 50 == 0)
        std::cout << "ray pass frames=" << ray_frames << '\n';
    }

    KeySet clearable_after_last_box;
    KeySet oracle_free_reference;
    for (const Key& key : ghost_candidates) {
      const VoxelStats& value = stats.at(key);
      if (value.maximum_nonbox_free_frame > value.last_box_hit_frame)
        clearable_after_last_box.insert(key);
      if (FreeKnown(oracle_tree, key)) oracle_free_reference.insert(key);
    }

    const auto raw_occupied = [&stats](const Key& key) {
      return stats.find(key) != stats.end();
    };
    const auto stable_occupied = [&stats, &options](const Key& key) {
      const auto found = stats.find(key);
      return found != stats.end() &&
             found->second.hit_frames >= options.stable_minimum_frames;
    };
    const auto masked_raw_occupied = [&stats](const Key& key) {
      const auto found = stats.find(key);
      return found != stats.end() && found->second.masked_hit_frames >= 1;
    };
    const auto masked_stable_occupied = [&stats, &options](const Key& key) {
      const auto found = stats.find(key);
      return found != stats.end() &&
             found->second.masked_hit_frames >= options.stable_minimum_frames;
    };
    std::vector<ModeMetrics> metrics;
    metrics.push_back(EvaluateMode(
        "raw_endpoint_union", stats.size(), ghost_candidates,
        persistent_nonbox_reference, oracle_free_reference, raw_occupied));
    std::size_t stable_count = 0;
    for (const auto& item : stats)
      if (item.second.hit_frames >= options.stable_minimum_frames)
        ++stable_count;
    metrics.push_back(EvaluateMode(
        "stable_endpoint_hits", stable_count, ghost_candidates,
        persistent_nonbox_reference, oracle_free_reference, stable_occupied));
    std::size_t masked_raw_count = 0;
    std::size_t masked_stable_count = 0;
    for (const auto& item : stats) {
      if (item.second.masked_hit_frames >= 1) ++masked_raw_count;
      if (item.second.masked_hit_frames >= options.stable_minimum_frames)
        ++masked_stable_count;
    }
    metrics.push_back(EvaluateMode(
        "s3_masked_endpoint_union", masked_raw_count, ghost_candidates,
        persistent_nonbox_reference, oracle_free_reference,
        masked_raw_occupied));
    metrics.push_back(EvaluateMode(
        "s3_masked_stable_endpoint_hits", masked_stable_count,
        ghost_candidates, persistent_nonbox_reference, oracle_free_reference,
        masked_stable_occupied));
    metrics.push_back(EvaluateMode(
        "octomap_all_depth", OccupiedLeafCount(all_tree), ghost_candidates,
        persistent_nonbox_reference, oracle_free_reference,
        [&all_tree](const Key& key) { return Occupied(all_tree, key); }));
    metrics.push_back(EvaluateMode(
        "octomap_s3_mask", OccupiedLeafCount(masked_tree), ghost_candidates,
        persistent_nonbox_reference, oracle_free_reference,
        [&masked_tree](const Key& key) { return Occupied(masked_tree, key); }));
    metrics.push_back(EvaluateMode(
        "octomap_oracle_box_filter", OccupiedLeafCount(oracle_tree),
        ghost_candidates, persistent_nonbox_reference, oracle_free_reference,
        [&oracle_tree](const Key& key) { return Occupied(oracle_tree, key); }));

    WriteStatsPly(options.output_directory / "raw_endpoint_union.ply",
                  stats, all_tree, 1);
    WriteStatsPly(options.output_directory / "stable_endpoint_hits.ply",
                  stats, all_tree, options.stable_minimum_frames);
    WriteStatsPly(options.output_directory / "s3_masked_endpoint_union.ply",
                  stats, all_tree, 1, true);
    WriteStatsPly(
        options.output_directory / "s3_masked_stable_endpoint_hits.ply",
        stats, all_tree, options.stable_minimum_frames, true);
    WriteTreePly(options.output_directory / "octomap_all_depth.ply", all_tree);
    WriteTreePly(options.output_directory / "octomap_s3_mask.ply", masked_tree);
    WriteTreePly(options.output_directory / "octomap_oracle_box_filter.ply",
                 oracle_tree);
    all_tree.write((options.output_directory / "octomap_all_depth.ot").string());
    masked_tree.write((options.output_directory / "octomap_s3_mask.ot").string());
    oracle_tree.write((options.output_directory /
                       "octomap_oracle_box_filter.ot").string());

    std::ofstream csv(options.output_directory / "voxel_metrics.csv");
    csv << "mode,occupied_leaf_nodes,ghost_candidate_voxels,"
           "ghost_retained_voxels,ghost_retained_fraction,"
           "persistent_nonbox_reference_voxels,"
           "persistent_nonbox_retained_voxels,"
           "persistent_nonbox_retained_fraction,"
           "oracle_free_reference_voxels,"
           "oracle_free_conflicts,oracle_free_conflict_fraction\n";
    for (const ModeMetrics& item : metrics) {
      csv << item.name << ',' << item.occupied_leaf_nodes << ','
          << item.ghost_candidate_voxels << ','
          << item.ghost_retained_voxels << ','
          << item.ghost_retained_fraction << ','
          << item.persistent_nonbox_reference_voxels << ','
          << item.persistent_nonbox_retained_voxels << ','
          << item.persistent_nonbox_retained_fraction << ','
          << item.oracle_free_reference_voxels << ','
          << item.oracle_free_conflicts << ','
          << item.oracle_free_conflict_fraction << '\n';
    }

    const auto end = std::chrono::steady_clock::now();
    const double runtime = std::chrono::duration<double>(end-begin).count();
    std::ofstream json(options.output_directory / "summary.json");
    json << std::fixed << std::setprecision(9);
    json << "{\n"
         << "  \"method\": \"fixed-trajectory offline endpoint and OctoMap audit\",\n"
         << "  \"map_trajectory\": \"" << options.map_trajectory.string() << "\",\n"
         << "  \"reference_camera_trajectory\": \""
         << options.reference_camera_trajectory.string() << "\",\n"
         << "  \"processed_frames\": " << processed_frames << ",\n"
         << "  \"missing_mask_frames\": " << missing_masks << ",\n"
         << "  \"missing_pose_frames\": " << missing_poses << ",\n"
         << "  \"pixel_stride\": " << options.pixel_stride << ",\n"
         << "  \"frame_step\": " << options.frame_step << ",\n"
         << "  \"maximum_depth_m\": " << options.maximum_depth << ",\n"
         << "  \"resolution_m\": " << options.resolution << ",\n"
         << "  \"stable_minimum_frames\": "
         << options.stable_minimum_frames << ",\n"
         << "  \"octomap_prob_hit\": " << all_tree.getProbHit() << ",\n"
         << "  \"octomap_prob_miss\": " << all_tree.getProbMiss() << ",\n"
         << "  \"octomap_occupancy_threshold\": "
         << all_tree.getOccupancyThres() << ",\n"
         << "  \"octomap_clamping_min\": "
         << all_tree.getClampingThresMin() << ",\n"
         << "  \"octomap_clamping_max\": "
         << all_tree.getClampingThresMax() << ",\n"
         << "  \"ghost_candidate_voxels\": "
         << ghost_candidates.size() << ",\n"
         << "  \"ghost_voxels_with_later_nonbox_free_ray\": "
         << clearable_after_last_box.size() << ",\n"
         << "  \"ghost_later_free_ray_fraction\": "
         << (ghost_candidates.empty() ? 0.0 :
             static_cast<double>(clearable_after_last_box.size()) /
             ghost_candidates.size()) << ",\n"
         << "  \"runtime_seconds\": " << runtime << ",\n"
         << "  \"modes\": [\n";
    for (std::size_t index = 0; index < metrics.size(); ++index) {
      const ModeMetrics& item = metrics[index];
      json << "    {\"name\": \"" << item.name
           << "\", \"occupied_leaf_nodes\": " << item.occupied_leaf_nodes
           << ", \"ghost_retained_voxels\": "
           << item.ghost_retained_voxels
           << ", \"ghost_retained_fraction\": "
           << item.ghost_retained_fraction
           << ", \"persistent_nonbox_retained_voxels\": "
           << item.persistent_nonbox_retained_voxels
           << ", \"persistent_nonbox_retained_fraction\": "
           << item.persistent_nonbox_retained_fraction
           << ", \"oracle_free_conflicts\": "
           << item.oracle_free_conflicts
           << ", \"oracle_free_conflict_fraction\": "
           << item.oracle_free_conflict_fraction << "}";
      if (index + 1 != metrics.size()) json << ',';
      json << '\n';
    }
    json << "  ]\n}\n";

    std::cout << "processed_frames=" << processed_frames
              << " ghost_candidates=" << ghost_candidates.size()
              << " later_free=" << clearable_after_last_box.size()
              << " runtime_s=" << runtime << '\n';
    for (const ModeMetrics& item : metrics)
      std::cout << item.name << " ghost=" << item.ghost_retained_fraction
                << " persistent_nonbox="
                << item.persistent_nonbox_retained_fraction
                << " free_conflict=" << item.oracle_free_conflict_fraction
                << " leaves=" << item.occupied_leaf_nodes << '\n';
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "error: " << error.what() << '\n';
    return 1;
  }
}
