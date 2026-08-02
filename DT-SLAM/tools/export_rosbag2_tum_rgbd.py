#!/usr/bin/env python3
"""Export raw ROS 2 RGB-D data and simulated odometry to TUM format."""

import argparse
import bisect
import math
from pathlib import Path

import cv2
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def stamp_seconds(stamp):
    return float(stamp.sec) + float(stamp.nanosec) * 1.0e-9


def quaternion_multiply(left, right):
    lx, ly, lz, lw = left
    rx, ry, rz, rw = right
    return np.array([
        lw * rx + lx * rw + ly * rz - lz * ry,
        lw * ry - lx * rz + ly * rw + lz * rx,
        lw * rz + lx * ry - ly * rx + lz * rw,
        lw * rw - lx * rx - ly * ry - lz * rz,
    ], dtype=np.float64)


def quaternion_from_rpy(roll, pitch, yaw):
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    return np.array([
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    ], dtype=np.float64)


def quaternion_rotate(quaternion, vector):
    xyz = quaternion[:3]
    scalar = quaternion[3]
    return (
        2.0 * np.dot(xyz, vector) * xyz
        + (scalar * scalar - np.dot(xyz, xyz)) * vector
        + 2.0 * scalar * np.cross(xyz, vector)
    )


def quaternion_slerp(left, right, alpha):
    left = left / np.linalg.norm(left)
    right = right / np.linalg.norm(right)
    dot = float(np.dot(left, right))
    if dot < 0.0:
        right = -right
        dot = -dot
    dot = min(1.0, max(-1.0, dot))
    if dot > 0.9995:
        result = left + alpha * (right - left)
        return result / np.linalg.norm(result)
    angle = math.acos(dot)
    scale = math.sin(angle)
    return (
        math.sin((1.0 - alpha) * angle) / scale * left
        + math.sin(alpha * angle) / scale * right
    )


def image_array(message):
    dtype = {
        "rgb8": np.uint8,
        "bgr8": np.uint8,
        "32FC1": np.float32,
        "16UC1": np.uint16,
    }.get(message.encoding)
    if dtype is None:
        raise ValueError("unsupported image encoding: {}".format(message.encoding))
    channels = 3 if message.encoding in ("rgb8", "bgr8") else 1
    row_values = message.step // np.dtype(dtype).itemsize
    array = np.frombuffer(message.data, dtype=dtype)
    array = array.reshape(message.height, row_values)
    array = array[:, :message.width * channels]
    if channels == 3:
        array = array.reshape(message.height, message.width, channels)
    return array.copy()


def odometry_pose(message):
    position = message.pose.pose.position
    orientation = message.pose.pose.orientation
    return (
        stamp_seconds(message.header.stamp),
        np.array([position.x, position.y, position.z], dtype=np.float64),
        np.array([
            orientation.x, orientation.y, orientation.z, orientation.w
        ], dtype=np.float64),
    )


def interpolate_pose(poses, timestamp, max_gap):
    times = [pose[0] for pose in poses]
    right = bisect.bisect_left(times, timestamp)
    if right == 0 or right == len(poses):
        return None
    left = right - 1
    t0, p0, q0 = poses[left]
    t1, p1, q1 = poses[right]
    if timestamp - t0 > max_gap or t1 - timestamp > max_gap or t1 <= t0:
        return None
    alpha = (timestamp - t0) / (t1 - t0)
    return (1.0 - alpha) * p0 + alpha * p1, quaternion_slerp(q0, q1, alpha)


def write_trajectory(path, timestamps, poses, camera_pose, max_gap):
    base_to_camera_translation = np.array([0.2, 0.0, 0.2])
    base_to_camera_rotation = quaternion_from_rpy(-math.pi / 2.0, 0.0, -math.pi / 2.0)
    written = 0
    with path.open("w", encoding="utf-8") as stream:
        stream.write("# timestamp tx ty tz qx qy qz qw\n")
        for timestamp in timestamps:
            interpolated = interpolate_pose(poses, timestamp, max_gap)
            if interpolated is None:
                continue
            position, quaternion = interpolated
            if camera_pose:
                position = position + quaternion_rotate(
                    quaternion, base_to_camera_translation)
                quaternion = quaternion_multiply(
                    quaternion, base_to_camera_rotation)
                quaternion /= np.linalg.norm(quaternion)
            stream.write(
                "{:.9f} {:.9f} {:.9f} {:.9f} {:.9f} {:.9f} {:.9f} {:.9f}\n".format(
                    timestamp, *position, *quaternion))
            written += 1
    return written


def nearest_associations(rgb_frames, depth_frames, tolerance):
    depth_times = [item[0] for item in depth_frames]
    associations = []
    for rgb_time, rgb_path in rgb_frames:
        index = bisect.bisect_left(depth_times, rgb_time)
        candidates = [i for i in (index - 1, index) if 0 <= i < len(depth_frames)]
        if not candidates:
            continue
        best = min(candidates, key=lambda i: abs(depth_times[i] - rgb_time))
        depth_time, depth_path = depth_frames[best]
        if abs(depth_time - rgb_time) <= tolerance:
            associations.append((rgb_time, rgb_path, depth_time, depth_path))
    return associations


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("bag", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--association-tolerance", type=float, default=0.005)
    parser.add_argument("--pose-gap", type=float, default=0.1)
    return parser.parse_args()


def main():
    args = parse_args()
    bag = args.bag.expanduser().resolve()
    output = args.output.expanduser().resolve()
    if output.exists() and any(output.iterdir()):
        raise SystemExit("output directory is not empty: {}".format(output))
    (output / "rgb").mkdir(parents=True, exist_ok=True)
    (output / "depth").mkdir(parents=True, exist_ok=True)

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("", ""))
    topic_types = {
        topic.name: get_message(topic.type)
        for topic in reader.get_all_topics_and_types()
    }

    rgb_frames = []
    depth_frames = []
    robot_poses = []
    box_poses = []
    camera_info = None

    while reader.has_next():
        topic, serialized, _ = reader.read_next()
        if topic not in topic_types:
            continue
        if topic not in (
                "/camera/image_raw", "/camera/depth/image_raw",
                "/camera/camera_info", "/odom", "/box_odom"):
            continue
        message = deserialize_message(serialized, topic_types[topic])
        if topic == "/camera/image_raw":
            timestamp = stamp_seconds(message.header.stamp)
            image = image_array(message)
            if message.encoding == "rgb8":
                image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
            relative = "rgb/{:.9f}.png".format(timestamp)
            if not cv2.imwrite(str(output / relative), image):
                raise RuntimeError("failed to write {}".format(relative))
            rgb_frames.append((timestamp, relative))
        elif topic == "/camera/depth/image_raw":
            timestamp = stamp_seconds(message.header.stamp)
            depth = image_array(message)
            if message.encoding == "32FC1":
                valid = np.isfinite(depth) & (depth > 0.0)
                scaled = np.zeros(depth.shape, dtype=np.uint16)
                scaled[valid] = np.clip(
                    np.rint(depth[valid] * 5000.0), 1, 65535).astype(np.uint16)
            else:
                scaled = depth.astype(np.uint16)
            relative = "depth/{:.9f}.png".format(timestamp)
            if not cv2.imwrite(str(output / relative), scaled):
                raise RuntimeError("failed to write {}".format(relative))
            depth_frames.append((timestamp, relative))
        elif topic == "/camera/camera_info" and camera_info is None:
            camera_info = message
        elif topic == "/odom":
            robot_poses.append(odometry_pose(message))
        elif topic == "/box_odom":
            box_poses.append(odometry_pose(message))

    associations = nearest_associations(
        rgb_frames, depth_frames, args.association_tolerance)
    with (output / "rgb.txt").open("w", encoding="utf-8") as stream:
        stream.write("# timestamp filename\n")
        for timestamp, relative in rgb_frames:
            stream.write("{:.9f} {}\n".format(timestamp, relative))
    with (output / "depth.txt").open("w", encoding="utf-8") as stream:
        stream.write("# timestamp filename\n")
        for timestamp, relative in depth_frames:
            stream.write("{:.9f} {}\n".format(timestamp, relative))
    with (output / "associations.txt").open("w", encoding="utf-8") as stream:
        for values in associations:
            stream.write("{:.9f} {} {:.9f} {}\n".format(*values))

    associated_times = [item[0] for item in associations]
    camera_count = write_trajectory(
        output / "groundtruth.txt", associated_times, robot_poses,
        camera_pose=True, max_gap=args.pose_gap)
    box_count = write_trajectory(
        output / "box_groundtruth.txt", associated_times, box_poses,
        camera_pose=False, max_gap=args.pose_gap)

    if camera_info is None:
        raise RuntimeError("bag does not contain /camera/camera_info")
    with (output / "camera.txt").open("w", encoding="utf-8") as stream:
        stream.write("width={}\nheight={}\n".format(
            camera_info.width, camera_info.height))
        stream.write("fx={:.12f}\nfy={:.12f}\ncx={:.12f}\ncy={:.12f}\n".format(
            camera_info.k[0], camera_info.k[4],
            camera_info.k[2], camera_info.k[5]))
        stream.write("distortion={}\n".format(" ".join(
            "{:.12f}".format(value) for value in camera_info.d)))
        stream.write("depth_map_factor=5000.0\n")

    print("rgb_frames={}".format(len(rgb_frames)))
    print("depth_frames={}".format(len(depth_frames)))
    print("associations={}".format(len(associations)))
    print("camera_groundtruth={}".format(camera_count))
    print("box_groundtruth={}".format(box_count))


if __name__ == "__main__":
    main()
