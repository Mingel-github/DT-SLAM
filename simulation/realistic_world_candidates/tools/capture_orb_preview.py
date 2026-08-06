#!/usr/bin/env python3
"""Capture one ROS 2 RGB frame and save a lightweight ORB distribution audit.

This is a scene-selection diagnostic, not the DT-SLAM ORB extractor and not an
algorithm evaluation. It uses OpenCV ORB with the same nominal feature count,
scale factor, and pyramid level count as the current ORB-SLAM2 configuration.
"""

import argparse
import json
from pathlib import Path

import cv2
from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image


class FrameCapture(Node):
    def __init__(self, topic: str):
        super().__init__('candidate_world_orb_preview')
        self.bridge = CvBridge()
        self.frame = None
        self.subscription = self.create_subscription(
            Image, topic, self._on_image, 10
        )

    def _on_image(self, message: Image) -> None:
        if self.frame is None:
            self.frame = self.bridge.imgmsg_to_cv2(message, 'bgr8')


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument('--topic', default='/camera/image_raw')
    parser.add_argument('--output-prefix', required=True)
    parser.add_argument('--timeout-sec', type=float, default=10.0)
    args = parser.parse_args()

    rclpy.init()
    node = FrameCapture(args.topic)
    start = node.get_clock().now()
    while rclpy.ok() and node.frame is None:
        rclpy.spin_once(node, timeout_sec=0.1)
        elapsed = (node.get_clock().now() - start).nanoseconds / 1e9
        if elapsed > args.timeout_sec:
            node.destroy_node()
            rclpy.shutdown()
            raise RuntimeError(f'No image received from {args.topic}')

    image = node.frame
    node.destroy_node()
    rclpy.shutdown()

    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    orb = cv2.ORB_create(
        nfeatures=1000,
        scaleFactor=1.2,
        nlevels=8,
        fastThreshold=20,
    )
    keypoints = orb.detect(gray, None)

    rows, cols = 3, 4
    height, width = gray.shape
    counts = [[0 for _ in range(cols)] for _ in range(rows)]
    for keypoint in keypoints:
        col = min(cols - 1, int(keypoint.pt[0] * cols / width))
        row = min(rows - 1, int(keypoint.pt[1] * rows / height))
        counts[row][col] += 1

    total = len(keypoints)
    nonempty_cells = sum(value > 0 for row in counts for value in row)
    dominant_fraction = (
        max(value for row in counts for value in row) / total if total else 0.0
    )

    overlay = cv2.drawKeypoints(
        image,
        keypoints,
        None,
        color=(0, 255, 0),
        flags=cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS,
    )
    for col in range(1, cols):
        x = int(col * width / cols)
        cv2.line(overlay, (x, 0), (x, height), (255, 255, 0), 1)
    for row in range(1, rows):
        y = int(row * height / rows)
        cv2.line(overlay, (0, y), (width, y), (255, 255, 0), 1)

    prefix = Path(args.output_prefix)
    prefix.parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(prefix.with_name(prefix.name + '_rgb.png')), image)
    cv2.imwrite(str(prefix.with_name(prefix.name + '_orb.png')), overlay)
    report = {
        'status': 'preview_proxy_not_dt_slam_extractor',
        'topic': args.topic,
        'image_width': width,
        'image_height': height,
        'keypoint_count': total,
        'grid_rows': rows,
        'grid_cols': cols,
        'grid_counts': counts,
        'nonempty_grid_cells': nonempty_cells,
        'dominant_cell_fraction': dominant_fraction,
    }
    prefix.with_name(prefix.name + '_orb.json').write_text(
        json.dumps(report, indent=2) + '\n', encoding='utf-8'
    )
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
