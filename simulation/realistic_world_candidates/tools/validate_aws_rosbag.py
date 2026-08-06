#!/usr/bin/env python3
"""Validate one raw AWS Small House RGB-D recording before DT-SLAM use."""

import argparse
import json
import math
from pathlib import Path

from rclpy.serialization import deserialize_message
from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
from rosidl_runtime_py.utilities import get_message


def stamp_ns(message):
    stamp = message.header.stamp
    return stamp.sec * 1_000_000_000 + stamp.nanosec


def update_extent(extents, name, position):
    values = extents.setdefault(
        name,
        {'min_x': math.inf, 'max_x': -math.inf,
         'min_y': math.inf, 'max_y': -math.inf},
    )
    values['min_x'] = min(values['min_x'], position.x)
    values['max_x'] = max(values['max_x'], position.x)
    values['min_y'] = min(values['min_y'], position.y)
    values['max_y'] = max(values['max_y'], position.y)


def update_track(tracks, name, position):
    point = (position.x, position.y, position.z)
    values = tracks.setdefault(name, {
        'start': point,
        'end': point,
        'previous': point,
        'sampled_path_length': 0.0,
    })
    previous = values['previous']
    values['sampled_path_length'] += math.dist(previous, point)
    values['previous'] = point
    values['end'] = point


def validate(bag_path: Path):
    reader = SequentialReader()
    reader.open(
        StorageOptions(uri=str(bag_path), storage_id='sqlite3'),
        ConverterOptions('', ''),
    )
    topic_types = {
        item.name: item.type for item in reader.get_all_topics_and_types()
    }
    message_types = {
        topic: get_message(type_name)
        for topic, type_name in topic_types.items()
    }

    counts = {}
    rgb_stamps = []
    depth_stamps = []
    image_metadata = {}
    camera_intrinsics = {}
    entity_extents = {}
    entity_tracks = {}
    required_entities = ('geo_bot', 'dynamic_box', 'actor_patrol')

    while reader.has_next():
        topic, serialized, _ = reader.read_next()
        counts[topic] = counts.get(topic, 0) + 1
        message = deserialize_message(serialized, message_types[topic])

        if topic == '/camera/image_raw':
            rgb_stamps.append(stamp_ns(message))
            image_metadata.setdefault(topic, {
                'width': message.width,
                'height': message.height,
                'encoding': message.encoding,
            })
        elif topic == '/camera/depth/image_raw':
            depth_stamps.append(stamp_ns(message))
            image_metadata.setdefault(topic, {
                'width': message.width,
                'height': message.height,
                'encoding': message.encoding,
            })
        elif topic in ('/camera/camera_info', '/camera/depth/camera_info'):
            camera_intrinsics.setdefault(topic, {
                'width': message.width,
                'height': message.height,
                'fx': message.k[0],
                'fy': message.k[4],
                'cx': message.k[2],
                'cy': message.k[5],
            })
        elif topic == '/simulation/model_states':
            index_by_name = {name: index for index, name in enumerate(message.name)}
            for entity in required_entities:
                if entity in index_by_name:
                    update_extent(
                        entity_extents,
                        entity,
                        message.pose[index_by_name[entity]].position,
                    )
                    update_track(
                        entity_tracks,
                        entity,
                        message.pose[index_by_name[entity]].position,
                    )

    rgb_set = set(rgb_stamps)
    depth_set = set(depth_stamps)
    paired = rgb_set & depth_set
    rgb_only = rgb_set - depth_set
    depth_only = depth_set - rgb_set
    paired_min = min(paired) if paired else None
    paired_max = max(paired) if paired else None
    unmatched = sorted(rgb_only | depth_only)
    unmatched_are_boundary_only = bool(paired) and all(
        timestamp < paired_min or timestamp > paired_max
        for timestamp in unmatched
    )

    for values in entity_extents.values():
        values['range_x'] = values['max_x'] - values['min_x']
        values['range_y'] = values['max_y'] - values['min_y']
    for values in entity_tracks.values():
        values['start_end_distance'] = math.dist(values['start'], values['end'])
        values.pop('previous')

    failures = []
    warnings = []
    for topic in (
        '/camera/image_raw',
        '/camera/depth/image_raw',
        '/camera/camera_info',
        '/camera/depth/camera_info',
        '/odom',
        '/box_odom',
        '/simulation/model_states',
    ):
        if counts.get(topic, 0) == 0:
            failures.append(f'missing required topic data: {topic}')

    if rgb_only or depth_only:
        mismatch_summary = (
            f'RGB/depth exact-stamp mismatch: rgb_only={len(rgb_only)}, '
            f'depth_only={len(depth_only)}'
        )
        if len(unmatched) <= 2 and unmatched_are_boundary_only:
            warnings.append(
                mismatch_summary + '; unmatched frames occur only outside the '
                'shared timestamp interval and will be excluded'
            )
        else:
            failures.append(mismatch_summary)
    for entity in required_entities:
        if entity not in entity_extents:
            failures.append(f'missing entity in model states: {entity}')

    box_extent = entity_extents.get('dynamic_box', {})
    actor_extent = entity_extents.get('actor_patrol', {})
    if box_extent and box_extent['range_x'] < 0.5:
        failures.append('box did not traverse at least 0.5 m in world X')
    if actor_extent and actor_extent['range_x'] < 0.5:
        failures.append('person actor did not traverse at least 0.5 m in world X')

    result = {
        'bag_path': str(bag_path),
        'passed': not failures,
        'failures': failures,
        'warnings': warnings,
        'counts': counts,
        'rgb_depth_sync': {
            'exact_pairs': len(paired),
            'rgb_only': len(rgb_only),
            'depth_only': len(depth_only),
            'unmatched_are_boundary_only': unmatched_are_boundary_only,
        },
        'image_metadata': image_metadata,
        'camera_intrinsics': camera_intrinsics,
        'entity_extents': entity_extents,
        'entity_tracks': entity_tracks,
    }
    return result


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('bag_path', type=Path)
    parser.add_argument('--json-output', type=Path)
    arguments = parser.parse_args()

    result = validate(arguments.bag_path)
    formatted = json.dumps(result, indent=2, sort_keys=True)
    print(formatted)
    if arguments.json_output:
        arguments.json_output.write_text(formatted + '\n', encoding='utf-8')
    raise SystemExit(0 if result['passed'] else 1)


if __name__ == '__main__':
    main()
