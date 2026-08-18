#!/usr/bin/env python3

"""
Analyze SLAM processing latency, pipeline delays, and TF timestamp deltas in a ROS 2 bag file.

Usage:
    bag_delay_analysis.py <path_to_bag>
    bag_delay_analysis.py <path_to_bag> --max-messages 5000
    bag_delay_analysis.py <path_to_bag> --verbose
"""

import argparse
import os
import sys
from collections import defaultdict

import numpy as np

try:
    import rclpy.serialization
    from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
    from rosidl_runtime_py.utilities import get_message
except ImportError as e:
    print(f'Error importing ROS 2 / rosbag2 packages: {e}')
    print('Make sure to source your ROS 2 workspace before running this script.')
    sys.exit(1)


# ORB-SLAM3 tracking state descriptions
TRACKING_STATES = {
    -1: 'SYSTEM_NOT_READY',
    0: 'NO_IMAGES_YET',
    1: 'NOT_INITIALIZED',
    2: 'TRACKING_OK',
    3: 'RECENTLY_LOST',
    4: 'TRACKING_LOST',
    5: 'TRACKING_OK_KLT',
}


def parse_args():
    parser = argparse.ArgumentParser(
        description='Analyze SLAM processing latency and TF timestamp deltas in a ROS 2 bag.'
    )
    parser.add_argument('bag_path', type=str, help='Path to the ROS 2 bag directory or MCAP file')
    parser.add_argument(
        '--storage-id', type=str, default='', help='Storage ID (mcap, sqlite3, or empty to auto-detect)'
    )
    parser.add_argument('-n', '--max-messages', type=int, default=0, help='Max messages to process (0 = all)')
    parser.add_argument('-v', '--verbose', action='store_true', help='Show detailed percentile breakdown')
    return parser.parse_args()


def format_stats(data):
    if not data:
        return 'N/A'
    arr = np.array(data)
    mean = np.mean(arr)
    median = np.median(arr)
    std = np.std(arr)
    min_v = np.min(arr)
    max_v = np.max(arr)
    return f'mean={mean:6.3f}s, med={median:6.3f}s, std={std:5.3f}s, min={min_v:6.3f}s, max={max_v:6.3f}s'


def print_table_row(name, count, arr, rate_hz=None):
    if not arr:
        return
    a = np.array(arr)
    rate_str = f'{rate_hz:5.1f} Hz' if rate_hz is not None else '   -    '
    print(
        f'{name:<26} | {count:>6} | {rate_str} | '
        f'{np.mean(a):>7.3f}s | {np.median(a):>7.3f}s | {np.std(a):>6.3f}s | '
        f'{np.min(a):>7.3f}s | {np.max(a):>7.3f}s'
    )


def print_percentiles(name, arr):
    if not arr:
        return
    a = np.array(arr)
    p50, p75, p90, p95, p99 = np.percentile(a, [50, 75, 90, 95, 99])
    print(f'  {name:<24}: p50={p50:.3f}s, p75={p75:.3f}s, p90={p90:.3f}s, p95={p95:.3f}s, p99={p99:.3f}s')


def analyze_bag(bag_path, storage_id='', max_messages=0, verbose=False):
    if not os.path.exists(bag_path):
        print(f'Error: Bag path does not exist: {bag_path}')
        sys.exit(1)

    # Detect storage id if not given
    if not storage_id:
        if os.path.isfile(bag_path) and bag_path.endswith('.mcap'):
            storage_id = 'mcap'
        elif os.path.isdir(bag_path):
            files = os.listdir(bag_path)
            if any(f.endswith('.mcap') for f in files):
                storage_id = 'mcap'
            elif any(f.endswith('.db3') for f in files):
                storage_id = 'sqlite3'
            else:
                storage_id = 'mcap'

    storage_options = StorageOptions(uri=bag_path, storage_id=storage_id)
    converter_options = ConverterOptions('', '')
    reader = SequentialReader()
    try:
        reader.open(storage_options, converter_options)
    except Exception as e:
        print(f'Failed to open bag {bag_path}: {e}')
        sys.exit(1)

    topics_and_types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    type_classes = {}
    for name, type_name in topics_and_types.items():
        try:
            type_classes[name] = get_message(type_name)
        except Exception:
            pass

    print('=' * 88)
    print(f'SLAM Delay & TF Latency Analysis: {os.path.basename(os.path.abspath(bag_path))}')
    print('=' * 88)

    # Metrics storage
    topic_delays = defaultdict(list)  # topic -> [bag_time - header_stamp]
    topic_stamps = defaultdict(list)  # topic -> [bag_time]
    tf_delays = defaultdict(list)  # (parent, child) -> [bag_time - stamp]
    tf_stamps = defaultdict(list)  # (parent, child) -> [(bag_time, stamp)]
    tracking_state_counts = defaultdict(int)
    tracked_points_counts = []

    count = 0
    start_bag_time = None
    end_bag_time = None

    while reader.has_next() and (max_messages == 0 or count < max_messages):
        topic, data, bag_stamp = reader.read_next()
        bag_time_sec = bag_stamp / 1e9

        if start_bag_time is None:
            start_bag_time = bag_time_sec
        end_bag_time = bag_time_sec

        msg_cls = type_classes.get(topic)
        if msg_cls is not None:
            try:
                msg = rclpy.serialization.deserialize_message(data, msg_cls)
            except Exception:
                count += 1
                continue

            # Record topic bag arrival time
            topic_stamps[topic].append(bag_time_sec)

            # Header delay (bag time - header.stamp)
            if hasattr(msg, 'header') and hasattr(msg.header, 'stamp'):
                msg_stamp_sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
                if msg_stamp_sec > 0:
                    topic_delays[topic].append(bag_time_sec - msg_stamp_sec)

            # Special topic inspections
            if topic == '/tf' or topic == '/tf_static':
                if hasattr(msg, 'transforms'):
                    for t in msg.transforms:
                        t_stamp = t.header.stamp.sec + t.header.stamp.nanosec * 1e-9
                        key = (t.header.frame_id, t.child_frame_id)
                        if t_stamp > 0:
                            tf_delays[key].append(bag_time_sec - t_stamp)
                            tf_stamps[key].append((bag_time_sec, t_stamp))

            elif topic == '/slam_status':
                if hasattr(msg, 'tracking_state'):
                    tracking_state_counts[msg.tracking_state] += 1
                if hasattr(msg, 'tracked_points') and hasattr(msg.tracked_points, 'width'):
                    if msg.tracking_state == 2:  # TRACKING_OK
                        tracked_points_counts.append(msg.tracked_points.width)

        count += 1

    duration = (end_bag_time - start_bag_time) if (start_bag_time and end_bag_time) else 0

    print(f'Processed {count:,} messages across {len(topics_and_types)} topics (Duration: {duration:.1f}s)')
    print()

    # 1. Pipeline Delays Table
    print('----------------------------------------------------------------------------------------')
    print(
        f'{"Topic / Transform":<26} | {"Msgs":>6} | {"Rate":>8} | {"Mean":>8} | {"Median":>8} | {"Std":>7} | {"Min":>8} | {"Max":>8}'
    )
    print('----------------------------------------------------------------------------------------')

    ordered_topics = [
        '/image_raw',
        '/annotated_image',
        '/slam_status',
        '/camera_pose',
        '/slam_pose',
        '/slam_delta',
        '/ekf_pose',
    ]

    for t in ordered_topics:
        if t in topic_delays:
            n = len(topic_delays[t])
            rate = (n / duration) if duration > 0 else 0
            print_table_row(t, n, topic_delays[t], rate)

    # TF transforms
    if ('map', 'slam') in tf_delays:
        n = len(tf_delays[('map', 'slam')])
        rate = (n / duration) if duration > 0 else 0
        print_table_row('tf: map -> slam', n, tf_delays[('map', 'slam')], rate)

    if ('map', 'base_link') in tf_delays:
        n = len(tf_delays[('map', 'base_link')])
        rate = (n / duration) if duration > 0 else 0
        print_table_row('tf: map -> base_link', n, tf_delays[('map', 'base_link')], rate)

    print('----------------------------------------------------------------------------------------')
    print()

    # 2. Key Insights
    print('--- Pipeline Delay Summary ---')
    if '/slam_status' in topic_delays:
        slam_med = np.median(topic_delays['/slam_status'])
        print(
            f'• SLAM Status Latency (Camera Frame -> SLAM Output): median {slam_med * 1000:.1f} ms (mean {np.mean(topic_delays["/slam_status"]) * 1000:.1f} ms)'
        )
    if ('map', 'slam') in tf_delays and ('map', 'base_link') in tf_delays:
        tf_slam_med = np.median(tf_delays[('map', 'slam')])
        tf_base_med = np.median(tf_delays[('map', 'base_link')])
        skew = tf_slam_med - tf_base_med
        print(f'• TF map->slam Latency:                                median {tf_slam_med * 1000:.1f} ms')
        print(f'• TF map->base_link Latency:                           median {tf_base_med * 1000:.1f} ms')
        print(f'• Relative TF Lag (map->slam vs map->base_link):       {skew:.3f} s ({skew * 1000:.1f} ms)')

    print()

    # 3. Tracking Status Breakdown
    if tracking_state_counts:
        print('--- SLAM Tracking State Breakdown ---')
        total_slam = sum(tracking_state_counts.values())
        for state_code, state_count in sorted(tracking_state_counts.items()):
            state_name = TRACKING_STATES.get(state_code, f'UNKNOWN({state_code})')
            pct = (state_count / total_slam) * 100
            print(f'  [{state_code:2d}] {state_name:<20}: {state_count:>5} frames ({pct:5.1f}%)')

    if tracked_points_counts:
        pts = np.array(tracked_points_counts)
        print(
            f'• Tracked Map Points (when TRACKING_OK): mean={np.mean(pts):.1f}, median={np.median(pts):.1f}, min={np.min(pts)}, max={np.max(pts)}'
        )

    print()

    # 4. Verbose Percentiles
    if verbose:
        print('--- Detailed Percentile Latencies ---')
        for t in ordered_topics:
            if t in topic_delays:
                print_percentiles(t, topic_delays[t])
        if ('map', 'slam') in tf_delays:
            print_percentiles('tf: map -> slam', tf_delays[('map', 'slam')])
        if ('map', 'base_link') in tf_delays:
            print_percentiles('tf: map -> base_link', tf_delays[('map', 'base_link')])
        print()


def main():
    args = parse_args()
    analyze_bag(args.bag_path, storage_id=args.storage_id, max_messages=args.max_messages, verbose=args.verbose)


if __name__ == '__main__':
    main()
