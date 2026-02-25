#!/usr/bin/env python3

import argparse
import time

import rclpy
from rclpy.node import Node


DEFAULT_TOPICS = [
    "/clock",
    "/odom",
    "/joint_states",
    "/imu/data_raw",
    "/scan",
    "/camera_front/image",
    "/camera_front/camera_info",
    "/camera_front/depth_image",
    "/camera_front/points",
    "/camera/image_raw",
    "/camera/camera_info",
]


class TopicProbe(Node):
    def __init__(self):
        super().__init__("sim_topic_probe")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Check required ROS topics are present"
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=45.0,
        help="Timeout in seconds for topic discovery",
    )
    parser.add_argument(
        "--topics",
        nargs="*",
        default=DEFAULT_TOPICS,
        help="Topic names that must be present",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    deadline = time.time() + args.timeout

    rclpy.init()
    node = TopicProbe()

    missing = set(args.topics)
    while time.time() < deadline and missing:
        names_and_types = node.get_topic_names_and_types()
        known_topics = {name for name, _ in names_and_types}
        missing = {topic for topic in args.topics if topic not in known_topics}
        if missing:
            time.sleep(0.5)

    node.destroy_node()
    rclpy.shutdown()

    if missing:
        print("Missing topics after timeout:")
        for topic in sorted(missing):
            print(f"- {topic}")
        return 1

    print("All required topics are present")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
