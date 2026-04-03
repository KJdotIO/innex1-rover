"""Launch tests for dual-EKF TF publisher ownership."""

from __future__ import annotations

import atexit
import os
from pathlib import Path
import threading
import time

import launch
from launch.actions import IncludeLaunchDescription
from launch.actions import SetEnvironmentVariable
from launch.actions import UnsetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
import pytest
import rclpy
from rclpy.node import Node as RclpyNode

TEST_ROS_DOMAIN_LOCK_FD = None
TEST_ROS_DOMAIN_LOCK_PATH = None


def _reserve_test_domain():
    """Reserve a ROS domain id for this test run on the current machine."""
    global TEST_ROS_DOMAIN_LOCK_FD
    global TEST_ROS_DOMAIN_LOCK_PATH

    domain_ids = list(range(1, 102)) + list(range(215, 233))
    start_index = (os.getpid() + time.time_ns()) % len(domain_ids)

    for offset in range(len(domain_ids)):
        domain_id = domain_ids[(start_index + offset) % len(domain_ids)]
        lock_path = Path(f"/tmp/lunabot_ros_domain_{domain_id}.lock")
        try:
            lock_fd = os.open(lock_path, os.O_CREAT | os.O_EXCL | os.O_RDWR)
        except FileExistsError:
            continue
        TEST_ROS_DOMAIN_LOCK_FD = lock_fd
        TEST_ROS_DOMAIN_LOCK_PATH = lock_path
        return str(domain_id)

    raise RuntimeError("No free ROS domain id available for launch test")


def _release_test_domain_reservation():
    """Release any reserved ROS domain id lock for this test run."""
    global TEST_ROS_DOMAIN_LOCK_FD
    global TEST_ROS_DOMAIN_LOCK_PATH

    if TEST_ROS_DOMAIN_LOCK_FD is not None:
        os.close(TEST_ROS_DOMAIN_LOCK_FD)
        TEST_ROS_DOMAIN_LOCK_FD = None

    if TEST_ROS_DOMAIN_LOCK_PATH is not None:
        TEST_ROS_DOMAIN_LOCK_PATH.unlink(missing_ok=True)
        TEST_ROS_DOMAIN_LOCK_PATH = None


TEST_ROS_DOMAIN_ID = _reserve_test_domain()
atexit.register(_release_test_domain_reservation)


class _GraphProbe(RclpyNode):
    """Minimal node for checking active graph publishers."""

    def __init__(self) -> None:
        super().__init__("localisation_tf_graph_probe")

    def publisher_names(self, topic_name):
        """Return publisher node names for a topic."""
        return {
            publisher.node_name
            for publisher in self.get_publishers_info_by_topic(topic_name)
        }


def _launch_localisation(*, lidar_costmap_phase):
    """Start the localisation launch file for the selected mode."""
    launch_path = (
        Path(__file__).resolve().parents[1] / "launch" / "localisation.launch.py"
    )
    description = launch.LaunchDescription(
        [
            SetEnvironmentVariable("ROS_DOMAIN_ID", TEST_ROS_DOMAIN_ID),
            UnsetEnvironmentVariable("FASTRTPS_DEFAULT_PROFILES_FILE"),
            UnsetEnvironmentVariable("RMW_FASTRTPS_USE_QOS_FROM_XML"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(launch_path)),
                launch_arguments={
                    "lidar_costmap_phase": "true" if lidar_costmap_phase else "false",
                    "enable_visual_slam": "false",
                    "use_sim_time": "true",
                    "enable_apriltag_debug": "false",
                }.items(),
            ),
        ]
    )

    launch_service = launch.LaunchService()
    launch_service.include_launch_description(description)
    thread = threading.Thread(target=launch_service.run, daemon=True)
    thread.start()
    return launch_service, thread


@pytest.mark.parametrize(
    (
        "lidar_costmap_phase",
        "expected_tf_publishers",
        "expected_tf_static_count",
    ),
    [
        (False, {"ekf_filter_node_odom", "ekf_filter_node_map"}, 0),
        (True, {"ekf_filter_node_odom"}, 1),
    ],
)
def test_localisation_launch_tf_publishers_match_expected_modes(
    lidar_costmap_phase, expected_tf_publishers, expected_tf_static_count
):
    original_env = {
        "ROS_DOMAIN_ID": os.environ.get("ROS_DOMAIN_ID"),
        "FASTRTPS_DEFAULT_PROFILES_FILE": os.environ.get(
            "FASTRTPS_DEFAULT_PROFILES_FILE"
        ),
        "RMW_FASTRTPS_USE_QOS_FROM_XML": os.environ.get(
            "RMW_FASTRTPS_USE_QOS_FROM_XML"
        ),
    }
    os.environ["ROS_DOMAIN_ID"] = TEST_ROS_DOMAIN_ID
    os.environ.pop("FASTRTPS_DEFAULT_PROFILES_FILE", None)
    os.environ.pop("RMW_FASTRTPS_USE_QOS_FROM_XML", None)

    launch_service, thread = _launch_localisation(
        lidar_costmap_phase=lidar_costmap_phase
    )
    rclpy.init()
    node = _GraphProbe()

    try:
        deadline = time.time() + 15.0
        tf_publishers = set()
        tf_static_publishers = set()

        while time.time() < deadline:
            tf_publishers = node.publisher_names("/tf")
            tf_static_publishers = node.publisher_names("/tf_static")
            if (
                tf_publishers == expected_tf_publishers
                and len(tf_static_publishers) == expected_tf_static_count
            ):
                break
            rclpy.spin_once(node, timeout_sec=0.1)
            time.sleep(0.1)

        assert tf_publishers == expected_tf_publishers
        assert len(tf_static_publishers) == expected_tf_static_count
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        launch_service.shutdown()
        thread.join(timeout=5.0)
        for name, value in original_env.items():
            if value is None:
                os.environ.pop(name, None)
            else:
                os.environ[name] = value

