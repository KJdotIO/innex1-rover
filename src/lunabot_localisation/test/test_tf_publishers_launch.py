"""Subprocess launch tests for localisation TF publishers."""

from __future__ import annotations

import atexit
import os
from pathlib import Path
import subprocess
import time

import pytest

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


def _test_environment():
    """Return a clean ROS environment for launch and graph probes."""
    env = os.environ.copy()
    env["ROS_DOMAIN_ID"] = TEST_ROS_DOMAIN_ID
    env.pop("FASTRTPS_DEFAULT_PROFILES_FILE", None)
    env.pop("RMW_FASTRTPS_USE_QOS_FROM_XML", None)
    return env


def _run_ros_command(env, *args):
    """Run a ROS CLI command and return combined output."""
    completed = subprocess.run(
        args,
        capture_output=True,
        text=True,
        env=env,
        check=False,
    )
    return completed.stdout + completed.stderr


def _publisher_count(topic_info_output):
    """Count publisher entries in `ros2 topic info -v` output."""
    return topic_info_output.count("Node name:")


@pytest.mark.parametrize(
    (
        "lidar_costmap_phase",
        "expected_nodes",
        "expected_tf_publishers",
        "expected_tf_static_publishers",
    ),
    [
        (
            False,
            {"/ekf_filter_node_odom", "/ekf_filter_node_map"},
            2,
            1,
        ),
        (
            True,
            {"/ekf_filter_node_odom"},
            1,
            2,
        ),
    ],
)
def test_localisation_launch_tf_publishers_match_expected_modes(
    lidar_costmap_phase,
    expected_nodes,
    expected_tf_publishers,
    expected_tf_static_publishers,
):
    env = _test_environment()
    launch_process = subprocess.Popen(
        [
            "ros2",
            "launch",
            "lunabot_localisation",
            "localisation.launch.py",
            f"lidar_costmap_phase:={'true' if lidar_costmap_phase else 'false'}",
            "enable_visual_slam:=false",
            "enable_apriltag_debug:=false",
            "use_sim_time:=true",
        ],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        env=env,
    )

    try:
        deadline = time.time() + 20.0
        node_list_output = ""
        tf_info_output = ""
        tf_static_info_output = ""

        while time.time() < deadline:
            node_list_output = _run_ros_command(env, "ros2", "node", "list")
            tf_info_output = _run_ros_command(env, "ros2", "topic", "info", "/tf", "-v")
            tf_static_info_output = _run_ros_command(
                env,
                "ros2",
                "topic",
                "info",
                "/tf_static",
                "-v",
            )

            nodes = {
                line.strip()
                for line in node_list_output.splitlines()
                if line.strip().startswith("/")
            }

            if (
                expected_nodes.issubset(nodes)
                and _publisher_count(tf_info_output) == expected_tf_publishers
                and _publisher_count(tf_static_info_output)
                == expected_tf_static_publishers
            ):
                break

            if launch_process.poll() is not None:
                break

            time.sleep(0.5)

        nodes = {
            line.strip()
            for line in node_list_output.splitlines()
            if line.strip().startswith("/")
        }
        assert expected_nodes.issubset(nodes), node_list_output
        assert _publisher_count(tf_info_output) == expected_tf_publishers, tf_info_output
        assert (
            _publisher_count(tf_static_info_output) == expected_tf_static_publishers
        ), tf_static_info_output
    finally:
        launch_process.terminate()
        try:
            launch_process.wait(timeout=5.0)
        except subprocess.TimeoutExpired:
            launch_process.kill()
            launch_process.wait(timeout=5.0)

