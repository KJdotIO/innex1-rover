"""Tests for the OAK-D front camera ROS topic contract."""

import pytest

from lunabot_bringup.oakd_camera_contract import (
    camera_launch_arguments,
    camera_topic_remappings,
    normalise_bool_text,
    select_depthai_launch_file,
)


@pytest.mark.parametrize(
    ("text", "expected"),
    [
        ("true", True),
        ("True", True),
        ("1", True),
        ("yes", True),
        ("on", True),
        ("false", False),
        ("False", False),
        ("0", False),
        ("no", False),
        ("off", False),
    ],
)
def test_normalise_bool_text_accepts_common_launch_values(text, expected):
    assert normalise_bool_text(text, "enable_depth") is expected


def test_normalise_bool_text_rejects_unknown_launch_value():
    with pytest.raises(ValueError, match="enable_depth"):
        normalise_bool_text("sometimes", "enable_depth")


def test_select_depthai_launch_file_uses_camera_launch_for_all_output_sets():
    assert select_depthai_launch_file(enable_pointcloud=True) == "camera.launch.py"
    assert select_depthai_launch_file(enable_pointcloud=False) == "camera.launch.py"


def test_camera_topic_remappings_match_existing_front_camera_contract():
    remappings = camera_topic_remappings(
        driver_name="oak_front",
        use_rectified_rgb=True,
        enable_depth=True,
        enable_pointcloud=True,
    )

    assert ("oak_front/rgb/image_rect", "/camera_front/image") in remappings
    assert ("oak_front/rgb/camera_info", "/camera_front/camera_info") in remappings
    assert ("oak_front/stereo/image_raw", "/camera_front/depth_image") in remappings
    assert ("oak_front/points", "/camera_front/points") in remappings


def test_camera_topic_remappings_can_disable_depth_outputs():
    remappings = camera_topic_remappings(
        driver_name="oak_front",
        use_rectified_rgb=False,
        enable_depth=False,
        enable_pointcloud=False,
    )

    assert remappings == [
        ("oak_front/rgb/image_raw", "/camera_front/image"),
        ("oak_front/rgb/camera_info", "/camera_front/camera_info"),
    ]


def test_camera_launch_arguments_keep_depth_optional_and_usb3_first():
    launch_args = camera_launch_arguments(
        driver_name="oak_front",
        camera_model="OAK-D-PRO",
        parent_frame="camera_front_link",
        params_file="/tmp/oakd_front.yaml",
        enable_depth=True,
        enable_pointcloud=False,
        use_rectified_rgb=True,
    )

    assert launch_args["name"] == "oak_front"
    assert launch_args["camera_model"] == "OAK-D-PRO"
    assert launch_args["parent_frame"] == "camera_front_link"
    assert launch_args["params_file"] == "/tmp/oakd_front.yaml"
    assert launch_args["enable_color"] == "true"
    assert launch_args["enable_depth"] == "true"
    assert launch_args["pointcloud.enable"] == "false"
    assert launch_args["rectify_rgb"] == "true"
