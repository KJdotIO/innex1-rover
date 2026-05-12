from pathlib import Path

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
LAUNCH_PATH = PACKAGE_ROOT / "launch" / "depth_camera_debug.launch.py"
RVIZ_PATH = PACKAGE_ROOT / "rviz" / "depth_camera_debug.rviz"


def test_depth_camera_debug_launch_opens_dedicated_rviz_profile():
    launch_text = LAUNCH_PATH.read_text(encoding="utf-8")

    assert "depth_camera_debug.rviz" in launch_text
    assert 'package="rviz2"' in launch_text
    assert "launch_rviz" in launch_text
    assert "use_sim_time" in launch_text


def test_depth_camera_debug_launch_can_run_crater_detector_on_demand():
    launch_text = LAUNCH_PATH.read_text(encoding="utf-8")

    assert "run_crater_detection" in launch_text
    assert 'package="lunabot_perception"' in launch_text
    assert 'executable="crater_detection"' in launch_text


def test_depth_camera_debug_rviz_tracks_project_camera_topics():
    rviz_text = RVIZ_PATH.read_text(encoding="utf-8")

    assert "/camera_front/image" in rviz_text
    assert "/camera_front/depth_image" in rviz_text
    assert "/camera_front/points" in rviz_text
    assert "/camera_rear/image" in rviz_text


def test_depth_camera_debug_rviz_includes_crater_debug_outputs():
    rviz_text = RVIZ_PATH.read_text(encoding="utf-8")

    assert "/crater_grid" in rviz_text
    assert "/crater_points_debug" in rviz_text
    assert "camera_front_rgb_camera_optical_frame" in rviz_text
