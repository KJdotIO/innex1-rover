from pathlib import Path

LAUNCH_PATH = (
    Path(__file__).resolve().parents[1]
    / "launch"
    / "foxglove_ground_control.launch.py"
)


def test_foxglove_ground_control_launch_uses_compressed_camera_republishers():
    launch_text = LAUNCH_PATH.read_text(encoding="utf-8")

    assert 'package="image_transport"' in launch_text
    assert '"compressed"' in launch_text
    assert '"compressedDepth"' in launch_text
    assert "out.jpeg_quality" in launch_text
    assert "/camera_front/image" in launch_text
    assert "/camera_front/depth_image" in launch_text
    assert "/camera_rear/image" in launch_text
    assert "enable_front_depth" in launch_text


def test_foxglove_ground_control_launch_uses_runtime_profile_allowlist():
    launch_text = LAUNCH_PATH.read_text(encoding="utf-8")

    assert "topic_whitelist" in launch_text
    assert "load_profiles(default_profiles_path())" in launch_text
    assert "send_buffer_limit" in launch_text
