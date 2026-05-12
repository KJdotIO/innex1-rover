from pathlib import Path

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
LAUNCH_PATH = PACKAGE_ROOT / "launch" / "oak_front_camera.launch.py"
CONFIG_PATH = PACKAGE_ROOT / "config" / "oak_front_rgbd.yaml"
SETUP_PATH = PACKAGE_ROOT / "setup.py"
PACKAGE_XML_PATH = PACKAGE_ROOT / "package.xml"


def test_oak_front_camera_launch_uses_depthai_driver_with_remaps():
    launch_text = LAUNCH_PATH.read_text(encoding="utf-8")

    assert "depthai_ros_driver" in launch_text
    assert "camera.launch.py" in launch_text
    assert "oak_front_rgbd.yaml" in launch_text
    assert "/camera_front" in launch_text
    assert "topic_tools" in launch_text


def test_oak_front_camera_launch_exposes_pointcloud_switch():
    launch_text = LAUNCH_PATH.read_text(encoding="utf-8")

    assert "enable_pointcloud" in launch_text
    assert "pointcloud.enable" in launch_text


def test_oak_front_camera_launch_maps_depthai_topics_to_rover_contract():
    launch_text = LAUNCH_PATH.read_text(encoding="utf-8")

    assert '"name": "camera_front"' in launch_text
    assert "/camera_front/rgb/image_raw" in launch_text
    assert "/camera_front/stereo/image_raw" in launch_text
    assert "/camera_front/rgb/camera_info" in launch_text
    assert "/camera_front/image" in launch_text
    assert "/camera_front/depth_image" in launch_text
    assert "/camera_front/camera_info" in launch_text
    assert "input_topic" in launch_text
    assert "output_topic" in launch_text


def test_oak_front_camera_uses_installed_packages_not_custom_bridge():
    setup_text = SETUP_PATH.read_text(encoding="utf-8")
    package_xml = PACKAGE_XML_PATH.read_text(encoding="utf-8")

    assert "oak_camera_bridge" not in setup_text
    assert "<exec_depend>depthai_ros_driver</exec_depend>" in package_xml
    assert "<exec_depend>topic_tools</exec_depend>" in package_xml


def test_oak_front_config_disables_neural_network_pipeline():
    config_text = CONFIG_PATH.read_text(encoding="utf-8")

    assert "/camera_front:" in config_text
    assert "i_nn_type: none" in config_text
    assert "i_subpixel: true" in config_text
