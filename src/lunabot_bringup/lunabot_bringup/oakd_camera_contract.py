"""OAK-D front camera topic contract helpers."""

TRUTHY_VALUES = {"1", "true", "yes", "on"}
FALSEY_VALUES = {"0", "false", "no", "off"}

FRONT_RGB_IMAGE_TOPIC = "/camera_front/image"
FRONT_CAMERA_INFO_TOPIC = "/camera_front/camera_info"
FRONT_DEPTH_IMAGE_TOPIC = "/camera_front/depth_image"
FRONT_POINTS_TOPIC = "/camera_front/points"


def normalise_bool_text(value: str, argument_name: str) -> bool:
    """Return a boolean from a launch-style text value, or fail closed."""
    normalised = str(value).strip().lower()
    if normalised in TRUTHY_VALUES:
        return True
    if normalised in FALSEY_VALUES:
        return False
    raise ValueError(
        f"Expected a boolean-style launch value for '{argument_name}', "
        f"got '{value}'."
    )


def bool_to_launch_text(value: bool) -> str:
    """Return a lowercase launch boolean string."""
    return "true" if value else "false"


def select_depthai_launch_file(enable_pointcloud: bool) -> str:
    """Return the DepthAI launch file needed for the requested output set."""
    _ = enable_pointcloud
    return "camera.launch.py"


def camera_topic_remappings(
    driver_name: str,
    use_rectified_rgb: bool,
    enable_depth: bool,
    enable_pointcloud: bool,
) -> list[tuple[str, str]]:
    """Return remappings from DepthAI topics to the rover front-camera contract."""
    rgb_suffix = "image_rect" if use_rectified_rgb else "image_raw"
    remappings = [
        (f"{driver_name}/rgb/{rgb_suffix}", FRONT_RGB_IMAGE_TOPIC),
        (f"{driver_name}/rgb/camera_info", FRONT_CAMERA_INFO_TOPIC),
    ]
    if enable_depth:
        remappings.append((f"{driver_name}/stereo/image_raw", FRONT_DEPTH_IMAGE_TOPIC))
    if enable_pointcloud:
        remappings.append((f"{driver_name}/points", FRONT_POINTS_TOPIC))
    return remappings


def camera_launch_arguments(
    driver_name: str,
    camera_model: str,
    parent_frame: str,
    params_file: str,
    enable_depth: bool,
    enable_pointcloud: bool,
    use_rectified_rgb: bool,
) -> dict[str, str]:
    """Return launch arguments for the DepthAI camera launch include."""
    return {
        "name": driver_name,
        "camera_model": camera_model,
        "parent_frame": parent_frame,
        "params_file": params_file,
        "enable_color": "true",
        "enable_depth": bool_to_launch_text(enable_depth),
        "pointcloud.enable": bool_to_launch_text(enable_pointcloud),
        "rectify_rgb": bool_to_launch_text(use_rectified_rgb),
    }
