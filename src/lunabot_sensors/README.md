# lunabot_sensors

This package owns hardware-facing sensor bring-up for the rover.

## What this package is responsible for

`lunabot_sensors` wraps external sensor drivers so the rest of the stack can keep using the repo's public ROS contracts.

For the OAK-D Pro, that means preserving the existing front camera interface:

- `/camera_front/image`
- `/camera_front/camera_info`
- `/camera_front/depth_image`
- `/camera_front/points`
- `camera_front_link`
- `camera_front_optical_frame`

## What this package is not responsible for

It does not own localisation, EKF fusion, AprilTag logic, or Nav2 costmaps. Those packages should not need to know that the upstream device happens to be an OAK-D Pro.

## Key files

- `launch/oakd_pro.launch.py`: hardware wrapper around the Humble `depthai_ros_driver_v3` launch.
- `lunabot_sensors/camera_contract_adapter.py`: normalises public camera topics and image header frame IDs.
- `config/oakd_usb2_degraded.yaml`: safe USB 2 profile for RGB plus intrinsics.
- `config/oakd_usb3_full.yaml`: fuller profile for depth and point cloud after hardware validation.

Point cloud data is republished on `/camera_front/points`, but the upstream frame is preserved by default unless you explicitly set `point_cloud_frame_id`. That is deliberate: changing the frame label without transforming the points would be fiction with extra steps.
