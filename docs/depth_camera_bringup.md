# Depth camera bring-up

This page is for testing the OAK-D cameras on the Jetson before we connect them to the full rover stack.

The aim is simple: prove that the camera is visible, prove that ROS is publishing the topics our stack expects, and only then test the crater pipeline. Don't start with RTAB-Map or Nav2. If the depth image or point cloud is wrong, the higher-level tools will only make the failure harder to read.

## What the rover expects

The front depth camera contract is:

| Topic | Type | Used by |
| --- | --- | --- |
| `/camera_front/image` | `sensor_msgs/Image` | operator view, AprilTag detection |
| `/camera_front/depth_image` | `sensor_msgs/Image` | localisation and depth checks |
| `/camera_front/camera_info` | `sensor_msgs/CameraInfo` | AprilTag and RGBD consumers |
| `/camera_front/points` | `sensor_msgs/PointCloud2` | crater detection |

The rear camera can use the same pattern later under `/camera_rear/...`.

## Why this matches the upstream driver

Luxonis recommends `depthai_ros_driver` for ROS 2. Our bring-up launch uses that driver directly, starts its RGBD point-cloud path, and uses the standard ROS `topic_tools relay` package for the short rover-facing aliases. There is no project camera bridge node to maintain.

The same docs say the driver can publish camera TF from calibration. That is useful for a camera-only check, but for the rover stack we still need the camera frames to agree with the rover URDF and robot state publisher. In other words, the OAK can tell ROS about its internal camera frames; the rover still owns where the camera sits on the chassis.

## Jetson setup check

On the Jetson:

```bash
cd ~/innex1-rover
source /opt/ros/humble/setup.bash
source install/setup.bash

lsusb
ros2 pkg list | grep depthai
python3 -c "import depthai; print(depthai.__version__)"
```

For ROS 2 Humble, start from:

```bash
sudo apt update
sudo apt install ros-humble-depthai-ros ros-humble-topic-tools
```

If the native Python test is needed too, install the Python dependencies in the Jetson environment used for testing.

The May 2026 bench test found one awkward but useful detail: the OAK-D Pro came
up at USB 3 speed over the USB-C path, while the USB-A cable tested that night
did not give a usable camera connection. Check `lsusb -t` before chasing ROS
launch problems.

## Native camera test

Use this before ROS:

```bash
cd ~/innex1-rover
python3 tools/oakd_apriltag_test.py --show-depth
```

This proves USB, camera power, DepthAI Python, RGB and stereo depth. If this does not work, don't debug Nav2 yet.

## ROS camera test

Once the driver is installed and the camera is visible:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch lunabot_bringup oak_front_camera.launch.py
```

In another terminal, inspect the actual topics:

```bash
ros2 topic list | grep -E 'camera_front|rgb|stereo|depth|points'
ros2 topic hz /camera_front/image
ros2 topic hz /camera_front/depth_image
ros2 topic hz /camera_front/points
```

The driver still exposes its native `/camera_front/rgb/...` and `/camera_front/stereo/...` topics. The rover-facing aliases are the project names listed at the top of this page.

## RViz view

Use the dedicated RViz profile:

```bash
ros2 launch lunabot_bringup oak_front_camera.launch.py use_rviz:=true
```

The fixed frame is `camera_front_rgb_camera_optical_frame`, so a camera-only point cloud can render without the whole navigation stack.

## Foxglove operator view

For a live operator check, do not stream raw RGB, raw depth or point clouds to
Foxglove. The bench result was clear: the heavy Foxglove debug layout fell
several seconds behind and hit the tab buffer limit. The lean setup, with
compressed RGB at about 8 Hz and no depth or point cloud panel, stayed close to
live.

Use `docs/foxglove/oak_camera_lean.layout.json` for this check. Keep only one
Foxglove tab connected.

Depth and point cloud are still useful, just not as the normal live operator
view. Use RViz or ROS topic checks for those, then stop them.

## Crater pipeline dry test

Only run this once `/camera_front/points` is publishing and TF exists from the point cloud frame to `odom`:

```bash
ros2 launch lunabot_bringup depth_camera_debug.launch.py run_crater_detection:=true
```

Then enable `Crater Grid` and `Crater Points Debug` in RViz. In a normal room this is not a meaningful crater test. It is only checking that the node receives point clouds, transforms them, and publishes debug outputs. A proper crater test still needs a shallow depression or a controlled test pit.

## What not to over-read

A room point cloud is useful for checking that the camera is alive and that nearby surfaces appear at plausible distances. It does not prove the rover can map the arena. Builder's sand, craters and low-texture ground are the real test.

Similarly, a clean RGB image does not prove AprilTag localisation. The tag test needs the printed tag, known tag size, camera calibration and repeatable measurement from the start-zone reference.
