# Ouster LiDAR Room Debug

This is the first bench check for the real Ouster LiDAR on the Jetson. The aim
is simple: prove that the Jetson can reach the sensor, the official ROS 2 driver
publishes `/ouster/points`, `/ouster/scan` and `/ouster/imu`, and RViz can show a
live point cloud without starting the whole rover stack.

Do not use the LiDAR Ethernet path for internet. Keep the Jetson on Wi-Fi or
Tailscale for SSH, and use the wired Ethernet port only for the Ouster.

## Wiring

1. Connect the Ouster sensor to its interface box.
2. Power the interface box with the correct Ouster supply.
3. Connect Ethernet from the interface box to the Jetson Ethernet port.
4. Wait about a minute for the sensor to boot.

On the Jetson, the wired Ethernet interface has been seen as `enP8p1s0`.

For the sponsored OS1-128 tested on 19 May 2026, discovery returned:

- sensor hostname: `os-122610007923.local`
- sensor IPv4: `169.254.7.93`
- Jetson Ethernet IPv4: `169.254.86.134`
- firmware: `3.1.0`

## Find the Sensor

SSH into the Jetson and check that the Ethernet link is up:

```bash
ip -br addr show enP8p1s0
```

If it still says `DOWN`, check the cable, interface box power and the LiDAR boot
state first.

Try discovering the sensor by hostname or neighbour table:

```bash
avahi-browse -art | grep -i ouster
ip neigh show dev enP8p1s0
```

If the sensor does not appear, give the Jetson Ethernet port a static address on
a private subnet, then check again:

```bash
sudo nmcli con add type ethernet ifname enP8p1s0 con-name ouster-static \
  ipv4.method manual ipv4.addresses 192.168.2.1/24 ipv6.method ignore
sudo nmcli con up ouster-static
ip -br addr show enP8p1s0
```

If the sensor has a known static IP, use that as `SENSOR_HOSTNAME`. If not, try
the Ouster `.local` hostname printed on the unit or shown by Avahi.

## Driver Install

Install the official Humble driver if it is not already present:

```bash
source /opt/ros/humble/setup.bash
ros2 pkg list | grep ouster_ros || sudo apt update
ros2 pkg list | grep ouster_ros || sudo apt install -y ros-humble-ouster-ros
```

The official driver is the important bit here. It configures the sensor and
publishes `/ouster/points` and `/ouster/imu`; we should not write a custom UDP
receiver for this.

## Launch

Set the sensor address and the Jetson Ethernet address:

```bash
source /opt/ros/humble/setup.bash
source ~/innex1-rover/install/setup.bash

export SENSOR_HOSTNAME=<sensor-ip-or-hostname>
export JETSON_ETH_IP=$(ip -4 addr show enP8p1s0 | awk '/inet / {print $2}' | cut -d/ -f1)
```

Start the low-rate debug launch:

```bash
ros2 launch lunabot_bringup ouster_room_debug.launch.py \
  sensor_hostname:=$SENSOR_HOSTNAME \
  udp_dest:=$JETSON_ETH_IP
```

The defaults are deliberately light: `512x10`, `xyz` points and vertical
reduction set to `2`. Once the basics work, rerun with full vertical resolution:

```bash
ros2 launch lunabot_bringup ouster_room_debug.launch.py \
  sensor_hostname:=$SENSOR_HOSTNAME \
  udp_dest:=$JETSON_ETH_IP \
  v_reduction:=1
```

## Verify Topics

In another SSH session:

```bash
source /opt/ros/humble/setup.bash
source ~/innex1-rover/install/setup.bash

ros2 topic list | grep ouster
ros2 topic hz /ouster/points
ros2 topic hz /ouster/scan
ros2 topic echo --once /ouster/imu
```

Open RViz on the Jetson desktop:

```bash
rviz2 -d ~/innex1-rover/install/lunabot_bringup/share/lunabot_bringup/rviz/ouster_room_debug.rviz
```

For a quick evidence bag:

```bash
mkdir -p ~/innex1_mission_evidence/lidar_debug
ros2 bag record -o ~/innex1_mission_evidence/lidar_debug/ouster_room_$(date -u +%Y%m%dT%H%M%SZ) \
  /ouster/points /ouster/scan /ouster/imu /tf /tf_static
```

## If Packets Drop

The Ouster docs recommend increasing Linux receive buffers if scans have missing
sections or the driver reports dropped UDP packets:

```bash
sudo sysctl -w net.core.rmem_max=2147483647
sudo sysctl -w net.core.rmem_default=134217728
```

Only do this when needed. For the first room check, get a live cloud working
before tuning the network.

During the first OS1-128 smoke test, `/ouster/points` published at about 9.5 Hz
in `512x10` mode. The driver warned that the default receive buffer was too
small, so the commands above were applied on the Jetson for the current session.
