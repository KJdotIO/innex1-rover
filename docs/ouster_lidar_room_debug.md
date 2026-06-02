# Ouster LiDAR Room Debug

This records the lab evidence for the OS1-128 path before the final rover mount
exists.

The purpose is not to claim full autonomy is ready. The purpose is to keep the
expensive sensor, router path, Jetson driver settings, and bandwidth assumptions
from becoming folklore.

## Slate Plus Lab Check, 2026-05-22

Working topology:

```text
Ouster interface box -> Slate Plus LAN
Jetson Ethernet      -> Slate Plus LAN
Mac / ground control -> Slate Plus Wi-Fi
```

Observed addresses:

```text
Jetson wired LAN: 192.168.8.20
Slate Plus:       192.168.8.1
Ouster OS1-128:   192.168.8.176
```

Validation:

- Jetson kept eduroam as the default internet and remote SSH route.
- Slate LAN stayed local-only.
- Mac on Slate Wi-Fi could ping the Jetson at `192.168.8.20` with 0 per cent
  packet loss.
- Mac on Slate Wi-Fi could ping the Ouster at `192.168.8.176` with 0 per cent
  packet loss.
- SSH worked from the Mac to the Jetson over Slate using the private
  `innex1-slate` alias.
- Jetson could reach the Slate router at `192.168.8.1`.
- Ouster was discovered by Avahi as `os-122610007923.local` at `192.168.8.176`.
- The normal Foxglove operator bridge on `8765` was reachable.

The OAK-D was not connected during this check, so the front camera panel was not
useful evidence.

## Historical Low-rate Baseline, 2026-05-22

Known-good room-debug baseline from 2026-05-22:

```text
sensor:        192.168.8.176
udp_dest:      192.168.8.20
mode:          512x10
point_type:    xyz
v_reduction:   2
```

Confirmed ROS topics:

- `/ouster/points`
- `/ouster/scan`
- `/ouster/imu`
- `/ouster/metadata`
- `/ouster/telemetry`

Observed `/ouster/points` behaviour:

```text
rate:          about 7-10 Hz
message size:  about 0.52 MB
ROS bandwidth: about 5.1 MB/s
```

Foxglove LiDAR debug was run separately on `ws://192.168.8.20:8766`, leaving the
normal competition/operator bridge on `8765`.

Foxglove showed the point cloud. It warned about many invalid values, which is
expected for an organised Ouster cloud where no-return or out-of-range slots are
still present in the message. Do not treat that warning as a failure by itself.

## Load Ladder, 2026-05-22

These profiles were tested while the lab-only Foxglove LiDAR bridge on `8766`
was subscribed:

| Profile | Cloud shape | Rate | Payload | Observed load |
|---|---:|---:|---:|---|
| `512x10`, `v_reduction=2`, `xyz` | `64 x 512` | about `9.3-10 Hz` | about `0.52 MB/msg`, `5 MB/s` | `os_driver` about `12%` CPU; Foxglove bridge about `24%` CPU |
| `512x10`, `v_reduction=1`, `xyz` | `128 x 512` | about `9.4-10 Hz` | about `1.05 MB/msg`, `10 MB/s` | `os_driver` about `11%` CPU; Foxglove bridge about `22%` CPU |
| `1024x10`, `v_reduction=2`, `xyz` | `64 x 1024` | about `9.7-10 Hz` | about `1.05 MB/msg`, `10-11 MB/s` | `os_driver` about `12%` CPU; Foxglove bridge about `41%` CPU |
| `1024x10`, `v_reduction=1`, `xyz` | `128 x 1024` | about `9.2-10 Hz` | about `2.10 MB/msg`, `20 MB/s` | `os_driver` about `18%` CPU; Foxglove bridge about `47%` CPU |

All profiles published `/ouster/points`, `/ouster/scan`, `/ouster/imu`,
`/ouster/metadata`, and `/ouster/telemetry`.

Current recommendation:

- Treat `src/lunabot_bringup/config/ouster_lidar_debug.yaml` as the current
  runnable debug config. At the time of writing it uses `1024x10`,
  `point_type=native`, and `v_reduction=4`.
- Keep the historical `512x10`, `point_type=xyz`, `v_reduction=2` profile as a
  fallback if the team needs the lightest previously observed point-cloud load.
- Do not stream heavier profiles through Foxglove except during deliberate LiDAR
  debugging. The Foxglove bridge becomes a bigger load source than the driver.

## Router Bring-up Commands

```bash
ip -br addr show
ping os-122610007923.local
sudo sysctl -w net.core.rmem_max=1048576 net.core.rmem_default=1048576
ros2 launch lunabot_bringup ouster_lidar_foxglove_debug.launch.py
ros2 topic hz /ouster/points
ros2 topic bw /ouster/points
```

The Ouster needs about 60 seconds after power-on before point cloud data should
be expected.

## Open Mount And TF Work

The remaining competition-readiness work is physical and contractual:

- decide the final mount position after the main frame is ready;
- record the measured transform from `base_link` to the Ouster hardware frame;
- align URDF, driver frame names, localisation config, preflight checks, and
  interface contracts;
- prove the legal LiDAR filter receives clouds with a valid TF chain.

The repo currently has historical evidence for a working room-debug path, but
the frame naming still needs a final sweep before treating LiDAR localisation as
ready.
