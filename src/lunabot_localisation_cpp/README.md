# lunabot_localisation_cpp

C++ localisation hot-path nodes for the rover stack.

The first node in this package is `legal_lidar_filter_cpp`, a field-preserving
filter for the Ouster OS1 localisation path. It subscribes to `/ouster/points`
by default, tests each point against the legal arena interior in `mask_frame`,
and republishes the surviving point records to
`/localisation/lidar/points_legal`.

The output cloud deliberately stays in the original LiDAR frame. The transform
is used only to decide whether each point is inside the legal mask. Each kept
record is copied byte-for-byte, so Ouster-native fields such as timestamp, ring,
reflectivity, intensity and near-infrared data are still available to LiDAR
odometry backends.

`lunabot_localisation/launch/localisation.launch.py` selects this C++ node by
default with `legal_lidar_filter_impl:=cpp`. Use
`legal_lidar_filter_impl:=python` only as a fallback when diagnosing the C++
path.

## Hardware Notes

The OS1 debug config currently publishes native 1024 x 32 clouds at about
10 Hz with `point_step` 48 and BEST_EFFORT QoS. On the Jetson, the C++ filter
has been smoke-tested against that live stream with `odom` statically attached
to `os_sensor`; it processed 32768-point native clouds in about 18 ms.

For quick hardware probes, do not publish a second static transform directly to
`os_lidar`. The Ouster driver already publishes `os_sensor -> os_lidar`, so a
test root should attach to `os_sensor` to avoid disconnected TF trees.
