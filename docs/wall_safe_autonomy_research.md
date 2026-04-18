# Wall-safe autonomy — research notes (UK Lunabotics)

This document records the evidence-first inputs used for the wall-safe autonomy stack. **Competition rules** are taken only from the UK Lunabotics rulebook (team PDF). Technical patterns below cite public ROS/robotics sources.

## UK rule (authoritative for compliance)

From the UK Lunabotics Competition Rule Book (Autonomy Rules): the arena walls shall not be used for mapping autonomous navigation and collision avoidance; teams must prove autonomy sensors do not use the walls. Full wording is in the team’s local PDF.

## Technical references (implementation rationale)

- **ROS 2 Nav2 costmaps**: separate observation sources and topics per layer; `nav2_costmap_2d` consumes `PointCloud2` topics configured in YAML ([Nav2 costmap configuration](https://docs.nav2.org/configuration/packages/configuring-costmaps.html)).
- **REP-105**: `map` → `odom` → `base_link` chain; when visual SLAM is disabled, identity `map`→`odom` is a documented debug pattern alongside odometry-only navigation ([REP-105](https://www.ros.org/reps/rep-0105.html)).
- **Point cloud preprocessing**: cropping and plane/region filtering are standard before downstream consumers (e.g. PCL crop box, height filters); we apply an axis-aligned **interior crop in `map`** to drop perimeter wall returns.
- **RTAB-Map**: uses RGB-D for SLAM; disabling it for competition mode removes wall structure from the global pose graph ([rtabmap_ros](https://github.com/introlab/rtabmap_ros)).

## Design choice summary

| Concern | Approach |
|--------|----------|
| No wall structure in global pose | Do not run RTAB-Map in `competition_safe_localisation`; publish static identity `map`→`odom` (same as odom-aligned map for rolling Nav2). |
| Start pose | Keep AprilTag + `start_zone_localiser` + EKF odom. |
| Nav/collision without walls | `wall_exclusion_filter` republishes clouds on `/perception/autonomy/*`; Nav2 and collision monitor subscribe to those topics in competition YAML. |

Last updated: implementation PR (wall-safe autonomy).
