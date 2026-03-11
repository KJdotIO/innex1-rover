# PR: Establish `feat/rewrite-from-ci-baseline` as the rewrite starting point

## Summary

This branch is intended to preserve the current `feat/ci-interface-sanity-checks` behavior as a known-good operational baseline before a ground-up rewrite.

The key result from validation is:

- the rover does navigate to goals successfully
- obstacle avoidance is working in practice in simulation
- the stack is not "fully healthy"
- the current success appears to come from the rest of the navigation stack being robust to degraded visual odometry, not from RTAB-Map RGB-D odometry being reliable

This PR does **not** claim that localization is solved. It documents the current state honestly so the rewrite starts from a stable and correctly-understood base.

## What Was Observed

During repeated validation runs on the `feat/ci-interface-sanity-checks` baseline:

- Nav2 accepted goals and completed them successfully
- RViz goaling in the `map` frame worked as expected
- the rover avoided obstacles well enough in the simulated arena
- `rgbd_odometry` frequently degraded to `quality=0`
- `rgbd_odometry` repeatedly reported registration failures and low-inlier failures
- `apriltag_ros` repeatedly warned that image and `camera_info` were poorly synchronized
- the local costmap occasionally warned that the sensor origin was outside costmap bounds for raytracing

Representative runtime evidence:

```text
[bt_navigator] Goal succeeded
[controller_server] Reached the goal!
```

```text
[rgbd_odometry] Odom: quality=0, std dev=0.000000m|0.000000rad
[rgbd_odometry] Registration failed: "Not enough inliers ..."
```

```text
[apriltag] Topics '/camera_front/image' and '/camera_front/camera_info' do not appear to be synchronized
```

```text
[local_costmap.local_costmap] Sensor origin ... is out of map bounds ... The costmap cannot raytrace for it.
```

## Interpretation

The important conclusion is:

- the system is operational
- the visual odometry subsystem is degraded
- those two statements are not contradictory

This branch still fuses wheel odometry, IMU, and visual odometry in the EKFs, but the continuous navigation backbone is effectively wheel odometry + IMU. Visual odometry is not the sole thing keeping Nav2 alive.

As a result, the rover can still:

- plan
- follow paths
- reach goals
- avoid obstacles

even while RTAB-Map RGB-D odometry quality collapses.

So the correct description of the current state is:

- navigation works
- obstacle avoidance works
- drift still exists
- VO is not trustworthy enough to be treated as the primary localization source

## What The Research Says

Relevant upstream references:

- RTAB-Map issue `#1399`: <https://github.com/introlab/rtabmap/issues/1399>
- RTAB-Map ROS issue `#1054`: <https://github.com/introlab/rtabmap_ros/issues/1054>
- RTAB-Map ROS issue `#1175`: <https://github.com/introlab/rtabmap_ros/issues/1175>
- RTAB-Map wiki, lost odometry notes: <https://github.com/introlab/rtabmap/wiki/Kinect-mapping>

Important takeaways:

- `quality=0` is not universally fatal in RTAB-Map
- however, in this project we are using RTAB-Map's own RGB-D odometry, not an external VINS backend
- repeated `quality=0` plus repeated `Not enough inliers` is therefore meaningful degradation here
- approximate sync producing pairs does not prove those pairs are good pairs
- large RGB/depth timestamp deltas are a known cause of weak odometry
- low-feature scenes, empty-space views, fast motion, and RGB-depth misalignment are all known causes of odometry loss

## What This Means For The Rewrite

This validation changes the architectural priority:

- do not treat RTAB-Map RGB-D odometry as the foundation of the system
- do not throw away depth cameras
- do not conclude that "VIO is useless"

The more defensible direction is:

1. Keep wheel odometry + IMU as the continuous control backbone.
2. Keep depth cameras as the primary obstacle perception source.
3. Use AprilTags or other absolute anchors to correct drift.
4. Treat visual odometry as optional or opportunistic until it proves robust in the arena.

## Why Branch From Here

This branch is the right rewrite base because it gives us:

- a stack that actually drives
- a stack that actually reaches goals
- a reproducible example of what still fails
- a clear boundary between "operational now" and "architecturally trustworthy later"

The mistake would be to rewrite from a branch that looked cleaner on paper but did not navigate in practice.

## Follow-Up Work

The rewrite should target:

- simplified localization ownership
- honest separation of control-critical odometry vs experimental VO
- better camera timing and synchronization
- removal of misleading assumptions that RTAB-Map VO is healthy when it is not
- costmap cleanup around the sensor-origin raytracing warning

## Non-Goals Of This PR

This PR does not:

- fix RTAB-Map RGB-D odometry
- fix AprilTag synchronization
- eliminate drift
- claim competition readiness

It only preserves the best currently-working baseline and documents the actual problem correctly before the rewrite starts.
