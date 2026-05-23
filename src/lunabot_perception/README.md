# lunabot_perception

Perception nodes for the innex1 rover, providing arena wall-exclusion and
crater (negative obstacle) detection for Nav2 costmap integration.

## Nodes

### `arena_boundary_filter`

Transforms an incoming `PointCloud2` into the arena frame, rejects returns
outside the configured legal interior, and republishes the remaining points for
autonomy consumers. Raw sensor topics stay available for debug and evidence,
but Nav2, collision monitor, and crater detection should consume only the
filtered `/perception/arena_boundary/...` topics.

The default arena bounds match the current simulation yard interior:
`x=[-1.0, 6.9]`, `y=[-3.3, 1.1]`, with a `0.35 m` wall-exclusion margin.

**Run manually (simulation):**

```bash
ros2 run lunabot_perception arena_boundary_filter --ros-args \
  -p use_sim_time:=true \
  -p input_topic:=/camera_front/points \
  -p output_topic:=/perception/arena_boundary/camera_front/points \
  -p source_name:=camera_front
```

**Subscriptions:**

| Topic | Type | Purpose |
|-------|------|---------|
| configurable `input_topic` | `sensor_msgs/PointCloud2` | Raw sensor cloud for one source |

**Publications:**

| Topic | Type | Purpose |
|-------|------|---------|
| configurable `output_topic` | `sensor_msgs/PointCloud2` | Wall-excluded cloud in `target_frame` |
| `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | Filter health, counts, rejected ratio and bounds |

**Key parameters:**

| Parameter | Default | Description |
|-----------|---------|-------------|
| `target_frame` | `odom` | Arena frame used for boundary checks |
| `arena_min_x` / `arena_max_x` | `-1.0` / `6.9` | Interior x bounds before margin |
| `arena_min_y` / `arena_max_y` | `-3.3` / `1.1` | Interior y bounds before margin |
| `wall_exclusion_margin_m` | `0.35` | Band removed around all arena walls |
| `stale_timeout_s` | `2.5` | Diagnostic stale threshold |

### `crater_detection`

Builds a rolling 2.5D elevation grid from the wall-excluded front depth camera
point cloud, identifies cells significantly below the local ground plane, and
publishes an `OccupancyGrid` that Nav2's costmap static layer consumes as
lethal cost.

**Run manually (simulation):**

```bash
ros2 run lunabot_perception crater_detection --ros-args -p use_sim_time:=true
```

**Subscriptions:**

| Topic | Type | Purpose |
|-------|------|---------|
| `/perception/arena_boundary/camera_front/points` | `sensor_msgs/PointCloud2` | Wall-excluded front depth camera cloud |

**Publications:**

| Topic | Type | Purpose |
|-------|------|---------|
| `/crater_grid` | `nav_msgs/OccupancyGrid` | Crater mask for Nav2 costmap (`crater_layer`) |
| `/crater_points_debug` | `sensor_msgs/PointCloud2` | Crater cell centroids for RViz visualisation |

**Key parameters:**

| Parameter | Default | Description |
|-----------|---------|-------------|
| `grid_resolution` | `0.05` | Cell size in metres |
| `grid_width` / `grid_height` | `8.0` / `5.0` | Grid extent in metres |
| `depth_threshold` | `0.08` | Depth below ground to flag as crater |
| `accumulator_decay` | `0.95` | Exponential decay for old observations |
| `inflation_cells` | `2` | Binary dilation radius around detected craters |
| `target_frame` | `odom` | TF frame for the elevation grid |
| `fixed_ground_z` | `NaN` | Set to a known floor height; `NaN` uses auto-percentile |
| `input_cloud_topic` | `/perception/arena_boundary/camera_front/points` | Wall-excluded depth cloud |

## Nav2 integration

The `crater_layer` in both the global and local costmaps is a
`nav2_costmap_2d::StaticLayer` subscribing to `/crater_grid`. See
`lunabot_navigation/config/nav2_params.yaml` for the full costmap plugin
stack. Nav2 obstacle and voxel layers should subscribe to the filtered
`/perception/arena_boundary/...` point-cloud topics, not raw sensor topics.

## Common failure modes

- Running without `use_sim_time:=true` in simulation: the grid timestamps
  will not match Nav2's TF timeline.
- Forgetting to launch this node: the `crater_layer` will repeatedly warn
  "Can't update static costmap layer, no map received" and craters will not
  appear in the costmap.

## Related docs

- Wiki: [Perception](https://github.com/KJdotIO/innex1-rover/wiki/Perception), [Sensors](https://github.com/KJdotIO/innex1-rover/wiki/Sensors), [SoftwareArchitecture](https://github.com/KJdotIO/innex1-rover/wiki/SoftwareArchitecture), [Contracts](https://github.com/KJdotIO/innex1-rover/wiki/Contracts)
- [Nav2 Costmap2D configuration](https://docs.nav2.org/configuration/packages/configuring-costmaps.html)
