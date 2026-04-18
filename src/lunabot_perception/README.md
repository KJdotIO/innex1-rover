# lunabot_perception

Perception nodes for the innex1 rover: crater (negative obstacle) detection
and optional **wall exclusion** republishers for UK wall-safe autonomy.

## Nodes

### `wall_exclusion_filter`

Subscribes to raw LiDAR and depth `PointCloud2` topics, transforms each cloud
into `map`, and republishes only points inside an axis-aligned **inset interior**
of the arena (drops perimeter wall returns). Used when
`competition_safe_localisation:=true` in `navigation.launch.py`; Nav2 and
collision monitor then consume `/perception/autonomy/*` topics.

Parameters are loaded from `config/wall_exclusion.yaml` (arena bounds match the
Moon Yard sim comments in `lunabot_bringup/config/arena_waypoints.yaml`; tune
for the real UK arena).

### `crater_detection`

Builds a rolling 2.5D elevation grid from the front depth camera's point
cloud, identifies cells significantly below the local ground plane, and
publishes an `OccupancyGrid` that Nav2's costmap static layer consumes as
lethal cost.

**Run manually (simulation):**

```bash
ros2 run lunabot_perception crater_detection --ros-args -p use_sim_time:=true
```

**Subscriptions:**

| Topic | Type | Purpose |
|-------|------|---------|
| `points_topic` param (default `/camera_front/points`) | `sensor_msgs/PointCloud2` | Depth camera point cloud; use `/perception/autonomy/camera_front/points` in wall-safe mode |

**Publications:**

| Topic | Type | Purpose |
|-------|------|---------|
| `/crater_grid` | `nav_msgs/OccupancyGrid` | Crater mask for Nav2 costmap (`crater_layer`) |
| `/crater_points_debug` | `sensor_msgs/PointCloud2` | Crater cell centroids for RViz visualisation |

**Key parameters:**

| Parameter | Default | Description |
|-----------|---------|-------------|
| `points_topic` | `/camera_front/points` | Input cloud; wall-safe launches use `/perception/autonomy/camera_front/points` |
| `grid_resolution` | `0.05` | Cell size in metres |
| `grid_width` / `grid_height` | `8.0` / `5.0` | Grid extent in metres |
| `depth_threshold` | `0.08` | Depth below ground to flag as crater |
| `accumulator_decay` | `0.95` | Exponential decay for old observations |
| `inflation_cells` | `2` | Binary dilation radius around detected craters |
| `target_frame` | `odom` | TF frame for the elevation grid |
| `fixed_ground_z` | `NaN` | Set to a known floor height; `NaN` uses auto-percentile |

## Nav2 integration

The `crater_layer` in both the global and local costmaps is a
`nav2_costmap_2d::StaticLayer` subscribing to `/crater_grid`. See
`lunabot_navigation/config/nav2_params.yaml` for the full costmap plugin
stack (obstacle layer + crater layer + inflation layer).

## Common failure modes

- Running without `use_sim_time:=true` in simulation: the grid timestamps
  will not match Nav2's TF timeline.
- Forgetting to launch this node: the `crater_layer` will repeatedly warn
  "Can't update static costmap layer, no map received" and craters will not
  appear in the costmap.

## Where to read next

- Wiki: [Perception](https://github.com/KJdotIO/innex1-rover/wiki/Perception), [Sensors](https://github.com/KJdotIO/innex1-rover/wiki/Sensors), [SoftwareArchitecture](https://github.com/KJdotIO/innex1-rover/wiki/SoftwareArchitecture), [Contracts](https://github.com/KJdotIO/innex1-rover/wiki/Contracts)
- [Nav2 Costmap2D configuration](https://docs.nav2.org/configuration/packages/configuring-costmaps.html)
