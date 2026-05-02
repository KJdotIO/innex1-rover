# Wall-safe autonomy — verification checklist

Use this with `competition_safe_localisation:=true` and the UK rulebook (local PDF) for judge-facing proof.

## Launch

- `ros2 launch lunabot_bringup navigation.launch.py competition_safe_localisation:=true`
- Expect: no `rtabmap` process; static identity `map`→`odom`; `wall_exclusion_filter` running; Nav2/collision monitor subscribed to `/perception/autonomy/*`.

## Topics (Foxglove / `ros2 topic list`)

| Raw (debug) | Autonomy (competition) |
|-------------|-------------------------|
| `/ouster/points` | `/perception/autonomy/ouster/points` |
| `/camera_front/points` | `/perception/autonomy/camera_front/points` |
| `/camera_rear/points` | `/perception/autonomy/camera_rear/points` |

## Automated tests (local CI)

- `lunabot_navigation` — `test_obstacle_avoidance_config.py` includes competition YAML checks.
- `lunabot_perception` — `test_wall_exclusion_geometry.py` exercises interior crop math.
- `lunabot_localisation` — launch validation includes `competition_safe_localisation`.

## Manual / bag replay

- Record a short bag with the rover near a wall; verify `/perception/autonomy/*` clouds drop returns outside the inset arena rectangle in `map` while interior obstacles remain.
- Run the same mission with `competition_safe_localisation:=false` only in simulation for SLAM development; do not use that mode for UK compliance demos.
