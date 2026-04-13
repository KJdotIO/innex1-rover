"""Validate the integrated obstacle avoidance configuration.

Checks that the Nav2 costmap layers and collision monitor config are
internally consistent: sensor topics, height bounds, plugin ordering,
and safety polygon definitions are all coherent.
"""

from pathlib import Path

import yaml

CONFIG_DIR = Path(__file__).resolve().parents[1] / "config"
NAV2_PARAMS_PATH = CONFIG_DIR / "nav2_params.yaml"
COLLISION_MONITOR_PATH = CONFIG_DIR / "collision_monitor.yaml"


def _load_yaml(path: Path) -> dict:
    return yaml.safe_load(path.read_text())


def _costmap_params(config: dict, costmap_key: str) -> dict:
    return config[costmap_key][costmap_key]["ros__parameters"]


class TestCostmapPluginOrder:
    """Verify both costmaps have consistent plugin ordering."""

    def setup_method(self):
        self.config = _load_yaml(NAV2_PARAMS_PATH)

    def test_inflation_layer_is_last_in_global(self):
        plugins = _costmap_params(self.config, "global_costmap")["plugins"]
        assert plugins[-1] == "inflation_layer"

    def test_inflation_layer_is_last_in_local(self):
        plugins = _costmap_params(self.config, "local_costmap")["plugins"]
        assert plugins[-1] == "inflation_layer"

    def test_global_and_local_have_same_plugins(self):
        global_plugins = _costmap_params(self.config, "global_costmap")["plugins"]
        local_plugins = _costmap_params(self.config, "local_costmap")["plugins"]
        assert global_plugins == local_plugins

    def test_obstacle_layer_present(self):
        for key in ("global_costmap", "local_costmap"):
            plugins = _costmap_params(self.config, key)["plugins"]
            assert "obstacle_layer" in plugins

    def test_crater_layer_present(self):
        for key in ("global_costmap", "local_costmap"):
            plugins = _costmap_params(self.config, key)["plugins"]
            assert "crater_layer" in plugins

    def test_each_plugin_has_config_block(self):
        for key in ("global_costmap", "local_costmap"):
            params = _costmap_params(self.config, key)
            for plugin_name in params["plugins"]:
                assert plugin_name in params, (
                    f"{key}: plugin '{plugin_name}' listed but has no config block"
                )


class TestObstacleLayerConfig:
    """Verify the depth camera obstacle layer configuration."""

    def setup_method(self):
        config = _load_yaml(NAV2_PARAMS_PATH)
        self.layers = []
        for key in ("global_costmap", "local_costmap"):
            params = _costmap_params(config, key)
            if "obstacle_layer" in params:
                self.layers.append((key, params["obstacle_layer"]))

    def test_depth_camera_topic(self):
        for costmap_key, layer in self.layers:
            assert layer["camera_points"]["topic"] == "/camera_front/points", (
                f"{costmap_key}: unexpected camera topic"
            )

    def test_height_bounds_are_positive(self):
        for costmap_key, layer in self.layers:
            pts = layer["camera_points"]
            assert pts["min_obstacle_height"] > 0.0
            assert pts["max_obstacle_height"] > pts["min_obstacle_height"]

    def test_marking_and_clearing_enabled(self):
        for costmap_key, layer in self.layers:
            pts = layer["camera_points"]
            assert pts["marking"] is True
            assert pts["clearing"] is True


class TestCraterLayerConfig:
    """Verify the crater layer references the correct topic."""

    def setup_method(self):
        config = _load_yaml(NAV2_PARAMS_PATH)
        self.layers = []
        for key in ("global_costmap", "local_costmap"):
            params = _costmap_params(config, key)
            if "crater_layer" in params:
                self.layers.append((key, params["crater_layer"]))

    def test_crater_grid_topic(self):
        for costmap_key, layer in self.layers:
            assert layer["map_topic"] == "/crater_grid"

    def test_static_layer_plugin(self):
        for costmap_key, layer in self.layers:
            assert layer["plugin"] == "nav2_costmap_2d::StaticLayer"


class TestInflationLayerConfig:
    """Verify inflation radii are set and reasonable."""

    def setup_method(self):
        config = _load_yaml(NAV2_PARAMS_PATH)
        self.global_inflation = _costmap_params(
            config, "global_costmap"
        )["inflation_layer"]
        self.local_inflation = _costmap_params(
            config, "local_costmap"
        )["inflation_layer"]

    def test_inflation_radius_positive(self):
        assert self.global_inflation["inflation_radius"] > 0.0
        assert self.local_inflation["inflation_radius"] > 0.0

    def test_global_inflation_at_least_local(self):
        assert (
            self.global_inflation["inflation_radius"]
            >= self.local_inflation["inflation_radius"]
        )


class TestCollisionMonitorConfig:
    """Verify the collision monitor YAML is well-formed."""

    def setup_method(self):
        config = _load_yaml(COLLISION_MONITOR_PATH)
        self.params = config["collision_monitor"]["ros__parameters"]

    def test_velocity_topics(self):
        assert self.params["cmd_vel_in_topic"] == "cmd_vel"
        assert self.params["cmd_vel_out_topic"] == "cmd_vel_safe"

    def test_stop_polygon_exists(self):
        assert "PolygonStop" in self.params["polygons"]
        assert self.params["PolygonStop"]["action_type"] == "stop"

    def test_slow_polygon_exists(self):
        assert "PolygonSlow" in self.params["polygons"]
        assert self.params["PolygonSlow"]["action_type"] == "slowdown"

    def test_slowdown_ratio_bounded(self):
        ratio = self.params["PolygonSlow"]["slowdown_ratio"]
        assert 0.0 < ratio < 1.0

    def test_observation_sources_not_empty(self):
        sources = self.params["observation_sources"]
        assert len(sources) >= 1

    def test_all_sources_have_topic(self):
        for source_name in self.params["observation_sources"]:
            source = self.params[source_name]
            assert "topic" in source, f"Source '{source_name}' missing topic"
            assert source["topic"].startswith("/"), (
                f"Source '{source_name}' topic should be absolute"
            )

    def test_height_bounds_consistent(self):
        for source_name in self.params["observation_sources"]:
            source = self.params[source_name]
            assert source["min_height"] < source["max_height"]

    def test_stop_polygon_inside_slow_polygon(self):
        stop_pts = self.params["PolygonStop"]["points"]
        slow_pts = self.params["PolygonSlow"]["points"]
        stop_x = [stop_pts[i] for i in range(0, len(stop_pts), 2)]
        slow_x = [slow_pts[i] for i in range(0, len(slow_pts), 2)]
        assert max(stop_x) <= max(slow_x), (
            "Stop polygon front edge should not exceed slow polygon"
        )
