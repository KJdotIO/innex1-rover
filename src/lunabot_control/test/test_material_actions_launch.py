"""Tests for the standalone material bench launch."""

import importlib.util
from pathlib import Path


def _load_launch_module():
    launch_path = Path(__file__).resolve().parents[1] / "launch" / "material_actions.launch.py"
    spec = importlib.util.spec_from_file_location("material_actions_launch", launch_path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def test_material_actions_launch_includes_bench_mock_and_servers():
    module = _load_launch_module()

    description = module.generate_launch_description()
    entities = [entity for entity in description.entities if hasattr(entity, "node_package")]
    node_names = {entity.node_name for entity in entities}

    assert node_names == {
        "excavation_telemetry_mock",
        "excavation_controller",
        "excavation_action_server",
        "material_action_server",
    }

