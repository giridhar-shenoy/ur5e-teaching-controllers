"""The controllers' model gravity must be the simulator's gravity.

SDF's default gravity is -9.8 m/s^2 and both dynamics backends default to -9.81; in v0.1 the
mismatch showed up as a constant ~0.1 N error in the hand-guiding hold force until the worlds set
<gravity> explicitly. This keeps the worlds and the controllers' gravity_z parameter in step.
"""

import xml.etree.ElementTree as ET
from pathlib import Path

import pytest
import yaml

ROOT = Path(__file__).resolve().parents[3]
BRINGUP = ROOT / "src" / "ur5e_teaching_bringup"
CONTROLLERS = ROOT / "src" / "ur5e_teaching_controllers" / "config"
EFFORT_CONTROLLERS = (
    "pid_ff_trajectory_controller",
    "gravity_comp_handguide_controller",
    "tcp_jog_controller",
)


def world_gravity_z(path: Path) -> float:
    gravity = ET.parse(path).getroot().find("world/gravity")
    assert gravity is not None, f"{path.name} has no <gravity>; SDF would default to -9.8"
    x, y, z = (float(v) for v in gravity.text.split())
    assert x == 0.0 and y == 0.0, f"{path.name}: gravity must point along -z"
    return z


@pytest.mark.parametrize("world", sorted((BRINGUP / "worlds").glob("*.sdf")), ids=lambda p: p.name)
def test_world_gravity_matches_controller_gravity_z(world):
    config = yaml.safe_load((BRINGUP / "config" / "controllers.yaml").read_text())
    g_world = world_gravity_z(world)
    for name in EFFORT_CONTROLLERS:
        assert config[name]["ros__parameters"]["gravity_z"] == g_world, name


def test_parameter_default_matches_the_worlds():
    worlds = {world_gravity_z(p) for p in (BRINGUP / "worlds").glob("*.sdf")}
    for name in EFFORT_CONTROLLERS:
        spec = yaml.safe_load((CONTROLLERS / f"{name}_parameters.yaml").read_text())
        assert {spec[name]["gravity_z"]["default_value"]} == worlds, name
