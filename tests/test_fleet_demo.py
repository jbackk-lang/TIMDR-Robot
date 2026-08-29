import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from demo.fleet_demo import build_demo_fleet
from timdr_robot.status import AxisHealth


def test_fleet_demo_alpha_is_healthy():
    fleet = build_demo_fleet()
    assert fleet.get_unit("robot_alpha").worst_level() == AxisHealth.OK


def test_fleet_demo_beta_gamma_delta_are_defect_with_expected_component():
    fleet = build_demo_fleet()
    assert fleet.get_unit("robot_beta").worst_level() == AxisHealth.DEFECT
    assert "axis_1" in fleet.get_unit("robot_beta").defect_components()

    assert fleet.get_unit("robot_gamma").worst_level() == AxisHealth.DEFECT
    assert "gripper_0" in fleet.get_unit("robot_gamma").defect_components()

    assert fleet.get_unit("robot_delta").worst_level() == AxisHealth.DEFECT
    assert "power_0" in fleet.get_unit("robot_delta").defect_components()


def test_fleet_demo_worst_fleet_level_is_defect():
    fleet = build_demo_fleet()
    assert fleet.worst_fleet_level() == AxisHealth.DEFECT
