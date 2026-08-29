import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from timdr_robot.fleet import Fleet, RobotUnit
from timdr_robot.status import AxisHealth, StatusEvent


def _event(level, cid="c0"):
    return StatusEvent(axis_id=cid, level=level, message="m", metrics_snapshot={})


def test_robot_unit_worst_level_empty_is_ok():
    unit = RobotUnit("r0")
    assert unit.worst_level() == AxisHealth.OK


def test_robot_unit_worst_level_picks_most_severe():
    unit = RobotUnit("r0", events={
        "a": _event(AxisHealth.OK, "a"),
        "b": _event(AxisHealth.SUSPECT, "b"),
        "c": _event(AxisHealth.DEFECT, "c"),
    })
    assert unit.worst_level() == AxisHealth.DEFECT


def test_robot_unit_defect_components_lists_only_defects():
    unit = RobotUnit("r0", events={
        "a": _event(AxisHealth.OK, "a"),
        "b": _event(AxisHealth.DEFECT, "b"),
        "c": _event(AxisHealth.DEFECT, "c"),
    })
    assert set(unit.defect_components()) == {"b", "c"}


def test_fleet_status_maps_unit_to_worst_level():
    fleet = Fleet()
    fleet.add_unit(RobotUnit("healthy", events={"a": _event(AxisHealth.OK)}))
    fleet.add_unit(RobotUnit("broken", events={"a": _event(AxisHealth.DEFECT)}))
    status = fleet.fleet_status()
    assert status == {"healthy": AxisHealth.OK, "broken": AxisHealth.DEFECT}


def test_fleet_units_at_level():
    fleet = Fleet()
    fleet.add_unit(RobotUnit("healthy", events={"a": _event(AxisHealth.OK)}))
    fleet.add_unit(RobotUnit("broken", events={"a": _event(AxisHealth.DEFECT)}))
    assert fleet.units_at_level(AxisHealth.DEFECT) == ["broken"]
    assert fleet.units_at_level(AxisHealth.OK) == ["healthy"]


def test_fleet_worst_fleet_level_empty_fleet_is_ok():
    fleet = Fleet()
    assert fleet.worst_fleet_level() == AxisHealth.OK


def test_fleet_worst_fleet_level_reflects_worst_unit():
    fleet = Fleet()
    fleet.add_unit(RobotUnit("healthy", events={"a": _event(AxisHealth.OK)}))
    fleet.add_unit(RobotUnit("suspect", events={"a": _event(AxisHealth.SUSPECT)}))
    assert fleet.worst_fleet_level() == AxisHealth.SUSPECT


def test_get_unit_unknown_raises():
    fleet = Fleet()
    with pytest.raises(KeyError):
        fleet.get_unit("does_not_exist")
