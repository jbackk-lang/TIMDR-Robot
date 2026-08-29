"""tests/test_rejected_pipeline.py — testy end-to-end sciezki REJECTED
(NC1 na wejsciu, NC2 na wyjsciu) przez cala warstwe: core.analyze_axis()
-> status.compute_*_status() -> control_bridge.ControlBridge ->
fleet.RobotUnit/Fleet. Dodane na wprost postawione pytanie uzytkownika o
formalny protokol Sanity/Negative Control (kroki 1 i 6)."""
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from timdr_robot import core, subsystem_core as sc
from timdr_robot.control_bridge import ControlBridge
from timdr_robot.fleet import Fleet, RobotUnit
from timdr_robot.sensor_bus import AxisSpec, generate_axis_trajectory
from timdr_robot.status import (
    AxisHealth,
    compute_axis_status,
    compute_component_status,
    compute_power_status,
)

SPEC = AxisSpec(name="axis_test", amplitude_deg=30.0, cycle_s=3.0)
N, DT = 2000, 0.01


def _clean_traj():
    return generate_axis_trajectory(SPEC, N, DT, seed=1, defect_type=None)


def test_analyze_axis_rejects_nan_position():
    traj = _clean_traj()
    pos = traj["position"].copy()
    pos[100] = np.nan
    metrics = core.analyze_axis("axis_test", traj["t"], pos, traj["velocity"], traj["accel"])
    assert metrics["rejected"] is True
    assert "NaN" in metrics["rejection_reason"]
    # metryki analityczne (torsja, model harmoniczny) NIE zostaly policzone
    assert "harmonic_anomaly_count" not in metrics


def test_analyze_axis_rejects_impossible_jump():
    traj = _clean_traj()
    pos = traj["position"].copy()
    pos[100] += 10000.0
    metrics = core.analyze_axis("axis_test", traj["t"], pos, traj["velocity"], traj["accel"],
                                 max_physical_jump_position=50.0)
    assert metrics["rejected"] is True


def test_analyze_axis_clean_signal_not_rejected():
    traj = _clean_traj()
    metrics = core.analyze_axis("axis_test", traj["t"], traj["position"], traj["velocity"], traj["accel"])
    assert metrics.get("rejected", False) is False


def test_compute_axis_status_maps_rejected_metrics_to_rejected_level():
    traj = _clean_traj()
    accel = traj["accel"].copy()
    accel[50] = np.inf
    metrics = core.analyze_axis("axis_test", traj["t"], traj["position"], traj["velocity"], accel)
    event = compute_axis_status(metrics)
    assert event.level == AxisHealth.REJECTED
    assert "ODRZUCONY" in event.message


def test_compute_component_status_maps_rejected_metrics_to_rejected_level():
    event = compute_component_status(
        "gripper_0", anomaly_count=0,
        metrics={"rejected": True, "rejection_reason": "test powod"},
    )
    assert event.level == AxisHealth.REJECTED


def test_compute_power_status_maps_rejected_metrics_to_rejected_level():
    event = compute_power_status({"component_id": "power_0", "rejected": True, "rejection_reason": "test powod"})
    assert event.level == AxisHealth.REJECTED


def test_analyze_gripper_rejects_nan_force():
    force = np.full(500, 10.0)
    force[10] = np.nan
    metrics = sc.analyze_gripper("gripper_0", np.arange(500) * DT, force)
    assert metrics["rejected"] is True


def test_analyze_vision_rejects_inf_error():
    err = np.zeros(500)
    err[10] = np.inf
    metrics = sc.analyze_vision("camera_0", np.arange(500) * DT, err)
    assert metrics["rejected"] is True


def test_control_bridge_rejected_triggers_stop_alarm_quarantine():
    traj = _clean_traj()
    accel = traj["accel"].copy()
    accel[50] = np.nan
    metrics = core.analyze_axis("axis_test", traj["t"], traj["position"], traj["velocity"], accel)
    event = compute_axis_status(metrics)
    bridge = ControlBridge()
    bridge.publish(event)
    actions = [e.action for e in bridge.reaction_log]
    assert actions == ["stop_axis", "alarm", "quarantine", "log_critical"]


def test_rejected_outranks_defect_in_fleet_worst_level():
    ok_event = compute_axis_status({
        "axis_id": "axis_0", "n_samples": 10, "harmonic_anomaly_count": 0,
        "torsion_spike_count": 0, "ringdown": None,
    })
    rejected_event = compute_axis_status({"axis_id": "axis_1", "rejected": True, "rejection_reason": "x"})
    unit = RobotUnit("robot_test", events={"axis_0": ok_event, "axis_1": rejected_event})
    assert unit.worst_level() == AxisHealth.REJECTED

    fleet = Fleet()
    fleet.add_unit(unit)
    assert fleet.worst_fleet_level() == AxisHealth.REJECTED
