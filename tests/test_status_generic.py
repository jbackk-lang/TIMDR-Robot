import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from timdr_robot.status import AxisHealth, compute_component_status, compute_power_status


def test_component_status_ok_below_threshold():
    event = compute_component_status("camera_0", anomaly_count=0, anomaly_threshold=3)
    assert event.level == AxisHealth.OK


def test_component_status_defect_at_or_above_threshold():
    event = compute_component_status("camera_0", anomaly_count=3, anomaly_threshold=3)
    assert event.level == AxisHealth.DEFECT


def test_power_status_ok_when_all_clean():
    metrics = {
        "component_id": "power_0",
        "voltage_absolute_violation_count": 0,
        "voltage_anomaly_count": 0,
        "thermal_anomaly_count": 0,
        "thermal_slope": 0.01,
    }
    event = compute_power_status(metrics)
    assert event.level == AxisHealth.OK


def test_power_status_absolute_violation_wins_priority():
    metrics = {
        "component_id": "power_0",
        "voltage_absolute_violation_count": 5,
        "voltage_anomaly_count": 10,
        "thermal_anomaly_count": 10,
        "thermal_slope": 0.5,
    }
    event = compute_power_status(metrics)
    assert event.level == AxisHealth.DEFECT
    assert "limitu" in event.message


def test_power_status_thermal_defect_when_no_absolute_violation():
    metrics = {
        "component_id": "power_0",
        "voltage_absolute_violation_count": 0,
        "voltage_anomaly_count": 0,
        "thermal_anomaly_count": 10,
        "thermal_slope": 0.5,
    }
    event = compute_power_status(metrics)
    assert event.level == AxisHealth.DEFECT
    assert "termicznego" in event.message


def test_power_status_suspect_for_transient_voltage_anomalies_only():
    metrics = {
        "component_id": "power_0",
        "voltage_absolute_violation_count": 0,
        "voltage_anomaly_count": 5,
        "thermal_anomaly_count": 0,
        "thermal_slope": 0.01,
    }
    event = compute_power_status(metrics)
    assert event.level == AxisHealth.SUSPECT
