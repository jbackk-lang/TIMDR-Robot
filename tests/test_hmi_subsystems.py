import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from timdr_robot import hmi
from timdr_robot.status import AxisHealth, StatusEvent


def test_describe_component_event_ok():
    event = StatusEvent(axis_id="camera_0", level=AxisHealth.OK, message="ok",
                         metrics_snapshot={"n_samples": 100, "anomaly_count": 0})
    text = hmi.describe_component_event(event, component_label="Kamera")
    assert "Kamera camera_0" in text
    assert "SYNTETYCZNYCH" in text


def test_describe_component_event_defect_mentions_anomaly_count():
    event = StatusEvent(axis_id="base_0", level=AxisHealth.DEFECT, message="defect",
                         metrics_snapshot={"n_samples": 200, "anomaly_count": 55})
    text = hmi.describe_component_event(event, component_label="Podstawa")
    assert "55" in text


def test_describe_power_event_reports_absolute_violation():
    event = StatusEvent(axis_id="power_0", level=AxisHealth.DEFECT, message="defect", metrics_snapshot={
        "n_samples": 2000, "voltage_absolute_violation_count": 12,
        "voltage_anomaly_count": 5, "thermal_anomaly_count": 0, "thermal_slope": 0.02,
    })
    text = hmi.describe_power_event(event)
    assert "12" in text
    assert "limitu" in text


def test_describe_scenario_supports_custom_label():
    events = {
        "gripper_0": StatusEvent(axis_id="gripper_0", level=AxisHealth.OK, message="ok", metrics_snapshot={}),
    }
    text = hmi.describe_scenario(events, label_plural="podsystemow")
    assert "podsystemow" in text
