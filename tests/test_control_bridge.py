import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from timdr_robot.control_bridge import ControlBridge
from timdr_robot.status import AxisHealth, StatusEvent


def _event(level, axis_id="axis_0"):
    return StatusEvent(axis_id=axis_id, level=level, message=f"test {level.value}", metrics_snapshot={})


def test_get_axis_health_returns_none_before_any_event():
    bridge = ControlBridge()
    assert bridge.get_axis_health("axis_0") is None


def test_get_axis_health_reflects_latest_published_event():
    bridge = ControlBridge()
    bridge.publish(_event(AxisHealth.OK))
    assert bridge.get_axis_health("axis_0") == AxisHealth.OK
    bridge.publish(_event(AxisHealth.DEFECT))
    assert bridge.get_axis_health("axis_0") == AxisHealth.DEFECT


def test_subscribers_are_notified_on_publish():
    bridge = ControlBridge()
    received = []
    bridge.subscribe_events(received.append)
    event = _event(AxisHealth.RESONANCE)
    bridge.publish(event)
    assert received == [event]


def test_ok_triggers_no_reaction():
    bridge = ControlBridge()
    bridge.publish(_event(AxisHealth.OK))
    assert bridge.reaction_log == []


def test_suspect_only_logs():
    bridge = ControlBridge()
    bridge.publish(_event(AxisHealth.SUSPECT))
    actions = [e.action for e in bridge.reaction_log]
    assert actions == ["log_info"]


def test_resonance_reduces_speed_and_warns():
    bridge = ControlBridge()
    bridge.publish(_event(AxisHealth.RESONANCE))
    actions = [e.action for e in bridge.reaction_log]
    assert "reduce_speed" in actions
    assert "log_warning" in actions
    assert "stop_axis" not in actions


def test_defect_triggers_full_reaction_chain():
    bridge = ControlBridge()
    bridge.publish(_event(AxisHealth.DEFECT))
    actions = [e.action for e in bridge.reaction_log]
    assert actions == ["reduce_speed", "increase_damping", "stop_axis", "alarm", "log_critical"]


def test_axes_are_tracked_independently():
    bridge = ControlBridge()
    bridge.publish(_event(AxisHealth.DEFECT, axis_id="axis_0"))
    bridge.publish(_event(AxisHealth.OK, axis_id="axis_1"))
    assert bridge.get_axis_health("axis_0") == AxisHealth.DEFECT
    assert bridge.get_axis_health("axis_1") == AxisHealth.OK
