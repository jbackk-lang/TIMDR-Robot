import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from timdr_robot.bridges.mqtt_stub import MQTTBridge
from timdr_robot.bridges.opcua_stub import OPCUABridge
from timdr_robot.bridges.ros2_stub import ROS2Bridge
from timdr_robot.status import AxisHealth, StatusEvent

EVENT = StatusEvent(axis_id="axis_3", level=AxisHealth.DEFECT, message="test defect", metrics_snapshot={})


def test_ros2_bridge_dry_run_when_rclpy_missing():
    bridge = ROS2Bridge()
    assert bridge.available is False
    assert bridge.import_error is not None
    payload = bridge.publish_status(EVENT)
    assert payload == {"component_id": "axis_3", "level": "DEFECT", "message": "test defect"}
    assert bridge.published_log == [payload]


def test_ros2_bridge_logs_every_call():
    bridge = ROS2Bridge()
    bridge.publish_status(EVENT)
    bridge.publish_status(EVENT)
    assert len(bridge.published_log) == 2


def test_mqtt_bridge_dry_run_when_paho_missing():
    bridge = MQTTBridge()
    assert bridge.available is False
    assert bridge.connected is False
    entry = bridge.publish_status("robot_beta", EVENT)
    assert entry["topic"] == "timdr/robot/robot_beta/axis_3/health"
    assert entry["payload"]["level"] == "DEFECT"


def test_mqtt_bridge_custom_topic_prefix():
    bridge = MQTTBridge(topic_prefix="custom/prefix")
    entry = bridge.publish_status("unit_x", EVENT)
    assert entry["topic"] == "custom/prefix/unit_x/axis_3/health"


def test_opcua_bridge_dry_run_when_asyncua_missing():
    bridge = OPCUABridge()
    assert bridge.available is False
    assert bridge.running is False
    result = bridge.update_node("robot_beta", EVENT)
    assert result["node_id"] == "ns=2;s=TIMDR.robot_beta.axis_3.Health"
    assert bridge.node_values[result["node_id"]]["level"] == "DEFECT"


def test_opcua_bridge_tracks_multiple_nodes():
    bridge = OPCUABridge()
    bridge.update_node("robot_beta", EVENT)
    ok_event = StatusEvent(axis_id="axis_0", level=AxisHealth.OK, message="ok", metrics_snapshot={})
    bridge.update_node("robot_beta", ok_event)
    assert len(bridge.node_values) == 2
