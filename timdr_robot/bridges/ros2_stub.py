"""timdr_robot/bridges/ros2_stub.py — kontrakt integracji z ROS2.
================================================================================
Prawdziwa integracja publikowalaby `StatusEvent` jako wiadomosc na topic
ROS2 (np. `std_msgs/String` z JSON-em w polu `data`, albo lepiej
dedykowany typ wiadomosci `.msg` - POZA zakresem tego stuba). Ten plik
NIE laczy sie z zadnym node'em ROS2 - `rclpy` jest importowany
DEFENSYWNIE (jesli brak pakietu, `available=False` i most dziala w
trybie dry-run, patrz docstring pakietu `bridges/__init__.py`).
"""
from __future__ import annotations

from typing import Dict, List

try:
    import rclpy  # type: ignore
    _IMPORT_ERROR = None
except Exception as e:  # pragma: no cover - zalezy od srodowiska
    rclpy = None
    _IMPORT_ERROR = str(e)

from ..status import StatusEvent


class ROS2Bridge:
    """Kontrakt: `publish_status(event)` publikuje status jednego
    komponentu na `topic`. Bez `rclpy` zainstalowanego dziala w trybie
    dry-run - `published_log` zawiera wszystko, co ZOSTALOBY wyslane."""

    def __init__(self, topic: str = "/timdr/component_health", node_name: str = "timdr_robot_bridge"):
        self.topic = topic
        self.node_name = node_name
        self.available = rclpy is not None
        self.import_error = _IMPORT_ERROR
        self.published_log: List[Dict] = []
        self._node = None
        if self.available:  # pragma: no cover - wymaga zainstalowanego rclpy
            self._init_real_node()

    def _init_real_node(self) -> None:  # pragma: no cover - wymaga rclpy
        """TODO integracyjne: `rclpy.init()`, `create_node(self.node_name)`,
        `create_publisher(String, self.topic, 10)`. Celowo puste w tym
        szkielecie - patrz docstring modulu."""
        pass

    def _to_payload(self, event: StatusEvent) -> Dict:
        return {"component_id": event.axis_id, "level": event.level.value, "message": event.message}

    def publish_status(self, event: StatusEvent) -> Dict:
        payload = self._to_payload(event)
        self.published_log.append(payload)
        if self.available:  # pragma: no cover - wymaga rclpy
            self._publish_real(payload)
        return payload

    def _publish_real(self, payload: Dict) -> None:  # pragma: no cover - wymaga rclpy
        """TODO integracyjne: serializacja `payload` do JSON, publikacja
        przez `self._publisher.publish(String(data=json.dumps(payload)))`."""
        pass
