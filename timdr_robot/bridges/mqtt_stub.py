"""timdr_robot/bridges/mqtt_stub.py — kontrakt integracji z MQTT.
================================================================================
Patrz docstring `bridges/__init__.py` po ogolny wzorzec. Prawdziwa
integracja publikowalaby `StatusEvent` jako wiadomosc JSON na topic MQTT
(np. `timdr/robot/<unit_id>/<component_id>/health`). `paho-mqtt` jest
importowany DEFENSYWNIE - bez niego most dziala w trybie dry-run.
"""
from __future__ import annotations

import json
from typing import Dict, List, Optional

try:
    import paho.mqtt.client as mqtt  # type: ignore
    _IMPORT_ERROR = None
except Exception as e:  # pragma: no cover - zalezy od srodowiska
    mqtt = None
    _IMPORT_ERROR = str(e)

from ..status import StatusEvent


class MQTTBridge:
    """Kontrakt: `publish_status(unit_id, event)` publikuje status na
    `topic_prefix/<unit_id>/<component_id>/health`. Bez `paho-mqtt`
    zainstalowanego dziala w trybie dry-run."""

    def __init__(self, broker_host: str = "localhost", broker_port: int = 1883, topic_prefix: str = "timdr/robot"):
        self.broker_host = broker_host
        self.broker_port = broker_port
        self.topic_prefix = topic_prefix
        self.available = mqtt is not None
        self.import_error = _IMPORT_ERROR
        self.connected = False
        self.published_log: List[Dict] = []
        self._client = None
        if self.available:  # pragma: no cover - wymaga zainstalowanego paho-mqtt
            self._init_real_client()

    def _init_real_client(self) -> None:  # pragma: no cover - wymaga paho-mqtt
        """TODO integracyjne: `mqtt.Client()`, `self._client.connect(...)`,
        ustawienie `self.connected=True` po potwierdzeniu polaczenia.
        Celowo puste w tym szkielecie - NIE laczy sie z zadnym brokerem."""
        pass

    def _topic_for(self, unit_id: str, component_id: str) -> str:
        return f"{self.topic_prefix}/{unit_id}/{component_id}/health"

    def publish_status(self, unit_id: str, event: StatusEvent) -> Dict:
        topic = self._topic_for(unit_id, event.axis_id)
        payload = {"level": event.level.value, "message": event.message}
        entry = {"topic": topic, "payload": payload}
        self.published_log.append(entry)
        if self.available and self.connected:  # pragma: no cover - wymaga polaczenia
            self._publish_real(topic, payload)
        return entry

    def _publish_real(self, topic: str, payload: Dict) -> None:  # pragma: no cover - wymaga paho-mqtt
        """TODO integracyjne: `self._client.publish(topic, json.dumps(payload))`."""
        pass
