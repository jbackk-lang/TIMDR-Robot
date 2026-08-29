"""timdr_robot/control_bridge.py — warstwa "TIMDR Integration" <-> "Robot
Control": API, ktorego uzylby prawdziwy sterownik robota.
================================================================================
**WSZYSTKIE REAKCJE PONIZEJ SA SYMULOWANE (LOGOWANE), NIE WYSYLAJA ZADNYCH
KOMEND DO PRAWDZIWEGO SPRZETU.** To jest interfejs/kontrakt do przyszlej
integracji (np. z ROS, sterownikiem PLC, czy inna magistrala) - dokladnie
w duchu "timdr_control_bridge" z dokumentu architektury: `get_axis_health
(axis_id)`, `subscribe_events()`. Realna implementacja tych reakcji
(rzeczywiste zwolnienie predkosci/zwiekszenie tlumienia/zatrzymanie osi)
wymaga sterownika konkretnego robota i jest jawnie POZA zakresem tego
szkieletu.

Polityka reakcji (celowo prosta, latwa do przeczytania i zmiany):
- OK:        brak reakcji.
- SUSPECT:   tylko log (poziom info) - brak zmiany zachowania robota.
- RESONANCE: `reduce_speed(axis_id)` (symulowane) + log ostrzegawczy.
- DEFECT:    `reduce_speed(axis_id)` + `increase_damping(axis_id)` +
             `stop_axis(axis_id)` (symulowane) + `alarm(axis_id)` + log
             krytyczny.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Callable, Dict, List, Optional

from .status import AxisHealth, StatusEvent


@dataclass
class ReactionLogEntry:
    axis_id: str
    action: str
    detail: str


class ControlBridge:
    """Trzyma najnowszy StatusEvent per os, wywoluje subskrybentow i
    (symulowane) reakcje wedlug polityki opisanej w docstringu modulu."""

    def __init__(self) -> None:
        self._latest: Dict[str, StatusEvent] = {}
        self._subscribers: List[Callable[[StatusEvent], None]] = []
        self.reaction_log: List[ReactionLogEntry] = []

    # -- API zgodne z dokumentem architektury -------------------------------

    def get_axis_health(self, axis_id: str) -> Optional[AxisHealth]:
        event = self._latest.get(axis_id)
        return event.level if event else None

    def subscribe_events(self, callback: Callable[[StatusEvent], None]) -> None:
        self._subscribers.append(callback)

    # -- Wejscie z TIMDR Core/Integration ------------------------------------

    def publish(self, event: StatusEvent) -> None:
        """Rejestruje nowy StatusEvent dla osi, powiadamia subskrybentow i
        wykonuje (symulowana) reakcje wedlug polityki."""
        self._latest[event.axis_id] = event
        for cb in self._subscribers:
            cb(event)
        self._react(event)

    def _react(self, event: StatusEvent) -> None:
        axis_id = event.axis_id
        if event.level == AxisHealth.OK:
            return
        if event.level == AxisHealth.SUSPECT:
            self._log(axis_id, "log_info", event.message)
            return
        if event.level == AxisHealth.RESONANCE:
            self.reduce_speed(axis_id)
            self._log(axis_id, "log_warning", event.message)
            return
        if event.level == AxisHealth.DEFECT:
            self.reduce_speed(axis_id)
            self.increase_damping(axis_id)
            self.stop_axis(axis_id)
            self.alarm(axis_id)
            self._log(axis_id, "log_critical", event.message)

    # -- Symulowane reakcje (BEZ prawdziwego I/O do sprzetu) -----------------

    def reduce_speed(self, axis_id: str, factor: float = 0.5) -> None:
        self.reaction_log.append(ReactionLogEntry(
            axis_id, "reduce_speed", f"symulacja: predkosc osi {axis_id} x{factor}"
        ))

    def increase_damping(self, axis_id: str, factor: float = 2.0) -> None:
        self.reaction_log.append(ReactionLogEntry(
            axis_id, "increase_damping", f"symulacja: tlumienie osi {axis_id} x{factor}"
        ))

    def stop_axis(self, axis_id: str) -> None:
        self.reaction_log.append(ReactionLogEntry(
            axis_id, "stop_axis", f"symulacja: zatrzymanie osi {axis_id}"
        ))

    def alarm(self, axis_id: str) -> None:
        self.reaction_log.append(ReactionLogEntry(
            axis_id, "alarm", f"symulacja: alarm dla osi {axis_id}"
        ))

    def _log(self, axis_id: str, level: str, message: str) -> None:
        self.reaction_log.append(ReactionLogEntry(axis_id, level, message))
