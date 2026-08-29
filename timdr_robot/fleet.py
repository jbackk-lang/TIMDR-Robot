"""timdr_robot/fleet.py — agregacja WIELU robotow (System Logic,
rozszerzenie ponad pojedyncza maszyne).
================================================================================
`RobotUnit` opakowuje juz policzone `StatusEvent` (z osi ramienia I/lub
podsystemow, patrz `status.py`/`subsystem_core.py`) dla JEDNEGO robota.
`Fleet` trzyma wiele `RobotUnit` i agreguje status na poziomie CALEJ
floty: ktory robot ma najgorszy status, ile robotow wymaga uwagi, itd.

**TO JEST WARSTWA CZYSTO AGREGUJACA** - nie liczy niczego nowego z
sygnalow, tylko sklada juz gotowe `StatusEvent` (wyprodukowane przez
`core.analyze_axis()`+`status.compute_axis_status()` lub
`subsystem_core.analyze_*()`+`status.compute_component_status()` /
`compute_power_status()`) w jeden widok. Priorytet powagi jest ten sam co
w `status.py`: DEFECT > RESONANCE > SUSPECT > OK.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, List

from .status import AxisHealth, StatusEvent

_LEVEL_ORDER = [AxisHealth.OK, AxisHealth.SUSPECT, AxisHealth.RESONANCE, AxisHealth.DEFECT]


@dataclass
class RobotUnit:
    """Jeden robot: `unit_id` + slownik `StatusEvent` per komponent
    (osie ramienia + podsystemy, jesli sa)."""
    unit_id: str
    events: Dict[str, StatusEvent] = field(default_factory=dict)

    def worst_level(self) -> AxisHealth:
        if not self.events:
            return AxisHealth.OK
        return max((e.level for e in self.events.values()), key=_LEVEL_ORDER.index)

    def defect_components(self) -> List[str]:
        return [cid for cid, e in self.events.items() if e.level == AxisHealth.DEFECT]

    def components_at_level(self, level: AxisHealth) -> List[str]:
        return [cid for cid, e in self.events.items() if e.level == level]


@dataclass
class Fleet:
    """Zbior `RobotUnit`."""
    units: Dict[str, RobotUnit] = field(default_factory=dict)

    def add_unit(self, unit: RobotUnit) -> None:
        self.units[unit.unit_id] = unit

    def unit_ids(self) -> List[str]:
        return list(self.units.keys())

    def get_unit(self, unit_id: str) -> RobotUnit:
        if unit_id not in self.units:
            raise KeyError(f"Nieznany robot: {unit_id!r}. Dostepne: {list(self.units)}")
        return self.units[unit_id]

    def fleet_status(self) -> Dict[str, AxisHealth]:
        """Mapa unit_id -> najgorszy status TEGO robota."""
        return {uid: unit.worst_level() for uid, unit in self.units.items()}

    def units_at_level(self, level: AxisHealth) -> List[str]:
        return [uid for uid, unit in self.units.items() if unit.worst_level() == level]

    def worst_fleet_level(self) -> AxisHealth:
        if not self.units:
            return AxisHealth.OK
        return max((unit.worst_level() for unit in self.units.values()), key=_LEVEL_ORDER.index)
