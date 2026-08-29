"""timdr_robot/status.py — warstwa "TIMDR Integration": mapuje surowe
metryki z core.analyze_axis() na dyskretny status zdrowia osi + zdarzenia.
================================================================================
Cztery poziomy (rosnaco wg powagi): OK -> SUSPECT -> RESONANCE -> DEFECT.
Reguly mapowania sa CELOWO proste i jawnie opisane (nie ukryty "czarny
box") - to jest szkielet do dalszej kalibracji na realnych danych, nie
gotowy, zwalidowany system decyzyjny:

- DEFECT: >= `harmonic_anomaly_threshold` anomalii reszty modelu
  harmonicznego (core.harmonic_law_residual(), patrz core.py po pelne
  uzasadnienie) wykrytych w oknie analizy - interpretacja: przyspieszenie
  osi systematycznie przestaje pasowac do prostego prawa ruchu, typowy
  objaw postepujacego luzu/backlash.
- RESONANCE: ringdown.is_oscillatory=True dla wykrytego zdarzenia -
  interpretacja: powrot do rownowagi po zdarzeniu jest oscylacyjny
  (mozliwe "dzwonienie" mechaniczne).
- SUSPECT: zadna z powyzszych, ale torsion_spike_count > 0 (geometria
  fazowa wykryla COS niestandardowego, ale nie na tyle duzo/wyraznie zeby
  klasyfikowac jako DEFECT/RESONANCE) - wymaga uwagi, nie alarmu.
- OK: brak wykrytych anomalii.

Priorytet gdy zachodzi wiecej niz jeden warunek: DEFECT > RESONANCE >
SUSPECT > OK (najbardziej dotkliwy wygrywa).
"""
from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from typing import Dict, Optional


class AxisHealth(str, Enum):
    OK = "OK"
    SUSPECT = "SUSPECT"
    RESONANCE = "RESONANCE"
    DEFECT = "DEFECT"


@dataclass
class StatusEvent:
    axis_id: str
    level: AxisHealth
    message: str
    metrics_snapshot: Dict


def compute_axis_status(
    metrics: Dict,
    harmonic_anomaly_threshold: int = 3,
) -> StatusEvent:
    """Mapuje wyjscie `core.analyze_axis()` na (AxisHealth, komunikat)."""
    axis_id = metrics["axis_id"]
    ringdown = metrics.get("ringdown")
    is_resonant = bool(ringdown and ringdown.get("is_oscillatory"))
    n_harmonic_anomalies = metrics.get("harmonic_anomaly_count", 0)
    n_torsion_spikes = metrics.get("torsion_spike_count", 0)

    if n_harmonic_anomalies >= harmonic_anomaly_threshold:
        level = AxisHealth.DEFECT
        message = (
            f"Os {axis_id}: {n_harmonic_anomalies} anomalii reszty modelu "
            f"harmonicznego wykrytych w oknie analizy (prog: "
            f"{harmonic_anomaly_threshold}) - mozliwy postepujacy luz "
            f"mechaniczny (backlash)."
        )
    elif is_resonant:
        level = AxisHealth.RESONANCE
        freq = ringdown.get("frequency_hz")
        freq_txt = f"{freq:.2f} Hz" if freq else "nieznana"
        level_note = (
            f"Os {axis_id}: powrot po zdarzeniu (idx="
            f"{metrics.get('ringdown_event_idx')}) jest OSCYLACYJNY, "
            f"czestotliwosc ~{freq_txt} - mozliwy rezonans mechaniczny."
        )
        message = level_note
    elif n_torsion_spikes > 0:
        level = AxisHealth.SUSPECT
        message = (
            f"Os {axis_id}: {n_torsion_spikes} skokow torsji portretu "
            f"fazowego bez potwierdzonego defektu/rezonansu - wymaga uwagi."
        )
    else:
        level = AxisHealth.OK
        message = f"Os {axis_id}: brak wykrytych anomalii."

    return StatusEvent(axis_id=axis_id, level=level, message=message, metrics_snapshot=metrics)
