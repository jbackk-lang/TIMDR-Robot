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


def compute_component_status(
    component_id: str,
    anomaly_count: int,
    metrics: Optional[Dict] = None,
    anomaly_threshold: int = 3,
    component_label: str = "komponent",
    ok_message: Optional[str] = None,
    defect_message: Optional[str] = None,
) -> StatusEvent:
    """Wersja compute_axis_status() UOGOLNIONA dla podsystemow innych niz
    obrotowa os ramienia (chwytak, podstawa mobilna, kamera, zasilanie -
    patrz `subsystem_core.py`). Te podsystemy nie maja pojecia "torsji
    portretu fazowego" ani "rezonansu po zdarzeniu" (te koncepcje sa
    specyficzne dla ruchu obrotowego osi) - maja WYLACZNIE licznik
    anomalii z jednego, wlasciwego dla siebie detektora
    (`baseline_residual`+`anomalies`, reszta kinematyczna,
    `thermal_drift_score`, itd. - patrz subsystem_core.py). Dlatego tylko
    dwa poziomy sa tu uzywane: OK i DEFECT (bez SUSPECT/RESONANCE, ktore
    wymagalyby dodatkowego, niezaleznego sygnalu, jakiego te podsystemy
    obecnie nie maja - `AxisHealth.SUSPECT`/`RESONANCE` sa nadal dostepne
    w enumie na przyszlosc, gdy podsystem dostanie drugi, niezalezny
    detektor).

    `StatusEvent.axis_id` jest tu uzywany jako generyczny identyfikator
    komponentu (nie zmieniono nazwy pola, zeby nie zlamac istniejacego
    kodu/testow dla osi ramienia - semantycznie dziala identycznie).
    """
    if anomaly_count >= anomaly_threshold:
        level = AxisHealth.DEFECT
        message = defect_message or (
            f"{component_label} {component_id}: {anomaly_count} anomalii "
            f"wykrytych w oknie analizy (prog: {anomaly_threshold})."
        )
    else:
        level = AxisHealth.OK
        message = ok_message or f"{component_label} {component_id}: brak wykrytych anomalii."

    return StatusEvent(axis_id=component_id, level=level, message=message, metrics_snapshot=metrics or {})


def compute_power_status(
    power_metrics: Dict,
    voltage_anomaly_threshold: int = 3,
    thermal_anomaly_threshold: int = 3,
) -> StatusEvent:
    """Dedykowana wersja dla `subsystem_core.analyze_power()`, bo laczy
    TRZY niezalezne sygnaly (limit bezwzgledny napiecia, wzgledna anomalia
    napiecia, dryft termiczny) - zbyt rozne, zeby upychac w jeden
    generyczny `anomaly_count` bez utraty informacji, KTORY z trzech
    problemow faktycznie wystapil. Priorytet: naruszenie limitu
    bezwzglednego (najpowazniejsze, bezposrednie zagrozenie) > dryft
    termiczny > wzgledna anomalia napiecia > OK."""
    component_id = power_metrics["component_id"]
    n_abs = power_metrics.get("voltage_absolute_violation_count", 0)
    n_volt = power_metrics.get("voltage_anomaly_count", 0)
    n_therm = power_metrics.get("thermal_anomaly_count", 0)

    if n_abs > 0:
        level = AxisHealth.DEFECT
        message = (
            f"Zasilanie {component_id}: {n_abs} probek ponizej "
            f"bezwzglednego limitu napiecia - bezposrednie zagrozenie."
        )
    elif n_therm >= thermal_anomaly_threshold:
        level = AxisHealth.DEFECT
        message = (
            f"Zasilanie {component_id}: {n_therm} anomalii dryftu "
            f"termicznego (nachylenie: {power_metrics.get('thermal_slope'):.3f}) - "
            f"mozliwy rozbiegajacy sie wzrost temperatury."
        )
    elif n_volt >= voltage_anomaly_threshold:
        level = AxisHealth.SUSPECT
        message = (
            f"Zasilanie {component_id}: {n_volt} przejsciowych anomalii "
            f"napiecia (wzgledem lokalnej mediany) - wymaga uwagi."
        )
    else:
        level = AxisHealth.OK
        message = f"Zasilanie {component_id}: brak wykrytych anomalii."

    return StatusEvent(axis_id=component_id, level=level, message=message, metrics_snapshot=power_metrics)
