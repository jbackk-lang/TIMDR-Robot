"""timdr_robot/hmi.py — warstwa "System Logic"/HMI: opis wynikow w jezyku
naturalnym, budowany WYLACZNIE z policzonych statystyk (bez LLM), tak jak
`_describe_result()` w fusion-tools/api.py.
================================================================================
Cel: czlowiek patrzacy na dashboard (lub log konsoli) dostaje jedno,
czytelne zdanie/akapit per os, zamiast musiec samemu czytac surowe liczby
z core.analyze_axis(). Zawsze konczy sie jawnym zastrzezeniem, ze to opis
statystyczny wyjscia pipeline'u, nie diagnoza mechaniczna.
"""
from __future__ import annotations

from typing import Dict

from .fleet import Fleet
from .status import AxisHealth, StatusEvent


def describe_axis_event(event: StatusEvent) -> str:
    metrics = event.metrics_snapshot
    axis_id = event.axis_id
    n = metrics.get("n_samples", 0)
    n_defects = metrics.get("harmonic_anomaly_count", 0)
    n_spikes = metrics.get("torsion_spike_count", 0)
    ringdown = metrics.get("ringdown")

    parts = [f"Os {axis_id} ({n} probek): status {event.level.value}."]

    if event.level == AxisHealth.OK:
        parts.append("Brak anomalii reszty modelu harmonicznego ani skokow torsji portretu fazowego.")
    else:
        if n_defects:
            parts.append(f"Wykryto {n_defects} anomalii reszty modelu harmonicznego (accel vs. -omega^2*pozycja).")
        if n_spikes:
            parts.append(f"Wykryto {n_spikes} skoki torsji portretu fazowego (geometria: pozycja-predkosc-przyspieszenie).")
        if ringdown and ringdown.get("is_oscillatory"):
            freq = ringdown.get("frequency_hz")
            damping = ringdown.get("damping_ratio")
            freq_txt = f"{freq:.2f} Hz" if freq else "nieznana"
            damping_txt = f"{damping:.3f}" if damping is not None else "nieznany"
            parts.append(
                f"Powrot po zdarzeniu jest oscylacyjny: czestotliwosc ~{freq_txt}, "
                f"wskaznik tlumienia ~{damping_txt} (mniejszy = wolniej gasnie)."
            )

    parts.append(
        "To jest opis statystyczny wyjscia pipeline'u TIMDR na danych "
        "SYNTETYCZNYCH, NIE zwalidowana diagnoza mechaniczna prawdziwego "
        "sprzetu."
    )
    return " ".join(parts)


def describe_scenario(events: Dict[str, StatusEvent], label_plural: str = "osi") -> str:
    """Podsumowanie calego scenariusza (wszystkie monitorowane komponenty
    na raz - osie ramienia i/lub nowe podsystemy, patrz `label_plural`)."""
    by_level = {level: [] for level in AxisHealth}
    for component_id, event in events.items():
        by_level[event.level].append(component_id)

    lines = []
    for level in (AxisHealth.DEFECT, AxisHealth.RESONANCE, AxisHealth.SUSPECT, AxisHealth.OK):
        ids = by_level[level]
        if ids:
            lines.append(f"{level.value}: {', '.join(ids)}")
    summary = " | ".join(lines) if lines else "brak danych"
    return (
        f"Podsumowanie scenariusza ({len(events)} {label_plural}): {summary}. "
        "Dane syntetyczne - patrz README, sekcja 'Status walidacji'."
    )


def describe_component_event(event: StatusEvent, component_label: str = "Komponent") -> str:
    """Generyczny opis dla podsystemow spoza oz ramienia (chwytak,
    podstawa mobilna, kamera) - te maja tylko `anomaly_count`, bez modelu
    harmonicznego/torsji/rezonansu specyficznych dla ruchu obrotowego."""
    metrics = event.metrics_snapshot
    n = metrics.get("n_samples", 0)
    n_anom = metrics.get("anomaly_count", 0)
    parts = [f"{component_label} {event.axis_id} ({n} probek): status {event.level.value}."]
    if event.level == AxisHealth.OK:
        parts.append("Brak wykrytych anomalii.")
    else:
        parts.append(f"Wykryto {n_anom} anomalii wzgledem oczekiwanego zachowania komponentu.")
    parts.append(
        "To jest opis statystyczny wyjscia pipeline'u TIMDR na danych "
        "SYNTETYCZNYCH, NIE zwalidowana diagnoza mechaniczna prawdziwego "
        "sprzetu."
    )
    return " ".join(parts)


def describe_fleet(fleet: Fleet) -> str:
    """Podsumowanie CALEJ FLOTY (wiele robotow) - patrz `fleet.py`."""
    status = fleet.fleet_status()
    by_level: Dict[AxisHealth, list] = {level: [] for level in AxisHealth}
    for unit_id, level in status.items():
        by_level[level].append(unit_id)

    lines = []
    for level in (AxisHealth.DEFECT, AxisHealth.RESONANCE, AxisHealth.SUSPECT, AxisHealth.OK):
        ids = by_level[level]
        if ids:
            lines.append(f"{level.value}: {', '.join(ids)}")
    summary = " | ".join(lines) if lines else "brak robotow we flocie"
    return (
        f"Flota ({len(fleet.units)} robotow): {summary}. "
        "Dane syntetyczne - patrz README, sekcja 'Status walidacji'."
    )


def describe_power_event(event: StatusEvent) -> str:
    """Dedykowany opis dla zasilania/baterii (subsystem_core.analyze_power())
    - trzy niezalezne sygnaly, patrz status.compute_power_status()."""
    metrics = event.metrics_snapshot
    n = metrics.get("n_samples", 0)
    n_abs = metrics.get("voltage_absolute_violation_count", 0)
    n_volt = metrics.get("voltage_anomaly_count", 0)
    n_therm = metrics.get("thermal_anomaly_count", 0)
    slope = metrics.get("thermal_slope")

    parts = [f"Zasilanie {event.axis_id} ({n} probek): status {event.level.value}."]
    if event.level == AxisHealth.OK:
        parts.append("Napiecie i temperatura w oczekiwanych granicach.")
    else:
        if n_abs:
            parts.append(f"{n_abs} probek ponizej bezwzglednego limitu napiecia.")
        if n_therm:
            slope_txt = f"{slope:.3f}" if slope is not None else "nieznane"
            parts.append(f"{n_therm} anomalii dryftu termicznego (nachylenie trendu: {slope_txt} jednostek/s).")
        if n_volt:
            parts.append(f"{n_volt} przejsciowych anomalii napiecia wzgledem lokalnej mediany.")
    parts.append(
        "To jest opis statystyczny wyjscia pipeline'u TIMDR na danych "
        "SYNTETYCZNYCH, NIE zwalidowana diagnoza mechaniczna prawdziwego "
        "sprzetu."
    )
    return " ".join(parts)
