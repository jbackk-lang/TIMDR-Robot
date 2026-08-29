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


def describe_scenario(events: Dict[str, StatusEvent]) -> str:
    """Podsumowanie calego scenariusza (wszystkie osie na raz)."""
    by_level = {level: [] for level in AxisHealth}
    for axis_id, event in events.items():
        by_level[event.level].append(axis_id)

    lines = []
    for level in (AxisHealth.DEFECT, AxisHealth.RESONANCE, AxisHealth.SUSPECT, AxisHealth.OK):
        axes = by_level[level]
        if axes:
            lines.append(f"{level.value}: {', '.join(axes)}")
    summary = " | ".join(lines) if lines else "brak danych"
    return (
        f"Podsumowanie scenariusza ({len(events)} osi): {summary}. "
        "Dane syntetyczne - patrz README, sekcja 'Status walidacji'."
    )
