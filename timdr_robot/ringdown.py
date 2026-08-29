"""timdr_robot/ringdown.py — czy powrot osi do poziomu odniesienia PO
zdarzeniu (np. wykrytym skoku/defekcie) jest REZONANSOWY (oscylacyjny)
================================================================================
Port 1:1 (bez zmian w matematyce) z jbackk-lang/universal-state-analyzer
(`timdr_core/ringdown.py`), TIMDR-Grid-Monitor, TIMDR-Earthquake-Core,
analizator-gieldowy-v3 i deliverable_timdr_finanse - tam metoda jest
zweryfikowana numerycznie na tlumionym oscylatorze o znanej czestotliwosci/
stalej czasowej.

KONTEKST ROBOTYCZNY (dlaczego tu w ogole jest): po zdarzeniu na osi (np.
nagly skok wykryty przez core.defect(), koniec fazy przyspieszania,
uderzenie/kontakt) sygnal (np. przyspieszenie z IMU) moze wrocic do
poziomu odniesienia PLYNNIE (dobrze, ukladane mechanicznie tlumienie) albo
OSCYLACYJNIE (mechaniczny rezonans/"dzwonienie" - np. luzna sruba,
rezonans ramienia pod obciazeniem, USZKODZONE lozysko). Ta funkcja
odroznia jedno od drugiego.

**UWAGA - TO JEST NARZEDZIE OPISOWE (POST-EVENT), NIE PREDYKCYJNE.** Sama
funkcja analizuje powrot do rownowagi PO zdarzeniu (potrzebuje
`event_idx`) - nie mowi nic o tym, czy rezonans da sie wykryc PRZED nim.

NIEZWALIDOWANE NA PRAWDZIWYM ROBOCIE w chwili portowania tej funkcji do
tego repo - zweryfikowane wylacznie na syntetycznym, czystym tlumionym
oscylatorze (jak we wszystkich innych portach w tym zestawie repo). Metoda
zero-crossing zaklada z grubsza JEDEN dominujacy tryb oscylacji - realne
drgania mechaniczne wieloosiowego ramienia moga byc superpozycja kilku
trybow jednoczesnie, co metoda moze bledine zinterpretowac.
"""
from __future__ import annotations

from typing import Optional

import numpy as np


def ringdown_resonance(
    t,
    s,
    event_idx: int,
    baseline: Optional[float] = None,
    pre_event_window: int = 10,
    max_lookahead: Optional[int] = None,
    noise_floor_factor: float = 3.0,
) -> dict:
    """Analizuje powrot `s` do poziomu odniesienia PO indeksie `event_idx`.
    Patrz docstring modulu po pelne uzasadnienie parametrow i metody.

    Zwraca dict: baseline, noise_floor, is_oscillatory, n_crossings,
    n_peaks_used, period_s, frequency_hz, log_decrement, damping_ratio,
    peak_times, peak_amplitudes.
    """
    t = np.asarray(t, dtype=float)
    s = np.asarray(s, dtype=float)
    n = len(s)
    if n == 0 or not (0 <= event_idx < n):
        raise ValueError(f"event_idx={event_idx} poza zakresem serii o dlugosci {n}")

    pre_start = max(0, event_idx - pre_event_window)
    pre_samples = s[pre_start:event_idx]

    if baseline is None:
        baseline = float(np.mean(pre_samples)) if len(pre_samples) else float(s[event_idx])

    noise_std = float(np.std(pre_samples)) if len(pre_samples) >= 2 else 0.0
    noise_floor = noise_floor_factor * noise_std

    end = n if max_lookahead is None else min(n, event_idx + max_lookahead)
    t_post = t[event_idx:end]
    d = s[event_idx:end] - baseline

    result: dict = {
        "baseline": float(baseline),
        "noise_floor": float(noise_floor),
        "is_oscillatory": False,
        "n_crossings": 0,
        "n_peaks_used": 0,
        "period_s": None,
        "frequency_hz": None,
        "log_decrement": None,
        "damping_ratio": None,
        "peak_times": [],
        "peak_amplitudes": [],
    }

    if len(d) < 3:
        return result

    band = noise_floor
    confirmed_idx: list = []
    state = 0
    for i in range(len(d)):
        if d[i] > band:
            new_state = 1
        elif d[i] < -band:
            new_state = -1
        else:
            continue
        if new_state != state:
            confirmed_idx.append(i)
            state = new_state

    crossing_times: list = []
    for prev_i, cur_i in zip(confirmed_idx[:-1], confirmed_idx[1:]):
        found = None
        for k in range(prev_i, cur_i):
            if d[k] == 0 or (d[k] > 0) != (d[k + 1] > 0):
                frac = 0.0 if d[k] == 0 else -d[k] / (d[k + 1] - d[k])
                found = float(t_post[k] + frac * (t_post[k + 1] - t_post[k]))
                break
        if found is None:
            found = float((t_post[prev_i] + t_post[cur_i]) / 2.0)
        crossing_times.append(found)

    bounds_idx = sorted(set([0] + confirmed_idx + [len(d) - 1]))
    peak_times: list = []
    peak_amps: list = []
    for a, b in zip(bounds_idx[:-1], bounds_idx[1:]):
        if b < a:
            continue
        seg = d[a:b + 1]
        local_idx = int(np.argmax(np.abs(seg)))
        peak_times.append(float(t_post[a + local_idx]))
        peak_amps.append(float(seg[local_idx]))

    used_crossings = crossing_times

    result["n_crossings"] = len(used_crossings)
    result["n_peaks_used"] = len(peak_amps)
    result["peak_times"] = peak_times
    result["peak_amplitudes"] = peak_amps

    if len(used_crossings) >= 2 and len(peak_amps) >= 2:
        result["is_oscillatory"] = True

        crossing_diffs = np.diff(used_crossings)
        if len(crossing_diffs) and np.median(crossing_diffs) > 0:
            period = 2.0 * float(np.median(crossing_diffs))
            result["period_s"] = period
            result["frequency_hz"] = 1.0 / period

        log_ratios = []
        for i in range(len(peak_amps) - 2):
            a, b = peak_amps[i], peak_amps[i + 2]
            if np.sign(a) == np.sign(b) and a != 0 and b != 0:
                ratio = abs(a) / abs(b)
                if ratio > 0:
                    log_ratios.append(np.log(ratio))
        if log_ratios:
            delta = float(np.mean(log_ratios))
            result["log_decrement"] = delta
            result["damping_ratio"] = float(delta / np.sqrt(4 * np.pi ** 2 + delta ** 2))

    return result
