"""timdr_robot/subsystems.py — Warstwa "Robot Hardware", rozszerzenie
poza pojedyncza oz obrotowa: chwytak (gripper), podstawa mobilna, kamera/
wizja, zasilanie/bateria.
================================================================================
Ten sam status jak `sensor_bus.py`: SYNTETYCZNE generatory, zero polaczenia
z prawdziwym sprzetem. Rozdzielone od `sensor_bus.py` (ktory zostaje
nietkniety - jest juz przetestowany i uzywany przez demo/api), zeby nie
ryzykowac regresji w istniejacym, dzialajacym kodzie tylko po to, by
dodac nowe typy czujnikow.

**WAZNA ROZNICA WZGLEDEM OSI RAMIENIA:** pozycja osi w ruchu to duza,
oscylujaca wartosc (dlatego oz wymagala modelu fizycznego -
`core.harmonic_law_residual()` - zamiast prostego wykrywania skokow, patrz
README/core.py po pelne uzasadnienie tamtej poprawki). Sygnaly w tym
pliku sa inne z natury - w WIEKSZOSCI PRZYPADKOW blisko stacjonarne
(sila chwytu w fazie trzymania, blad sledzenia kamery, napiecie/pradu
baterii) - dla nich `core.anomalies()`/`core.defect()` (port z
TIMDR-Grid-Monitor) sa WLASCIWYM narzedziem, dokladnie tak jak zostaly
zaprojektowane. To swiadomy wybor: dopasowanie detektora do KSZTALTU
sygnalu, nie mechaniczne uzycie tego samego mlotka wszedzie - to jest ta
sama lekcja, ktora wczesniej trzeba bylo naprawiac empirycznie przy
oz ramienia.

CZTERY NOWE PODSYSTEMY:

1. **Chwytak (gripper)**: sila chwytu w cyklu chwyc-trzymaj-pusc
   (wygladzony trapez, nie ostre skoki - zeby jerk byl bliski zeru poza
   wstrzykniętym zdarzeniem). `defect_type="slip_event"`: krotkie,
   rosnace w czasie zapadniecia sily w fazie trzymania (poslizg
   chwytanego przedmiotu).

2. **Podstawa mobilna (mobile base)**: dwa kola (napad rozniczkowy).
   Prawo kinematyczne: predkosc katowa obrotu = (v_prawe - v_lewe) /
   rozstaw_kol, liczona z ENKODEROW kol. Niezalezny pomiar (symulowany
   "zyroskop IMU") mierzy PRAWDZIWA predkosc obrotu. Reszta = pomiar -
   model. `defect_type="wheel_slip"`: kolo traci przyczepnosc - enkoder
   nadal "widzi" pelny obrot, ale prawdziwy ruch (zyroskop) juz nie
   nadaza - reszta rosnie. To ten sam wzorzec co harmonic_law_residual()
   dla osi (prawo fizyczne jako model odniesienia), tylko z innym prawem.

3. **Kamera/wizja**: blad sledzenia celu (piksele), bliski zeru przy
   stabilnym sledzeniu. `defect_type="tracking_loss"`: okresowa utrata
   sledzenia (np. okluzja) - duzy, przejsciowy wzrost bledu.

4. **Zasilanie/bateria**: napiecie (powoli opadajace pod obciazeniem),
   prad, temperatura (powoli rosnaca). `defect_type="voltage_sag"`:
   nagly, krotki spadek napiecia pod skokiem obciazenia.
   `defect_type="thermal_runaway"`: zamiast powolnego, liniowego wzrostu
   temperatury - przyspieszajacy (nie liniowy) wzrost od pewnego momentu.
   Wykrywanie tego DRUGIEGO przypadku (zmiana NACHYLENIA trendu, nie
   nagly skok) wymaga innego detektora niz `defect()` - patrz
   `core.thermal_drift_score()`.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, Optional

import numpy as np


def _smoothed_periodic_trapezoid(t: np.ndarray, cycle_s: float, duty: float, edge_frac: float) -> np.ndarray:
    """Gladki, powtarzalny trapez w [0, 1] - `duty` to udzial czasu na
    poziomie 1 (plus narastanie/opadanie), `edge_frac` to udzial cyklu na
    kazde zbocze (tanh, nie ostry kant - zeby jerk byl bliski zeru poza
    wstrzykniętymi zdarzeniami, tak samo jak w sensor_bus.py dla ruchu
    osi)."""
    phase = np.mod(t, cycle_s) / cycle_s
    edge_w = max(edge_frac, 1e-3)
    up = 0.5 * (1.0 + np.tanh((phase - edge_w) / (edge_w * 0.3)))
    down = 0.5 * (1.0 - np.tanh((phase - duty) / (edge_w * 0.3)))
    return np.clip(up * down, 0.0, 1.0)


def _gaussian_pulses_at(n_samples: int, centers: np.ndarray, widths: np.ndarray, amplitudes: np.ndarray) -> np.ndarray:
    """Suma gaussowskich impulsow (te same ksztalty co "backlash" w
    sensor_bus.py) - pomocnicze dla wstrzykiwanych zdarzen ponizej."""
    out = np.zeros(n_samples)
    idx = np.arange(n_samples)
    for c, w, a in zip(centers, widths, amplitudes):
        out += a * np.exp(-0.5 * ((idx - c) / max(w, 1)) ** 2)
    return out


# ---------------------------------------------------------------------
# 1. Chwytak (gripper)
# ---------------------------------------------------------------------

@dataclass
class GripperSpec:
    name: str = "gripper"
    hold_force_n: float = 20.0
    cycle_s: float = 5.0
    duty: float = 0.7
    noise_std_force: float = 0.15


def generate_gripper_trajectory(
    spec: GripperSpec,
    n_samples: int,
    dt: float,
    seed: int,
    defect_type: Optional[str] = None,
    defect_start_frac: float = 0.6,
    defect_severity: float = 1.0,
) -> Dict[str, np.ndarray]:
    rng = np.random.default_rng(seed)
    t = np.arange(n_samples) * dt
    profile = _smoothed_periodic_trapezoid(t, spec.cycle_s, spec.duty, edge_frac=0.1)
    grip_force = spec.hold_force_n * profile

    if defect_type == "slip_event":
        start_idx = int(n_samples * defect_start_frac)
        # zdarzenia poslizgu tylko tam, gdzie sila jest juz bliska
        # poziomowi trzymania (profile > 0.8) - "trzymanie" chwytanego
        # przedmiotu, nie faza otwierania/zamykania
        holding = profile > 0.8
        candidate_idx = np.where(holding)[0]
        candidate_idx = candidate_idx[candidate_idx >= start_idx]
        n_events = max(1, int(len(candidate_idx) / (spec.cycle_s / dt) * 3))
        if len(candidate_idx) > 0:
            centers = rng.choice(candidate_idx, size=min(n_events, len(candidate_idx)), replace=False)
            severities = np.linspace(0.3, 1.0, len(centers)) * defect_severity
            widths = np.full(len(centers), max(1, int(0.02 * spec.cycle_s / dt)))
            dips = _gaussian_pulses_at(n_samples, centers, widths, -spec.hold_force_n * 0.6 * severities)
            grip_force = grip_force + dips

    grip_force = grip_force + rng.normal(0.0, spec.noise_std_force, n_samples)
    return {"t": t, "grip_force": grip_force}


# ---------------------------------------------------------------------
# 2. Podstawa mobilna (mobile base, napad rozniczkowy)
# ---------------------------------------------------------------------

@dataclass
class MobileBaseSpec:
    name: str = "mobile_base"
    wheel_base_m: float = 0.5
    nominal_speed_mps: float = 0.3
    turn_amplitude_mps: float = 0.05
    cycle_s: float = 8.0
    noise_std_wheel: float = 0.003
    noise_std_gyro: float = 0.01


def generate_mobile_base_trajectory(
    spec: MobileBaseSpec,
    n_samples: int,
    dt: float,
    seed: int,
    defect_type: Optional[str] = None,
    defect_start_frac: float = 0.6,
    defect_severity: float = 1.0,
) -> Dict[str, np.ndarray]:
    """Zwraca: t, v_left, v_right (odczyty enkoderow kol, m/s),
    heading_rate_gyro (niezalezny pomiar predkosci obrotu, rad/s).

    Zdrowy robot: heading_rate_gyro ~ (v_right - v_left) / wheel_base
    (z szumem pomiarowym). `defect_type="wheel_slip"`: prawe kolo traci
    przyczepnosc od `defect_start_frac` - ENKODER nadal raportuje pelna
    predkosc komendowana (jakby kolo się kręciło normalnie), ale
    PRAWDZIWY ruch (zyroskop) już nie nadąża, bo kolo się poslizguje.
    """
    rng = np.random.default_rng(seed)
    t = np.arange(n_samples) * dt
    omega = 2.0 * np.pi / spec.cycle_s

    turn = spec.turn_amplitude_mps * np.sin(omega * t)
    v_left_cmd = spec.nominal_speed_mps - turn
    v_right_cmd = spec.nominal_speed_mps + turn

    v_left = v_left_cmd + rng.normal(0.0, spec.noise_std_wheel, n_samples)
    v_right = v_right_cmd + rng.normal(0.0, spec.noise_std_wheel, n_samples)

    slip_factor = np.zeros(n_samples)
    if defect_type == "wheel_slip":
        start_idx = int(n_samples * defect_start_frac)
        if start_idx < n_samples:
            span = n_samples - start_idx
            slip_factor[start_idx:] = defect_severity * np.linspace(0.0, 0.5, span)

    v_right_true = v_right_cmd * (1.0 - slip_factor)
    heading_rate_true = (v_right_true - v_left_cmd) / spec.wheel_base_m
    heading_rate_gyro = heading_rate_true + rng.normal(0.0, spec.noise_std_gyro, n_samples)

    return {
        "t": t,
        "v_left": v_left,
        "v_right": v_right,
        "heading_rate_gyro": heading_rate_gyro,
    }


# ---------------------------------------------------------------------
# 3. Kamera / wizja
# ---------------------------------------------------------------------

@dataclass
class CameraSpec:
    name: str = "camera"
    noise_std_px: float = 1.2


def generate_vision_trajectory(
    spec: CameraSpec,
    n_samples: int,
    dt: float,
    seed: int,
    defect_type: Optional[str] = None,
    defect_start_frac: float = 0.6,
    defect_severity: float = 1.0,
) -> Dict[str, np.ndarray]:
    """Zwraca: t, tracking_error_px (blad sledzenia celu w pikselach,
    bliski zeru przy stabilnym sledzeniu)."""
    rng = np.random.default_rng(seed)
    t = np.arange(n_samples) * dt
    tracking_error = rng.normal(0.0, spec.noise_std_px, n_samples)

    if defect_type == "tracking_loss":
        start_idx = int(n_samples * defect_start_frac)
        burst_len = min(n_samples - start_idx, int(1.5 / dt))
        if burst_len > 0:
            local = np.arange(burst_len)
            envelope = np.sin(np.pi * local / burst_len) ** 2  # gladki wzrost/spadek
            loss = defect_severity * 40.0 * envelope * rng.choice([-1, 1], size=burst_len)
            tracking_error[start_idx:start_idx + burst_len] += loss

    return {"t": t, "tracking_error_px": tracking_error}


# ---------------------------------------------------------------------
# 4. Zasilanie / bateria
# ---------------------------------------------------------------------

@dataclass
class PowerSpec:
    name: str = "power"
    nominal_voltage_v: float = 24.0
    nominal_current_a: float = 2.0
    nominal_temp_c: float = 35.0
    voltage_min_v: float = 21.0   # absolutny limit (analogiczny do EN 50160 w grid_core.py)
    noise_std_voltage: float = 0.05
    noise_std_current: float = 0.1
    noise_std_temp: float = 0.2


def generate_power_trajectory(
    spec: PowerSpec,
    n_samples: int,
    dt: float,
    seed: int,
    defect_type: Optional[str] = None,
    defect_start_frac: float = 0.6,
    defect_severity: float = 1.0,
) -> Dict[str, np.ndarray]:
    """Zwraca: t, voltage, current, temperature.

    Zdrowy przebieg: napiecie powoli opada pod obciazeniem (rozladowanie),
    prad oscyluje wokol nominalnego poboru, temperatura powoli rosnie
    (normalna praca). `defect_type="voltage_sag"`: nagly, krotki spadek
    napiecia (skok obciazenia). `defect_type="thermal_runaway"`: zamiast
    liniowego wzrostu temperatury - PRZYSPIESZAJACY wzrost od
    `defect_start_frac` (zmiana nachylenia trendu, nie pojedynczy skok -
    wymaga core.thermal_drift_score(), nie core.defect())."""
    rng = np.random.default_rng(seed)
    t = np.arange(n_samples) * dt
    duration = n_samples * dt

    voltage = spec.nominal_voltage_v - 1.5 * (t / max(duration, 1e-9))
    current = spec.nominal_current_a + 0.3 * np.sin(2 * np.pi * t / 6.0)
    temperature = spec.nominal_temp_c + 5.0 * (t / max(duration, 1e-9))

    if defect_type == "voltage_sag":
        start_idx = int(n_samples * defect_start_frac)
        n_events = 3
        centers = np.linspace(start_idx, n_samples - 1, n_events).astype(int)
        widths = np.full(n_events, max(1, int(0.3 / dt)))
        dips = _gaussian_pulses_at(n_samples, centers, widths, -defect_severity * 3.0 * np.ones(n_events))
        voltage = voltage + dips

    elif defect_type == "thermal_runaway":
        start_idx = int(n_samples * defect_start_frac)
        if start_idx < n_samples:
            span = n_samples - start_idx
            tau = np.arange(span) * dt
            runaway = defect_severity * 8.0 * (np.exp(tau / (0.4 * span * dt)) - 1.0)
            runaway = np.clip(runaway, 0, 60.0)
            temperature[start_idx:] += runaway

    voltage = voltage + rng.normal(0.0, spec.noise_std_voltage, n_samples)
    current = current + rng.normal(0.0, spec.noise_std_current, n_samples)
    temperature = temperature + rng.normal(0.0, spec.noise_std_temp, n_samples)

    return {"t": t, "voltage": voltage, "current": current, "temperature": temperature}
