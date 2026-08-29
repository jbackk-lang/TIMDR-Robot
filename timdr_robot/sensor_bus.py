"""timdr_robot/sensor_bus.py — Warstwa "Robot Hardware": SYNTETYCZNE dane
z enkoderow, IMU i czujnikow sily/momentu dla wieloosiowego ramienia
================================================================================
**TO JEST GENERATOR DANYCH SYNTETYCZNYCH, NIE STEROWNIK SPRZETU.** Zgodnie
z ustalonym podejsciem ("najpierw szkielet + dane syntetyczne, potem
realne testy") ten modul NIE laczy sie z zadnym prawdziwym robotem, PLC
ani magistrala (CAN/EtherCAT/ROS) - generuje wiarygodne KSZTALTEM sygnaly
(pozycja/predkosc/przyspieszenie/moment na oz), z opcjonalnie wstrzykniętą
syntetyczna wada mechaniczna, zeby dalo sie zbudowac i przetestowac cala
reszte pipeline'u (TIMDR Core -> TIMDR Integration -> System Logic) zanim
podlaczy sie cokolwiek prawdziwego.

MODEL RUCHU JEDNEJ OSI (na sesje demo): trapezoidalny profil predkosci
(przyspiesz -> stala predkosc -> zwolnij) powtarzany cyklicznie, plus szum
pomiarowy gaussowski. Przyspieszenie liczone analitycznie z pochodnej
profilu (nie roznicowaniem numerycznym pozycji), zeby nie mieszac szumu
numerycznego z szumem symulowanym - to swiadomy wybor, inaczej kazda
"wada" bylaby po czesci artefaktem wlasnego roznicowania.

DWA TYPY WSTRZYKIWANEJ WADY (parametr `defect_type`):
- "backlash": rosnacy w czasie "martwy skok"/drzenie pozycji przy kazdej
  zmianie kierunku ruchu (typowy objaw zuzytego luzu w przekladni) -
  amplituda rosnie liniowo od `defect_start_frac` do konca serii.
- "resonance_burst": pojedynczy, tlumiony oscylacyjny impuls (symulowane
  "dzwonienie" mechaniczne) wstrzykniety w jednym, ustalonym momencie pod
  obciazeniem - do tego wlasnie sluzy ringdown_resonance().

Zdrowa os (defect_type=None) ma tylko szum pomiarowy, zero wstrzykniętych
anomalii - to jest "kontrola" uzywana w core.negative_control_check().
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, Optional

import numpy as np


@dataclass
class AxisSpec:
    """Parametry fizyczne jednej syntetycznej osi (jednostki umowne, nie
    skalibrowane do konkretnego robota - to jest szkielet)."""
    name: str
    amplitude_deg: float = 45.0       # polowa zakresu ruchu (stopnie)
    cycle_s: float = 4.0              # okres jednego pelnego cyklu ruchu
    noise_std_pos: float = 0.02       # szum pomiarowy pozycji (stopnie)
    noise_std_force: float = 0.5      # szum pomiarowy momentu (Nm, umownie)
    force_base: float = 5.0           # bazowe obciazenie osi (Nm, umownie)


def _trapezoid_position(t: np.ndarray, amplitude: float, cycle_s: float) -> np.ndarray:
    """Gladki, powtarzalny ruch osi: `amplitude * sin(2*pi*t/cycle_s)` -
    wybrany zamiast prawdziwego trapezu predkosci, zeby analityczne
    pochodne (predkosc, przyspieszenie) byly trywialne i dokladne (sin/cos),
    bez wprowadzania wlasnych artefaktow numerycznych do sygnalu, ktory
    materma byc "czysty" tam, gdzie nie wstrzykujemy celowo wady."""
    omega = 2.0 * np.pi / cycle_s
    return amplitude * np.sin(omega * t)


def generate_axis_trajectory(
    spec: AxisSpec,
    n_samples: int,
    dt: float,
    seed: int,
    defect_type: Optional[str] = None,
    defect_start_frac: float = 0.6,
    defect_severity: float = 1.0,
) -> Dict[str, np.ndarray]:
    """Generuje syntetyczna trajektorie jednej osi.

    Zwraca dict: t, position, velocity, accel, force (wszystkie ndarray
    dlugosci n_samples).
    """
    rng = np.random.default_rng(seed)
    t = np.arange(n_samples) * dt
    omega = 2.0 * np.pi / spec.cycle_s

    position = _trapezoid_position(t, spec.amplitude_deg, spec.cycle_s)
    velocity = spec.amplitude_deg * omega * np.cos(omega * t)
    accel = -spec.amplitude_deg * (omega ** 2) * np.sin(omega * t)
    force = spec.force_base + 0.3 * spec.force_base * np.sin(omega * t)

    if defect_type == "backlash":
        start_idx = int(n_samples * defect_start_frac)
        ramp = np.zeros(n_samples)
        if start_idx < n_samples:
            span = n_samples - start_idx
            ramp[start_idx:] = np.linspace(0.0, 1.0, span)
        # "martwy skok" przy kazdej zmianie kierunku: dodaj krotki impuls
        # do pozycji/predkosci tam, gdzie predkosc zmienia znak (przejscie
        # przez zero), z amplituda rosnaca wg `ramp` i `defect_severity`.
        sign_changes = np.where(np.diff(np.sign(velocity)) != 0)[0]
        backlash_signal = np.zeros(n_samples)
        pulse_width = max(1, int(0.05 * spec.cycle_s / dt))
        for idx in sign_changes:
            lo = max(0, idx - pulse_width // 2)
            hi = min(n_samples, idx + pulse_width // 2)
            local = np.arange(lo, hi) - idx
            pulse = np.exp(-0.5 * (local / max(pulse_width / 4, 1)) ** 2)
            backlash_signal[lo:hi] += pulse
        backlash_signal *= ramp * defect_severity * 0.8  # skala: stopnie
        position = position + backlash_signal
        velocity = velocity + np.gradient(backlash_signal, dt)
        accel = accel + np.gradient(np.gradient(backlash_signal, dt), dt)

    elif defect_type == "resonance_burst":
        start_idx = int(n_samples * defect_start_frac)
        burst_freq_hz = 15.0  # znaczaco szybsze niz ruch osi -> latwe do odroznienia
        burst_damping = 6.0   # 1/s, tlumienie wykladnicze
        burst_len = min(n_samples - start_idx, int(2.0 / dt))
        if burst_len > 0:
            tau = np.arange(burst_len) * dt
            burst = (
                defect_severity
                * 3.0
                * np.exp(-burst_damping * tau)
                * np.sin(2.0 * np.pi * burst_freq_hz * tau)
            )
            accel_burst = np.zeros(n_samples)
            accel_burst[start_idx:start_idx + burst_len] = burst
            accel = accel + accel_burst
            force = force + accel_burst * 0.2

    position = position + rng.normal(0.0, spec.noise_std_pos, n_samples)
    force = force + rng.normal(0.0, spec.noise_std_force, n_samples)

    return {
        "t": t,
        "position": position,
        "velocity": velocity,
        "accel": accel,
        "force": force,
    }


@dataclass
class SensorBus:
    """Zbior syntetycznych osi ramienia - jeden `generate_axis_trajectory()`
    wynik na oz, wygenerowany z gory (nie strumieniowo) na potrzeby demo i
    dashboardu. Uzycie strumieniowe (`.step()` w czasie rzeczywistym z
    prawdziwego sprzetu) to kolejny krok POZA zakresem tego szkieletu -
    patrz README, sekcja "Co dalej"."""
    axes: Dict[str, Dict[str, np.ndarray]] = field(default_factory=dict)

    def add_axis(self, axis_id: str, trajectory: Dict[str, np.ndarray]) -> None:
        self.axes[axis_id] = trajectory

    def read_axis(self, axis_id: str) -> Dict[str, np.ndarray]:
        if axis_id not in self.axes:
            raise KeyError(f"Nieznana os: {axis_id!r}. Dostepne: {list(self.axes)}")
        return self.axes[axis_id]

    def axis_ids(self):
        return list(self.axes.keys())


def build_arm_scenario(
    n_axes: int = 6,
    n_samples: int = 2000,
    dt: float = 0.01,
    seed: int = 42,
    defect_axis_index: Optional[int] = 3,
    defect_type: Optional[str] = "backlash",
    defect_start_frac: float = 0.6,
    defect_severity: float = 1.0,
) -> SensorBus:
    """Generuje scenariusz demonstracyjny: `n_axes`-osiowe ramie robotyczne,
    gdzie jedna wybrana os (domyslnie indeks 3, zgodnie z przykladem
    "ramie robotyczne" z dokumentu architektury) ma wstrzykniete
    pogarszajace sie zuzycie (domyslnie "backlash"), a pozostale osie sa
    "czyste" (tylko szum pomiarowy).
    """
    bus = SensorBus()
    for i in range(n_axes):
        axis_id = f"axis_{i}"
        spec = AxisSpec(
            name=axis_id,
            amplitude_deg=30.0 + 5.0 * i,
            cycle_s=3.0 + 0.3 * i,
        )
        this_defect = defect_type if (defect_axis_index is not None and i == defect_axis_index) else None
        traj = generate_axis_trajectory(
            spec, n_samples, dt, seed=seed + i,
            defect_type=this_defect,
            defect_start_frac=defect_start_frac,
            defect_severity=defect_severity,
        )
        bus.add_axis(axis_id, traj)
    return bus
