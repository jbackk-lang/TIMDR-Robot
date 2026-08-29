"""timdr_robot/geometry.py — krzywizna i torsja (Freneta-Serreta) trajektorii
stanu jednej osi robota w przestrzeni fazowej (pozycja, predkosc, przyspieszenie)
================================================================================
Port 1:1 matematyki z TIMDR-META-DYNAMICS/analysis/meta_torsion.py
(field_curvature/field_torsion na trajektorii (Lambda, tau, rho)) -
uogolniony tutaj do DOWOLNEJ trajektorii 3D, nie tylko stanu meta. Ten sam
kod byl juz wczesniej portowany 1:1 z FLIGHT-TRACKING-TIMDR.frenet_serret()
(zweryfikowane tam numerycznie na helisie kolowej, blad wzgledny < 1e-5).

CO TU JEST LICZONE (i czego to NIE jest):
Dla jednej osi robota mamy trzy zsynchronizowane sygnaly: polozenie (kat
enkodera), predkosc katowa, przyspieszenie katowe. Traktujemy trojke
(pozycja(t), predkosc(t), przyspieszenie(t)) jako punkt w R^3 w kazdej
chwili t - czyli portret fazowy tej osi. Krzywizna i torsja TEJ krzywej
(nie samego sygnalu polozenia) mierza, jak bardzo ten portret fazowy
"skreca sie" w czasie:

    kappa   = |r' x r''| / |r'|^3
    torsion = ((r' x r'') . r''') / |r' x r''|^2

gdzie r', r'', r''' to pierwsza/druga/trzecia pochodna pozycji (Nx3) po
parametrze t.

**WAZNE ROZROZNIENIE (ta sama pulapka co w meta_torsion.py, tam
udokumentowana explicite):** to jest GEOMETRYCZNA torsja krzywej w
przestrzeni fazowej - NIE jest to fizyczne skrecenie mechaniczne
(torque/skrecenie walu pod obciazeniem). Nazwa "torsion" w tym pliku
odnosi sie WYLACZNIE do definicji z geometrii rozniczkowej. Hipoteza
robocza (NIEZWALIDOWANA na realnym robocie w chwili napisania tego kodu):
gladki, powtarzalny ruch osi daje mala/stabilna torsje portretu fazowego,
a mechaniczna wada (np. rosnacy luz/backlash, rezonans pod obciazeniem)
wprowadza dodatkowe skladowe wysokoczestotliwosciowe, ktore objawiaja sie
jako skoki tej torsji. To tylko hipoteza do sprawdzenia na syntetycznych
danych w tym repo (patrz demo/run_demo.py) - i pozniej, jesli w ogole, na
prawdziwym sprzecie.

OGRANICZENIE (identyczne jak w oryginale): torsja wymaga TRZECIEJ
pochodnej - surowe roznicowanie wzmacnia szum. Pozycja jest wygladzana
filtrem Savitzky-Golay (jesli scipy dostepne) i rozniczkowana analitycznie
z dopasowanego wielomianu. Krotkie serie (< poly+3 probek) zwracaja same
zera zamiast rzucac wyjatek w srodku pipeline'u.
"""
from __future__ import annotations

from typing import Tuple

import numpy as np

try:
    from scipy.signal import savgol_filter
    _HAS_SCIPY = True
except ImportError:  # pragma: no cover - zalezy od srodowiska
    _HAS_SCIPY = False


def frenet_serret_curvature_torsion(
    pos: np.ndarray,
    dt: float = 1.0,
    poly: int = 3,
) -> Tuple[np.ndarray, np.ndarray]:
    """Liczy (kappa, torsion) dla trajektorii `pos` (Nx3 ndarray).

    `dt`: krok miedzy kolejnymi probkami, zakladany STALY - jesli probki
    nie sa rownoodlegle, wynik bedzie bledny (to samo zalozenie co w
    meta_torsion.py/frenet_serret.py w innych repo tego zestawu).

    Zwraca (kappa, torsion), oba tablice numpy dlugosci len(pos).
    """
    pos = np.asarray(pos, dtype=float)
    if pos.ndim != 2 or pos.shape[1] != 3:
        raise ValueError(f"pos musi miec ksztalt (N, 3), dostalem {pos.shape}")

    n = len(pos)
    n_needed = poly + 3
    if n < n_needed:
        z = np.zeros(n)
        return z, z.copy()

    if _HAS_SCIPY:
        max_win = n if n % 2 == 1 else n - 1
        win = min(9, max_win)
        if win <= poly:
            win = poly + 2 if (poly + 2) % 2 == 1 else poly + 3
            win = min(win, max_win)
        if win <= poly:
            z = np.zeros(n)
            return z, z.copy()

        v = savgol_filter(pos, window_length=win, polyorder=poly, deriv=1, delta=dt, axis=0)
        a = savgol_filter(pos, window_length=win, polyorder=poly, deriv=2, delta=dt, axis=0)
        j = savgol_filter(pos, window_length=win, polyorder=poly, deriv=3, delta=dt, axis=0)
    else:  # pragma: no cover - brak scipy: roznicowanie skonczone, gorsza jakosc
        v = np.gradient(pos, dt, axis=0)
        a = np.gradient(v, dt, axis=0)
        j = np.gradient(a, dt, axis=0)

    cross_va = np.cross(v, a)
    speed = np.linalg.norm(v, axis=1)
    cross_norm = np.linalg.norm(cross_va, axis=1)

    kappa = np.zeros(n)
    torsion = np.zeros(n)
    ok_speed = speed > 1e-9
    kappa[ok_speed] = cross_norm[ok_speed] / (speed[ok_speed] ** 3)
    ok_cross = cross_norm > 1e-9
    numer = np.einsum('ij,ij->i', cross_va, j)
    torsion[ok_cross] = numer[ok_cross] / (cross_norm[ok_cross] ** 2)
    return kappa, torsion


def phase_portrait(position: np.ndarray, velocity: np.ndarray, accel: np.ndarray) -> np.ndarray:
    """Sklada trzy zsynchronizowane sygnaly osi (pozycja, predkosc,
    przyspieszenie) w jedna trajektorie Nx3 - wygodny helper zeby wywolanie
    w core.py bylo czytelne (`phase_portrait(pos, vel, acc)` zamiast
    `np.column_stack([pos, vel, acc])` rozsianego po kodzie)."""
    position = np.asarray(position, dtype=float)
    velocity = np.asarray(velocity, dtype=float)
    accel = np.asarray(accel, dtype=float)
    if not (len(position) == len(velocity) == len(accel)):
        raise ValueError(
            f"pozycja/predkosc/przyspieszenie musza miec te sama dlugosc, "
            f"dostalem {len(position)}/{len(velocity)}/{len(accel)}"
        )
    return np.column_stack([position, velocity, accel])
