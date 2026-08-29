"""timdr_robot/core.py — warstwa "TIMDR Core": laczy prymitywy anomalii
(MAD-z, port z TIMDR-Grid-Monitor), geometrie fazowa (geometry.py) i
analize rezonansu (ringdown.py) w jedna analize per-os.
================================================================================
`_mad_z()` i `defect()` sa portem 1:1 (bez zmian w matematyce) z
TIMDR-Grid-Monitor/grid_core.py, gdzie ten sam wzorzec byl juz uzyty w
timdr_core_finance.py/bio_core.py/catalog_core.py z poprzednich repo tego
zestawu - odporny z-score (MAD) do wykrywania nagle odstajacych probek,
"defect()" do wykrywania naglych SKOKOW miedzy kolejnymi probkami wzgledem
lokalnego rozstepu roznic (nie rozstepu poziomow - udokumentowana pulapka
w kodzie referencyjnym timdr_core_finance.py).

PIPELINE analyze_axis():
1. harmonic_law_residual() na (pozycja, przyspieszenie) -> reszta modelu
   prostego oscylatora harmonicznego (accel = -omega^2 * pozycja),
   dopasowanego na POCZATKOWYM oknie kalibracyjnym serii (patrz uwaga
   nizej). anomalies() (MAD-z) na tej reszcie -> indeksy anomalii.
2. frenet_serret_curvature_torsion() na portrecie fazowym (pozycja,
   predkosc, przyspieszenie) -> geometryczna torsja calej trajektorii.
   Skoki torsji powyzej progu MAD-z sa traktowane jako dodatkowe
   kandydatury na zdarzenia.
3. Dla NAJWIEKSZEGO wykrytego zdarzenia (jesli jakiekolwiek) ->
   ringdown_resonance() na sygnale przyspieszenia, zeby sprawdzic, czy
   powrot do poziomu odniesienia jest oscylacyjny (rezonans).
4. Zwraca dict metryk - status.py mapuje ten dict na AxisHealth.

**DLACZEGO NIE `defect()` NA SUROWEJ POZYCJI (pierwsza wersja tego pliku
tak robila i to byl realny, znaleziony empirycznie blad):** `defect()` w
wersji z TIMDR-Grid-Monitor zaklada sygnal w przewazajacej czesci PLASKI,
z rzadkimi, wyraznymi skokami (napiecie/czestotliwosc sieci). Pozycja osi
robota w ruchu to CIAGLE duza, oscylujaca wartosc (sinusoida) - lokalny
rozstep roznic (`spread`) w oknie 20 probek jest tego samego rzedu co
sama roznica krok-po-kroku wiekszosc czasu, wiec prog `jump_factor*spread`
wychodzil MNIEJSZY niz normalny krok ruchu i ~50% probek bylo falszywie
zglaszanych jako "skok" NAWET na czystym, syntetycznym sygnale bez
zadnej wstrzykniętej wady (potwierdzone empirycznie: negative_control_check
dawal false_positive_rate=1.00 na czystej osi przed ta poprawka). Zamiast
tego uzywamy modelu FIZYCZNEGO (prawo ruchu oscylatora harmonicznego)
zamiast czysto statystycznego skoku - reszta tego modelu jest bliska zeru
dla gladkiego ruchu niezaleznie od tego, jak duza jest sama amplituda
ruchu, i rosnie wyraznie tylko wtedy, gdy przyspieszenie przestaje pasowac
do prostego prawa `accel = -omega^2 * pozycja` (co robia zarowno
backlash, jak i resonance_burst w sensor_bus.py).

OGRANICZENIE tej metody: `omega^2` jest dopasowywane metoda najmniejszych
kwadratow na POCZATKOWYM oknie kalibracyjnym (domyslnie pierwsze 30%
probek) - zaklada to, ze os na poczatku analizowanej serii jest zdrowa/
scharakteryzowana. Jesli wada jest obecna JUZ od pierwszej probki, to
dopasowanie samo bedzie skazone i metoda nie wykryje niczego
nietypowego - to jest znane, udokumentowane ograniczenie tego szkieletu,
nie ukryta wada. Dziala tylko dla ruchu w przyblizeniu jednoczestosciowego
(jedna dominujaca sinusoida) - dowolniejszy profil ruchu (prawdziwy
trapez predkosci, wielosegmentowa trajektoria) wymagalby innego modelu
referencyjnego (np. dopasowanego wielomianu/filtra zamiast pojedynczego
omega) - kolejny krok POZA zakresem tego szkieletu.

NEGATIVE CONTROL (negative_control_check()): uruchamia analyze_axis() na
CZYSTYCH (bez wstrzykniętej wady) syntetycznych powtorzeniach tej samej
osi i liczy, jak czesto pipeline mimo to zglasza DEFECT/RESONANCE
(false positive rate). To NIE jest rygorystyczny test statystyczny
(brak formalnego modelu null/permutacji z pelnego protokolu
pre-rejestracji) - to podstawowy test poczytalnosci: "czy na czystych
danych alarmujemy zbyt czesto". Wynik nalezy raportowac wprost, nie
interpretowac jako dowod poprawnosci progu.
"""
from __future__ import annotations

from typing import Dict, List, Optional

import numpy as np

from . import geometry
from .ringdown import ringdown_resonance


# ---------------------------------------------------------------------
# Port 1:1 z TIMDR-Grid-Monitor/grid_core.py
# ---------------------------------------------------------------------

def _mad_z(x: np.ndarray) -> np.ndarray:
    """Odporny z-score: (x - mediana) / (1.4826 * MAD), z fallbackiem na
    rozstep/4 gdy MAD=0 (plaski sygnal)."""
    x = np.asarray(x, dtype=float)
    med = np.median(x)
    mad = np.median(np.abs(x - med))
    if mad > 1e-12:
        scale = 1.4826 * mad
    else:
        spread = np.max(x) - np.min(x)
        scale = spread / 4.0 if spread > 1e-12 else 1.0
    return (x - med) / scale


def _rolling_percentile_spread(x: np.ndarray, window: int, lo: float = 10, hi: float = 90) -> np.ndarray:
    """Rozstep percentyli w oknie kroczacym, TYLKO wstecz (przyczynowe)."""
    n = len(x)
    out = np.zeros(n)
    for i in range(n):
        start = max(0, i - window)
        seg = x[start:i + 1]
        if len(seg) < 3:
            out[i] = 0.0
        else:
            out[i] = np.percentile(seg, hi) - np.percentile(seg, lo)
    return out


def anomalies(x: np.ndarray, factor: float = 3.5) -> np.ndarray:
    """Indeksy probek odstajacych wg odpornego z-score (MAD-z)."""
    x = np.asarray(x, dtype=float)
    if len(x) == 0:
        return np.array([], dtype=int)
    z = _mad_z(x)
    return np.where(np.abs(z) > factor)[0]


def defect(x: np.ndarray, window: int = 20, jump_factor: float = 3.5,
           min_floor_frac: float = 1e-4) -> np.ndarray:
    """Nagle skoki miedzy kolejnymi probkami wzgledem rozstepu RoZNIC w
    oknie wstecznym. Podloga zapobiega zapadaniu sie progu do zera na
    plaskich odcinkach."""
    x = np.asarray(x, dtype=float)
    n = len(x)
    if n < 2:
        return np.array([], dtype=int)
    diffs = np.abs(np.diff(x))
    diffs_padded = np.concatenate([[0.0], diffs])
    spread = _rolling_percentile_spread(diffs_padded, window)
    floor = min_floor_frac * (np.median(np.abs(x)) + 1e-9)
    threshold = np.maximum(jump_factor * spread, floor)
    idx = np.where(diffs_padded > threshold)[0]
    return idx[idx > 0]


# ---------------------------------------------------------------------
# Model fizyczny: prosty oscylator harmoniczny jako "oczekiwany" ruch osi
# ---------------------------------------------------------------------

def harmonic_law_residual(
    position: np.ndarray,
    accel: np.ndarray,
    calib_frac: float = 0.3,
    min_calib_samples: int = 10,
) -> Dict:
    """Dopasowuje `omega^2` w modelu `accel = -omega^2 * position` metoda
    najmniejszych kwadratow (z zerowym wyrazem wolnym) na POCZATKOWYM
    oknie `calib_frac` probek, po czym liczy reszte tego modelu na CALEJ
    serii. Patrz docstring modulu po pelne uzasadnienie i ograniczenia.

    Zwraca dict: omega_sq, calib_n, residual (ndarray).
    """
    position = np.asarray(position, dtype=float)
    accel = np.asarray(accel, dtype=float)
    n = len(position)
    calib_n = max(min_calib_samples, int(n * calib_frac))
    calib_n = min(calib_n, n)

    p_calib = position[:calib_n]
    a_calib = accel[:calib_n]
    denom = float(np.sum(p_calib * p_calib))
    omega_sq = -float(np.sum(a_calib * p_calib) / denom) if denom > 1e-9 else 0.0

    residual = accel + omega_sq * position
    return {"omega_sq": omega_sq, "calib_n": calib_n, "residual": residual}


# ---------------------------------------------------------------------
# Analiza per-os: geometria fazowa + model harmoniczny + rezonans
# ---------------------------------------------------------------------

def analyze_axis(
    axis_id: str,
    t: np.ndarray,
    position: np.ndarray,
    velocity: np.ndarray,
    accel: np.ndarray,
    force: Optional[np.ndarray] = None,
    dt: Optional[float] = None,
    torsion_factor: float = 4.0,
    harmonic_anomaly_factor: float = 6.0,
    harmonic_calib_frac: float = 0.3,
    ringdown_pre_window: int = 20,
    ringdown_lookahead: int = 300,
) -> Dict:
    """Pelna analiza jednej osi. Zwraca dict metryk (nigdy nie rzuca
    wyjatku na poprawnych wejsciach - brakujace wykrycia daja puste listy/
    None, nie None-owy caly wynik, zeby wywolujacy zawsze mial spojna
    strukture do dalszego przetwarzania)."""
    t = np.asarray(t, dtype=float)
    position = np.asarray(position, dtype=float)
    velocity = np.asarray(velocity, dtype=float)
    accel = np.asarray(accel, dtype=float)
    n = len(position)

    if dt is None:
        dt = float(np.median(np.diff(t))) if n >= 2 else 1.0

    harmonic = harmonic_law_residual(position, accel, calib_frac=harmonic_calib_frac)
    harmonic_anomaly_idx = anomalies(harmonic["residual"], factor=harmonic_anomaly_factor)

    portrait = geometry.phase_portrait(position, velocity, accel)
    kappa, torsion = geometry.frenet_serret_curvature_torsion(portrait, dt=dt)
    torsion_spike_idx = anomalies(torsion, factor=torsion_factor) if len(torsion) else np.array([], dtype=int)

    candidate_events = sorted(set(harmonic_anomaly_idx.tolist()) | set(torsion_spike_idx.tolist()))

    ringdown_result = None
    ringdown_event_idx = None
    if candidate_events and n >= 10:
        # najwieksza reszta modelu harmonicznego sposrod kandydatow
        # (najbardziej prawdopodobne "prawdziwe" zdarzenie mechaniczne,
        # nie pojedynczy szum pomiarowy)
        resid_abs = np.abs(harmonic["residual"])
        ringdown_event_idx = max(candidate_events, key=lambda i: resid_abs[i] if i < n else 0.0)
        if ringdown_event_idx < n - 3:
            # Rezonans liczony na RESZCIE modelu harmonicznego, nie na
            # surowym przyspieszeniu: przyspieszenie osi w ruchu samo w
            # sobie oscyluje (sinusoida ruchu) wiec okno przed-zdarzeniowe
            # nie ma stabilnego poziomu odniesienia, ktorego wymaga
            # ringdown_resonance() (empirycznie potwierdzony blad - na
            # surowym accel is_oscillatory wychodzilo czesto False mimo
            # wyraznego wstrzykniętego rezonansu, bo noise_floor byl
            # zawyzony przez naturalna krzywizne sinusoidy ruchu w oknie
            # pre_event_window). Reszta modelu jest ~0 poza zdarzeniami,
            # wiec spelnia zalozenie "plaski poziom odniesienia PRZED
            # zdarzeniem", dla ktorego ta funkcja zostala zwalidowana.
            ringdown_result = ringdown_resonance(
                t, harmonic["residual"], event_idx=ringdown_event_idx,
                pre_event_window=ringdown_pre_window,
                max_lookahead=ringdown_lookahead,
            )

    return {
        "axis_id": axis_id,
        "n_samples": n,
        "dt": dt,
        "omega_sq": harmonic["omega_sq"],
        "harmonic_calib_n": harmonic["calib_n"],
        "harmonic_anomaly_count": int(len(harmonic_anomaly_idx)),
        "harmonic_anomaly_idx": harmonic_anomaly_idx.tolist(),
        "torsion_spike_count": int(len(torsion_spike_idx)),
        "torsion_spike_idx": torsion_spike_idx.tolist(),
        "torsion_max_abs": float(np.max(np.abs(torsion))) if len(torsion) else 0.0,
        "torsion_series": torsion.tolist(),
        "kappa_series": kappa.tolist(),
        "ringdown_event_idx": ringdown_event_idx,
        "ringdown": ringdown_result,
        "force_mean": float(np.mean(force)) if force is not None and len(force) else None,
    }


def negative_control_check(
    spec_factory,
    n_trials: int = 20,
    n_samples: int = 2000,
    dt: float = 0.01,
    base_seed: int = 9000,
) -> Dict:
    """Uruchamia analyze_axis() na `n_trials` CZYSTYCH (bez wstrzykniętej
    wady) syntetycznych powtorzeniach tej samej osi (rozne ziarna szumu) i
    liczy, jak czesto pipeline mimo to zglasza zdarzenie. `spec_factory`
    to funkcja `(seed) -> dict z kluczami t/position/velocity/accel/force`
    (typowo `sensor_bus.generate_axis_trajectory` z `defect_type=None`
    czesciowo zaaplikowane przez `functools.partial` lub lambde w
    wywolujacym kodzie).

    Zwraca: n_trials, n_false_positive, false_positive_rate, disclaimer.
    To NIE jest formalny test permutacyjny/null-model - patrz docstring
    modulu.
    """
    n_fp = 0
    for i in range(n_trials):
        traj = spec_factory(base_seed + i)
        result = analyze_axis(
            "negative_control", traj["t"], traj["position"], traj["velocity"],
            traj["accel"], traj.get("force"), dt=dt,
        )
        flagged = result["harmonic_anomaly_count"] > 0 or result["torsion_spike_count"] > 0
        if flagged and result["ringdown"] is not None and result["ringdown"]["is_oscillatory"]:
            n_fp += 1
        elif flagged and result["harmonic_anomaly_count"] > 2:
            n_fp += 1
    return {
        "n_trials": n_trials,
        "n_false_positive": n_fp,
        "false_positive_rate": n_fp / n_trials if n_trials else 0.0,
        "disclaimer": (
            "Podstawowy test poczytalnosci na czystych danych syntetycznych, "
            "NIE formalny test permutacyjny/null-model. Wysoki odsetek "
            "falszywych alarmow tutaj oznacza, ze progi (torsion_factor, "
            "harmonic_anomaly_factor) sa za czule na TEN typ syntetycznego "
            "sygnalu - nie mowi nic o zachowaniu na prawdziwym sprzecie."
        ),
    }
