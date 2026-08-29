"""timdr_robot/subsystem_core.py — TIMDR Core dla nowych podsystemow
(chwytak, podstawa mobilna, kamera, zasilanie) + generyczne prymitywy
(reszta wzgledem ruchomej mediany, dryft trendu, dryft widmowy) uzywane
przez te podsystemy.
================================================================================
Oddzielony od `core.py` (ktory zostaje NIETKNIETY - jest juz przetestowany
i strojony pod oz ramienia) celowo: te sygnaly maja INNY ksztalt niz
oscylujaca pozycja osi, wiec potrzebuja innych narzedzi. Zaimportowane z
`core.py`: `_mad_z`, `anomalies`, `defect` (bez duplikowania kodu).

CZTERY NOWE analyze_*() FUNKCJE + DWA NOWE GENERYCZNE PRYMITYWY:

- `baseline_residual(x, window)`: reszta wzgledem RUCHOMEJ MEDIANY w
  oknie `window` (filtr wygladzajacy nieliniowy - odporny na same
  anomalie, bo mediana nie "widzi" pojedynczych odstajacych probek jako
  bardzo, o ile nie stanowia > polowy okna). Dobry dla anomalii
  LOKALNYCH (impuls/dip), ZLY dla dryftu TRENDU (wolno rosnaca
  temperatura) - patrz `thermal_drift_score` po ten drugi przypadek.
  Empirycznie potwierdzone przy budowie tego pliku: proba wykrycia
  poslizgu chwytaka (`slip_event`) i zapadu napiecia (`voltage_sag`)
  bezposrednio przez `core.defect()` (jak dla oz) zawodzila, bo
  wstrzykniety impuls byl SZEROKI wzgledem domyslnego okna `defect()`
  (window=20) - lokalny rozstep roznic w tym oknie byl juz "wewnatrz"
  samego impulsu, wiec sam siebie maskowal. `baseline_residual()` z
  odpowiednio DLUZSZYM oknem (>> szerokosc impulsu) usuwa ten problem,
  bo porownuje probke do mediany SZEROKIEGO otoczenia, nie do lokalnego
  rozstepu roznic.

- `thermal_drift_score(x, dt, window, calib_frac)`: dla anomalii TRENDU
  (przyspieszajacy wzrost temperatury), nie impulsu. Liczy roznice miedzy
  wartoscia probki a przedluzeniem LINIOWEGO trendu dopasowanego na oknie
  kalibracyjnym - to jest DOKLADNIE ten sam wzorzec co
  `core.harmonic_law_residual()` dla osi (dopasuj prawo na czystym
  poczatku, sprawdzaj reszte na calej serii), tylko z prawem liniowym
  zamiast harmonicznego.

- `spectral_energy_drift(x, dt, window)`: ogolny, PROSTY proxy dla
  "analizy widma wibracji lozyska" / "sygnatury pradowej silnika" -
  liczy udzial energii WYSOKOCZESTOTLIWOSCIOWEJ (powyzej polowy pasma) w
  oknie kroczacym metoda FFT, sledzi zmiane tego udzialu w czasie.
  **JAWNE OGRANICZENIE:** to NIE jest certyfikowana metoda diagnostyki
  lozysk (brakuje formuł BPFO/BPFI, ktore wymagaja geometrii lozyska:
  srednica kulek, srednica podzialowa, kat kontaktu, liczba kulek) ani
  prawdziwa analiza sygnatury pradowej silnika (MCSA, ktora szuka
  konkretnych wstazek bocznych wokol czestotliwosci sieciowej). To jest
  GENERYCZNY wskaznik "czy udzial wysokich czestotliwosci w tym sygnale
  rosnie w czasie" - uzyteczny jako pierwszy, przesiewowy sygnal, NIE
  jako diagnoza konkretnej usterki mechanicznej.
"""
from __future__ import annotations

from typing import Dict, Optional

import numpy as np

from .core import _mad_z, anomalies, defect  # noqa: F401 (defect re-eksportowany dla spojnosci API)


# ---------------------------------------------------------------------
# Generyczne prymitywy
# ---------------------------------------------------------------------

def baseline_residual(x: np.ndarray, window: int = 61) -> np.ndarray:
    """Reszta wzgledem ruchomej mediany w oknie `window` (symetrycznym,
    przyczynowo-NIEprzyczynowym - patrzy w obie strony, wiec nie nadaje
    sie do strumieniowania w czasie rzeczywistym bez modyfikacji, tylko
    do analizy juz zakonczonej serii - patrz README ograniczenia)."""
    x = np.asarray(x, dtype=float)
    n = len(x)
    half = max(1, window // 2)
    med = np.empty(n)
    for i in range(n):
        lo, hi = max(0, i - half), min(n, i + half + 1)
        med[i] = np.median(x[lo:hi])
    return x - med


def thermal_drift_score(
    x: np.ndarray,
    dt: float,
    calib_frac: float = 0.3,
    min_calib_samples: int = 20,
) -> Dict:
    """Dopasowuje linie prosta (trend) na POCZATKOWYM oknie kalibracyjnym
    (`calib_frac` probek), ekstrapoluje ja na cala serie i liczy reszte.
    Dla zdrowego, liniowego trendu reszta jest bliska zeru (szum); dla
    PRZYSPIESZAJACEGO wzrostu (np. thermal runaway) reszta rosnie
    systematycznie, nie tylko punktowo - dlatego `anomalies()` na tej
    reszcie wykrywa koniec serii (gdzie odchylenie od przedluzonej prostej
    jest najwieksze), a nie pojedynczy wyizolowany punkt.

    Zwraca dict: slope, intercept, calib_n, residual (ndarray).
    """
    x = np.asarray(x, dtype=float)
    n = len(x)
    t = np.arange(n) * dt
    calib_n = max(min_calib_samples, int(n * calib_frac))
    calib_n = min(calib_n, n)

    t_c = t[:calib_n]
    x_c = x[:calib_n]
    # regresja liniowa metoda najmniejszych kwadratow (z wyrazem wolnym)
    A = np.column_stack([t_c, np.ones_like(t_c)])
    (slope, intercept), *_ = np.linalg.lstsq(A, x_c, rcond=None)

    predicted = slope * t + intercept
    residual = x - predicted
    return {"slope": float(slope), "intercept": float(intercept), "calib_n": calib_n, "residual": residual}


def spectral_energy_drift(
    x: np.ndarray,
    dt: float,
    window: int = 128,
    step: Optional[int] = None,
    ratio_factor: float = 8.0,
    floor: float = 0.02,
) -> Dict:
    """Windowed FFT: dla kazdego okna liczy udzial energii
    wysokoczestotliwosciowej (> polowa Nyquista) w calkowitej energii
    widma. Zwraca serie tego udzialu w czasie (jedna wartosc na okno) +
    indeksy okien, w ktorych udzial jest anomalny.

    **DLACZEGO PROG WZGLEDNY (`ratio_factor * mediana`, z podloga
    `floor`), NIE `anomalies()`/MAD-z:** przy typowej liczbie okien w
    serii demo (rzedu dziesiatek, nie tysiecy probek) `anomalies()`
    zawodzil empirycznie - mediana udzialu wysokich czestotliwosci na
    czystym sygnale jest bardzo mala (rzedu 0.001-0.01), wiec MAD tez
    jest mikroskopijny, i zwykle statystyczne wahania miedzy oknami
    (n=30 to malo probek do stabilnego oszacowania MAD) dawaly z-score
    > 4 na WIELU czystych oknach (falszywe alarmy). Prog wzgledem
    mediany z jawna podloga jest prostszy i empirycznie stabilniejszy
    dla tej konkretnej, silnie skosnej (bounded w [0,1], bliskiej zeru)
    statystyki.

    Zwraca dict: window_centers_idx, high_freq_fraction (ndarray),
    anomaly_window_idx.
    """
    x = np.asarray(x, dtype=float)
    n = len(x)
    if step is None:
        step = window // 2
    if n < window:
        return {"window_centers_idx": np.array([], dtype=int), "high_freq_fraction": np.array([]), "anomaly_window_idx": np.array([], dtype=int)}

    centers = []
    fractions = []
    win_func = np.hanning(window)
    for start in range(0, n - window + 1, step):
        seg = x[start:start + window] * win_func
        spec = np.abs(np.fft.rfft(seg))
        power = spec ** 2
        total = np.sum(power)
        if total <= 1e-12:
            fractions.append(0.0)
        else:
            half = len(power) // 2
            fractions.append(float(np.sum(power[half:]) / total))
        centers.append(start + window // 2)

    fractions = np.array(fractions)
    centers = np.array(centers, dtype=int)
    if len(fractions) >= 3:
        med = np.median(fractions)
        threshold = max(ratio_factor * med, floor)
        anomaly_idx = np.where(fractions > threshold)[0]
    else:
        anomaly_idx = np.array([], dtype=int)
    return {"window_centers_idx": centers, "high_freq_fraction": fractions, "anomaly_window_idx": anomaly_idx}


# ---------------------------------------------------------------------
# 1. Chwytak (gripper)
# ---------------------------------------------------------------------

def analyze_gripper(
    component_id: str,
    t: np.ndarray,
    grip_force: np.ndarray,
    residual_window: int = 61,
    anomaly_factor: float = 5.0,
) -> Dict:
    residual = baseline_residual(grip_force, window=residual_window)
    idx = anomalies(residual, factor=anomaly_factor)
    return {
        "component_id": component_id,
        "n_samples": len(grip_force),
        "anomaly_count": int(len(idx)),
        "anomaly_idx": idx.tolist(),
        "residual_max_abs": float(np.max(np.abs(residual))) if len(residual) else 0.0,
    }


# ---------------------------------------------------------------------
# 2. Podstawa mobilna
# ---------------------------------------------------------------------

def analyze_mobile_base(
    component_id: str,
    t: np.ndarray,
    v_left: np.ndarray,
    v_right: np.ndarray,
    heading_rate_gyro: np.ndarray,
    wheel_base_m: float,
    anomaly_factor: float = 4.0,
) -> Dict:
    """Reszta = pomiar niezalezny (zyroskop) - model kinematyczny z
    enkoderow kol. Patrz docstring modulu `subsystems.py` po pelne
    uzasadnienie fizyczne (poslizg kola: enkoder "klamie", zyroskop nie)."""
    v_left = np.asarray(v_left, dtype=float)
    v_right = np.asarray(v_right, dtype=float)
    heading_rate_gyro = np.asarray(heading_rate_gyro, dtype=float)
    heading_rate_expected = (v_right - v_left) / wheel_base_m
    residual = heading_rate_gyro - heading_rate_expected
    idx = anomalies(residual, factor=anomaly_factor)
    return {
        "component_id": component_id,
        "n_samples": len(v_left),
        "anomaly_count": int(len(idx)),
        "anomaly_idx": idx.tolist(),
        "residual_max_abs": float(np.max(np.abs(residual))) if len(residual) else 0.0,
    }


# ---------------------------------------------------------------------
# 3. Kamera / wizja
# ---------------------------------------------------------------------

def analyze_vision(
    component_id: str,
    t: np.ndarray,
    tracking_error_px: np.ndarray,
    anomaly_factor: float = 4.0,
) -> Dict:
    """Blad sledzenia jest juz z natury bliski zeru (stacjonarny) - wiec
    `anomalies()` (MAD-z) mozna zastosowac WPROST, bez posredniego modelu
    fizycznego (w odroznieniu od pozycji osi w ruchu)."""
    tracking_error_px = np.asarray(tracking_error_px, dtype=float)
    idx = anomalies(tracking_error_px, factor=anomaly_factor)
    return {
        "component_id": component_id,
        "n_samples": len(tracking_error_px),
        "anomaly_count": int(len(idx)),
        "anomaly_idx": idx.tolist(),
        "max_abs_error_px": float(np.max(np.abs(tracking_error_px))) if len(tracking_error_px) else 0.0,
    }


# ---------------------------------------------------------------------
# 4. Zasilanie / bateria
# ---------------------------------------------------------------------

def analyze_power(
    component_id: str,
    t: np.ndarray,
    voltage: np.ndarray,
    current: np.ndarray,
    temperature: np.ndarray,
    dt: Optional[float] = None,
    voltage_min_v: float = 21.0,
    voltage_residual_window: int = 61,
    voltage_anomaly_factor: float = 5.0,
    thermal_calib_frac: float = 0.3,
    thermal_anomaly_factor: float = 5.0,
) -> Dict:
    """Laczy DWA rozne mechanizmy, tak jak grid_core.py dla napiecia/
    czestotliwosci sieci:
    (a) bezwzgledny limit fizyczny (`voltage_min_v`) - nigdy nie
        przeoczy realnego niedopuszczalnego spadku, niezaleznie od tego,
        jak "przyzwyczajona" jest bateria do niskiego stanu naladowania;
    (b) adaptacyjna reszta wzgledem ruchomej mediany (`baseline_residual`)
        - lapie WZGLEDNE, przejsciowe zapady napiecia (voltage_sag),
        nawet jesli mieszcza sie w limicie bezwzglednym.
    Temperatura uzywa `thermal_drift_score` (dryft trendu), NIE
    `baseline_residual` (dryft trendu jest z definicji NIEWIDOCZNY dla
    filtra mediany - mediana w szerokim oknie podaza za powolnym
    trendem tak samo jak sam sygnal, kasujac reszte; potwierdzone
    empirycznie przy budowie tego pliku)."""
    t = np.asarray(t, dtype=float)
    voltage = np.asarray(voltage, dtype=float)
    current = np.asarray(current, dtype=float)
    temperature = np.asarray(temperature, dtype=float)
    n = len(voltage)
    if dt is None:
        dt = float(np.median(np.diff(t))) if n >= 2 else 1.0

    absolute_violation_idx = np.where(voltage < voltage_min_v)[0]

    voltage_residual = baseline_residual(voltage, window=voltage_residual_window)
    voltage_anomaly_idx = anomalies(voltage_residual, factor=voltage_anomaly_factor)

    thermal = thermal_drift_score(temperature, dt, calib_frac=thermal_calib_frac)
    thermal_anomaly_idx = anomalies(thermal["residual"], factor=thermal_anomaly_factor)

    return {
        "component_id": component_id,
        "n_samples": n,
        "voltage_absolute_violation_count": int(len(absolute_violation_idx)),
        "voltage_absolute_violation_idx": absolute_violation_idx.tolist(),
        "voltage_anomaly_count": int(len(voltage_anomaly_idx)),
        "voltage_anomaly_idx": voltage_anomaly_idx.tolist(),
        "thermal_slope": thermal["slope"],
        "thermal_anomaly_count": int(len(thermal_anomaly_idx)),
        "thermal_anomaly_idx": thermal_anomaly_idx.tolist(),
        "thermal_residual_max_abs": float(np.max(np.abs(thermal["residual"]))) if n else 0.0,
    }
