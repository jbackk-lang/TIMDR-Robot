"""tests/test_core.py — testy timdr_robot/core.py, ze szczegolnym naciskiem
na regresje po realnym, empirycznie znalezionym bledzie: pierwsza wersja
`analyze_axis()` uzywala `defect()` (skoki wzgledem lokalnego rozstepu
roznic) BEZPOSREDNIO na surowej, oscylujacej pozycji osi w ruchu, co
falszywie flagowalo ~50% probek na CZYSTYM sygnale (potwierdzone: uruchomienie
demo/run_demo.py przed poprawka dawalo negative_control false_positive_rate
= 1.00 na wszystkich 6 osiach, wliczajac te bez wstrzykniętej wady). Poprawka:
`harmonic_law_residual()` (dopasowanie prawa oscylatora harmonicznego na
oknie kalibracyjnym) zamiast `defect()` na pozycji. Testy ponizej
odtwarzaja dokladnie ten scenariusz, zeby regresja byla wykrywalna
automatycznie, nie tylko przy recznym uruchomieniu demo."""
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from timdr_robot import core
from timdr_robot.sensor_bus import AxisSpec, generate_axis_trajectory

SPEC = AxisSpec(name="test_axis", amplitude_deg=30.0, cycle_s=3.0)
N_SAMPLES = 2000
DT = 0.01


def test_clean_moving_axis_has_no_harmonic_anomalies():
    """To jest DOKLADNIE test, ktory pierwsza wersja pipeline'u zawalala:
    zdrowa, w pelni ruchoma (nie plaska) os nie powinna byc flagowana."""
    traj = generate_axis_trajectory(SPEC, N_SAMPLES, DT, seed=1, defect_type=None)
    result = core.analyze_axis(
        "axis_clean", traj["t"], traj["position"], traj["velocity"], traj["accel"],
    )
    assert result["harmonic_anomaly_count"] == 0


def test_backlash_defect_is_detected_after_onset():
    traj = generate_axis_trajectory(
        SPEC, N_SAMPLES, DT, seed=1, defect_type="backlash",
        defect_start_frac=0.6, defect_severity=1.0,
    )
    result = core.analyze_axis(
        "axis_backlash", traj["t"], traj["position"], traj["velocity"], traj["accel"],
    )
    assert result["harmonic_anomaly_count"] > 10
    onset_idx = int(N_SAMPLES * 0.6)
    assert min(result["harmonic_anomaly_idx"]) >= onset_idx - 5


def test_resonance_burst_is_detected_and_is_oscillatory():
    traj = generate_axis_trajectory(
        SPEC, N_SAMPLES, DT, seed=1, defect_type="resonance_burst",
        defect_start_frac=0.6, defect_severity=1.0,
    )
    result = core.analyze_axis(
        "axis_resonance", traj["t"], traj["position"], traj["velocity"], traj["accel"],
    )
    assert result["harmonic_anomaly_count"] > 0
    assert result["ringdown"] is not None
    assert result["ringdown"]["is_oscillatory"] is True


def test_negative_control_false_positive_rate_is_low_on_clean_axis():
    from functools import partial
    nc = core.negative_control_check(
        partial(generate_axis_trajectory, SPEC, N_SAMPLES, DT, defect_type=None),
        n_trials=10, dt=DT,
    )
    assert nc["false_positive_rate"] <= 0.2, (
        "negative control zbyt wysoki - to byl dokladnie objaw bledu z "
        "pierwszej wersji pipeline'u (defect() na surowej pozycji)"
    )


def test_harmonic_law_residual_recovers_expected_omega():
    traj = generate_axis_trajectory(SPEC, N_SAMPLES, DT, seed=1, defect_type=None)
    result = core.harmonic_law_residual(traj["position"], traj["accel"])
    expected_omega_sq = (2 * np.pi / SPEC.cycle_s) ** 2
    assert result["omega_sq"] == pytest.approx(expected_omega_sq, rel=0.01)


def test_mad_z_matches_grid_monitor_port_on_known_input():
    """Sanity check ze port `_mad_z`/`anomalies`/`defect` z TIMDR-Grid-Monitor
    dziala tak samo jak oryginal na prostym, znanym wejsciu (kilka wyraznych
    outlierow na plaskim tle - dokladnie przypadek, dla ktorego defect()
    zostal zaprojektowany, w odroznieniu od oscylujacej pozycji osi)."""
    x = np.zeros(100)
    x[50] = 10.0
    idx = core.anomalies(x, factor=3.5)
    assert 50 in idx
