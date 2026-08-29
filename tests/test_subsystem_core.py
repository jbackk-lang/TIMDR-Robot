"""tests/test_subsystem_core.py — testy detektorow dla nowych podsystemow.
Regresje empiryczne, znalezione podczas budowy (patrz README):
`core.defect()` z domyslnym oknem nie wykrywal `slip_event`/`voltage_sag`
bo wstrzykniety impuls byl SZEROKI wzgledem okna - naprawione
`baseline_residual()` z dluzszym oknem. `anomalies()`/MAD-z na
`high_freq_fraction` (n~30 okien) dawal falszywe alarmy na czystych
danych - naprawione progiem wzglednym z podloga."""
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from timdr_robot import subsystem_core as sc
from timdr_robot import subsystems as ss

N, DT = 2000, 0.01


def test_baseline_residual_is_near_zero_on_flat_signal():
    x = np.full(200, 5.0) + np.random.default_rng(0).normal(0, 0.01, 200)
    resid = sc.baseline_residual(x, window=41)
    assert np.max(np.abs(resid)) < 0.1


def test_baseline_residual_flags_localized_pulse():
    x = np.full(300, 5.0)
    x[150:160] -= 3.0
    resid = sc.baseline_residual(x, window=61)
    assert np.max(np.abs(resid[145:165])) > 1.0


def test_thermal_drift_score_near_zero_for_pure_linear_trend():
    t = np.arange(1000) * DT
    x = 30.0 + 0.01 * t
    result = sc.thermal_drift_score(x, DT)
    assert np.max(np.abs(result["residual"])) < 0.5


def test_thermal_drift_score_grows_for_accelerating_trend():
    t = np.arange(1000) * DT
    x = np.concatenate([30.0 + 0.01 * t[:600], 30.0 + 0.01 * t[600] + 0.2 * (t[600:] - t[600])])
    result = sc.thermal_drift_score(x, DT, calib_frac=0.3)
    baseline = sc.thermal_drift_score(30.0 + 0.01 * t, DT, calib_frac=0.3)
    # reszta na przyspieszajacym trendzie musi byc wyraznie wieksza niz na
    # czysto liniowym trendzie o tym samym nachyleniu poczatkowym (wartosc
    # bezwzgledna zalezy od dlugosci/wielkosci skoku nachylenia, wiec
    # sprawdzamy separacje, nie arbitralna liczbe)
    assert result["residual"][-1] > baseline["residual"][-1] + 0.5


def test_analyze_gripper_clean_has_few_anomalies():
    g = ss.generate_gripper_trajectory(ss.GripperSpec(), N, DT, seed=1, defect_type=None)
    result = sc.analyze_gripper("gripper_0", g["t"], g["grip_force"])
    assert result["anomaly_count"] <= 3


def test_analyze_gripper_detects_slip_event():
    gd = ss.generate_gripper_trajectory(ss.GripperSpec(), N, DT, seed=1, defect_type="slip_event", defect_start_frac=0.6)
    result = sc.analyze_gripper("gripper_0", gd["t"], gd["grip_force"])
    assert result["anomaly_count"] > 10


def test_analyze_mobile_base_clean_has_no_anomalies():
    spec = ss.MobileBaseSpec()
    mb = ss.generate_mobile_base_trajectory(spec, N, DT, seed=1, defect_type=None)
    result = sc.analyze_mobile_base("base_0", mb["t"], mb["v_left"], mb["v_right"], mb["heading_rate_gyro"], spec.wheel_base_m)
    assert result["anomaly_count"] == 0


def test_analyze_mobile_base_detects_wheel_slip():
    spec = ss.MobileBaseSpec()
    mbd = ss.generate_mobile_base_trajectory(spec, N, DT, seed=1, defect_type="wheel_slip", defect_start_frac=0.6)
    result = sc.analyze_mobile_base("base_0", mbd["t"], mbd["v_left"], mbd["v_right"], mbd["heading_rate_gyro"], spec.wheel_base_m)
    assert result["anomaly_count"] > 50


def test_analyze_vision_clean_has_no_anomalies():
    c = ss.generate_vision_trajectory(ss.CameraSpec(), N, DT, seed=1, defect_type=None)
    result = sc.analyze_vision("camera_0", c["t"], c["tracking_error_px"])
    assert result["anomaly_count"] == 0


def test_analyze_vision_detects_tracking_loss():
    cd = ss.generate_vision_trajectory(ss.CameraSpec(), N, DT, seed=1, defect_type="tracking_loss", defect_start_frac=0.6)
    result = sc.analyze_vision("camera_0", cd["t"], cd["tracking_error_px"])
    assert result["anomaly_count"] > 10


def test_analyze_power_clean_reports_no_violations():
    spec = ss.PowerSpec()
    p = ss.generate_power_trajectory(spec, N, DT, seed=1, defect_type=None)
    result = sc.analyze_power("power_0", p["t"], p["voltage"], p["current"], p["temperature"], voltage_min_v=spec.voltage_min_v)
    assert result["voltage_absolute_violation_count"] == 0
    assert result["thermal_anomaly_count"] == 0


def test_analyze_power_detects_voltage_sag_both_ways():
    spec = ss.PowerSpec()
    pv = ss.generate_power_trajectory(spec, N, DT, seed=1, defect_type="voltage_sag", defect_start_frac=0.6)
    result = sc.analyze_power("power_0", pv["t"], pv["voltage"], pv["current"], pv["temperature"], voltage_min_v=spec.voltage_min_v)
    assert result["voltage_absolute_violation_count"] > 0
    assert result["voltage_anomaly_count"] > 0


def test_analyze_power_detects_thermal_runaway():
    spec = ss.PowerSpec()
    pt = ss.generate_power_trajectory(spec, N, DT, seed=1, defect_type="thermal_runaway", defect_start_frac=0.6)
    result = sc.analyze_power("power_0", pt["t"], pt["voltage"], pt["current"], pt["temperature"], voltage_min_v=spec.voltage_min_v)
    assert result["thermal_anomaly_count"] > 10


def test_spectral_energy_drift_clean_sine_has_no_anomaly_windows():
    t = np.arange(N) * DT
    rng = np.random.default_rng(0)
    clean = np.sin(2 * np.pi * 0.3 * t) + 0.05 * rng.normal(size=N)
    result = sc.spectral_energy_drift(clean, DT, window=128)
    assert len(result["anomaly_window_idx"]) == 0


def test_spectral_energy_drift_detects_injected_high_frequency_burst():
    t = np.arange(N) * DT
    rng = np.random.default_rng(0)
    clean = np.sin(2 * np.pi * 0.3 * t) + 0.05 * rng.normal(size=N)
    noisy = clean.copy()
    noisy[1400:1600] += 0.8 * np.sin(2 * np.pi * 40 * t[1400:1600])
    result = sc.spectral_energy_drift(noisy, DT, window=128)
    assert len(result["anomaly_window_idx"]) > 0
    flagged_centers = result["window_centers_idx"][result["anomaly_window_idx"]]
    assert np.all((flagged_centers > 1300) & (flagged_centers < 1700))
