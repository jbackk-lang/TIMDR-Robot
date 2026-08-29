import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from timdr_robot import subsystems as ss

N, DT = 2000, 0.01


def test_gripper_clean_stays_near_zero_or_hold_level():
    g = ss.generate_gripper_trajectory(ss.GripperSpec(), N, DT, seed=1, defect_type=None)
    assert g["grip_force"].min() > -2.0
    assert g["grip_force"].max() < ss.GripperSpec().hold_force_n + 2.0


def test_gripper_slip_event_dips_below_clean_signal():
    clean = ss.generate_gripper_trajectory(ss.GripperSpec(), N, DT, seed=1, defect_type=None)
    slip = ss.generate_gripper_trajectory(ss.GripperSpec(), N, DT, seed=1, defect_type="slip_event", defect_start_frac=0.6)
    onset = int(N * 0.6)
    diff_before = np.max(np.abs(slip["grip_force"][:onset] - clean["grip_force"][:onset]))
    diff_after = np.max(np.abs(slip["grip_force"][onset:] - clean["grip_force"][onset:]))
    # przed onsetem sygnaly musza byc DOKLADNIE identyczne (ta sama
    # realizacja szumu) - regresja na empirycznie znaleziony blad: wybor
    # losowych centrow impulsow poslizgu przesuwal stan glownego RNG, wiec
    # "czysta" i "uszkodzona" trajektoria mialy rozne szumy nawet przed
    # wystapieniem wady (patrz sensor_bus/subsystems.py, poprawka RNG)
    assert diff_before == 0.0
    assert diff_after > 3.0


def test_mobile_base_clean_residual_is_small():
    spec = ss.MobileBaseSpec()
    mb = ss.generate_mobile_base_trajectory(spec, N, DT, seed=1, defect_type=None)
    resid = mb["heading_rate_gyro"] - (mb["v_right"] - mb["v_left"]) / spec.wheel_base_m
    assert np.max(np.abs(resid)) < 0.1


def test_mobile_base_wheel_slip_grows_residual_after_onset():
    spec = ss.MobileBaseSpec()
    mbd = ss.generate_mobile_base_trajectory(spec, N, DT, seed=1, defect_type="wheel_slip", defect_start_frac=0.6, defect_severity=1.0)
    resid = mbd["heading_rate_gyro"] - (mbd["v_right"] - mbd["v_left"]) / spec.wheel_base_m
    onset = int(N * 0.6)
    assert np.max(np.abs(resid[:onset])) < 0.1
    assert np.max(np.abs(resid[onset:])) > 0.2


def test_vision_clean_error_is_near_zero_noise():
    c = ss.generate_vision_trajectory(ss.CameraSpec(), N, DT, seed=1, defect_type=None)
    assert c["tracking_error_px"].std() < 3.0


def test_vision_tracking_loss_spikes():
    cd = ss.generate_vision_trajectory(ss.CameraSpec(), N, DT, seed=1, defect_type="tracking_loss", defect_start_frac=0.6)
    assert np.max(np.abs(cd["tracking_error_px"])) > 15.0


def test_power_clean_voltage_stays_above_absolute_limit():
    spec = ss.PowerSpec()
    p = ss.generate_power_trajectory(spec, N, DT, seed=1, defect_type=None)
    assert p["voltage"].min() > spec.voltage_min_v


def test_power_voltage_sag_can_cross_absolute_limit():
    spec = ss.PowerSpec()
    pv = ss.generate_power_trajectory(spec, N, DT, seed=1, defect_type="voltage_sag", defect_start_frac=0.6, defect_severity=1.0)
    assert pv["voltage"].min() < spec.voltage_min_v


def test_power_thermal_runaway_exceeds_clean_max_temperature():
    spec = ss.PowerSpec()
    p = ss.generate_power_trajectory(spec, N, DT, seed=1, defect_type=None)
    pt = ss.generate_power_trajectory(spec, N, DT, seed=1, defect_type="thermal_runaway", defect_start_frac=0.6)
    assert pt["temperature"].max() > p["temperature"].max() + 20.0


def test_unknown_defect_types_are_ignored_not_raising():
    ss.generate_gripper_trajectory(ss.GripperSpec(), 200, DT, seed=0, defect_type="bogus")
    ss.generate_mobile_base_trajectory(ss.MobileBaseSpec(), 200, DT, seed=0, defect_type="bogus")
    ss.generate_vision_trajectory(ss.CameraSpec(), 200, DT, seed=0, defect_type="bogus")
    ss.generate_power_trajectory(ss.PowerSpec(), 200, DT, seed=0, defect_type="bogus")
