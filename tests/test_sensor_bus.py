import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from timdr_robot.sensor_bus import AxisSpec, SensorBus, build_arm_scenario, generate_axis_trajectory

SPEC = AxisSpec(name="axis_test", amplitude_deg=20.0, cycle_s=2.0)


def test_generate_axis_trajectory_shapes_and_keys():
    traj = generate_axis_trajectory(SPEC, 500, 0.01, seed=0)
    assert set(traj.keys()) == {"t", "position", "velocity", "accel", "force"}
    for key in traj:
        assert len(traj[key]) == 500


def test_clean_trajectory_stays_within_expected_amplitude_envelope():
    traj = generate_axis_trajectory(SPEC, 1000, 0.01, seed=0, defect_type=None)
    # amplituda + margines na szum pomiarowy (kilka odchylen standardowych)
    margin = 10 * SPEC.noise_std_pos
    assert np.max(np.abs(traj["position"])) <= SPEC.amplitude_deg + margin


def test_backlash_defect_only_appears_after_start_frac():
    clean = generate_axis_trajectory(SPEC, 2000, 0.01, seed=5, defect_type=None)
    defected = generate_axis_trajectory(
        SPEC, 2000, 0.01, seed=5, defect_type="backlash", defect_start_frac=0.6, defect_severity=1.0,
    )
    onset = int(2000 * 0.6)
    diff_before = np.abs(defected["position"][:onset] - clean["position"][:onset])
    diff_after = np.abs(defected["position"][onset:] - clean["position"][onset:])
    # przed wystapieniem wady sygnaly powinny byc niemal identyczne (to samo
    # ziarno szumu) - po wystapieniu wady, wyraznie sie roznia
    assert np.max(diff_before) < 0.05
    assert np.max(diff_after) > 0.5


def test_resonance_burst_adds_high_frequency_energy_to_accel():
    clean = generate_axis_trajectory(SPEC, 2000, 0.01, seed=5, defect_type=None)
    resonant = generate_axis_trajectory(
        SPEC, 2000, 0.01, seed=5, defect_type="resonance_burst", defect_start_frac=0.6, defect_severity=1.0,
    )
    onset = int(2000 * 0.6)
    diff_accel = np.abs(resonant["accel"][onset:onset + 200] - clean["accel"][onset:onset + 200])
    assert np.max(diff_accel) > 1.0


def test_unknown_defect_type_leaves_signal_unchanged():
    clean = generate_axis_trajectory(SPEC, 500, 0.01, seed=2, defect_type=None)
    unknown = generate_axis_trajectory(SPEC, 500, 0.01, seed=2, defect_type="not_a_real_defect")
    assert np.allclose(clean["position"], unknown["position"])


def test_sensor_bus_stores_and_retrieves_axes():
    bus = SensorBus()
    traj = generate_axis_trajectory(SPEC, 100, 0.01, seed=0)
    bus.add_axis("axis_0", traj)
    assert bus.axis_ids() == ["axis_0"]
    assert bus.read_axis("axis_0") is traj


def test_sensor_bus_unknown_axis_raises_key_error():
    bus = SensorBus()
    with pytest.raises(KeyError):
        bus.read_axis("does_not_exist")


def test_build_arm_scenario_injects_defect_only_on_target_axis():
    bus = build_arm_scenario(n_axes=6, n_samples=1000, defect_axis_index=3, defect_type="backlash")
    clean_axis = generate_axis_trajectory(
        AxisSpec(name="axis_1", amplitude_deg=35.0, cycle_s=3.3), 1000, 0.01, seed=43, defect_type=None,
    )
    # axis_1 (indeks != 3) nie powinien miec wady wstrzykniętej
    axis1 = bus.read_axis("axis_1")
    assert np.allclose(axis1["position"], clean_axis["position"])
    assert bus.axis_ids() == [f"axis_{i}" for i in range(6)]
