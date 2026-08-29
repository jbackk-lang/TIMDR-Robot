import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from timdr_robot.sensor_bus import AxisSpec, generate_axis_trajectory
from timdr_robot.status import AxisHealth
from timdr_robot.streaming import SlidingWindowAnalyzer

SPEC = AxisSpec(name="axis_test", amplitude_deg=30.0, cycle_s=3.0)
N, DT = 2000, 0.01


def test_no_recompute_before_min_samples():
    analyzer = SlidingWindowAnalyzer("a", window_size=600, recompute_every=100, min_samples=300, dt=DT)
    for i in range(299):
        recomputed = analyzer.push(i * DT, 0.0, 0.0, 0.0)
        assert recomputed is False
    assert analyzer.latest_metrics is None
    assert analyzer.latest_event is None


def test_recompute_triggers_after_min_samples_and_interval():
    analyzer = SlidingWindowAnalyzer("a", window_size=600, recompute_every=100, min_samples=300, dt=DT)
    traj = generate_axis_trajectory(SPEC, 350, DT, seed=1, defect_type=None)
    recompute_indices = []
    for i in range(350):
        if analyzer.push(traj["t"][i], traj["position"][i], traj["velocity"][i], traj["accel"][i]):
            recompute_indices.append(i)
    assert recompute_indices == [299, 399] or recompute_indices == [299]
    assert analyzer.latest_event is not None


def test_streaming_stays_ok_on_clean_axis():
    analyzer = SlidingWindowAnalyzer("a", window_size=600, recompute_every=100, min_samples=300, dt=DT)
    traj = generate_axis_trajectory(SPEC, N, DT, seed=1, defect_type=None)
    for i in range(N):
        analyzer.push(traj["t"][i], traj["position"][i], traj["velocity"][i], traj["accel"][i])
    assert analyzer.latest_event.level == AxisHealth.OK
    assert analyzer.n_recomputes > 0


def test_streaming_detects_backlash_shortly_after_onset():
    analyzer = SlidingWindowAnalyzer("a", window_size=600, recompute_every=100, min_samples=300, dt=DT)
    traj = generate_axis_trajectory(SPEC, N, DT, seed=1, defect_type="backlash", defect_start_frac=0.6, defect_severity=1.0)
    onset = int(N * 0.6)
    first_defect_idx = None
    for i in range(N):
        if analyzer.push(traj["t"][i], traj["position"][i], traj["velocity"][i], traj["accel"][i]):
            if analyzer.latest_event.level == AxisHealth.DEFECT and first_defect_idx is None:
                first_defect_idx = i
    assert first_defect_idx is not None
    assert first_defect_idx > onset
    # wykryte niedlugo po onsetcie (w obrebie jednego okna analizy), nie
    # tylko na samym koncu strumienia
    assert first_defect_idx < onset + 600
    assert analyzer.latest_event.level == AxisHealth.DEFECT


def test_buffer_len_respects_window_size():
    analyzer = SlidingWindowAnalyzer("a", window_size=50, recompute_every=10, min_samples=10, dt=DT)
    for i in range(200):
        analyzer.push(i * DT, 0.0, 0.0, 0.0)
    assert analyzer.buffer_len() == 50
