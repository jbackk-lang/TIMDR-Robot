import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from timdr_robot.status import AxisHealth, compute_axis_status


def _base_metrics(**overrides):
    metrics = {
        "axis_id": "axis_0",
        "harmonic_anomaly_count": 0,
        "torsion_spike_count": 0,
        "ringdown_event_idx": None,
        "ringdown": None,
    }
    metrics.update(overrides)
    return metrics


def test_ok_when_nothing_detected():
    event = compute_axis_status(_base_metrics())
    assert event.level == AxisHealth.OK


def test_suspect_when_only_torsion_spikes():
    event = compute_axis_status(_base_metrics(torsion_spike_count=1))
    assert event.level == AxisHealth.SUSPECT


def test_resonance_when_ringdown_oscillatory():
    event = compute_axis_status(_base_metrics(
        torsion_spike_count=1,
        ringdown={"is_oscillatory": True, "frequency_hz": 12.5},
    ))
    assert event.level == AxisHealth.RESONANCE
    assert "12.5" in event.message or "12.50" in event.message


def test_defect_when_harmonic_anomalies_above_threshold():
    event = compute_axis_status(_base_metrics(
        harmonic_anomaly_count=5,
        ringdown={"is_oscillatory": True, "frequency_hz": 1.0},
    ))
    # DEFECT ma priorytet nad RESONANCE, nawet jesli oba warunki zachodza
    assert event.level == AxisHealth.DEFECT


def test_defect_threshold_is_configurable():
    metrics = _base_metrics(harmonic_anomaly_count=2)
    assert compute_axis_status(metrics, harmonic_anomaly_threshold=3).level != AxisHealth.DEFECT
    assert compute_axis_status(metrics, harmonic_anomaly_threshold=2).level == AxisHealth.DEFECT
