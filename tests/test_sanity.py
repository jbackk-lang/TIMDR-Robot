import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from timdr_robot.sanity import sanity_check_metrics, sanity_check_signal


def test_clean_signal_passes():
    x = np.sin(np.linspace(0, 10, 200))
    result = sanity_check_signal(x)
    assert result.ok is True
    assert result.reason is None


def test_empty_signal_fails():
    result = sanity_check_signal(np.array([]))
    assert result.ok is False
    assert "dlugosc" in result.reason


def test_nan_signal_fails():
    x = np.array([1.0, 2.0, np.nan, 4.0])
    result = sanity_check_signal(x)
    assert result.ok is False
    assert "NaN" in result.reason


def test_inf_signal_fails():
    x = np.array([1.0, 2.0, np.inf, 4.0])
    result = sanity_check_signal(x)
    assert result.ok is False
    assert "Inf" in result.reason


def test_negative_inf_also_fails():
    x = np.array([1.0, -np.inf, 4.0])
    result = sanity_check_signal(x)
    assert result.ok is False


def test_impossible_jump_fails_when_limit_given():
    x = np.array([1.0, 1.1, 1.2, 500.0, 1.3])
    result = sanity_check_signal(x, max_physical_jump=10.0)
    assert result.ok is False
    assert "skok" in result.reason


def test_no_jump_check_when_limit_not_given():
    x = np.array([1.0, 1.1, 1.2, 500.0, 1.3])
    result = sanity_check_signal(x, max_physical_jump=None)
    assert result.ok is True


def test_jump_within_limit_passes():
    x = np.array([1.0, 1.1, 1.2, 5.0, 5.1])
    result = sanity_check_signal(x, max_physical_jump=10.0)
    assert result.ok is True


def test_metrics_sanity_passes_on_clean_dict():
    metrics = {"torsion_max_abs": 1.5, "anomaly_count": 3}
    result = sanity_check_metrics(metrics, numeric_keys=["torsion_max_abs", "anomaly_count"])
    assert result.ok is True


def test_metrics_sanity_fails_on_nan_value():
    metrics = {"torsion_max_abs": float("nan"), "anomaly_count": 3}
    result = sanity_check_metrics(metrics, numeric_keys=["torsion_max_abs", "anomaly_count"])
    assert result.ok is False
    assert "NaN" in result.reason or "niefizyczny" in result.reason


def test_metrics_sanity_fails_on_negative_count():
    metrics = {"anomaly_count": -1}
    result = sanity_check_metrics(metrics, numeric_keys=["anomaly_count"])
    assert result.ok is False
    assert "ujemna" in result.reason


def test_metrics_sanity_ignores_missing_or_none_keys():
    metrics = {"torsion_max_abs": 1.0}
    result = sanity_check_metrics(metrics, numeric_keys=["torsion_max_abs", "missing_key", "none_key"])
    assert result.ok is True
