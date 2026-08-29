import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from timdr_robot.ringdown import ringdown_resonance


def _damped_oscillator(t, freq_hz, damping, amplitude=1.0):
    omega = 2 * np.pi * freq_hz
    return amplitude * np.exp(-damping * t) * np.sin(omega * t)


def test_detects_oscillatory_return_and_recovers_frequency():
    dt = 0.001
    t = np.arange(0, 2.0, dt)
    baseline = 5.0
    s = np.full_like(t, baseline)
    event_idx = 200
    tau = t[event_idx:] - t[event_idx]
    s[event_idx:] += _damped_oscillator(tau, freq_hz=10.0, damping=3.0, amplitude=2.0)

    result = ringdown_resonance(t, s, event_idx=event_idx, pre_event_window=50)
    assert result["is_oscillatory"] is True
    assert result["frequency_hz"] == pytest.approx(10.0, rel=0.15)


def test_monotonic_return_is_not_oscillatory():
    dt = 0.001
    t = np.arange(0, 2.0, dt)
    baseline = 5.0
    s = np.full_like(t, baseline)
    event_idx = 200
    tau = t[event_idx:] - t[event_idx]
    s[event_idx:] += 3.0 * np.exp(-4.0 * tau)  # czysto wykladniczy powrot, bez oscylacji

    result = ringdown_resonance(t, s, event_idx=event_idx, pre_event_window=50)
    assert result["is_oscillatory"] is False


def test_event_idx_out_of_range_raises():
    t = np.linspace(0, 1, 10)
    s = np.zeros(10)
    with pytest.raises(ValueError):
        ringdown_resonance(t, s, event_idx=99)
