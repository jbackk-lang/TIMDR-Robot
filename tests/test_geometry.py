import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from timdr_robot.geometry import frenet_serret_curvature_torsion, phase_portrait


def test_helix_has_constant_curvature_and_torsion():
    """Sanity check klasyczny dla wzorow Freneta-Serreta: helisa kolowa ma
    STALA krzywizne i torsje wzdluz calej dlugosci - to samo zrodlo prawdy
    co przy oryginalnej walidacji tej matematyki we FLIGHT-TRACKING-TIMDR."""
    t = np.linspace(0, 4 * np.pi, 500)
    dt = t[1] - t[0]
    r = 2.0
    c = 1.0
    pos = np.column_stack([r * np.cos(t), r * np.sin(t), c * t])

    kappa, torsion = frenet_serret_curvature_torsion(pos, dt=dt)

    expected_kappa = r / (r ** 2 + c ** 2)
    expected_torsion = c / (r ** 2 + c ** 2)

    # pomijamy brzegi (artefakty filtra Savitzky-Golay na koncach serii)
    mid = slice(20, -20)
    assert np.allclose(kappa[mid], expected_kappa, rtol=0.05)
    assert np.allclose(torsion[mid], expected_torsion, rtol=0.05)


def test_planar_curve_has_zero_torsion():
    """Krzywa plaska (np. okrag w plaszczyznie XY) ma torsje ~0 wszedzie -
    torsja mierzy WYLACZNIE wyjscie poza plaszczyzne oscylujaca."""
    t = np.linspace(0, 4 * np.pi, 300)
    dt = t[1] - t[0]
    pos = np.column_stack([np.cos(t), np.sin(t), np.zeros_like(t)])

    _, torsion = frenet_serret_curvature_torsion(pos, dt=dt)
    mid = slice(20, -20)
    assert np.max(np.abs(torsion[mid])) < 1e-6


def test_short_series_returns_zeros_not_exception():
    pos = np.random.default_rng(0).normal(size=(3, 3))
    kappa, torsion = frenet_serret_curvature_torsion(pos, dt=0.01)
    assert len(kappa) == 3
    assert np.all(kappa == 0)
    assert np.all(torsion == 0)


def test_frenet_serret_rejects_wrong_shape():
    import pytest
    with pytest.raises(ValueError):
        frenet_serret_curvature_torsion(np.zeros((10, 2)), dt=0.01)


def test_phase_portrait_stacks_columns():
    pos = np.array([1.0, 2.0, 3.0])
    vel = np.array([0.1, 0.2, 0.3])
    acc = np.array([0.01, 0.02, 0.03])
    portrait = phase_portrait(pos, vel, acc)
    assert portrait.shape == (3, 3)
    assert np.allclose(portrait[:, 0], pos)
    assert np.allclose(portrait[:, 1], vel)
    assert np.allclose(portrait[:, 2], acc)


def test_phase_portrait_rejects_mismatched_lengths():
    import pytest
    with pytest.raises(ValueError):
        phase_portrait(np.zeros(5), np.zeros(4), np.zeros(5))
