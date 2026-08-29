"""timdr_robot/streaming.py — most miedzy dzisiejsza wsadowa generacja
(`SensorBus` trzyma cala serie z gory) a prawdziwym strumieniowaniem w
czasie rzeczywistym (probka po probce z prawdziwego sprzetu).
================================================================================
`SlidingWindowAnalyzer` przyjmuje probki JEDNA PO DRUGIEJ (`push()`),
trzyma je w buforze o STALEJ dlugosci (`collections.deque(maxlen=...)` -
najstarsze probki same wypadaja), i re-analizuje CALY bufor pelnym
`core.analyze_axis()` co `recompute_every` nowych probek (NIE na kazda
pojedyncza probke - re-analiza Freneta-Serreta + modelu harmonicznego
na kazdej probce byloby zbyt kosztowne obliczeniowo dla strumienia o
wysokiej czestotliwosci probkowania).

**JAWNE OGRANICZENIE (nie ukryta wada):** `core.harmonic_law_residual()`
kalibruje `omega^2` na PIERWSZYCH `calib_frac` probkach bufora - w oknie
KROCZACYM te "pierwsze probki" to po prostu NAJSTARSZE zachowane probki,
ktore ciagle sie przesuwaja w przod w miare uplywu czasu (stare probki
wypadaja z bufora). Oznacza to, ze kalibracja NIE jest przywiazana do
jednego, stalego "zdrowego poczatku" tak jak w analizie wsadowej calej
serii - jesli wada byla obecna wystarczajaco dlugo, by zdominowac
"najstarszy" fragment aktualnego okna, `omega^2` moze sie do niej czesciowo
dostroic, zanizajac czulosc detekcji. To jest znany kompromis buforowania
o stalej dlugosci, nie blad implementacji - w pelnej wersji streamingowej
(POZA zakresem tego szkieletu) kalibracja powinna byc odswiezana rzadziej
i tylko wtedy, gdy okno jest juz jawnie potwierdzone jako zdrowe (np. po
resecie/przegladzie), nie automatycznie przy kazdym przesunieciu bufora.
"""
from __future__ import annotations

from collections import deque
from typing import Dict, Optional

import numpy as np

from . import core
from .status import StatusEvent, compute_axis_status


class SlidingWindowAnalyzer:
    """Przyjmuje probki (t, position, velocity, accel) jedna po drugiej,
    re-analizuje bufor co `recompute_every` probek. `latest_metrics`/
    `latest_event` sa `None` dopoki bufor nie osiagnie `min_samples`."""

    def __init__(
        self,
        axis_id: str,
        window_size: int = 1000,
        recompute_every: int = 100,
        min_samples: int = 100,
        dt: float = 0.01,
        analyze_kwargs: Optional[Dict] = None,
        status_kwargs: Optional[Dict] = None,
    ):
        self.axis_id = axis_id
        self.window_size = window_size
        self.recompute_every = recompute_every
        self.min_samples = min(min_samples, window_size)
        self.dt = dt
        self.analyze_kwargs = analyze_kwargs or {}
        self.status_kwargs = status_kwargs or {}

        self._t = deque(maxlen=window_size)
        self._position = deque(maxlen=window_size)
        self._velocity = deque(maxlen=window_size)
        self._accel = deque(maxlen=window_size)
        self._since_recompute = 0
        self.n_pushed = 0
        self.n_recomputes = 0
        self.latest_metrics: Optional[Dict] = None
        self.latest_event: Optional[StatusEvent] = None

    def push(self, t: float, position: float, velocity: float, accel: float) -> bool:
        """Dodaje jedna probke. Zwraca True jesli ta probka wywolala
        ponowna analize (czyli `latest_metrics`/`latest_event` zostaly
        zaktualizowane)."""
        self._t.append(t)
        self._position.append(position)
        self._velocity.append(velocity)
        self._accel.append(accel)
        self._since_recompute += 1
        self.n_pushed += 1

        if len(self._t) >= self.min_samples and self._since_recompute >= self.recompute_every:
            self._recompute()
            self._since_recompute = 0
            return True
        return False

    def _recompute(self) -> None:
        t = np.array(self._t, dtype=float)
        position = np.array(self._position, dtype=float)
        velocity = np.array(self._velocity, dtype=float)
        accel = np.array(self._accel, dtype=float)
        self.latest_metrics = core.analyze_axis(
            self.axis_id, t, position, velocity, accel, dt=self.dt, **self.analyze_kwargs,
        )
        self.latest_event = compute_axis_status(self.latest_metrics, **self.status_kwargs)
        self.n_recomputes += 1

    def buffer_len(self) -> int:
        return len(self._t)
