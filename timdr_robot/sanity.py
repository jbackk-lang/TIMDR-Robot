"""timdr_robot/sanity.py — bramki NC1 (wejscie) i NC2 (wyjscie), zgodnie
z 8-krokowym schematem zaproponowanym przez uzytkownika: krok 1
(Sanity/Negative Control na sygnale wejsciowym) i krok 6 (Negative
Control na policzonych wynikach P/T/R, PRZED klasyfikacja).
================================================================================
**Dlaczego osobny plik, nie wrzucone do core.py/subsystem_core.py:** te
same dwie bramki sa uzywane przez WSZYSTKIE analyze_*() (os ramienia i
cztery podsystemy) - jeden wspolny, przetestowany raz kod zamiast
osobnej, potencjalnie rozjezdzajacej sie kopii w kazdym miejscu (patrz
timdr-signal-framework skill sec.10 o duplication-drift - dokladnie tego
unikamy).

NC1 (`sanity_check_signal`): sygnal jest ODRZUCANY (nie analizowany
dalej) jesli zawiera NaN/Inf, jest pusty, lub (opcjonalnie, jesli
wywolujacy poda `max_physical_jump`) ma skok miedzy kolejnymi probkami
wiekszy niz fizycznie mozliwy dla TEGO konkretnego czujnika. `max_
physical_jump` jest CELOWO opcjonalny i podawany przez wywolujacego, nie
wyliczany automatycznie z danych - to jest twardy limit fizyczny
("sensowny Kmax" wg uzytkownika), rozny dla kazdego typu czujnika, a nie
kolejny statystyczny prog (te juz sa w core.anomalies()/defect()).

NC2 (`sanity_check_metrics`): PO policzeniu torsji/reszty modelu/
rezonansu, ale PRZED klasyfikacja - sprawdza, czy same WYNIKI analizy sa
skonczone i sensowne (np. torsion_max_abs nie jest NaN/Inf, liczniki
anomalii sa nieujemne). To lapie sytuacje, w ktorych sam sygnal wejsciowy
przeszedl NC1, ale jakis krok posredni (dzielenie, log, pierwiastek)
wyprodukowal cos niefizycznego - w praktyce bardzo rzadkie przy
poprawnym kodzie, ale to WLASNIE dlatego jest to tania bramka, nie
kosztowna: gdy nigdy sie nie uruchamia, nic nie kosztuje; gdy sie
uruchomi, ratuje przed przekazaniem smieci do klasyfikacji/decyzji.

Zaden z tych mechanizmow nie byl obecny w pierwszej wersji tego repo -
dodany na wprost postawione pytanie uzytkownika o formalny protokol
Sanity/NC. Wczesniej jedyna forma "kontroli poczytalnosci" w tym repo to
`core.negative_control_check()` - ktora jest czyms INNYM (statystyczny
test odsetka falszywych alarmow na WIELU przebiegach syntetycznych, nie
bramka per-wywolanie na KONKRETNYM sygnale).
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Optional, Sequence

import numpy as np


@dataclass
class SanityResult:
    ok: bool
    reason: Optional[str] = None


def sanity_check_signal(
    x: Sequence[float],
    name: str = "signal",
    max_physical_jump: Optional[float] = None,
    min_length: int = 1,
) -> SanityResult:
    """NC1: sprawdza, czy `x` nalezy do klasy sygnalow dopuszczalnych
    PRZED jakakolwiek dalsza analiza.

    Warunki (w kolejnosci sprawdzania):
    1. dlugosc >= `min_length`
    2. brak NaN
    3. brak +-Inf
    4. (opcjonalnie) |x[i+1] - x[i]| <= max_physical_jump dla wszystkich i
    """
    arr = np.asarray(x, dtype=float)

    if len(arr) < min_length:
        return SanityResult(False, f"{name}: dlugosc {len(arr)} < wymagane minimum {min_length}")

    if np.isnan(arr).any():
        n_nan = int(np.isnan(arr).sum())
        return SanityResult(False, f"{name}: zawiera {n_nan} wartosci NaN")

    if np.isinf(arr).any():
        n_inf = int(np.isinf(arr).sum())
        return SanityResult(False, f"{name}: zawiera {n_inf} wartosci +-Inf")

    if max_physical_jump is not None and len(arr) >= 2:
        diffs = np.abs(np.diff(arr))
        bad = np.where(diffs > max_physical_jump)[0]
        if len(bad):
            idx = int(bad[0])
            return SanityResult(
                False,
                f"{name}: skok |x[{idx+1}]-x[{idx}]|={diffs[idx]:.4g} "
                f"przekracza max_physical_jump={max_physical_jump:.4g} "
                f"({len(bad)} takich skokow lacznie) - fizycznie niemozliwe "
                f"dla tego czujnika",
            )

    return SanityResult(True, None)


def sanity_check_metrics(metrics: dict, numeric_keys: Sequence[str]) -> SanityResult:
    """NC2: sprawdza, czy wybrane POLICZONE metryki (`numeric_keys` z
    `metrics`) sa skonczone i sensowne, PRZED klasyfikacja statusu.
    Liczniki anomalii (klucze konczace sie na `_count`) musza dodatkowo
    byc nieujemnymi liczbami calkowitymi."""
    for key in numeric_keys:
        if key not in metrics or metrics[key] is None:
            continue
        value = metrics[key]
        try:
            fval = float(value)
        except (TypeError, ValueError):
            return SanityResult(False, f"metryka {key!r}={value!r} nie jest liczba")
        if np.isnan(fval) or np.isinf(fval):
            return SanityResult(False, f"metryka {key!r}={fval} jest NaN/Inf - wynik niefizyczny")
        if key.endswith("_count") and fval < 0:
            return SanityResult(False, f"metryka {key!r}={fval} jest ujemna - licznik anomalii nie moze byc ujemny")
    return SanityResult(True, None)
