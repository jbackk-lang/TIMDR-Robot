"""api.py — lokalny dashboard (FastAPI) dla TIMDR-Robot.

Uruchamia scenariusz demonstracyjny (6-osiowe ramie, os 3 z wstrzykniętym
luzem) PRZY STARCIE, trzyma wynik w pamieci i serwuje go pod /api/*. Zeby
przeliczyc na nowo (np. z innym ziarnem/inna wada), trzeba zrestartowac
serwer - to jest szkielet demonstracyjny na danych SYNTETYCZNYCH, nie
serwis produkcyjny podlaczony do zywego robota (patrz README).

Chart.js jest wczesniej ustalona lekcja z tego samego zestawu repo
(fusion-tools, SYNOPTYK-ARCTIC): CDN bywa niedostepny na sieciach
firmowych z restrykcjami (Device Guard i podobne), wiec od razu
wektorujemy Chart.js LOKALNIE pod /static/vendor/, bez zadnego
zewnetrznego hosta w index.html.

Endpointy:
- GET /                  — dashboard (statyczny HTML + JS)
- GET /api/scenario      — pelne wyniki scenariusza demo (status per os,
                            opisy, log reakcji, negative control)
- GET /api/axis/{axis_id} — surowa trajektoria + metryki jednej osi (do
                            wykresow czasowych na dashboardzie)
"""
from __future__ import annotations

from functools import partial
from pathlib import Path
from typing import Dict

from fastapi import FastAPI, HTTPException
from fastapi.responses import HTMLResponse
from fastapi.staticfiles import StaticFiles

from timdr_robot import core, hmi
from timdr_robot.control_bridge import ControlBridge
from timdr_robot.sensor_bus import AxisSpec, build_arm_scenario, generate_axis_trajectory
from timdr_robot.status import compute_axis_status

BASE_DIR = Path(__file__).resolve().parent
STATIC_DIR = BASE_DIR / "static"

app = FastAPI(title="TIMDR-Robot Dashboard")
app.mount("/static", StaticFiles(directory=str(STATIC_DIR)), name="static")


def _run_scenario() -> Dict:
    bus = build_arm_scenario(
        n_axes=6, n_samples=2000, dt=0.01, seed=42,
        defect_axis_index=3, defect_type="backlash",
        defect_start_frac=0.6, defect_severity=1.0,
    )
    bridge = ControlBridge()
    events = {}
    metrics_by_axis = {}
    trajectories = {}
    for axis_id in bus.axis_ids():
        traj = bus.read_axis(axis_id)
        metrics = core.analyze_axis(
            axis_id, traj["t"], traj["position"], traj["velocity"],
            traj["accel"], traj.get("force"),
        )
        event = compute_axis_status(metrics)
        bridge.publish(event)
        events[axis_id] = event
        metrics_by_axis[axis_id] = metrics
        trajectories[axis_id] = traj

    clean_spec = AxisSpec(name="negative_control_axis")
    negative_control = core.negative_control_check(
        partial(generate_axis_trajectory, clean_spec, 2000, 0.01, defect_type=None),
        n_trials=20,
    )

    return {
        "events": events,
        "metrics": metrics_by_axis,
        "trajectories": trajectories,
        "reaction_log": bridge.reaction_log,
        "negative_control": negative_control,
        "summary": hmi.describe_scenario(events),
    }


_SCENARIO = _run_scenario()


@app.get("/api/scenario")
def api_scenario() -> dict:
    axes_out = {}
    for axis_id, event in _SCENARIO["events"].items():
        metrics = _SCENARIO["metrics"][axis_id]
        axes_out[axis_id] = {
            "level": event.level.value,
            "message": event.message,
            "description": hmi.describe_axis_event(event),
            "harmonic_anomaly_count": metrics["harmonic_anomaly_count"],
            "torsion_spike_count": metrics["torsion_spike_count"],
            "torsion_max_abs": metrics["torsion_max_abs"],
            "omega_sq": metrics["omega_sq"],
            "ringdown": metrics["ringdown"],
        }
    return {
        "axes": axes_out,
        "summary": _SCENARIO["summary"],
        "reaction_log": [
            {"axis_id": e.axis_id, "action": e.action, "detail": e.detail}
            for e in _SCENARIO["reaction_log"]
        ],
        "negative_control": _SCENARIO["negative_control"],
        "disclaimer": (
            "Wszystkie dane sa SYNTETYCZNE (wygenerowane przez "
            "timdr_robot/sensor_bus.py) - to jest szkielet demonstracyjny, "
            "NIE polaczenie z prawdziwym robotem. Patrz README, sekcja "
            "'Status walidacji'."
        ),
    }


@app.get("/api/axis/{axis_id}")
def api_axis(axis_id: str) -> dict:
    if axis_id not in _SCENARIO["trajectories"]:
        raise HTTPException(404, f"Nieznana os: {axis_id!r}. Dostepne: {list(_SCENARIO['trajectories'])}")
    traj = _SCENARIO["trajectories"][axis_id]
    metrics = _SCENARIO["metrics"][axis_id]
    # decymacja do wykresu (co N-ta probka), zeby nie wysylac 2000 punktow
    # x 6 osi naraz do przegladarki bez potrzeby - dashboard rysuje trend,
    # nie potrzebuje pelnej rozdzielczosci probkowania
    step = 4
    return {
        "axis_id": axis_id,
        "t": traj["t"][::step].tolist(),
        "position": traj["position"][::step].tolist(),
        "velocity": traj["velocity"][::step].tolist(),
        "accel": traj["accel"][::step].tolist(),
        "torsion_series": metrics["torsion_series"][::step],
        "harmonic_anomaly_idx": metrics["harmonic_anomaly_idx"],
        "torsion_spike_idx": metrics["torsion_spike_idx"],
    }


@app.get("/", response_class=HTMLResponse)
def index() -> str:
    html_path = STATIC_DIR / "index.html"
    return html_path.read_text(encoding="utf-8")
