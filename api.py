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

import sys as _sys
from pathlib import Path as _Path
_sys.path.insert(0, str(_Path(__file__).resolve().parent))

from demo.fleet_demo import build_demo_fleet
from timdr_robot import core, hmi, subsystem_core as sc
from timdr_robot import subsystems as ss
from timdr_robot.control_bridge import ControlBridge
from timdr_robot.sensor_bus import AxisSpec, build_arm_scenario, generate_axis_trajectory
from timdr_robot.status import compute_axis_status, compute_component_status, compute_power_status

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

    # --- nowe podsystemy (chwytak, podstawa mobilna, kamera, zasilanie) ---
    N, DT = 2000, 0.01
    subsystem_events = {}
    subsystem_metrics = {}
    subsystem_raw = {}

    g = ss.generate_gripper_trajectory(ss.GripperSpec(), N, DT, seed=100, defect_type="slip_event", defect_start_frac=0.6)
    gm = sc.analyze_gripper("gripper_0", g["t"], g["grip_force"])
    ge = compute_component_status("gripper_0", gm["anomaly_count"], metrics=gm, anomaly_threshold=5, component_label="Chwytak")
    bridge.publish(ge)
    subsystem_events["gripper_0"] = ge
    subsystem_metrics["gripper_0"] = gm
    subsystem_raw["gripper_0"] = {"t": g["t"], "series": {"grip_force": g["grip_force"]}}

    mb_spec = ss.MobileBaseSpec()
    mb = ss.generate_mobile_base_trajectory(mb_spec, N, DT, seed=101, defect_type="wheel_slip", defect_start_frac=0.6)
    mbm = sc.analyze_mobile_base("mobile_base_0", mb["t"], mb["v_left"], mb["v_right"], mb["heading_rate_gyro"], mb_spec.wheel_base_m)
    mbe = compute_component_status("mobile_base_0", mbm["anomaly_count"], metrics=mbm, anomaly_threshold=10, component_label="Podstawa mobilna")
    bridge.publish(mbe)
    subsystem_events["mobile_base_0"] = mbe
    subsystem_metrics["mobile_base_0"] = mbm
    subsystem_raw["mobile_base_0"] = {"t": mb["t"], "series": {"v_left": mb["v_left"], "v_right": mb["v_right"], "heading_rate_gyro": mb["heading_rate_gyro"]}}

    cam = ss.generate_vision_trajectory(ss.CameraSpec(), N, DT, seed=102, defect_type="tracking_loss", defect_start_frac=0.6)
    cm = sc.analyze_vision("camera_0", cam["t"], cam["tracking_error_px"])
    ce = compute_component_status("camera_0", cm["anomaly_count"], metrics=cm, anomaly_threshold=5, component_label="Kamera")
    bridge.publish(ce)
    subsystem_events["camera_0"] = ce
    subsystem_metrics["camera_0"] = cm
    subsystem_raw["camera_0"] = {"t": cam["t"], "series": {"tracking_error_px": cam["tracking_error_px"]}}

    p_spec = ss.PowerSpec()
    p = ss.generate_power_trajectory(p_spec, N, DT, seed=103, defect_type="thermal_runaway", defect_start_frac=0.6)
    pm = sc.analyze_power("power_0", p["t"], p["voltage"], p["current"], p["temperature"], voltage_min_v=p_spec.voltage_min_v)
    pe = compute_power_status(pm)
    bridge.publish(pe)
    subsystem_events["power_0"] = pe
    subsystem_metrics["power_0"] = pm
    subsystem_raw["power_0"] = {"t": p["t"], "series": {"voltage": p["voltage"], "current": p["current"], "temperature": p["temperature"]}}

    all_events = {**events, **subsystem_events}

    return {
        "events": events,
        "metrics": metrics_by_axis,
        "trajectories": trajectories,
        "reaction_log": bridge.reaction_log,
        "negative_control": negative_control,
        "summary": hmi.describe_scenario(all_events, label_plural="komponentow"),
        "subsystem_events": subsystem_events,
        "subsystem_metrics": subsystem_metrics,
        "subsystem_raw": subsystem_raw,
    }


_SCENARIO = _run_scenario()
_FLEET = build_demo_fleet()


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


@app.get("/api/subsystems")
def api_subsystems() -> dict:
    out = {}
    for component_id, event in _SCENARIO["subsystem_events"].items():
        if component_id == "power_0":
            description = hmi.describe_power_event(event)
        else:
            label = {"gripper_0": "Chwytak", "mobile_base_0": "Podstawa mobilna", "camera_0": "Kamera"}.get(component_id, "Komponent")
            description = hmi.describe_component_event(event, component_label=label)
        out[component_id] = {
            "level": event.level.value,
            "message": event.message,
            "description": description,
            "metrics": _SCENARIO["subsystem_metrics"][component_id],
        }
    return {
        "subsystems": out,
        "disclaimer": (
            "Wszystkie dane sa SYNTETYCZNE (timdr_robot/subsystems.py) - "
            "szkielet demonstracyjny, NIE polaczenie z prawdziwym sprzetem."
        ),
    }


@app.get("/api/component/{component_id}")
def api_component(component_id: str) -> dict:
    if component_id not in _SCENARIO["subsystem_raw"]:
        raise HTTPException(404, f"Nieznany komponent: {component_id!r}. Dostepne: {list(_SCENARIO['subsystem_raw'])}")
    raw = _SCENARIO["subsystem_raw"][component_id]
    step = 4
    return {
        "component_id": component_id,
        "t": raw["t"][::step].tolist(),
        "series": {name: values[::step].tolist() for name, values in raw["series"].items()},
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


@app.get("/api/fleet")
def api_fleet() -> dict:
    units_out = {}
    for unit_id in _FLEET.unit_ids():
        unit = _FLEET.get_unit(unit_id)
        units_out[unit_id] = {
            "level": unit.worst_level().value,
            "defect_components": unit.defect_components(),
            "components": {cid: e.level.value for cid, e in unit.events.items()},
        }
    return {
        "units": units_out,
        "fleet_level": _FLEET.worst_fleet_level().value,
        "summary": hmi.describe_fleet(_FLEET),
        "disclaimer": (
            "Wszystkie dane sa SYNTETYCZNE (demo/fleet_demo.py) - "
            "szkielet demonstracyjny, NIE polaczenie z prawdziwa flota."
        ),
    }


@app.get("/", response_class=HTMLResponse)
def index() -> str:
    html_path = STATIC_DIR / "index.html"
    return html_path.read_text(encoding="utf-8")
