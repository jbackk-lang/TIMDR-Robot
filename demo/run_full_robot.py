"""demo/run_full_robot.py — scenariusz rozszerzony: ramie 6-osiowe (jak w
demo/run_demo.py) PLUS cztery nowe podsystemy (chwytak, podstawa mobilna,
kamera, zasilanie), kazdy z wlasnym wstrzykniętym zdarzeniem.
================================================================================
Ten skrypt NIE zastepuje demo/run_demo.py (ktory zostaje - jest juz
przetestowany i uzywany przez istniejace demo) - to jest ROZSZERZONY
scenariusz pokazujacy pelna szerokosc warstwy TIMDR Robot po dodaniu
nowych podsystemow. Wszystkie dane SYNTETYCZNE.

Wstrzykniete zdarzenia (jedno na podsystem, zeby kazdy detektor mial cos
do wykrycia w demo):
- os 3 ramienia: backlash (jak w run_demo.py)
- chwytak: slip_event (poslizg trzymanego przedmiotu)
- podstawa mobilna: wheel_slip (utrata przyczepnosci prawego kola)
- kamera: tracking_loss (przejsciowa utrata sledzenia celu)
- zasilanie: thermal_runaway (przyspieszajacy wzrost temperatury)
"""
from __future__ import annotations

import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from timdr_robot import core, hmi, subsystem_core as sc
from timdr_robot.control_bridge import ControlBridge
from timdr_robot.sensor_bus import build_arm_scenario
from timdr_robot.status import compute_axis_status, compute_component_status, compute_power_status
from timdr_robot import subsystems as ss

DATA_DIR = Path(__file__).resolve().parent.parent / "data"
N, DT = 2000, 0.01


def main() -> None:
    DATA_DIR.mkdir(exist_ok=True)
    bridge = ControlBridge()
    events = {}
    printed = []
    bridge.subscribe_events(lambda e: printed.append(f"[{e.level.value}] {e.message}"))

    # --- ramie (jak w run_demo.py) -----------------------------------
    bus = build_arm_scenario(
        n_axes=6, n_samples=N, dt=DT, seed=42,
        defect_axis_index=3, defect_type="backlash",
        defect_start_frac=0.6, defect_severity=1.0,
    )
    for axis_id in bus.axis_ids():
        traj = bus.read_axis(axis_id)
        metrics = core.analyze_axis(axis_id, traj["t"], traj["position"], traj["velocity"], traj["accel"], traj.get("force"))
        event = compute_axis_status(metrics)
        bridge.publish(event)
        events[axis_id] = event

    # --- chwytak ------------------------------------------------------
    g = ss.generate_gripper_trajectory(ss.GripperSpec(), N, DT, seed=100, defect_type="slip_event", defect_start_frac=0.6)
    gm = sc.analyze_gripper("gripper_0", g["t"], g["grip_force"])
    ge = compute_component_status("gripper_0", gm["anomaly_count"], metrics=gm, anomaly_threshold=5, component_label="Chwytak")
    bridge.publish(ge)
    events["gripper_0"] = ge

    # --- podstawa mobilna ----------------------------------------------
    mb_spec = ss.MobileBaseSpec()
    mb = ss.generate_mobile_base_trajectory(mb_spec, N, DT, seed=101, defect_type="wheel_slip", defect_start_frac=0.6)
    mbm = sc.analyze_mobile_base("mobile_base_0", mb["t"], mb["v_left"], mb["v_right"], mb["heading_rate_gyro"], mb_spec.wheel_base_m)
    mbe = compute_component_status("mobile_base_0", mbm["anomaly_count"], metrics=mbm, anomaly_threshold=10, component_label="Podstawa mobilna")
    bridge.publish(mbe)
    events["mobile_base_0"] = mbe

    # --- kamera ---------------------------------------------------------
    cam = ss.generate_vision_trajectory(ss.CameraSpec(), N, DT, seed=102, defect_type="tracking_loss", defect_start_frac=0.6)
    cm = sc.analyze_vision("camera_0", cam["t"], cam["tracking_error_px"])
    ce = compute_component_status("camera_0", cm["anomaly_count"], metrics=cm, anomaly_threshold=5, component_label="Kamera")
    bridge.publish(ce)
    events["camera_0"] = ce

    # --- zasilanie -------------------------------------------------------
    p_spec = ss.PowerSpec()
    p = ss.generate_power_trajectory(p_spec, N, DT, seed=103, defect_type="thermal_runaway", defect_start_frac=0.6)
    pm = sc.analyze_power("power_0", p["t"], p["voltage"], p["current"], p["temperature"], voltage_min_v=p_spec.voltage_min_v)
    pe = compute_power_status(pm)
    bridge.publish(pe)
    events["power_0"] = pe

    print("=== TIMDR-Robot: pelny scenariusz (ramie + 4 podsystemy, dane SYNTETYCZNE) ===")
    for line in printed:
        print(line)
    print()
    for component_id, event in events.items():
        if component_id.startswith("axis_"):
            print(hmi.describe_axis_event(event))
        elif component_id == "power_0":
            print(hmi.describe_power_event(event))
        else:
            label = {"gripper_0": "Chwytak", "mobile_base_0": "Podstawa mobilna", "camera_0": "Kamera"}.get(component_id, "Komponent")
            print(hmi.describe_component_event(event, component_label=label))
    print()
    print(hmi.describe_scenario(events, label_plural="komponentow"))

    out = {
        cid: {"level": e.level.value, "message": e.message}
        for cid, e in events.items()
    }
    out_path = DATA_DIR / "full_robot_events.json"
    with open(out_path, "w", encoding="utf-8") as f:
        json.dump(out, f, ensure_ascii=False, indent=2)
    print()
    print(f"Zapisano: {out_path}")


if __name__ == "__main__":
    main()
