"""demo/fleet_demo.py — flota kilku robotow naraz (agregacja ponad
pojedyncza maszyna, `timdr_robot/fleet.py`).
================================================================================
Symuluje 4 roboty (te same generatory co demo/run_demo.py i
demo/run_full_robot.py, rozne ziarna/rozne wstrzykniete wady):
- robot_alpha: zdrowy (wszystkie osie/podsystemy czyste)
- robot_beta: os 3 ramienia z backlash (jak w run_demo.py)
- robot_gamma: chwytak ze slip_event
- robot_delta: zasilanie z thermal_runaway

Wszystkie dane SYNTETYCZNE.
"""
from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from timdr_robot import core, hmi, subsystem_core as sc
from timdr_robot import subsystems as ss
from timdr_robot.fleet import Fleet, RobotUnit
from timdr_robot.sensor_bus import build_arm_scenario
from timdr_robot.status import compute_axis_status, compute_component_status, compute_power_status

N, DT = 1000, 0.01


def _arm_events(seed: int, defect_axis_index=None, defect_type=None) -> dict:
    bus = build_arm_scenario(
        n_axes=3, n_samples=N, dt=DT, seed=seed,
        defect_axis_index=defect_axis_index, defect_type=defect_type,
        defect_start_frac=0.6, defect_severity=1.0,
    )
    events = {}
    for axis_id in bus.axis_ids():
        traj = bus.read_axis(axis_id)
        metrics = core.analyze_axis(axis_id, traj["t"], traj["position"], traj["velocity"], traj["accel"], traj.get("force"))
        events[axis_id] = compute_axis_status(metrics)
    return events


def _gripper_event(seed: int, defect_type=None) -> dict:
    g = ss.generate_gripper_trajectory(ss.GripperSpec(), N, DT, seed=seed, defect_type=defect_type, defect_start_frac=0.6)
    gm = sc.analyze_gripper("gripper_0", g["t"], g["grip_force"])
    return {"gripper_0": compute_component_status("gripper_0", gm["anomaly_count"], metrics=gm, anomaly_threshold=5, component_label="Chwytak")}


def _power_event(seed: int, defect_type=None) -> dict:
    spec = ss.PowerSpec()
    p = ss.generate_power_trajectory(spec, N, DT, seed=seed, defect_type=defect_type, defect_start_frac=0.6)
    pm = sc.analyze_power("power_0", p["t"], p["voltage"], p["current"], p["temperature"], voltage_min_v=spec.voltage_min_v)
    return {"power_0": compute_power_status(pm)}


def build_demo_fleet() -> Fleet:
    fleet = Fleet()

    alpha_events = {}
    alpha_events.update(_arm_events(seed=1))
    alpha_events.update(_gripper_event(seed=200))
    fleet.add_unit(RobotUnit("robot_alpha", alpha_events))

    beta_events = {}
    beta_events.update(_arm_events(seed=10, defect_axis_index=1, defect_type="backlash"))
    beta_events.update(_gripper_event(seed=201))
    fleet.add_unit(RobotUnit("robot_beta", beta_events))

    gamma_events = {}
    gamma_events.update(_arm_events(seed=20))
    gamma_events.update(_gripper_event(seed=202, defect_type="slip_event"))
    fleet.add_unit(RobotUnit("robot_gamma", gamma_events))

    delta_events = {}
    delta_events.update(_arm_events(seed=30))
    delta_events.update(_power_event(seed=203, defect_type="thermal_runaway"))
    fleet.add_unit(RobotUnit("robot_delta", delta_events))

    return fleet


def main() -> None:
    fleet = build_demo_fleet()
    print("=== TIMDR-Robot: demo floty (4 roboty, dane SYNTETYCZNE) ===")
    for unit_id in fleet.unit_ids():
        unit = fleet.get_unit(unit_id)
        print(f"{unit_id}: {unit.worst_level().value} (defekty: {unit.defect_components() or 'brak'})")
    print()
    print(hmi.describe_fleet(fleet))


if __name__ == "__main__":
    main()
