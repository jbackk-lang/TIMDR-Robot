"""demo/run_demo.py — scenariusz demonstracyjny: 6-osiowe ramie robotyczne,
os 3 ma wstrzykniety rosnacy luz mechaniczny (backlash), pozostale osie sa
"czyste".
================================================================================
Odtwarza przyklad "ramie robotyczne" z dokumentu architektury: TIMDR Core
wykrywa anomalie na osi 3, TIMDR Integration mapuje to na status/zdarzenie,
ControlBridge wykonuje (symulowana) reakcje.

Uruchomienie: `python demo/run_demo.py` z katalogu glownego repo (albo
`run.bat`, ktory robi to samo w srodowisku .venv).

Zapisuje takze `data/demo_run.csv` (surowe trajektorie wszystkich osi) i
`data/demo_events.json` (wykryte statusy/zdarzenia) - te dwa pliki sluzy
dashboardowi (api.py) do wyswietlenia wynikow bez ponownego liczenia.
"""
from __future__ import annotations

import csv
import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from timdr_robot import core, hmi
from timdr_robot.control_bridge import ControlBridge
from timdr_robot.sensor_bus import build_arm_scenario
from timdr_robot.status import compute_axis_status

DATA_DIR = Path(__file__).resolve().parent.parent / "data"


def main() -> None:
    DATA_DIR.mkdir(exist_ok=True)

    bus = build_arm_scenario(
        n_axes=6, n_samples=2000, dt=0.01, seed=42,
        defect_axis_index=3, defect_type="backlash",
        defect_start_frac=0.6, defect_severity=1.0,
    )

    bridge = ControlBridge()
    printed = []

    def on_event(event):
        printed.append(f"[{event.level.value}] {event.message}")

    bridge.subscribe_events(on_event)

    events = {}
    metrics_by_axis = {}
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

    print("=== TIMDR-Robot: demo scenariusz (ramie 6-osiowe, dane SYNTETYCZNE) ===")
    for line in printed:
        print(line)
    print()
    for axis_id, event in events.items():
        print(hmi.describe_axis_event(event))
    print()
    print(hmi.describe_scenario(events))
    print()
    print("Log reakcji ControlBridge:")
    for entry in bridge.reaction_log:
        print(f"  {entry.axis_id}: {entry.action} — {entry.detail}")

    # Negative control na osi "czystej" (np. axis_0, ktora nie ma wady)
    from functools import partial
    from timdr_robot.sensor_bus import AxisSpec, generate_axis_trajectory
    clean_spec = AxisSpec(name="negative_control_axis")
    nc = core.negative_control_check(
        partial(generate_axis_trajectory, clean_spec, 2000, 0.01, defect_type=None),
        n_trials=20,
    )
    print()
    print(f"Negative control (os czysta, {nc['n_trials']} powtorzen): "
          f"false_positive_rate={nc['false_positive_rate']:.2f}")
    print(f"  {nc['disclaimer']}")

    # Zapis CSV surowych trajektorii
    csv_path = DATA_DIR / "demo_run.csv"
    with open(csv_path, "w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(["axis_id", "t", "position", "velocity", "accel", "force"])
        for axis_id in bus.axis_ids():
            traj = bus.read_axis(axis_id)
            for i in range(len(traj["t"])):
                writer.writerow([
                    axis_id, traj["t"][i], traj["position"][i],
                    traj["velocity"][i], traj["accel"][i], traj["force"][i],
                ])

    # Zapis JSON zdarzen/statusow (bez surowych serii - te sa w CSV)
    events_path = DATA_DIR / "demo_events.json"
    events_out = {
        axis_id: {
            "level": event.level.value,
            "message": event.message,
            "description": hmi.describe_axis_event(event),
            "harmonic_anomaly_count": event.metrics_snapshot["harmonic_anomaly_count"],
            "torsion_spike_count": event.metrics_snapshot["torsion_spike_count"],
            "torsion_max_abs": event.metrics_snapshot["torsion_max_abs"],
            "ringdown": event.metrics_snapshot["ringdown"],
        }
        for axis_id, event in events.items()
    }
    events_out["_summary"] = hmi.describe_scenario(events)
    events_out["_negative_control"] = nc
    events_out["_reaction_log"] = [
        {"axis_id": e.axis_id, "action": e.action, "detail": e.detail}
        for e in bridge.reaction_log
    ]
    with open(events_path, "w", encoding="utf-8") as f:
        json.dump(events_out, f, ensure_ascii=False, indent=2)

    print()
    print(f"Zapisano: {csv_path}")
    print(f"Zapisano: {events_path}")


if __name__ == "__main__":
    main()
