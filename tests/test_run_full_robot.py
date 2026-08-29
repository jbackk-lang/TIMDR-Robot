import json
import subprocess
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent


def test_run_full_robot_end_to_end():
    result = subprocess.run(
        [sys.executable, str(REPO_ROOT / "demo" / "run_full_robot.py")],
        cwd=str(REPO_ROOT), capture_output=True, text=True, timeout=60,
    )
    assert result.returncode == 0, result.stderr
    assert "DEFECT" in result.stdout

    out_path = REPO_ROOT / "data" / "full_robot_events.json"
    assert out_path.exists()
    events = json.loads(out_path.read_text(encoding="utf-8"))

    assert events["axis_3"]["level"] == "DEFECT"
    assert events["gripper_0"]["level"] == "DEFECT"
    assert events["mobile_base_0"]["level"] == "DEFECT"
    assert events["camera_0"]["level"] == "DEFECT"
    assert events["power_0"]["level"] == "DEFECT"
    assert events["axis_1"]["level"] in ("OK", "SUSPECT")
