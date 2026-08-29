import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from fastapi.testclient import TestClient

from api import app

client = TestClient(app)


def test_index_serves_html():
    r = client.get("/")
    assert r.status_code == 200
    assert "TIMDR-Robot" in r.text
    assert "text/html" in r.headers["content-type"]


def test_index_does_not_reference_external_cdn():
    """Regression test - lekcja z fusion-tools/SYNOPTYK-ARCTIC: Chart.js
    musi byc serwowany lokalnie, nigdy z zewnetrznego hosta."""
    r = client.get("/")
    assert r.status_code == 200
    assert 'src="http' not in r.text
    assert '<script src="/static/vendor/chart.umd.js"></script>' in r.text


def test_vendored_chartjs_is_served():
    r = client.get("/static/vendor/chart.umd.js")
    assert r.status_code == 200
    assert b"Chart.js" in r.content
    assert len(r.content) > 100_000


def test_scenario_endpoint_reports_six_axes_with_defect_on_axis_3():
    r = client.get("/api/scenario")
    assert r.status_code == 200
    body = r.json()
    assert set(body["axes"].keys()) == {f"axis_{i}" for i in range(6)}
    assert body["axes"]["axis_3"]["level"] == "DEFECT"
    assert body["axes"]["axis_1"]["level"] in ("OK", "SUSPECT")
    assert "disclaimer" in body
    assert "SYNTETYCZNE" in body["disclaimer"]


def test_scenario_endpoint_includes_reaction_log_for_defect_axis():
    r = client.get("/api/scenario")
    body = r.json()
    actions = [e["action"] for e in body["reaction_log"] if e["axis_id"] == "axis_3"]
    assert "stop_axis" in actions
    assert "alarm" in actions


def test_scenario_endpoint_includes_negative_control():
    r = client.get("/api/scenario")
    body = r.json()
    nc = body["negative_control"]
    assert nc["n_trials"] > 0
    assert 0.0 <= nc["false_positive_rate"] <= 1.0


def test_axis_endpoint_returns_time_series():
    r = client.get("/api/axis/axis_0")
    assert r.status_code == 200
    body = r.json()
    assert body["axis_id"] == "axis_0"
    assert len(body["t"]) == len(body["position"]) == len(body["torsion_series"])
    assert len(body["t"]) > 0


def test_axis_endpoint_unknown_axis_returns_404():
    r = client.get("/api/axis/does_not_exist")
    assert r.status_code == 404


def test_subsystems_endpoint_reports_four_components_all_defect():
    r = client.get("/api/subsystems")
    assert r.status_code == 200
    body = r.json()
    assert set(body["subsystems"].keys()) == {"gripper_0", "mobile_base_0", "camera_0", "power_0"}
    for component_id, data in body["subsystems"].items():
        assert data["level"] == "DEFECT", f"{component_id} spodziewany DEFECT (wstrzykniete zdarzenie), dostal {data['level']}"
    assert "SYNTETYCZNE" in body["disclaimer"]


def test_component_endpoint_returns_series():
    r = client.get("/api/component/gripper_0")
    assert r.status_code == 200
    body = r.json()
    assert "grip_force" in body["series"]
    assert len(body["t"]) == len(body["series"]["grip_force"])


def test_component_endpoint_unknown_returns_404():
    r = client.get("/api/component/does_not_exist")
    assert r.status_code == 404


def test_fleet_endpoint_reports_four_units_with_expected_levels():
    r = client.get("/api/fleet")
    assert r.status_code == 200
    body = r.json()
    assert set(body["units"].keys()) == {"robot_alpha", "robot_beta", "robot_gamma", "robot_delta"}
    assert body["units"]["robot_alpha"]["level"] == "OK"
    assert body["units"]["robot_beta"]["level"] == "DEFECT"
    assert body["fleet_level"] == "DEFECT"
    assert "SYNTETYCZNE" in body["disclaimer"]
