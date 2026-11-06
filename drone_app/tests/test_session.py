"""Desktop app controls and workers without a drone or visible GUI."""

import math
from pathlib import Path
import sys
import threading
import time
from types import SimpleNamespace
from unittest.mock import Mock

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from drone_app import REPO_ROOT
from drone_app import flight
from drone_app.session import DroneSession, Route
from protocol import CMD_HOLD, CMD_LAND, CMD_START, TelemetryPacket


def packet(drone_id=4, x=0):
    return TelemetryPacket(drone_id, x, 0, 0, 0, -1, 0, [False] * 32, False, 65535)


def test_detector_cache_is_separate_from_existing_viewer():
    from drone_app.detector import _host
    import apriltag_host
    assert _host is not apriltag_host
    assert _host.BUILD_DIR == REPO_ROOT / "drone_app/.cache/apriltag-host"
    assert apriltag_host.BUILD_DIR == REPO_ROOT / "laptop/.apriltag-host"


def test_included_routes_validate_and_preview_without_network():
    for path in (REPO_ROOT / "trajectory").glob("*.csv"):
        route = Route.load(path)
        assert len(route.points) <= 2400
        assert route.peak_speed <= .30 + 1e-6
        assert np.allclose(route.points[0], (0, 0, 0))
        assert not route.points.flags.writeable


@pytest.mark.parametrize("interval", [0, -.1, math.nan, math.inf, .0001, 1.1])
def test_invalid_sample_interval_rejected(interval):
    with pytest.raises(ValueError, match="Sample interval"):
        Route.load(REPO_ROOT / "trajectory/ellipse_min_snap_traj.csv", interval)


def test_overspeed_and_invalid_route_rejected(tmp_path):
    path = tmp_path / "route.csv"
    path.write_text("t,x,y,z\n0,0,0,0\n1,1,0,0\n")
    with pytest.raises(ValueError, match="Peak speed"):
        Route.load(path)
    path.write_text("t,x,y,z\n0,0,0,0\n1,nan,0,0\n")
    with pytest.raises(ValueError, match="finite"):
        Route.load(path)


def test_session_filters_both_drone_id_and_ip():
    session = DroneSession()
    session._on_telemetry(packet(5), "192.168.1.204")
    session._on_telemetry(packet(), "192.168.1.205")
    assert session.snapshot()[0] is None
    sample = packet()
    session._on_telemetry(sample, "192.168.1.204")
    latest, _, samples, _, _ = session.snapshot()
    assert latest is sample and len(samples) == 1
    assert session.snapshot()[2] == []


def test_commands_never_target_an_unverified_id_or_ip():
    session = DroneSession()
    session.comms = Mock()
    session.comms.known_drones.return_value = {4: "192.168.1.205"}
    with pytest.raises(ConnectionError, match="verified"):
        session.send_raw(4, b"test")
    session.comms.send_raw.assert_not_called()
    session.comms.known_drones.return_value = {4: session.esp_ip}
    assert session.send_raw(4, b"test")


def test_demo_generates_frames_without_opening_sockets(monkeypatch):
    forbidden = Mock(side_effect=AssertionError("demo must not access hardware"))
    monkeypatch.setattr("drone_app.session.socket.socket", forbidden)
    monkeypatch.setattr("drone_app.session.Detector", forbidden)
    session = DroneSession(demo=True)
    session.start()
    deadline = time.monotonic() + 2
    while session.snapshot()[4] is None and time.monotonic() < deadline:
        time.sleep(.02)
    _, _, _, count, bundle = session.snapshot()
    assert count > 0 and bundle is not None
    assert bundle[1].shape == (360, 480)
    assert len(bundle[2]) == 3
    with pytest.raises(ConnectionError, match="disabled"):
        session.interrupt(CMD_LAND)
    session.close()
    forbidden.assert_not_called()


@pytest.mark.parametrize("command", [CMD_HOLD, CMD_LAND])
def test_stop_serializes_against_queued_start(command):
    control = flight.FlightControl()
    comms = Mock()
    control.interrupt(comms, 4, command)
    with pytest.raises(flight.FlightCancelled):
        control.send_raw(comms, 4, b"trajectory start")
    with pytest.raises(flight.FlightCancelled):
        control.send_command(comms, 4, SimpleNamespace(cmd_type=CMD_START))
    comms.send_raw.assert_not_called()
    assert comms.send_command.call_args.args[1].cmd_type == command


def test_cancellation_during_upload_sends_no_start_or_cleanup_land():
    control = flight.FlightControl()
    feedback = flight.FlightFeedback(4)
    feedback.update(packet(), "drone")
    comms = Mock()
    comms.send_raw.side_effect = lambda *args: control.cancelled.set() or True
    with pytest.raises(flight.FlightCancelled):
        flight.run_flight(comms, feedback, np.zeros((81, 3)),
                          flight.FlightOptions(4, takeoff=True), control=control)
    comms.send_command.assert_not_called()
    assert comms.send_raw.call_count == 1


def test_takeoff_only_uses_existing_legacy_feedback_and_leaves_hover(monkeypatch):
    now = [0.0]
    feedback = flight.FlightFeedback(4)
    comms = Mock()
    commands = []

    def sleep(seconds):
        now[0] += seconds
        feedback.update(packet(), "drone")

    def send(_id, command):
        commands.append(command.cmd_type)
        return True

    monkeypatch.setattr(flight, "time", SimpleNamespace(monotonic=lambda: now[0], sleep=sleep))
    feedback.update(packet(), "drone")
    comms.send_command.side_effect = send
    flight.run_flight(comms, feedback, None, flight.FlightOptions(4, takeoff=True, takeoff_wait=1))
    assert now[0] >= 1
    assert commands == [CMD_START]
    comms.send_raw.assert_not_called()


def test_stale_feedback_prevents_any_operation():
    session = DroneSession()
    session.comms = Mock()
    session.comms.known_drones.return_value = {4: session.esp_ip}
    with pytest.raises(ConnectionError, match="No drone"):
        session.launch(takeoff=True)
    session.comms.send_command.assert_not_called()
    assert not session.busy
