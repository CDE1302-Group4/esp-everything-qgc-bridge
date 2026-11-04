"""CSV routes exercise the shared exploration lifecycle without flight hardware."""

import csv
import json
from pathlib import Path
import sys
from unittest.mock import Mock

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import simple_waypoint_mission as waypoint
import simple_exploration_mission as runner
from mission_tags import PoseTracker
from protocol import CMD_GOTO, CMD_HOLD, CMD_LAND, CMD_START, NAV_IDLE, TelemetryPacket


def make_csv(tmp_path, content):
    path = tmp_path / "route.csv"
    path.write_text(content, encoding="utf-8")
    return path


@pytest.mark.parametrize("content", [
    "x,y\n1,2\n-3,4\n", "1,2\n-3,4\n",
    '\ufeff# test route\n\n X , Y \n"1","2"\n# comment\n-3,4\n',
])
def test_csv_formats_preserve_order(tmp_path, content):
    path = make_csv(tmp_path, content)
    assert waypoint.load_waypoints_from_csv(str(path)) == [(1, 2), (-3, 4)]
    assert waypoint.load_waypoints_from_file(str(path)) == [(1, 2), (-3, 4)]


@pytest.mark.parametrize("content", [
    "", "# comments only\n", "x,y\n", "x,y\n1,\n", "x,y\n1,2,3\n",
    "t,x,y,z\n0,1,2,-0.5\n", "x,y\n1,nan\n", "x,y\ninf,2\n",
    "x,y\n1,2\nx,y\n", 'x,y\n"1,2\n', "x,y\n1,2\nbad,3\n",
])
def test_invalid_csv_is_rejected_in_full(tmp_path, content):
    with pytest.raises(ValueError):
        waypoint.load_waypoints_from_csv(str(make_csv(tmp_path, content)))


def test_csv_error_identifies_source_line(tmp_path):
    with pytest.raises(ValueError, match=r"route.csv:4:"):
        waypoint.load_waypoints_from_csv(str(make_csv(tmp_path, "# route\nx,y\n1,2\nwrong,4\n")))


def test_legacy_waypoints_and_negative_arguments(tmp_path):
    path = tmp_path / "route.txt"
    path.write_text("# route\n1 2\n-3,4\n")
    assert waypoint.load_waypoints_from_file(str(path)) == [(1, 2), (-3, 4)]
    args = waypoint.parse_args(["--drone-id", "22", "--waypoint", "-1,2"])
    assert args.waypoints == [(-1, 2)]
    with pytest.raises(SystemExit):
        waypoint.parse_args(["--drone-id", "22", "--waypoint", "nan,2"])


def test_dry_run_and_invalid_files_never_connect(tmp_path, monkeypatch, capsys):
    monkeypatch.setattr(runner, "CommsNode", Mock(side_effect=AssertionError("opened network")))
    path = make_csv(tmp_path, "x,y\n0.5,0\n-1,2\n")
    assert waypoint.main(["--drone-id", "22", "--waypoints-csv", str(path), "--dry-run"]) == 0
    assert "1: 0.500, 0.000" in capsys.readouterr().out
    path.write_text("x,y\n0.5,0\ninvalid,2\n")
    assert waypoint.main(["--drone-id", "22", "--waypoints-csv", str(path)]) == 1
    assert waypoint.main(["--drone-id", "22", "--waypoints-csv", str(tmp_path / "missing.csv")]) == 1
    assert waypoint.main(["--drone-id", "22"]) == 1


@pytest.fixture
def fake_flight(tmp_path, monkeypatch):
    pkt = TelemetryPacket(22, 2, 3, 0, NAV_IDLE, -1, 0, [False] * 32, False, 65535)
    tracker, recorder, comms = Mock(), Mock(), Mock()
    tracker.latest.side_effect = lambda: waypoint.TelemetrySnapshot(pkt, "drone", runner.time.monotonic())
    tracker_factory = Mock(return_value=tracker)
    comms_factory = Mock(return_value=comms)
    monkeypatch.setattr(runner, "PoseTracker", tracker_factory)
    monkeypatch.setattr(runner, "CommsNode", comms_factory)
    monkeypatch.setattr(runner, "TagRecorder", Mock(return_value=recorder))
    monkeypatch.setattr(runner, "checked_wait", lambda duration, check: check())
    monkeypatch.setattr(runner.time, "sleep", lambda _: None)
    monkeypatch.setattr(runner, "wait_for_arrival", Mock(return_value=True))
    path = make_csv(tmp_path, "x,y\n0.5,0\n1,1\n")
    log_path = tmp_path / "mission.jsonl"
    args = ["--drone-id", "22", "--waypoints-csv", str(path), "--log", str(log_path)]
    return args, comms, recorder, log_path, tracker_factory, comms_factory


def commands(comms):
    return [call.args[1] for call in comms.send_command.call_args_list]


def goals(comms):
    return [(c.goal_x, c.goal_y) for c in commands(comms) if c.cmd_type == CMD_GOTO]


def test_csv_flight_order_return_home_and_logging(fake_flight):
    args, comms, recorder, log_path, _, _ = fake_flight
    assert waypoint.main(args) == 0
    assert commands(comms)[0].cmd_type == CMD_START
    assert goals(comms) == [(2.5, 3), (3, 4), (3, 4), (2.5, 3), (2, 3)]
    assert [c.cmd_type for c in commands(comms)[-3:]] == [CMD_LAND] * 3
    records = [json.loads(line) for line in log_path.read_text().splitlines()]
    assert records[0]["waypoints_m"] == [[0.5, 0], [1, 1]]
    assert records[0]["waypoint_frame"] == "relative_to_recorded_start"
    assert [r["phase"] for r in records if r["type"] == "phase"] == ["takeoff", "exploring", "returning"]
    recorder.start.assert_called_once()
    recorder.stop.assert_called_once()
    comms.stop.assert_called_once()


def test_mission_can_write_csv_output(fake_flight):
    args, _, _, log_path, _, _ = fake_flight
    csv_path = log_path.with_suffix(".csv")
    args[args.index("--log") + 1] = str(csv_path)
    assert waypoint.main(args) == 0
    with csv_path.open(newline="") as source:
        records = list(csv.DictReader(source))
    assert records[0]["type"] == "mission"
    assert json.loads(records[0]["details_json"])["waypoints_m"] == [[0.5, 0], [1, 1]]
    goals = [r for r in records if r["type"] == "goal"]
    assert float(goals[-1]["goal_local_x_m"]) == 2
    assert float(goals[-1]["goal_local_y_m"]) == 3
    assert records[-1]["type"] == "landing_command_sent"
    assert not log_path.exists()


def test_map_frame_uses_absolute_goals(fake_flight, tmp_path):
    args, comms, _, log_path, tracker_factory, comms_factory = fake_flight
    config = tmp_path / "setup.yaml"
    config.write_text("drones:\n  22: {start_x: 10, start_y: -5}\n")
    assert waypoint.main(args + ["--map-frame", "--config", str(config)]) == 0
    assert goals(comms) == [(0.5, 0), (1, 1), (1, 1), (0.5, 0), (2, 3)]
    assert tracker_factory.call_args.args[2] == (10, -5)
    assert comms_factory.call_args.kwargs["config_path"] == str(config)
    comms.set_nav_tags.assert_not_called()
    first = json.loads(log_path.read_text().splitlines()[0])
    assert first["waypoint_frame"] == "arena"


def test_tag_history_subtracts_map_offset_but_navigation_keeps_map_pose():
    tracker = PoseTracker(22, map_offset=(10, -5))
    pkt = TelemetryPacket(22, 12, -2, 0, NAV_IDLE, -1, 0, [False] * 32, False, 65535)
    tracker.callback(pkt, "drone")
    latest = tracker.latest()
    local = tracker.pose_at(latest.received_at, 0.5)
    assert (latest.packet.ned_x, latest.packet.ned_y) == (12, -2)
    assert (local.packet.ned_x, local.packet.ned_y) == (2, 3)


def test_explicit_hold_finishes_at_home_without_fallback_land(fake_flight):
    args, comms, _, _, _, _ = fake_flight
    assert waypoint.main(args + ["--finish-action", "hold"]) == 0
    assert goals(comms)[-1] == (2, 3)
    assert commands(comms)[-1].cmd_type == CMD_HOLD
    assert CMD_LAND not in [c.cmd_type for c in commands(comms)]


def test_missed_csv_waypoint_returns_without_visiting_later_goals(fake_flight, monkeypatch):
    args, comms, _, _, _, _ = fake_flight
    monkeypatch.setattr(runner, "wait_for_arrival", Mock(side_effect=[False, True]))
    assert waypoint.main(args) == 1
    assert goals(comms) == [(2.5, 3), (2, 3)]
    assert [c.cmd_type for c in commands(comms)[-3:]] == [CMD_LAND] * 3


@pytest.mark.parametrize("failure,code", [(ConnectionError("lost camera"), 1), (KeyboardInterrupt(), 130)])
def test_failure_in_csv_flight_lands(fake_flight, monkeypatch, failure, code):
    args, comms, _, _, _, _ = fake_flight
    monkeypatch.setattr(runner, "wait_for_arrival", Mock(side_effect=failure))
    assert waypoint.main(args) == code
    assert commands(comms)[0].cmd_type == CMD_START
    assert [c.cmd_type for c in commands(comms)[-3:]] == [CMD_LAND] * 3


def test_missing_map_start_is_rejected_before_connecting(tmp_path, monkeypatch):
    monkeypatch.setattr(runner, "CommsNode", Mock(side_effect=AssertionError("opened network")))
    config = tmp_path / "setup.yaml"
    config.write_text("drones: {}\n")
    assert waypoint.main(["--drone-id", "22", "--waypoint", "1,2", "--map-frame",
                          "--config", str(config)]) == 1
