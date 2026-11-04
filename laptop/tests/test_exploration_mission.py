"""Mission planning, pose transforms and flight failures, without flight hardware."""

import json
import math
from pathlib import Path
import sys
from unittest.mock import MagicMock, Mock

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from apriltag_host import Detection
import mission_tags as tags
from protocol import CMD_GOTO, CMD_HOLD, CMD_LAND, CMD_START, NAV_IDLE, TelemetryPacket
import simple_exploration_mission as mission
from simple_waypoint_mission import TelemetrySnapshot, wait_for_arrival


def packet(drone_id=22, x=2, y=3, heading=0):
    return TelemetryPacket(drone_id, x, y, heading, NAV_IDLE, -1, 0, [False] * 32, False, 65535)


def detection(**kwargs):
    fields = dict(id=5, hamming=0, decision_margin=100, centre=(160, 120),
                  corners=((0, 0),) * 4, pose_valid=True, t=(0.2, 0, 1),
                  R=(1, 0, 0, 0, 1, 0, 0, 0, 1), pose_err=0.001)
    return Detection(**(fields | kwargs))


def test_sweep_covers_edges_and_respects_spacing():
    bounds = (-1, 2, -0.5, 1.2)
    route = mission.lawnmower(bounds, 0.6)
    assert len(route) == 8
    ys = sorted(set(y for _, y in route))
    assert ys[0] == bounds[2] and ys[-1] == bounds[3]
    assert max(b - a for a, b in zip(ys, ys[1:])) <= 0.6
    for a, b in zip(route[::2], route[1::2]):
        assert a[1] == b[1]
        assert {a[0], b[0]} == {-1, 2}
    for a, b in zip(route[1::2], route[2::2]):
        assert a[0] == b[0]  # Turns are adjacent rows, not diagonal crossings.


@pytest.mark.parametrize("bounds,spacing", [((0, 0, 0, 1), 1), ((1, 0, 0, 1), 1),
                                           ((0, 1, 0, 1), 0), ((0, math.nan, 0, 1), 1),
                                           ((0, 1, 0, 1), math.inf)])
def test_invalid_search_rejected(bounds, spacing):
    with pytest.raises(ValueError):
        mission.lawnmower(bounds, spacing)


def test_localization_uses_mount_yaw_and_drone_position():
    # Camera right becomes body right; camera forward is 45 degrees down.
    snap = TelemetrySnapshot(packet(heading=math.pi / 2), "drone", 10)
    result = tags.detection_record(detection(), snap, 10.2, assumed_altitude=0.5)
    assert result["local_position_ned_m"] == pytest.approx(
        (2 - 0.2, 3 + 0.02 + math.sqrt(0.5), math.sqrt(0.5) - 0.5))
    assert result["relative_camera_rotation_row_major"] == (1, 0, 0, 0, 1, 0, 0, 0, 1)
    assert result["local_rotation_row_major"][:3] == pytest.approx((-1, 0, 0))
    assert result["telemetry_age_at_frame_s"] == pytest.approx(0.2)


def test_unmeasured_z_is_not_fabricated():
    result = tags.detection_record(detection(), TelemetrySnapshot(packet(), "drone", 10), 10.1)
    assert result["local_position_ned_m"][2] is None


def test_detection_details_include_image_geometry_range_and_quality():
    det = detection()
    result = tags.detection_record(det, TelemetrySnapshot(packet(), "drone", 10), 10.1)
    assert result["tag_family"] == "tag16h5"
    assert result["image_center_px"] == det.centre
    assert result["image_corners_px"] == det.corners
    assert result["camera_range_m"] == pytest.approx(math.sqrt(1.04))
    assert result["quality_accepted"] is True
    assert result["quality_rejection_reason"] is None
    assert result["pose_valid"] is True


@pytest.mark.parametrize("fields,reason", [({"decision_margin": 20}, "margin<=55"),
                                         ({"hamming": 2}, "hamming>1")])
def test_low_quality_tag_is_recorded_without_localization(fields, reason):
    result = tags.detection_record(detection(**fields), TelemetrySnapshot(packet(), "drone", 10), 10.1)
    assert result["tag_id"] == 5
    assert result["quality_accepted"] is False
    assert result["quality_rejection_reason"] == reason
    assert result["localization_status"] == "rejected_quality"
    assert result["local_position_ned_m"] is None
    assert result["local_rotation_row_major"] is None
    assert result["uav_local_xy_m"] == (2, 3)


def test_invalid_numeric_detection_still_serializes():
    result = tags.detection_record(detection(decision_margin=math.nan, t=(math.nan, 0, 1),
                                             pose_err=math.inf),
                                   TelemetrySnapshot(packet(), "drone", 10), 10.1)
    encoded = json.dumps(result, allow_nan=False)
    assert json.loads(encoded)["tag_id"] == 5
    assert result["quality_accepted"] is False
    assert result["decision_margin"] is None
    assert result["pose_error"] is None
    assert result["relative_body_translation_m"] is None


def test_missing_pose_or_telemetry_preserves_id():
    result = tags.detection_record(detection(), TelemetrySnapshot(), 10)
    assert result["tag_id"] == 5
    assert result["local_position_ned_m"] is None
    assert result["localization_status"] == "no_matching_telemetry"
    result = tags.detection_record(detection(pose_valid=False, t=None, R=None, pose_err=None),
                                   TelemetrySnapshot(packet(), "drone", 9.9), 10)
    assert result["tag_id"] == 5
    assert result["localization_status"] == "invalid_pose"
    assert result["uav_local_xy_m"] == (2, 3)
    assert result["image_center_px"] == (160, 120)


def test_history_uses_pre_image_pose_and_excludes_other_drone(monkeypatch):
    now = [10.0]
    monkeypatch.setattr(tags.time, "monotonic", lambda: now[0])
    tracker = tags.PoseTracker(22)
    tracker.callback(packet(x=1), "drone")
    now[0] = 10.2
    tracker.callback(packet(x=9), "drone")
    tracker.callback(packet(drone_id=23, x=100), "other")
    assert tracker.pose_at(10.1, 0.5).packet.ned_x == 1
    assert tracker.pose_at(10.3, 0.5).packet.ned_x == 9
    assert tracker.pose_at(11, 0.5).packet is None
    assert tracker.pose_at(9, 0.5).packet is None


def test_journal_flushes_and_refuses_overwrite(tmp_path):
    path = tmp_path / "detections.jsonl"
    journal = tags.Journal(path)
    try:
        journal.write("detection", **tags.detection_record(detection(), TelemetrySnapshot(), 10))
        record = json.loads(path.read_text())
        assert record["tag_id"] == 5
        assert record["timestamp_utc"].endswith("+00:00")
        with pytest.raises(FileExistsError):
            tags.Journal(path)
    finally:
        journal.close()


@pytest.fixture
def flight(monkeypatch):
    args = mission.parse_args(["--drone-id", "22", "--bounds", "0", "1", "0", "1"])
    tracker = Mock()
    tracker.latest.side_effect = lambda: TelemetrySnapshot(packet(), "drone", mission.time.monotonic())
    comms, recorder, journal = Mock(), Mock(), Mock()
    obj = mission.Mission(args, comms, tracker, recorder, journal)
    monkeypatch.setattr(mission, "checked_wait", lambda duration, health: health())
    monkeypatch.setattr(mission.time, "sleep", lambda _: None)
    return obj


def sent_commands(flight):
    return [call.args[1] for call in flight.comms.send_command.call_args_list]


def test_complete_mission_returns_to_actual_home_before_land(flight, monkeypatch):
    monkeypatch.setattr(mission, "wait_for_arrival", lambda *a, **kw: True)
    assert flight.fly((2, 3), [(0, 1), (1, 1)]) == 0
    commands = sent_commands(flight)
    assert commands[0].cmd_type == CMD_START
    assert [(c.goal_x, c.goal_y) for c in commands if c.cmd_type == CMD_GOTO] == [
        (2, 4), (3, 4), (3, 4), (2, 4), (2, 3)]
    assert [c.cmd_type for c in commands[-3:]] == [CMD_LAND] * 3
    assert flight.land_sent


def test_search_timeout_returns_and_reports_incomplete(flight, monkeypatch):
    arrivals = iter([True, False, True, True])
    monkeypatch.setattr(mission, "wait_for_arrival", lambda *a, **kw: next(arrivals))
    assert flight.fly((2, 3), [(0, 1), (1, 1), (2, 1)]) == 1
    commands = sent_commands(flight)
    assert CMD_HOLD in [c.cmd_type for c in commands]
    assert [(c.goal_x, c.goal_y) for c in commands if c.cmd_type == CMD_GOTO][-2:] == [(2, 4), (2, 3)]
    assert flight.land_sent


def test_return_failure_is_not_reported_as_success(flight, monkeypatch):
    arrivals = iter([True, False])
    monkeypatch.setattr(mission, "wait_for_arrival", lambda *a, **kw: next(arrivals))
    with pytest.raises(TimeoutError, match="Return waypoint"):
        flight.fly((2, 3), [(0, 1)])
    assert not flight.land_sent  # main's finally performs the fallback.


def test_health_failure_checked_before_goto(flight):
    flight.recorder.check_health.side_effect = ConnectionError("camera lost")
    with pytest.raises(ConnectionError, match="camera lost"):
        flight.goto((4, 5), 1)
    assert not flight.comms.send_command.called


def test_health_hook_interrupts_arrival_wait():
    def broken_camera():
        raise ConnectionError("camera lost")
    with pytest.raises(ConnectionError, match="camera lost"):
        wait_for_arrival(Mock(), Mock(), 1, 3, 0, 0, 0.2, check_health=broken_camera)


@pytest.mark.parametrize("failure,exit_code", [(RuntimeError("lost link"), 1), (KeyboardInterrupt(), 130)])
def test_main_lands_and_cleans_up_on_failure(tmp_path, monkeypatch, failure, exit_code):
    comms, recorder = Mock(), Mock()
    tracker = Mock()
    tracker.latest.return_value = TelemetrySnapshot(packet(), "drone", 0)
    monkeypatch.setattr(mission, "CommsNode", lambda **kw: comms)
    monkeypatch.setattr(mission, "PoseTracker", lambda *a: tracker)
    monkeypatch.setattr(mission, "TagRecorder", lambda *a, **kw: recorder)
    monkeypatch.setattr(mission.Mission, "check_health", lambda self: None)
    monkeypatch.setattr(mission, "checked_wait", lambda *a: None)
    monkeypatch.setattr(mission.time, "sleep", lambda _: None)

    def fail(self, *args):
        self.start_attempted = True
        raise failure

    monkeypatch.setattr(mission.Mission, "fly", fail)
    result = mission.main(["--drone-id", "22", "--bounds", "0", "1", "0", "1",
                           "--log", str(tmp_path / "mission.jsonl")])
    assert result == exit_code
    assert [call.args[1].cmd_type for call in comms.send_command.call_args_list] == [CMD_LAND] * 3
    recorder.stop.assert_called_once()
    comms.stop.assert_called_once()


def test_dry_run_never_opens_comms(monkeypatch, capsys):
    monkeypatch.setattr(mission, "CommsNode", Mock(side_effect=AssertionError("network opened")))
    assert mission.main(["--drone-id", "22", "--bounds", "0", "1", "0", "1", "--dry-run"]) == 0
    assert "Return:" in capsys.readouterr().out


def test_recorder_surfaces_worker_failure():
    recorder = tags.TagRecorder("127.0.0.1", 5006, Mock(), Mock())
    recorder._error = OSError("disk full")
    with pytest.raises(RuntimeError, match="disk full"):
        recorder.check_health()


def test_recorder_stale_camera_fails():
    recorder = tags.TagRecorder("127.0.0.1", 5006, Mock(), Mock())
    with pytest.raises(ConnectionError, match="Camera stream is stale"):
        recorder.check_health()


@pytest.mark.parametrize("include_rejected", [False, True])
def test_camera_jpeg_to_detection_journal(tmp_path, monkeypatch, include_rejected):
    """Exercise JPEG recording with a real tag and with mixed repeated sightings."""
    cv2 = pytest.importorskip("cv2")
    np = pytest.importorskip("numpy")
    from camera_stream import Frame
    from protocol import build_camera_stream_command

    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_16h5)
    gray = np.full((240, 320), 255, np.uint8)
    gray[60:180, 100:220] = cv2.aruco.generateImageMarker(dictionary, 5, 120)
    ok, jpeg = cv2.imencode(".jpg", gray)
    assert ok
    tracker = tags.PoseTracker(22)
    tracker.callback(packet(), "drone")
    frame = Frame(7, jpeg.tobytes(), 20, 0, tags.time.monotonic())
    path = tmp_path / "camera.jsonl"
    journal = tags.Journal(path)
    recorder = tags.TagRecorder("drone", 5006, tracker, journal)
    recorder.set_phase("exploring")
    if include_rejected:
        detector = MagicMock()
        detector.__enter__.return_value = detector
        detector.detect.return_value = [detection(), detection(id=7, decision_margin=20),
                                       detection(id=9, pose_valid=False, t=None, R=None, pose_err=None)]
        monkeypatch.setattr(tags, "Detector", lambda: detector)
    rx, tx = MagicMock(), MagicMock()
    rx.__enter__.return_value = rx
    tx.__enter__.return_value = tx
    monkeypatch.setattr(tags.socket, "socket", Mock(side_effect=[rx, tx]))
    monkeypatch.setattr(tags.select, "select", lambda *a: None)

    frame_count = 0

    def receive(*args):
        nonlocal frame_count
        frame_count += 1
        if frame_count == (2 if include_rejected else 1):
            recorder._stop.set()
        return Frame(6 + frame_count, frame.jpeg, frame.age_ms, frame.esp_drops, frame.first_rx), 1, None

    monkeypatch.setattr(tags, "drain_socket", receive)
    try:
        recorder._run()
        recorder.check_health()
        records = [json.loads(line) for line in path.read_text().splitlines()]
        assert len(records) == (6 if include_rejected else 1)
        assert records[0]["tag_id"] == 5
        assert records[0]["frame_id"] == 7
        assert records[0]["phase"] == "exploring"
        assert records[0]["local_position_ned_m"][:2] != [None, None]
        assert records[0]["frame_received_utc"].endswith("+00:00")
        assert all(r["timestamp_utc"].endswith("+00:00") for r in records)
        if include_rejected:
            assert [r["tag_id"] for r in records] == [5, 7, 9, 5, 7, 9]
            assert [r["frame_id"] for r in records] == [7, 7, 7, 8, 8, 8]
            assert records[1]["quality_accepted"] is False
            assert records[1]["local_position_ned_m"] is None
            assert records[2]["pose_valid"] is False
            assert records[2]["local_position_ned_m"] is None
        assert tx.sendto.call_args.args == (build_camera_stream_command(False), ("drone", 5006))
    finally:
        journal.close()


def test_preflight_camera_failure_never_sends_start(tmp_path, monkeypatch):
    comms, tracker, recorder = Mock(), Mock(), Mock()
    tracker.latest.return_value = TelemetrySnapshot(packet(), "drone", 0)
    recorder.start.side_effect = TimeoutError("no camera")
    monkeypatch.setattr(mission, "CommsNode", lambda **kw: comms)
    monkeypatch.setattr(mission, "PoseTracker", lambda *a: tracker)
    monkeypatch.setattr(mission, "TagRecorder", lambda *a, **kw: recorder)
    monkeypatch.setattr(mission, "checked_wait", lambda *a: None)
    assert mission.main(["--drone-id", "22", "--bounds", "0", "1", "0", "1",
                         "--log", str(tmp_path / "mission.jsonl")]) == 1
    comms.send_command.assert_not_called()
    recorder.stop.assert_called_once()


def test_landing_command_precedes_journal_write(flight):
    flight.journal.write.side_effect = OSError("disk full")
    with pytest.raises(OSError, match="disk full"):
        flight.land()
    assert [c.cmd_type for c in sent_commands(flight)] == [CMD_LAND] * 3
    assert flight.land_sent
