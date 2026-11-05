"""Live plot behaviour, exercised without a drone or GUI windows."""

import math
import socket
import struct
import sys
from dataclasses import replace
from contextlib import nullcontext
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
pytest.importorskip("cv2")
matplotlib = pytest.importorskip("matplotlib")
matplotlib.use("Agg")

import tag_stream
from apriltag_host import Detection, POSE_ERR_LIMIT


@pytest.fixture
def plot(monkeypatch):
    monkeypatch.setattr(tag_stream.plt, "show", lambda **kwargs: None)
    result = tag_stream.TagPlot()
    yield result
    tag_stream.plt.close(result.figure)


def detection(tag_id=5, **changes):
    # Camera pose that maps to body (forward=3, right=4, down=0), range 5 m.
    det = Detection(
        id=tag_id, hamming=0, decision_margin=100.0,
        centre=(100, 100), corners=((90, 90), (110, 90), (110, 110), (90, 110)),
        pose_valid=True, t=(4, -2.98 / math.sqrt(2), 2.98 / math.sqrt(2)),
        R=None, pose_err=1e-6)
    return replace(det, **changes)


def pose(x=0, y=0, heading=0):
    return SimpleNamespace(ned_x=x, ned_y=y, heading_rad=heading)


def telemetry_bytes(x=0, y=0, heading=0):
    return struct.pack("<BBfffBbf32sBH", 1, 22, x, y, heading,
                       0, -1, 0, bytes(32), 0, 65535)


def test_positions_and_distances_use_drone_centre_and_tag_ids(plot):
    plot.map.update_pose(pose(), 0.4)
    plot.update([detection(), detection(12, t=(0, 0, 1))], 0.5)
    plot.refresh(0.5)
    marker = plot.markers[5]
    assert marker.get_xdata() == pytest.approx([4])  # right
    assert marker.get_ydata() == pytest.approx([3])  # forward
    assert plot.position_ax.get_xlim()[1] > 4  # expands beyond initial bounds
    assert plot.position_ax.get_ylim()[1] > 3
    labels = [annotation.get_text() for annotation in plot.annotations]
    assert any("Tag 5: 5.00 m" in label and "down" in label for label in labels)
    assert plot.markers[12].get_color() != marker.get_color()
    assert plot.map.tags[5] == pytest.approx((3, 4))
    assert plot.map_artists[5][0].get_xdata() == pytest.approx([4])
    assert "Tag 5\n5.00 m from start" == plot.map_artists[5][1].get_text()
    assert "time" not in plot.map_ax.get_xlabel().lower()


def test_missing_tags_stay_on_the_permanent_map(plot):
    plot.map.update_pose(pose(), 0)
    plot.update([detection()], 0)
    plot.update([], 0.5)
    plot.update([], 0.7)
    plot.refresh(0.7)
    assert not plot.markers[5].get_xdata()
    assert plot.map.tags[5] == pytest.approx((3, 4))
    assert plot.map_artists[5][0].get_xdata() == pytest.approx([4])
    assert plot.map.counts[5] == 1


def test_translation_and_turns_keep_a_stationary_tag_fixed():
    mapping = tag_stream.StartMap()
    mapping.update_pose(pose(10, 20), 0)
    mapping.observe({5: (3, 4, 1)}, 0)
    mapping.update_pose(pose(11, 20), 0.1)
    mapping.observe({5: (2, 4, 1)}, 0.1)
    mapping.update_pose(pose(11, 20, math.pi / 2), 0.2)
    mapping.observe({5: (4, -2, 1)}, 0.2)
    assert mapping.tags[5] == pytest.approx((3, 4))
    assert mapping.counts[5] == 3
    assert mapping.drone_path == [(0, 0), (1, 0), (1, 0)]


def test_initial_heading_sets_map_axes_and_pose_is_matched_to_capture_time():
    mapping = tag_stream.StartMap()
    mapping.update_pose(pose(10, 20, math.pi / 2), 0)
    mapping.update_pose(pose(10, 21, math.pi / 2), 0.1)
    mapping.update_pose(pose(10, 21, math.pi), 0.2)
    # Frame captured before the turn: use the pose at 0.1 s, not the newest.
    mapping.observe({12: (2, 4, 0)}, 0.1)
    assert mapping.tags[12] == pytest.approx((3, 4))
    assert mapping.drone_path[-1] == pytest.approx((1, 0))


def test_map_pauses_without_recent_pose_and_resumes_without_resetting_origin():
    mapping = tag_stream.StartMap()
    mapping.observe({5: (3, 4, 0)}, 0)
    assert mapping.tags == {}
    mapping.update_pose(pose(), 0)
    mapping.observe({5: (3, 4, 0)}, 0)
    mapping.observe({5: (99, 99, 0), 12: (99, 99, 0)}, 1)
    assert mapping.tags == {5: (3, 4)}
    assert mapping.counts[5] == 1
    mapping.update_pose(pose(1, 0), 1.1)
    mapping.observe({5: (2, 4, 0), 12: (1, 1, 0)}, 1.1)
    assert mapping.origin == (0, 0, 0)
    assert mapping.tags[5] == pytest.approx((3, 4))
    assert mapping.tags[12] == pytest.approx((2, 1))
    assert math.isnan(mapping.drone_path[-2][0])  # no invented path across a gap


def test_repeated_observations_refine_one_marker_per_id():
    mapping = tag_stream.StartMap()
    mapping.update_pose(pose(), 0)
    mapping.observe({5: (3, 4, 0)}, 0)
    mapping.observe({5: (3.2, 4.2, 0)}, 0.1)
    assert mapping.tags[5] == pytest.approx((3.1, 4.1))
    assert mapping.counts == {5: 2}


def test_invalid_telemetry_does_not_set_or_move_the_origin():
    mapping = tag_stream.StartMap()
    mapping.update_pose(pose(math.nan), 0)
    assert mapping.origin is None
    mapping.update_pose(pose(1, 2), 0)
    mapping.update_pose(pose(9, 9, math.inf), 0.1)
    mapping.update_pose(pose(9, 9), -1)
    assert mapping.origin == (1, 2, 0)
    assert len(mapping.poses) == 1


@pytest.mark.parametrize("changes", [
    {"pose_valid": False, "t": None, "pose_err": None},
    {"hamming": 2},
    {"decision_margin": 55},
    {"pose_err": POSE_ERR_LIMIT},
    {"pose_err": math.nan},
    {"t": (math.nan, 0, 1)},
])
def test_unusable_poses_are_not_plotted(plot, changes):
    plot.map.update_pose(pose(), 0)
    plot.update([detection(**changes)], 0)
    assert plot.markers == {}
    assert plot.map.tags == {}
    assert plot.current == {}


def test_stalled_stream_clears_live_positions_but_preserves_map(plot):
    plot.map.update_pose(pose(), 0)
    plot.update([detection()], 0)
    plot.refresh(1.1)
    assert plot.current == {}
    assert plot.map.tags[5] == pytest.approx((3, 4))
    assert plot.map_artists[5][0].get_xdata() == pytest.approx([4])
    assert not plot.drone_marker.get_xdata()


def test_save_renders_latest_data_after_window_closed(plot, tmp_path):
    plot.map.update_pose(pose(), 0)
    plot.update([detection()], 0)
    tag_stream.plt.close(plot.figure)
    path = tmp_path / "plots" / "tags.png"
    plot.save(path, 0.1)
    assert path.read_bytes().startswith(b"\x89PNG\r\n\x1a\n")
    assert plot.map_artists[5][0].get_ydata() == pytest.approx([3])


def test_telemetry_reader_filters_drone_sources_and_accepts_local_copies(plot):
    sock = Mock()
    sock.recvfrom.side_effect = [
        (telemetry_bytes(99, 99), ("192.168.1.223", 1234)),
        (b"bad", ("192.168.1.222", 1234)),
        (socket.inet_aton("192.168.1.223") + telemetry_bytes(99, 99), ("127.0.0.1", 5005)),
        (telemetry_bytes(10, 20), ("192.168.1.222", 1234)),
        (socket.inet_aton("192.168.1.222") + telemetry_bytes(11, 20), ("127.0.0.1", 5005)),
        BlockingIOError,
    ]
    tag_stream.drain_telemetry(sock, "192.168.1.222", plot, tag_stream.time.monotonic())
    assert plot.map.origin == (10, 20, 0)
    assert len(plot.map.poses) == 2
    assert plot.map.drone_path[-1] == (1, 0)


@pytest.mark.parametrize("mirror_fails", [False, True])
def test_comms_mirrors_telemetry_without_interrupting_flight_callbacks(monkeypatch, mirror_fails):
    from comms import CommsNode
    node = CommsNode()
    node._sock = Mock()
    mirror = Mock()
    monkeypatch.setattr("comms.socket.socket", lambda *args: nullcontext(mirror))
    wire = telemetry_bytes(10, 20)
    node._sock.recvfrom.side_effect = [(wire, ("192.168.1.222", 1234)), OSError]
    if mirror_fails:
        mirror.sendto.side_effect = OSError
    callback = Mock()
    node.on_telemetry(callback)
    node._running = True
    node._recv_loop()
    node._sock.sendto.assert_not_called()
    mirror.sendto.assert_called_once_with(
        socket.inet_aton("192.168.1.222") + wire,
        ("127.0.0.1", tag_stream.TELEM_MIRROR_PORT))
    callback.assert_called_once()
    packet, source_ip = callback.call_args.args
    assert packet.ned_x == 10 and packet.ned_y == 20
    assert source_ip == "192.168.1.222"


@pytest.mark.parametrize("exit_mode", ["q", "interrupt", "close_plot"])
def test_stream_exit_saves_plot_and_releases_resources(monkeypatch, tmp_path, exit_mode):
    path = tmp_path / "session.png"
    monkeypatch.setattr(sys, "argv", ["tag_stream.py", "--esp-ip", "127.0.0.1",
                                     "--plot-output", str(path)])
    rx, tx, telemetry = Mock(), Mock(), Mock()
    monkeypatch.setattr(tag_stream.socket, "socket", Mock(side_effect=[rx, tx, telemetry]))
    telemetry.recvfrom.side_effect = [
        (socket.inet_aton("127.0.0.1") + telemetry_bytes(), ("127.0.0.1", 5005)),
        BlockingIOError,
    ]
    monkeypatch.setattr(tag_stream.time, "monotonic", lambda: 1000)
    monkeypatch.setattr(tag_stream.select, "select", lambda *args: None)
    detector = Mock()
    detector.detect.return_value = [detection()]
    monkeypatch.setattr(tag_stream, "Detector", lambda: detector)
    ok, jpeg = tag_stream.cv2.imencode(".jpg", tag_stream.np.zeros((240, 320), "uint8"))
    assert ok
    frame = SimpleNamespace(jpeg=jpeg.tobytes(), first_rx=1000, age_ms=0)
    monkeypatch.setattr(tag_stream, "drain_socket", lambda *args: (frame, 0, 0))
    for name in ("namedWindow", "imshow", "destroyAllWindows"):
        monkeypatch.setattr(tag_stream.cv2, name, Mock())
    monkeypatch.setattr(tag_stream.plt, "show", lambda **kwargs: None)
    plots = []
    constructor = tag_stream.TagPlot

    def create_plot():
        result = constructor()
        plots.append(result)
        return result

    monkeypatch.setattr(tag_stream, "TagPlot", create_plot)

    def quit_stream(delay):
        if exit_mode == "interrupt":
            raise KeyboardInterrupt
        if exit_mode == "close_plot":
            tag_stream.plt.close("all")
            return -1
        return ord("q")

    monkeypatch.setattr(tag_stream.cv2, "waitKey", quit_stream)
    tag_stream.main()
    assert path.read_bytes().startswith(b"\x89PNG\r\n\x1a\n")
    detector.detect.assert_called_once()
    detector.close.assert_called_once()
    rx.close.assert_called_once()
    tx.close.assert_called_once()
    telemetry.close.assert_called_once()
    assert plots[0].map.tags[5] == pytest.approx((3, 4))
    assert not tag_stream.plt.get_fignums()


def flight_pose(x=0, y=0, armed=False, offset=(0, 0), fresh=True):
    return SimpleNamespace(ned_x=x + offset[0], ned_y=y + offset[1], heading_rad=0,
                           flight_status_version=1, map_offset_x=offset[0], map_offset_y=offset[1],
                           px4_armed=int(armed), px4_link_ok=fresh)


def test_extended_preflight_drift_is_visible_but_not_a_flown_path(plot):
    plot.map.update_pose(flight_pose(1, 2), 0)
    plot.map.update_pose(flight_pose(1.04, 2), 0.1)
    plot.update([detection()], 0.1)
    plot.refresh(0.1)
    assert plot.map.drone_path == []
    assert plot.map.tags == {}
    assert plot.map.preflight_delta == pytest.approx((0.04, 0))
    assert 'Disarmed' in plot.map_ax.get_title()
    assert '+0.040' in plot.map_ax.get_title()


def test_map_offset_change_does_not_move_the_flight_path():
    mapping = tag_stream.StartMap()
    mapping.update_pose(flight_pose(1, 2), 0)
    mapping.update_pose(flight_pose(1, 2, offset=(-2, 3)), 0.1)
    assert mapping.preflight_delta == (0, 0)
    mapping.update_pose(flight_pose(1, 2, armed=True, offset=(-2, 3)), 0.2)
    mapping.update_pose(flight_pose(1, 2, armed=True, offset=(10, -8)), 0.3)
    assert mapping.drone_path == [(0, 0), (0, 0)]
    assert mapping.origin == (1, 2, 0)


def test_landing_preserves_map_and_next_arming_starts_new_flight():
    mapping = tag_stream.StartMap()
    mapping.update_pose(flight_pose(), 0)
    mapping.update_pose(flight_pose(armed=True), 0.1)
    mapping.update_pose(flight_pose(1, armed=True), 0.2)
    mapping.observe({5: (1, 0, 0)}, 0.2)
    mapping.update_pose(flight_pose(1), 0.3)
    assert mapping.drone_path == [(0, 0), (1, 0)]
    assert 5 in mapping.tags
    mapping.update_pose(flight_pose(1, armed=True), 0.4)
    assert mapping.drone_path == [(0, 0)]
    assert mapping.tags == {}


def test_reset_clears_map_artists_and_fixed_scale_does_not_change(plot):
    plot.map.update_pose(pose(), 0)
    plot.update([detection()], 0)
    plot.refresh(0)
    before = (plot.map_ax.get_xlim(), plot.map_ax.get_ylim())
    assert 5 in plot.map_artists
    plot._on_key(SimpleNamespace(key='r'))
    plot.refresh(0.2)
    assert plot.map_artists == {}
    assert len(plot.path_line.get_xdata()) == 0
    assert before == (plot.map_ax.get_xlim(), plot.map_ax.get_ylim())


def test_extended_stale_px4_data_not_drawn():
    mapping = tag_stream.StartMap()
    mapping.update_pose(flight_pose(), 0)
    mapping.update_pose(flight_pose(5, fresh=False), 0.1)
    assert mapping.preflight_delta == (0, 0)
    assert len(mapping.poses) == 1
