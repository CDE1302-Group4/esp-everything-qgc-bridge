"""Height/status protocol and trajectory gating without sockets or flight hardware."""
from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock
import struct
import sys

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import send_trajectory as sender
from protocol import (TelemetryPacket, MISSION_READY, MISSION_FLYING, MISSION_BUSY,
                      NAV_IDLE, NAV_TRAJ, NAV_ARRIVED, CMD_START, CMD_LAND, parse_telemetry)


def packet(**changes):
    p = TelemetryPacket(15, 0, 0, 0, NAV_IDLE, -1, 0, [False]*32, False, 65535,
                        flight_status_version=1, local_z=-0.2, local_vz=0,
                        ground_z=-0.2, px4_armed=0, px4_custom_main_mode=0,
                        mission_phase=MISSION_READY, px4_pos_age_ms=0, px4_hb_age_ms=0,
                        map_offset_x=0, map_offset_y=0)
    return replace(p, **changes)


def test_wire_extension_and_legacy():
    prefix = struct.pack('<BBfffBbf32sBH', 1, 15, 1, 2, 0, 0, -1, 0, bytes(32), 0, 65535)
    suffix = struct.pack('<4sfffBBBIIff', b'TF01', -0.7, 0.02, -0.2, 1, 6, 2, 20, 150, -2, 3)
    p = parse_telemetry(prefix + suffix)
    assert len(prefix + suffix) == 90
    assert p.local_z == pytest.approx(-0.7)
    assert p.ground_z - p.local_z == pytest.approx(0.5)
    assert (p.px4_armed, p.mission_phase, p.map_offset_x, p.map_offset_y) == (1, 2, -2, 3)
    assert p.px4_link_ok
    assert parse_telemetry(prefix).flight_status_version is None
    assert parse_telemetry(prefix + suffix[:10]).flight_status_version is None
    assert parse_telemetry(prefix + suffix.replace(b'TF01', b'TF99')).flight_status_version is None


@pytest.mark.parametrize('changes,match', [
    ({'flight_status_version': 99}, 'Unsupported'),
    ({'px4_pos_age_ms': 600}, 'stale'),
    ({'px4_hb_age_ms': 2100}, 'stale'),
    ({'local_z': float('nan')}, 'invalid'),
])
def test_rejects_unusable_feedback(changes, match):
    feedback = sender.FlightFeedback(15)
    feedback.update(packet(**changes), 'drone')
    with pytest.raises((RuntimeError, ConnectionError), match=match):
        feedback.packet()


def legacy_packet(**changes):
    raw = struct.pack('<BBfffBbf32sBH', 1, 15, 0, 0, 0, NAV_IDLE, -1, 0, bytes(32), 0, 65535)
    return replace(parse_telemetry(raw), **changes)


def test_legacy_feedback_accepts_only_available_fields_and_rejects_format_changes():
    feedback = sender.FlightFeedback(15)
    old = legacy_packet()
    feedback.update(old, 'drone')
    assert feedback.packet() is old
    assert old.local_z is None and old.px4_armed is None
    feedback.update(legacy_packet(ned_x=float('nan')), 'drone')
    with pytest.raises(RuntimeError, match='invalid'):
        feedback.packet()
    feedback.update(packet(), 'drone')
    with pytest.raises(RuntimeError, match='format changed'):
        feedback.packet()


def test_extended_feedback_never_silently_downgrades_to_legacy():
    feedback = sender.FlightFeedback(15)
    feedback.update(packet(), 'drone')
    feedback.packet()
    feedback.update(legacy_packet(), 'drone')
    with pytest.raises(RuntimeError, match='format changed'):
        feedback.packet()


def test_stale_laptop_packet_and_wrong_drone(monkeypatch):
    now = [0.0]
    monkeypatch.setattr(sender, 'time', SimpleNamespace(monotonic=lambda: now[0]))
    feedback = sender.FlightFeedback(15)
    feedback.update(packet(drone_id=14), 'other')
    with pytest.raises(ConnectionError):
        feedback.packet()
    feedback.update(packet(), 'drone')
    now[0] = 1.1
    with pytest.raises(ConnectionError, match='stale'):
        feedback.packet()


def test_hover_uses_ground_relative_height_and_phase():
    p = packet(px4_armed=1, px4_custom_main_mode=6, mission_phase=MISSION_FLYING, local_z=-0.7)
    assert sender.at_hover(p, 0.5)
    assert not sender.at_hover(replace(p, local_z=-0.5), 0.5)
    assert not sender.at_hover(replace(p, local_vz=-0.2), 0.5)
    assert not sender.at_hover(replace(p, mission_phase=MISSION_BUSY), 0.5)


@pytest.mark.parametrize('rows', ['0,0,0,nan\n1,0,0,0', '0,0,0,0\n1,0,0,inf',
                                 '0,0,0,0\n1,0,bad,0', '0,0,0,0\n0,0,0,0',
                                 '0,0,0,0,5\n1,0,0,0,5'])
def test_invalid_csv_not_silently_dropped(tmp_path, rows):
    path = tmp_path / 'bad.csv'
    path.write_text('t,x,y,z\n' + rows)
    with pytest.raises(ValueError):
        sender.load_trajectory(path, 0.05)


def test_constant_height_and_partial_endpoint(tmp_path):
    path = tmp_path / 'height.csv'
    path.write_text('t,x,y,z\n0,0,0,-1\n0.12,0.02,0,-1\n')
    p = sender.load_trajectory(path, 0.05)
    assert len(p) == 4
    assert np.all(p[:, 2] == 0)
    assert p[-1, 0] == 0.02


@pytest.mark.parametrize('failure', [None, 'wrong_height', 'stale', 'takeover', 'stalled_endpoint'])
def test_sender_flight_flow(tmp_path, monkeypatch, failure):
    path = tmp_path / 'route.csv'
    path.write_text('t,x,y,z\n0,0,0,0\n2,0.1,0,0\n')
    now = [0.0]
    commands, uploads = [], []
    state = {'start': None, 'traj': None}
    comms = Mock()
    callback = [None]
    def deliver():
        p = packet()
        if state['start'] is not None:
            elapsed = now[0] - state['start']
            if failure == 'stale' and elapsed > 0.2:
                return
            p = replace(p, px4_armed=1, px4_custom_main_mode=6,
                        mission_phase=MISSION_FLYING if elapsed > 1 else MISSION_BUSY,
                        local_z=-0.5 if failure == 'wrong_height' else -0.7)
            if failure == 'takeover' and elapsed > 2:
                p = replace(p, px4_custom_main_mode=3)
        if state['traj'] is not None:
            done = now[0] - state['traj'] >= 2
            p = replace(p, nav_state=NAV_ARRIVED if done else NAV_TRAJ,
                        ned_x=0.1 if done and failure != 'stalled_endpoint' else 0)
            if done and failure == 'stalled_endpoint':
                p = replace(p, ned_x=-1)
        callback[0](p, 'drone')
    def sleep(seconds):
        now[0] += seconds
        deliver()
    def send_command(_id, command):
        commands.append(command.cmd_type)
        if command.cmd_type == CMD_START:
            state['start'] = now[0]
        deliver()
        return True
    def send_raw(_id, data):
        uploads.append(data[1])
        if data[1] == 9:
            assert now[0] - state['start'] >= 2  # confirmed hover, not a blind timer
            state['traj'] = now[0]
        deliver()
        return True
    comms.on_telemetry.side_effect = lambda cb: callback.__setitem__(0, cb)
    comms.start.side_effect = deliver
    comms.send_command.side_effect = send_command
    comms.send_raw.side_effect = send_raw
    monkeypatch.setattr(sender, 'CommsNode', lambda **kw: comms)
    monkeypatch.setattr(sender, 'time', SimpleNamespace(monotonic=lambda: now[0], sleep=sleep))
    args = ['--drone-id', '15', '--trajectory', str(path), '--takeoff',
            '--takeoff-wait', '0', '--takeoff-timeout', '5']
    if failure:
        with pytest.raises((RuntimeError, ConnectionError, TimeoutError)):
            sender.main(args)
        assert commands[-3:] == [CMD_LAND]*3
        if failure != 'stalled_endpoint':
            assert 9 not in uploads
    else:
        sender.main(args)
        assert commands == [CMD_START, CMD_LAND, CMD_LAND, CMD_LAND]
        assert uploads == [8, 9]
    comms.stop.assert_called_once()


@pytest.mark.parametrize('failure,takeoff,finish', [
    (None, True, 'land'),
    (None, False, 'hold'),
    ('takeoff_stale', True, 'land'),
    ('playback_stale', True, 'land'),
    ('no_start', True, 'land'),
    ('interrupted', True, 'land'),
    ('stalled_endpoint', True, 'land'),
    ('format_change', True, 'land'),
])
def test_legacy_sender_flight_flow(tmp_path, monkeypatch, caplog, failure, takeoff, finish):
    path = tmp_path / 'legacy.csv'
    # Include vertical motion: old firmware plays Z offsets even though it
    # cannot report Z back. It must not be replaced with an invented height.
    path.write_text('t,x,y,z\n0,0,0,0\n2,0.1,0,-0.2\n')
    now = [0.0]
    commands, uploads = [], []
    state = {'start': None, 'traj': None}
    callback = [None]
    comms = Mock()

    def deliver():
        if (failure == 'takeoff_stale' and state['start'] is not None
                and now[0] - state['start'] > 0.2):
            return
        p = legacy_packet()
        if failure == 'format_change' and state['start'] is not None:
            p = packet()
        if state['traj'] is not None:
            elapsed = now[0] - state['traj']
            if failure == 'playback_stale' and elapsed > 0.2:
                return
            done = elapsed >= 2
            p = replace(p, nav_state=NAV_ARRIVED if done else NAV_TRAJ,
                        ned_x=0.1 if done and failure != 'stalled_endpoint' else 0)
            if done and failure == 'stalled_endpoint':
                p = replace(p, ned_x=-1)
            if failure == 'interrupted' and elapsed > 0.2:
                p = replace(p, nav_state=NAV_IDLE)
        callback[0](p, 'drone')

    def sleep(seconds):
        now[0] += seconds
        deliver()

    def send_command(_id, command):
        commands.append(command.cmd_type)
        if command.cmd_type == CMD_START:
            state['start'] = now[0]
        deliver()
        return True

    def send_raw(_id, data):
        uploads.append(data[1])
        if data[1] == 9:
            if takeoff:
                assert now[0] - state['start'] >= 12  # default compatibility wait
            if failure != 'no_start':
                state['traj'] = now[0]
        deliver()
        return True

    comms.on_telemetry.side_effect = lambda cb: callback.__setitem__(0, cb)
    comms.start.side_effect = deliver
    comms.send_command.side_effect = send_command
    comms.send_raw.side_effect = send_raw
    monkeypatch.setattr(sender, 'CommsNode', lambda **kw: comms)
    monkeypatch.setattr(sender, 'time', SimpleNamespace(monotonic=lambda: now[0], sleep=sleep))
    monkeypatch.setattr('builtins.input', lambda prompt: 'TRAJ-15')
    args = ['--drone-id', '15', '--trajectory', str(path), '--confirm', '--finish', finish]
    if takeoff:
        args += ['--takeoff']
    if failure:
        with pytest.raises((RuntimeError, ConnectionError, TimeoutError)):
            sender.main(args)
        assert commands[-3:] == [CMD_LAND] * 3
        if failure in ('takeoff_stale', 'format_change'):
            assert 9 not in uploads
    else:
        sender.main(args)
        from protocol import CMD_HOLD
        expected = [CMD_START] if takeoff else []
        expected += [CMD_LAND] * 3 if finish == 'land' else [CMD_HOLD]
        assert commands == expected
        assert uploads == [8, 9]
        assert 'Legacy telemetry detected' in caplog.text
    comms.stop.assert_called_once()


def test_dry_run_never_connects(tmp_path, monkeypatch):
    path = tmp_path / 'route.csv'
    path.write_text('0,0,0,0\n1,0.1,0,0\n')
    comms = Mock(side_effect=AssertionError('must not connect'))
    monkeypatch.setattr(sender, 'CommsNode', comms)
    sender.main(['--drone-id', '15', '--trajectory', str(path), '--dry-run'])
    comms.assert_not_called()
