"""Exercise the app's separate flight runner, including its stop semantics."""

from dataclasses import replace
from pathlib import Path
import sys
from types import SimpleNamespace
from unittest.mock import Mock

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from drone_app import flight
from protocol import (CMD_START, CMD_LAND, CMD_HOLD, MISSION_READY, MISSION_BUSY,
                      MISSION_FLYING, NAV_IDLE, NAV_TRAJ, NAV_ARRIVED, TelemetryPacket)


@pytest.mark.parametrize("extended,failure", [
    (False, None), (True, None),
    (False, "stale"), (True, "stale"),
    (False, "no_start"), (True, "no_start"),
    (False, "bad_endpoint"), (True, "bad_endpoint"),
    (False, "hold"), (True, "hold"),
    (True, "wrong_height"), (True, "takeover"),
])
def test_app_flight_workflow(monkeypatch, extended, failure):
    now = [0.0]
    state = {"start": None, "traj": None, "interrupted": False}
    comms = Mock()
    control = flight.FlightControl()
    feedback = flight.FlightFeedback(4)
    commands, raw_types = [], []

    def publish():
        p = TelemetryPacket(4, 0, 0, 0, NAV_IDLE, -1, 0, [False] * 32, False, 65535)
        if extended:
            p = replace(p, flight_status_version=1, local_z=-.2, local_vz=0,
                        ground_z=-.2, px4_armed=0, px4_custom_main_mode=0,
                        mission_phase=MISSION_READY, px4_pos_age_ms=0, px4_hb_age_ms=0,
                        map_offset_x=0, map_offset_y=0)
        if state["start"] is not None:
            since_start = now[0] - state["start"]
            if failure == "stale" and since_start > .2:
                return
            if extended:
                p = replace(p, px4_armed=1, px4_custom_main_mode=6,
                            mission_phase=MISSION_FLYING if since_start > 1 else MISSION_BUSY,
                            local_z=-.5 if failure == "wrong_height" else -.7)
                if failure == "takeover" and since_start > 2:
                    p = replace(p, px4_custom_main_mode=3)
        if state["traj"] is not None:
            elapsed = now[0] - state["traj"]
            done = elapsed >= 2
            p = replace(p, nav_state=NAV_ARRIVED if done else NAV_TRAJ,
                        ned_x=(-1 if failure == "bad_endpoint" else .1) if done else 0)
            if failure == "hold" and elapsed > .2 and not state["interrupted"]:
                state["interrupted"] = True
                control.interrupt(comms, 4, CMD_HOLD)
        feedback.update(p, "drone")

    def sleep(seconds):
        now[0] += seconds
        publish()

    def send_command(_id, command):
        commands.append(command.cmd_type)
        if command.cmd_type == CMD_START:
            state["start"] = now[0]
        publish()
        return True

    def send_raw(_id, data):
        raw_types.append(data[1])
        if data[1] == 9:
            assert now[0] - state["start"] >= 2
            if failure != "no_start":
                state["traj"] = now[0]
        publish()
        return True

    comms.send_command.side_effect = send_command
    comms.send_raw.side_effect = send_raw
    monkeypatch.setattr(flight, "time", SimpleNamespace(monotonic=lambda: now[0], sleep=sleep))
    publish()
    points = np.column_stack((np.linspace(0, .1, 41), np.zeros((41, 2))))
    options = flight.FlightOptions(4, 50, True, 2, 6, "land")
    if failure == "hold":
        with pytest.raises(flight.FlightCancelled):
            flight.run_flight(comms, feedback, points, options, control=control)
        assert commands == [CMD_START, CMD_HOLD]
    elif failure:
        with pytest.raises((RuntimeError, TimeoutError, ConnectionError)):
            flight.run_flight(comms, feedback, points, options, control=control)
        assert commands[-3:] == [CMD_LAND] * 3
    else:
        flight.run_flight(comms, feedback, points, options, control=control)
        assert commands == [CMD_START, CMD_LAND, CMD_LAND, CMD_LAND]
        assert raw_types == [8, 9]
