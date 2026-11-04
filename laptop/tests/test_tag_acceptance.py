"""False-ID filtering, including temporal resets and duplicate frames."""

import math
from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from tag_acceptance import TagAcceptance
from simple_exploration_mission import parse_args as exploration_args
from simple_waypoint_mission import parse_args as waypoint_args


def record(**overrides):
    return dict(tag_id=5, hamming=0, decision_margin=100, quality_accepted=True,
                pose_valid=True, local_position_ned_m=(1, 2, None),
                relative_camera_translation_m=(0, 0, 1)) | overrides


def frame(gate, number, records=None, nonce=1, timestamp=None):
    return gate.process_frame([record()] if records is None else records,
                              (nonce, number), number * 0.2 if timestamp is None else timestamp)


def test_three_consistent_frames_then_continued_sightings():
    gate = TagAcceptance()
    assert [frame(gate, i)[0][0] for i in range(1, 5)] == [
        "detection_candidate", "detection_candidate", "detection", "detection"]


@pytest.mark.parametrize("changes,reason", [
    ({"hamming": 1}, "corrected_bits"),
    ({"decision_margin": 80}, "low_decision_margin"),
    ({"decision_margin": math.nan}, "low_decision_margin"),
    ({"decision_margin": None}, "low_decision_margin"),
    ({"pose_valid": False}, "invalid_quality_or_pose"),
    ({"quality_accepted": False}, "invalid_quality_or_pose"),
    ({"local_position_ned_m": None}, "no_local_position"),
    ({"local_position_ned_m": (math.nan, 2, None)}, "no_local_position"),
    ({"relative_camera_translation_m": (0, 0, -1)}, "pose_behind_camera"),
    ({"tag_id": 22}, "unsupported_id"),
])
def test_bad_observations_never_confirm(changes, reason):
    gate = TagAcceptance()
    for i in range(1, 5):
        [(kind, result)] = frame(gate, i, [record(**changes)])
        assert kind == "detection_rejected"
        assert result["acceptance_reason"] == reason
        assert not result["mission_accepted"]


@pytest.mark.parametrize("interruption", ["missing", "bad", "other_id", "reboot", "gap", "time_reversal"])
def test_interrupted_streak_must_reconfirm(interruption):
    gate = TagAcceptance()
    frame(gate, 1)
    frame(gate, 2)
    nonce, start = 1, 0.6
    if interruption == "missing":
        frame(gate, 3, [])
    elif interruption == "bad":
        frame(gate, 3, [record(hamming=1)])
    elif interruption == "other_id":
        frame(gate, 3, [record(tag_id=6)])
    elif interruption == "reboot":
        nonce = 2
    elif interruption == "gap":
        start = 3.0
    else:
        start = 0.1
    for i in range(3):
        [(kind, result)] = frame(gate, 4 + i, nonce=nonce, timestamp=start + i * 0.2)
        assert kind == ("detection" if i == 2 else "detection_candidate")
        assert result["confirmation_count"] == i + 1


def test_duplicate_frame_does_not_count_or_emit_confirmed_detection():
    gate = TagAcceptance()
    frame(gate, 1)
    for _ in range(5):
        assert frame(gate, 1)[0][0] == "detection_rejected"
    assert frame(gate, 2)[0][0] == "detection_candidate"
    assert frame(gate, 3)[0][0] == "detection"
    assert frame(gate, 3)[0][0] == "detection_rejected"


def test_duplicate_id_in_one_frame_is_ambiguous():
    gate = TagAcceptance()
    for i in range(1, 5):
        results = frame(gate, i, [record(), record()])
        assert all(r["acceptance_reason"] == "ambiguous_duplicate_id" for _, r in results)


def test_xy_jump_resets_and_anchor_prevents_gradual_drift():
    gate = TagAcceptance()
    frame(gate, 1)
    frame(gate, 2, [record(local_position_ned_m=(1.3, 2, None))])
    [(kind, result)] = frame(gate, 3, [record(local_position_ned_m=(1.6, 2, None))])
    assert kind == "detection_candidate"
    assert result["confirmation_count"] == 1


def test_allowlist_and_each_id_confirm_independently():
    gate = TagAcceptance(allowed_ids=[5, 6])
    for i in range(1, 4):
        results = frame(gate, i, [record(tag_id=j) for j in (5, 6, 7)])
    assert [kind for kind, _ in results] == ["detection", "detection", "detection_rejected"]
    assert results[-1][1]["acceptance_reason"] == "id_not_allowed"


@pytest.mark.parametrize("parse,base", [
    (exploration_args, ["--drone-id", "15", "--bounds", "0", "1", "0", "1"]),
    (waypoint_args, ["--drone-id", "15", "--waypoint", "0,0"]),
])
def test_both_cli_filter_options_and_validation(parse, base):
    args = parse(base + ["--tag-min-margin", "95", "--tag-confirm-frames", "4", "--tag-ids", "5", "6"])
    assert (args.tag_min_margin, args.tag_confirm_frames, args.tag_ids) == (95, 4, [5, 6])
    for bad in (["--tag-min-margin", "nan"], ["--tag-min-margin", "54"],
                ["--tag-confirm-frames", "1"], ["--tag-ids", "22"]):
        with pytest.raises(SystemExit):
            parse(base + bad)
