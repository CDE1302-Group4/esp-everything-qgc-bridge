"""CSV output retains pose details and events while preserving JSONL support."""

import csv
import json
from pathlib import Path
import sys
from concurrent.futures import ThreadPoolExecutor

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from apriltag_host import Detection
from mission_tags import Journal, detection_record
from protocol import NAV_IDLE, TelemetryPacket
from simple_waypoint_mission import TelemetrySnapshot
from simple_exploration_mission import parse_args


def read_rows(path):
    with path.open(newline="", encoding="utf-8") as source:
        return list(csv.DictReader(source))


def test_csv_localization_and_metadata(tmp_path):
    path = tmp_path / "observations.csv"
    det = Detection(5, 0, 100, (160, 120), ((1, 2), (3, 4), (5, 6), (7, 8)),
                    True, (0.2, 0, 1), (1, 0, 0, 0, 1, 0, 0, 0, 1), 0.001)
    pkt = TelemetryPacket(22, 2, 3, 0, NAV_IDLE, -1, 0, [False] * 32, False, 65535)
    record = detection_record(det, TelemetrySnapshot(pkt, "drone", 10), 10.1)
    journal = Journal(path)
    try:
        journal.write("mission", waypoints_m=[(1, 2), (3, 4)], route_source='a,"b"\n.csv')
        for _ in range(2):
            journal.write("detection", drone_id=22, frame_received_utc="2026-11-04T12:00:00+00:00",
                          **record)
        journal.write("landing_command_sent", landing_confirmed=False)
        # Read before close: records are already flushed, including quoted metadata.
        rows = read_rows(path)
        assert len(rows) == 4
        assert json.loads(rows[0]["details_json"]) == {
            "waypoints_m": [[1, 2], [3, 4]], "route_source": 'a,"b"\n.csv'}
        observed = rows[1]
        assert observed["tag_id"] == "5"
        assert observed["quality_accepted"] == "True"
        assert observed["timestamp_utc"].endswith("+00:00")
        assert observed["frame_received_utc"] == "2026-11-04T12:00:00+00:00"
        assert float(observed["local_x_m"]) == pytest.approx(record["local_position_ned_m"][0])
        assert float(observed["local_y_m"]) == pytest.approx(record["local_position_ned_m"][1])
        assert observed["local_z_m"] == ""  # Unknown is not reported as zero.
        assert observed["image_corner_3_y_px"] == "8"
        assert observed["camera_r00"] == "1"
        assert float(observed["body_forward_m"]) == pytest.approx(record["relative_body_translation_m"][0])
        assert rows[2]["tag_id"] == "5"  # Repeated sightings are preserved.
        assert rows[3]["landing_confirmed"] == "False"
    finally:
        journal.close()


def test_csv_rejected_detection_has_blank_position(tmp_path):
    path = tmp_path / "rejected.CSV"
    det = Detection(7, 0, 20, (160, 120), ((1, 2),) * 4, False, None, None, None)
    journal = Journal(path)
    try:
        journal.write("detection", **detection_record(det, TelemetrySnapshot(), 0))
        row = read_rows(path)[0]
        assert row["quality_accepted"] == "False"
        assert row["quality_rejection_reason"] == "margin<=55"
        assert row["local_x_m"] == row["local_y_m"] == row["local_z_m"] == ""
        assert row["tag_id"] == "7"
    finally:
        journal.close()


@pytest.mark.parametrize("suffix", [".csv", ".jsonl"])
def test_journal_rejects_overwrite_and_nonfinite_values(tmp_path, suffix):
    path = tmp_path / ("log" + suffix)
    journal = Journal(path)
    try:
        with pytest.raises(FileExistsError):
            Journal(path)
        with pytest.raises(ValueError):
            journal.write("detection", decision_margin=float("nan"))
        journal.write("cancelled")
        if suffix == ".csv":
            assert [r["type"] for r in read_rows(path)] == ["cancelled"]
        else:
            assert json.loads(path.read_text())["type"] == "cancelled"
    finally:
        journal.close()


def test_csv_concurrent_writers_share_one_header(tmp_path):
    path = tmp_path / "concurrent.csv"
    journal = Journal(path)
    try:
        with ThreadPoolExecutor(max_workers=4) as pool:
            list(pool.map(lambda i: journal.write("detection", tag_id=i), range(80)))
        rows = read_rows(path)
        assert len(rows) == 80
        assert {int(row["tag_id"]) for row in rows} == set(range(80))
    finally:
        journal.close()


def test_exploration_defaults_to_csv():
    args = parse_args(["--drone-id", "22", "--bounds", "0", "2", "-1", "1"])
    assert args.log.suffix == ".csv"
