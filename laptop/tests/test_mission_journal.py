"""CSV contains only tag IDs and coordinates; JSONL retains detailed records."""

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


@pytest.mark.parametrize("altitude", [None, 0.5])
def test_csv_contains_only_tag_ids_and_local_coordinates(tmp_path, altitude):
    path = tmp_path / "observations.csv"
    det = Detection(5, 0, 100, (160, 120), ((1, 2), (3, 4), (5, 6), (7, 8)),
                    True, (0.2, 0, 1), (1, 0, 0, 0, 1, 0, 0, 0, 1), 0.001)
    pkt = TelemetryPacket(22, 2, 3, 0, NAV_IDLE, -1, 0, [False] * 32, False, 65535)
    record = detection_record(det, TelemetrySnapshot(pkt, "drone", 10), 10.1, altitude)
    journal = Journal(path)
    try:
        journal.write("mission", waypoints_m=[(1, 2), (3, 4)], route_source='a,"b"\n.csv')
        for _ in range(2):
            journal.write("detection", drone_id=22, frame_received_utc="2026-11-04T12:00:00+00:00",
                          **record)
        journal.write("landing_command_sent", landing_confirmed=False)
        # Read before close: detections are flushed, mission-event rows omitted.
        rows = read_rows(path)
        assert len(rows) == 2
        observed = rows[0]
        assert list(observed) == ["tag_id", "local_x_m", "local_y_m", "local_z_m"]
        assert observed["tag_id"] == "5"
        assert float(observed["local_x_m"]) == pytest.approx(record["local_position_ned_m"][0])
        assert float(observed["local_y_m"]) == pytest.approx(record["local_position_ned_m"][1])
        if altitude is None:
            assert observed["local_z_m"] == ""  # Unknown is not reported as zero.
        else:
            assert float(observed["local_z_m"]) == pytest.approx(record["local_position_ned_m"][2])
        assert rows[1] == rows[0]  # Repeated sightings are preserved.
    finally:
        journal.close()


def test_csv_omits_rejected_and_unconfirmed_detections(tmp_path):
    path = tmp_path / "rejected.CSV"
    det = Detection(7, 0, 20, (160, 120), ((1, 2),) * 4, False, None, None, None)
    journal = Journal(path)
    try:
        for kind in ("detection_rejected", "detection_candidate"):
            journal.write(kind, **detection_record(det, TelemetrySnapshot(), 0))
        assert read_rows(path) == []
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
            journal.write("detection", tag_id=5, local_position_ned_m=(float("nan"), 1, 2))
        journal.write("cancelled")
        if suffix == ".csv":
            assert read_rows(path) == []
            assert path.read_text().strip() == "tag_id,local_x_m,local_y_m,local_z_m"
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


def test_jsonl_keeps_detailed_events_and_observations(tmp_path):
    path = tmp_path / "detailed.jsonl"
    journal = Journal(path)
    try:
        journal.write("mission", waypoints_m=[(1, 2)])
        journal.write("detection", tag_id=5, local_position_ned_m=(1, 2, None),
                      image_center_px=(160, 120), quality_accepted=True)
        journal.write("landing_command_sent", landing_confirmed=False)
        records = [json.loads(line) for line in path.read_text().splitlines()]
        assert [r["type"] for r in records] == ["mission", "detection", "landing_command_sent"]
        assert records[0]["waypoints_m"] == [[1, 2]]
        assert records[1]["image_center_px"] == [160, 120]
        assert records[1]["timestamp_utc"].endswith("+00:00")
        assert records[1]["quality_accepted"] is True
    finally:
        journal.close()
