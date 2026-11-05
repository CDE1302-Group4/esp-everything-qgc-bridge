import struct
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from protocol import parse_telemetry


def test_parse_telemetry_includes_px4_status_fields() -> None:
    raw = struct.pack(
        "<BBfffBbf32sBH",
        0x01,
        2,
        1.0,
        2.0,
        0.3,
        0,
        -1,
        0.5,
        b"\x00" * 32,
        0,
        0,
    )
    raw += struct.pack("<4sfffBBBIIff", b"TF01", -0.7, 0.0, -0.2,
                       1, 6, 2, 0, 0, 0.0, 0.0)

    pkt = parse_telemetry(raw)
    assert pkt is not None
    assert pkt.px4_link_ok is True
    assert pkt.px4_hb_age_ms == 0
    assert pkt.px4_armed == 1
    assert pkt.px4_custom_main_mode == 6
