"""Conservative mission acceptance, separate from the raw firmware detector."""

from collections import Counter
import math

from apriltag_host import EXPECTED_NCODES, GATE_MIN_MARGIN


def add_tag_arguments(parser):
    parser.add_argument("--tag-min-margin", type=float, default=80.0,
                        help="Require decode decision margin above this value (default: 80).")
    parser.add_argument("--tag-confirm-frames", type=int, default=3,
                        help="Consecutive consistent frames required per tag (default: 3, minimum: 2).")
    parser.add_argument("--tag-ids", type=int, nargs="+",
                        help="Optional allowed tag IDs; by default consider all supported IDs (0-21).")


def validate_tag_arguments(args):
    TagAcceptance(args.tag_min_margin, args.tag_confirm_frames, args.tag_ids)


class TagAcceptance:
    """Confirm exact decodes at consistent local XY across distinct camera frames.

    A missing/rejected sighting breaks the streak. A gap over 1.5 s, reboot,
    or XY change over 0.5 m from the streak's first observation starts over.
    Z is deliberately excluded because it is not measured by mission telemetry.
    """

    MAX_GAP_S = 1.5
    MAX_XY_CHANGE_M = 0.5

    def __init__(self, min_margin=80.0, confirm_frames=3, allowed_ids=None):
        if not math.isfinite(min_margin) or min_margin < GATE_MIN_MARGIN:
            raise ValueError(f"--tag-min-margin must be finite and at least {GATE_MIN_MARGIN:g}.")
        if not isinstance(confirm_frames, int) or confirm_frames < 2:
            raise ValueError("--tag-confirm-frames must be at least 2.")
        if allowed_ids is not None and any(not 0 <= i < EXPECTED_NCODES for i in allowed_ids):
            raise ValueError(f"--tag-ids must be between 0 and {EXPECTED_NCODES - 1}.")
        self.min_margin = min_margin
        self.confirm_frames = confirm_frames
        self.allowed_ids = None if allowed_ids is None else frozenset(allowed_ids)
        self._streaks = {}
        self._last_key = None
        self._last_time = None

    def _rejection(self, record):
        if not 0 <= record["tag_id"] < EXPECTED_NCODES:
            return "unsupported_id"
        if self.allowed_ids is not None and record["tag_id"] not in self.allowed_ids:
            return "id_not_allowed"
        if record["hamming"] != 0:
            return "corrected_bits"
        margin = record["decision_margin"]
        if margin is None or not math.isfinite(margin) or margin <= self.min_margin:
            return "low_decision_margin"
        if not record["quality_accepted"] or not record["pose_valid"]:
            return "invalid_quality_or_pose"
        position = record["local_position_ned_m"]
        if position is None or not all(v is not None and math.isfinite(v) for v in position[:2]):
            return "no_local_position"
        translation = record["relative_camera_translation_m"]
        if translation is None or translation[2] <= 0:
            return "pose_behind_camera"
        return None

    def process_frame(self, records, frame_key, received_at):
        """Return (journal event type, detailed observation) for every decode."""
        duplicate = frame_key == self._last_key
        if (self._last_key is not None and frame_key[0] != self._last_key[0]
                or self._last_time is not None
                and not 0 < received_at - self._last_time <= self.MAX_GAP_S):
            if not duplicate:
                self._streaks.clear()
        counts = Counter(r["tag_id"] for r in records)
        next_streaks = {}
        results = []
        for record in records:
            tag_id = record["tag_id"]
            reason = self._rejection(record)
            if duplicate:
                reason = "duplicate_frame"
            elif counts[tag_id] > 1:
                reason = "ambiguous_duplicate_id"
            count = 0
            if reason is None:
                xy = tuple(record["local_position_ned_m"][:2])
                previous_count, anchor = self._streaks.get(tag_id, (0, xy))
                if math.dist(xy, anchor) > self.MAX_XY_CHANGE_M:
                    previous_count, anchor = 0, xy
                count = min(previous_count + 1, self.confirm_frames)
                next_streaks[tag_id] = (count, anchor)
                if count < self.confirm_frames:
                    reason = "awaiting_confirmation"
            accepted = reason is None
            kind = "detection" if accepted else (
                "detection_candidate" if reason == "awaiting_confirmation" else "detection_rejected")
            results.append((kind, dict(record, mission_accepted=accepted,
                                       acceptance_reason=reason, confirmation_count=count)))
        if not duplicate:
            self._streaks = next_streaks
            self._last_key, self._last_time = frame_key, received_at
        return results
