"""Load the existing detector source with an app-private compiled-library cache."""

import importlib.util
from pathlib import Path
import shutil
import sys

from . import LAPTOP_DIR

_name = "drone_app._host_detector"
_spec = importlib.util.spec_from_file_location(_name, LAPTOP_DIR / "apriltag_host.py")
_host = importlib.util.module_from_spec(_spec)
sys.modules[_name] = _host
_spec.loader.exec_module(_host)
_host.BUILD_DIR = Path(__file__).resolve().parent / ".cache" / "apriltag-host"

class Detector(_host.Detector):
    def __init__(self, *args, **kwargs):
        # Seed from the working viewer's cache by reading/copying only. The
        # normal source fingerprint still checks whether the copy is current.
        if not (_host.BUILD_DIR / _host._LIB_NAME).exists():
            original = LAPTOP_DIR / ".apriltag-host"
            if (original / _host._LIB_NAME).is_file():
                _host.BUILD_DIR.mkdir(parents=True, exist_ok=True)
                for name in (_host._LIB_NAME, _host._STAMP_NAME):
                    if (original / name).is_file():
                        shutil.copy2(original / name, _host.BUILD_DIR / name)
        super().__init__(*args, **kwargs)


Detection = _host.Detection
CAM_POS_M = _host.CAM_POS_M
R_BODY_CAM = _host.R_BODY_CAM
passes_gate = _host.passes_gate
