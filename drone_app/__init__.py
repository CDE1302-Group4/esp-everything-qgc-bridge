"""Standalone desktop app; existing laptop scripts are read-only dependencies."""

from pathlib import Path
import sys

REPO_ROOT = Path(__file__).resolve().parents[1]
LAPTOP_DIR = REPO_ROOT / "laptop"
if str(LAPTOP_DIR) not in sys.path:
    sys.path.append(str(LAPTOP_DIR))
