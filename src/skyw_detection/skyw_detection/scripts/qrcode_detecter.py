#!/usr/bin/env python3
"""Legacy launcher: prefer `ros2 run skyw_detection qrcode_detector` after `colcon build`."""

from __future__ import annotations

import sys
from pathlib import Path

_pkg_root = Path(__file__).resolve().parents[2]
if str(_pkg_root) not in sys.path:
    sys.path.insert(0, str(_pkg_root))

from skyw_detection.python.qr_code_detector_node import main

if __name__ == "__main__":
    main()
