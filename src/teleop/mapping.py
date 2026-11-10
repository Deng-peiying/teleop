"""Named L20 controls and raw conversion shared by glove and simulation."""
import math
import xml.etree.ElementTree as ET
from pathlib import Path

L20_URDF = Path(__file__).resolve().parents[2] / "models/L20/right/l20_right_manual.urdf"
CONTROLS = [
    ("thumb_cmc_roll", "thumb_roll"), ("thumb_cmc_yaw", "thumb_rotation"),
    ("thumb_cmc_pitch", "thumb_root"), ("thumb_mcp", "thumb_tip"),
    *[(f"{finger}_{joint}", f"{finger}_{label}")
      for finger in ("index", "middle", "ring", "pinky")
      for joint, label in (("mcp_roll", "abd"), ("mcp_pitch", "root"), ("pip", "tip"))],
]


# User-facing manual names keyed by the official SDK channel.
DISPLAY_NAMES = {channel: joint for joint, channel in CONTROLS}


def manual_model_joints():
    return {j.get("name"): j for j in ET.parse(L20_URDF).findall("joint")}



_joints = manual_model_joints()
MANUAL_RANGES = {
    channel: tuple(float(_joints[joint].find("limit").get(k)) for k in ("lower", "upper"))
    for joint, channel in CONTROLS
}


def raw_endpoints(joint):
    """Nominal physical lower/upper to SDK percent endpoints used by teleop."""
    mapping = joint.percent_mapping
    if mapping is None:
        raise ValueError(f"No SDK endpoint mapping for {joint.name}")
    return mapping.raw_at_hundred, mapping.raw_at_zero


def map_angle(angle, open_angle, bent_angle, raw_open, raw_bent):
    values = (angle, open_angle, bent_angle, raw_open, raw_bent)
    if not all(math.isfinite(v) for v in values):
        raise ValueError("Calibration and input values must be finite")
    if abs(bent_angle - open_angle) < 1e-6:
        raise ValueError("Open and bent samples must differ")
    if not all(0 <= v <= 255 for v in (raw_open, raw_bent)):
        raise ValueError("L20 raw endpoints must be within 0..255")
    fraction = max(0.0, min(1.0, (angle-open_angle)/(bent_angle-open_angle)))
    return round(raw_open + fraction*(raw_bent-raw_open))
