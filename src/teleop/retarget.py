"""Right Wuji 21-DoF angles to right L20's 16 active channels.

Joint definitions come from the official models; the orientation fitting and
coupled-angle projection are this project's cross-hand retargeting policy.
"""
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation

from .mapping import CONTROLS, MANUAL_RANGES, manual_model_joints, map_angle, raw_endpoints

WUJI_URDF = Path(__file__).resolve().parents[2] / "models/Wuji/right.urdf"
SOURCE_JOINTS = (
    ("thumb_cmc_rot", "thumb_cmc_flex", "thumb_cmc_abd", "thumb_mcp", "thumb_ip"),
    *[(f"{f}_mcp_flex", f"{f}_mcp_abd", f"{f}_pip", f"{f}_dip")
      for f in ("index_finger", "middle_finger", "ring_finger", "pinky")],
)


def _origin_rotation(joint):
    origin = joint.find("origin")
    rpy = "0 0 0" if origin is None else origin.get("rpy", "0 0 0")
    return Rotation.from_euler("xyz", np.fromstring(rpy, sep=" ")).as_matrix()


class WujiToL20:
    def __init__(self, capabilities):
        self.joints = manual_model_joints()
        source = {j.get("name"): j for j in ET.parse(WUJI_URDF).findall("joint")}
        source_order = tuple(j.get("name") for j in source.values() if j.get("type") == "revolute")
        if source_order != tuple(n for finger in SOURCE_JOINTS for n in finger):
            raise ValueError("Unsupported Wuji right-hand model joint order")
        self.names = capabilities.joint_names
        if set(self.names) != {name for _, name in CONTROLS}:
            raise ValueError("Expected the 16 named L20 channels")
        self.endpoints = {j.name: raw_endpoints(j) for j in capabilities.joints}
        self.limits = {name: tuple(float(self.joints[joint].find("limit").get(k))
                                  for k in ("lower", "upper")) for joint, name in CONTROLS}
        self._source_cmc = self._chain(source, SOURCE_JOINTS[0][:3])
        self._target_cmc = self._chain(self.joints, [j for j, _ in CONTROLS[:3]])
        # Anatomical local-frame association: human flexion X -> robot Y,
        # human distal -Z -> robot +Z. This is not a factory calibration.
        self._basis = np.array([[0., 1., 0.], [1., 0., 0.], [0., 0., -1.]])
        source_zero = self._rotation(self._source_cmc, np.zeros(3))
        target_zero = self._rotation(self._target_cmc, np.zeros(3))
        self._alignment = target_zero @ self._basis @ source_zero.T
        self._thumb = np.zeros(3)
        self._bounds = np.array([self.limits[name] for _, name in CONTROLS[:3]]).T

    @staticmethod
    def _chain(joints, names):
        return [(_origin_rotation(joints[name]),
                 np.fromstring(joints[name].find("axis").get("xyz"), sep=" ")) for name in names]

    @staticmethod
    def _rotation(chain, angles):
        result = np.eye(3)
        for (origin, axis), angle in zip(chain, angles, strict=True):
            result = result @ origin @ Rotation.from_rotvec(axis / np.linalg.norm(axis) * angle).as_matrix()
        return result

    def _coupled(self, master, slave, first, second):
        mimic = self.joints[slave].find("mimic")
        if mimic is None or mimic.get("joint") != master:
            raise ValueError(f"Expected {slave} to mimic {master}")
        k, offset = float(mimic.get("multiplier", "1")), float(mimic.get("offset", "0"))
        if k <= 0:
            raise ValueError("Expected a positive L20 mimic multiplier")
        # Minimize (q-first)^2 + (k*q+offset-second)^2, then constrain both joints.
        q = (first + k * (second - offset)) / (1 + k*k)
        limit, slave_limit = self.joints[master].find("limit"), self.joints[slave].find("limit")
        lower = max(float(limit.get("lower")), (float(slave_limit.get("lower"))-offset)/k)
        upper = min(float(limit.get("upper")), (float(slave_limit.get("upper"))-offset)/k)
        return float(np.clip(q, lower, upper))

    def joint_angles(self, frame, min_confidence=0.5):
        if not math.isfinite(min_confidence) or not 0 <= min_confidence <= 1:
            raise ValueError("min_confidence must be within [0, 1]")
        if len(frame.fingers) != 5:
            raise ValueError("Expected five Wuji fingers in thumb-to-pinky order")
        fingers = []
        for i, finger in enumerate(frame.fingers):
            if not math.isfinite(finger.confidence) or finger.confidence < min_confidence:
                raise ValueError(f"Wuji finger {i} confidence below threshold")
            values = list(finger.angles)
            if len(values) != 5 or not all(math.isfinite(x) for x in values):
                raise ValueError("Expected five finite Wuji angle slots per finger")
            if i and abs(values[4]) > 1e-7:
                raise ValueError("Non-thumb Wuji slot 4 must be the zero placeholder")
            fingers.append(values)
        desired = self._alignment @ self._rotation(self._source_cmc, fingers[0][:3]) @ self._basis.T
        previous = self._thumb.copy()

        def residual(q):
            rotation_error = Rotation.from_matrix(
                self._rotation(self._target_cmc, q).T @ desired).as_rotvec()
            # Select a continuous solution near singular/ambiguous CMC poses.
            return np.concatenate((rotation_error, 1e-4 * (q - previous)))

        if np.linalg.norm(residual(previous)[:3]) > 1e-8:
            # A trust-region step starting exactly at all lower bounds can
            # terminate before moving; start slightly inside the feasible box.
            initial = np.clip(previous, self._bounds[0]+1e-3, self._bounds[1]-1e-3)
            fit = least_squares(residual, initial, bounds=self._bounds, max_nfev=100,
                                ftol=1e-8, xtol=1e-8, gtol=1e-8)
            if not fit.success or not np.all(np.isfinite(fit.x)):
                raise RuntimeError("Thumb CMC orientation fit did not converge")
            self._thumb = fit.x
        target = dict(zip((name for _, name in CONTROLS[:3]), self._thumb.tolist(), strict=True))
        target["thumb_tip"] = self._coupled("thumb_mcp", "thumb_dip", *fingers[0][3:5])
        for finger, values in zip(("index", "middle", "ring", "pinky"), fingers[1:], strict=True):
            target[f"{finger}_root"] = values[0]
            target[f"{finger}_abd"] = values[1]
            target[f"{finger}_tip"] = self._coupled(f"{finger}_pip", f"{finger}_dip", *values[2:4])
        return {name: float(np.clip(target[name], *self.limits[name])) for name in self.names}

    def map(self, frame, min_confidence=0.5):
        angles = self.joint_angles(frame, min_confidence)
        return {name: map_angle(angles[name], *MANUAL_RANGES[name], *self.endpoints[name])
                for name in self.names}
