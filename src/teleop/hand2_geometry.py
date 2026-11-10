from __future__ import annotations
import numpy as np
FINGERS = ('thumb', 'index_finger', 'middle_finger', 'ring_finger', 'pinky')
KP = np.array([[1, 2, 3, 4], [5, 6, 7, 8], [9, 10, 11, 12], [13, 14, 15, 16], [17, 18, 19, 20]])

def unit(v):
    return v / max(float(np.linalg.norm(v)), 1e-10)

def palm_basis(points):
    """Right-handed frame using wrist and MCPs; invariant to camera orientation."""
    along = unit(points[9] - points[0])
    across = points[5] - points[17]
    across = unit(across - np.dot(across, along) * along)
    if np.linalg.norm(np.cross(across, along)) < 0.5:
        raise ValueError('Degenerate hand palm frame')
    return np.column_stack([across, along, unit(np.cross(across, along))])

def point_segment(p, a, b):
    v = b - a
    t = np.clip(np.dot(p - a, v) / max(np.dot(v, v), 1e-14), 0, 1)
    return (a + t * v, float(t))

def segment_segment(a, b, c, d):
    """Closest points on finite segments, including parallel/zero-length cases."""
    u, v, w = (b - a, d - c, a - c)
    aa, bb, cc = (np.dot(u, u), np.dot(u, v), np.dot(v, v))
    dd, ee = (np.dot(u, w), np.dot(v, w))
    if aa < 1e-14:
        q, t = point_segment(a, c, d)
        return (a, q, 0.0, t)
    if cc < 1e-14:
        p, s = point_segment(c, a, b)
        return (p, c, s, 0.0)
    denom = aa * cc - bb * bb
    s = np.clip((bb * ee - cc * dd) / denom, 0, 1) if denom > 1e-14 else 0.0
    t = (bb * s + ee) / cc
    if t < 0:
        t, s = (0.0, np.clip(-dd / aa, 0, 1))
    elif t > 1:
        t, s = (1.0, np.clip((bb - dd) / aa, 0, 1))
    return (a + s * u, c + t * v, float(s), float(t))

def arc_point(points, s):
    lengths = np.linalg.norm(np.diff(points, axis=0), axis=1)
    total = max(lengths.sum(), 1e-10)
    position = np.clip(s, 0, 1) * total
    index = min(int(np.searchsorted(np.cumsum(lengths), position, side='right')), 2)
    t = np.clip((position - lengths[:index].sum()) / max(lengths[index], 1e-10), 0, 1)
    return (points[index] * (1 - t) + points[index + 1] * t, index, float(t))

def nearest_arc(p, points, previous=None, continuity=0.0):
    lengths = np.linalg.norm(np.diff(points, axis=0), axis=1)
    total = max(lengths.sum(), 1e-10)
    candidates = []
    for j in range(3):
        closest, t = point_segment(p, points[j], points[j + 1])
        s = (lengths[:j].sum() + t * lengths[j]) / total
        distance = np.linalg.norm(p - closest)
        penalty = continuity * (s - previous) ** 2 if previous is not None else 0.0
        candidates.append((distance ** 2 + penalty, s, closest, distance))
    _, s, closest, distance = min(candidates, key=lambda x: x[0])
    return (float(s), closest, float(distance))

def frame_names(side='left', anatomical_base=False):
    p = 'l_' if side == 'left' else 'r_'
    return [[p + f + ('_proximal' if anatomical_base and i > 0 else '_proximal_abd'), p + f + '_middle', p + f + '_distal', p + f + '_tip'] for i, f in enumerate(FINGERS)]

class Morphology:
    """Freeze per-segment length adaptation from input, independent of any solver."""

    def __init__(self, robot, sample_keypoints, scales=None, anatomical_base=False):
        self.names = frame_names(anatomical_base=anatomical_base)
        self.anatomical_base = anatomical_base
        robot.compute_forward_kinematics(np.zeros(robot.dof))
        self.robot_points = np.array([[robot.get_link_pose(robot.get_link_index(n))[:3, 3] for n in finger] for finger in self.names])
        reference = np.zeros((21, 3))
        for i in range(5):
            reference[KP[i]] = self.robot_points[i]
        self.robot_basis = palm_basis(reference)
        lengths = np.linalg.norm(np.diff(self.robot_points, axis=1), axis=2)
        self.lengths = lengths
        human_width = np.median(np.linalg.norm(sample_keypoints[:, 5] - sample_keypoints[:, 17], axis=1))
        robot_width = np.linalg.norm(reference[5] - reference[17])
        self.palm_scale = float(np.clip(robot_width / max(human_width, 1e-06), 0.5, 2.0))
        self.scales = np.ones((5, 3)) if scales is None else np.asarray(scales, float)
        if self.scales.shape != (5, 3) or np.any(self.scales <= 0):
            raise ValueError('segment scale must be positive (5,3)')

    def align(self, kp):
        return (kp - kp[0]) @ palm_basis(kp) @ self.robot_basis.T

    def adapt(self, kp):
        aligned = self.align(kp)
        result = aligned * self.palm_scale
        for f in range(5):
            if self.anatomical_base and f > 0:
                result[KP[f, 0]] = self.robot_points[f, 0]
            for j in range(3):
                delta = aligned[KP[f, j + 1]] - aligned[KP[f, j]]
                result[KP[f, j + 1]] = result[KP[f, j]] + unit(delta) * self.lengths[f, j] * self.scales[f, j]
        return result
