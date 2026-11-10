from __future__ import annotations
import argparse
import contextlib
import hashlib
import importlib.metadata
import json
import time
from datetime import datetime
from pathlib import Path
import mujoco
import mujoco.viewer
import numpy as np
from wuji_sdk import Handedness, HandModel, RetargetSession, SdkManager
ROOT = Path(__file__).resolve().parents[2]

def firmware_joint_names(side):
    prefix = 'l_' if side == 'left' else 'r_'
    names = ['thumb_cmc_flex', 'thumb_cmc_abd', 'thumb_mcp', 'thumb_ip']
    for finger in ('index_finger', 'middle_finger', 'ring_finger', 'pinky'):
        names.extend((f'{finger}_{joint}' for joint in ('mcp_flex', 'mcp_abd', 'pip', 'dip')))
    return [prefix + name for name in names]

def actuator_permutation(model, side):
    """Only adapt firmware joint ordering to XML actuator ordering, by name."""
    source = firmware_joint_names(side)
    actual = [mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, int(model.actuator_trnid[i, 0])) for i in range(model.nu)]
    if len(actual) != 20 or set(actual) != set(source):
        raise ValueError(f'Hand2 actuator names do not match SDK order: {actual}')
    return np.array([source.index(name) for name in actual], dtype=int)

def retarget_frame(session, keypoints):
    """Pass raw SDK skeleton positions directly to the official session."""
    keypoints = np.asarray(keypoints, dtype=np.float32)
    if keypoints.shape != (21, 3) or not np.isfinite(keypoints).all():
        raise ValueError('Expected finite (21, 3) skeleton in meters')
    q = np.asarray(session.step(keypoints))
    if q.shape != (20,) or not np.isfinite(q).all():
        raise ValueError('SDK returned an invalid joint command')
    return q

def latest_frame(sub):
    latest = None
    for _ in range(4096):
        frame = sub.recv()
        if frame is None:
            return latest
        latest = frame
    raise RuntimeError('Skeleton queue did not drain')
