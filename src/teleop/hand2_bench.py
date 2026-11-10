import hashlib
from pathlib import Path
import numpy as np
import mujoco
from .wuji_hand2_vendor import ROOT, actuator_permutation
from .hand2_geometry import FINGERS
METHODS=('thumb_track_b4',)
CONFIG=ROOT/'profiles/hand2_retarget_experiment.json'
MJCF=ROOT/'models/Wuji/hand2_beta1/body/mjcf/left.xml'
def file_hash(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()
class SharedSimulation:
    """Every method gets the same unchanged model, zero state and physics clock."""

    def __init__(self, hz):
        self.model = mujoco.MjModel.from_xml_path(str(MJCF))
        self.data = mujoco.MjData(self.model)
        self.command_data = mujoco.MjData(self.model)
        self.perm = actuator_permutation(self.model, 'left')
        self.ids = self.model.jnt_qposadr[self.model.actuator_trnid[:, 0].astype(int)]
        self.tip_ids = [mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, 'l_' + f + '_tip') for f in FINGERS]
        self.dt = 1 / hz
        self.clock = 0.0
        mujoco.mj_forward(self.model, self.data)
        neutral = np.vstack((np.zeros(3), self.data.site_xpos[self.tip_ids]))
        self.camera_center = (neutral.min(axis=0) + neutral.max(axis=0)) / 2

    def step(self, q):
        control = np.asarray(q)[self.perm]
        self.data.ctrl[:] = control
        self.clock += self.dt
        while self.data.time + 0.5 * self.model.opt.timestep < self.clock:
            mujoco.mj_step(self.model, self.data)
        self.command_data.qpos[self.ids] = control
        mujoco.mj_forward(self.model, self.command_data)
        return (self.data.qpos[self.ids].copy(), self.command_data.site_xpos[self.tip_ids].copy(), self.data.site_xpos[self.tip_ids].copy(), int(self.data.ncon))
