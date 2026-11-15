import unittest
import numpy as np
import mujoco
from teleop.mujoco_model import build_control_model, display_targets

class ArmModelTest(unittest.TestCase):
    def test_seven_axis_degree_targets_without_hand(self):
        model=build_control_model(preview=True)
        self.assertEqual(model.nu,7)
        self.assertEqual(model.nq,7)
        data=mujoco.MjData(model)
        data.ctrl[:]=np.mean(model.actuator_ctrlrange,axis=1)
        display_targets(model,data)
        for i in range(7):
            self.assertAlmostEqual(data.qpos[model.jnt_qposadr[model.actuator_trnid[i,0]]],np.deg2rad(data.ctrl[i]))
        self.assertFalse(any('l20' in model.body(i).name.lower() for i in range(model.nbody)))
