"""Regression checks for the shared arm/hand mesh basename."""
import unittest

import mujoco
import numpy as np

from teleop.mujoco_model import build_control_model, display_targets


class CombinedModelTest(unittest.TestCase):
    def test_both_base_meshes_survive_import(self):
        hand = build_control_model("hand")
        combined = build_control_model("arm-hand")
        self.assertEqual(combined.nmesh, hand.nmesh + 8)
        self.assertGreaterEqual(combined.mesh("rm75_base_link").id, 0)
        for i in range(hand.nmesh):
            name = hand.mesh(i).name
            j = combined.mesh(name).id
            a = hand.mesh_vert[hand.mesh_vertadr[i]:hand.mesh_vertadr[i] + hand.mesh_vertnum[i]]
            b = combined.mesh_vert[combined.mesh_vertadr[j]:combined.mesh_vertadr[j] + combined.mesh_vertnum[j]]
            np.testing.assert_allclose(a, b, atol=1e-7)

    def test_hand_kinematics_survive_mount(self):
        hand = build_control_model("hand")
        combined = build_control_model("arm-hand")
        a, b = mujoco.MjData(hand), mujoco.MjData(combined)
        a.ctrl[:] = (hand.actuator_ctrlrange[:, 0] + hand.actuator_ctrlrange[:, 1]) / 2
        b.ctrl[7:] = a.ctrl
        display_targets(hand, a)
        display_targets(combined, b)
        yaw = np.radians(160)
        mount = np.array([[np.cos(yaw), -np.sin(yaw), 0],
                          [np.sin(yaw), np.cos(yaw), 0], [0, 0, 1]])
        flange = b.body("Link7")
        rotation = flange.xmat.reshape(3, 3) @ mount
        for i in range(1, hand.nbody):
            name = hand.body(i).name
            np.testing.assert_allclose(
                b.body("l20_" + name).xpos,
                flange.xpos + rotation @ a.body(name).xpos, atol=1e-7)


if __name__ == "__main__":
    unittest.main()
