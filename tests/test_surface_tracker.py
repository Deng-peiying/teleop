import unittest
from types import SimpleNamespace

import numpy as np

from teleop.hand2_vendor_side import HandSurfaces
from teleop.surface_tracker import TrackedSurfacePatch, ContinuousSurfacePatch


class SurfaceTrackerTest(unittest.TestCase):
    def make_tracker(self):
        s=HandSurfaces();m=s.model
        method=SimpleNamespace(surface=s,thumb_bodies=[s.pad_body],
            index_bodies=[m.body('l_index_finger_'+n).id for n in ('proximal_abd','middle','distal')],
            thumb_tip=m.site('l_thumb_tip').id,index_tip=m.site('l_index_finger_tip').id,
            dt=1/30,morphology=SimpleNamespace(palm_scale=1.))
        return TrackedSurfacePatch(method)

    def test_stationary_contact_stays_active_and_release_clears_history(self):
        tracker=self.make_tracker()
        kp=np.zeros((21,3));rot=np.repeat(np.eye(3)[None],21,axis=0)
        for f in range(4):
            kp[5+4*f:9+4*f]=[[-.035*f,0,z] for z in (-.08,-.11,-.14,-.17)]
        kp[3:5]=[[.008,0,-.12],[.008,0,-.14]]
        for _ in range(8):
            tracker.prepare(kp,rot);tracker.finalize()
        self.assertGreater(tracker.strength,.1)
        self.assertTrue(tracker.previous_outputs)
        self.assertTrue(np.isfinite(tracker.cost()))
        kp[3:5,1]+=.05
        tracker.prepare(kp,rot)
        self.assertEqual(tracker.cost(),0)
        self.assertFalse(tracker.previous_outputs)
        self.assertIsNone(tracker.selected_finger)

    def test_separation_and_bidirectional_contact_for_each_finger(self):
        kp = np.zeros((21,3))
        rotations = np.repeat(np.eye(3)[None],21,axis=0)
        for finger in range(4):
            kp[5+4*finger:9+4*finger] = [[-.035*finger,0,z] for z in (-.08,-.11,-.14,-.17)]
        for finger in range(4):
            tracker = ContinuousSurfacePatch(self.make_tracker().method)
            for lateral, z in [(.014,-.12),(.008,-.12),(.008,-.15),(.008,-.12),(.014,-.12)]:
                kp[3:5] = [[-.035*finger+lateral,0,z+.008],[-.035*finger+lateral,0,z-.008]]
                tracker.prepare(kp,rotations)
                tracker.finalize()
                if lateral > tracker.skin_offset_m:
                    self.assertEqual(tracker.strength,0)
                    self.assertEqual(tracker.cost(),0)
                else:
                    self.assertGreater(tracker.strength,0)
                    self.assertEqual(tracker.selected_finger,finger)

    def test_a5_refuses_unmeasured_calibration(self):
        tracker=self.make_tracker()
        with self.assertRaises(ValueError):
            TrackedSurfacePatch(tracker.method,5,{'status':'template'})


if __name__=='__main__':unittest.main()
