import unittest
from types import SimpleNamespace
from unittest.mock import patch

import nlopt
import numpy as np

from teleop.thumb_relative import ThumbRelative


class RoundoffTest(unittest.TestCase):
    def test_accepts_only_evaluated_feasible_current_frame(self):
        morphology = SimpleNamespace(lengths=np.full((5,3),.03),robot_basis=np.eye(3),
                                     palm_scale=1.,align=lambda points:points)
        method = ThumbRelative(None,morphology,1/30)
        points = np.zeros((21,3))
        points[1:5] = np.vstack((method.surface.data.xpos[method.thumb_bodies],
                                method.surface.data.site_xpos[method.thumb_tip]))
        points[5:9] = np.vstack((method.surface.data.xpos[method.index_bodies],
                                method.surface.data.site_xpos[method.index_tip]))
        native = nlopt.opt

        class InterruptedOptimizer:
            def __init__(self,*args):
                self.native = native(*args)

            def __getattr__(self,name):
                return getattr(self.native,name)

            def set_min_objective(self,objective):
                self.objective = objective

            def optimize(self,seed):
                self.objective(seed,np.empty(4))
                raise nlopt.RoundoffLimited()

        with patch('teleop.thumb_relative.nlopt.opt',InterruptedOptimizer):
            result = method.refine(np.zeros(20),points)
        self.assertEqual(method.last_status,-4)
        np.testing.assert_allclose(result,np.zeros(20),atol=1e-12)
        self.assertGreaterEqual(method.surface.distances().min(),.000149)


if __name__=='__main__':
    unittest.main()
