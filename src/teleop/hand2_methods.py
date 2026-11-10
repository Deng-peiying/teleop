import numpy as np
from .robot_kinematics import RobotWrapper
from wuji_sdk import RetargetSession, HandModel, Handedness
from .wuji_hand2_vendor import ROOT
from .hand2_geometry import Morphology
from .thumb_relative import ThumbRelative
from .surface_tracker import ContinuousSurfacePatch

URDF = ROOT/'models/Wuji/hand2_beta1/body/urdf/left.urdf'

def build_method(name, samples, config, dt):
    if name != 'thumb_track_b4':
        raise ValueError('Supported method: thumb_track_b4')
    robot = RobotWrapper(str(URDF))
    morphology = Morphology(robot, np.asarray(samples), anatomical_base=True)
    session = RetargetSession.for_hand(HandModel.WujiHand2, Handedness.Left)
    method = ThumbRelative(session, morphology, dt, collision=True)
    method.use_rotations = True
    method.patch = ContinuousSurfacePatch(method)
    method.analytic_gradient = True
    return method
