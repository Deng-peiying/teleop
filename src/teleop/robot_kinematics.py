"""Pinocchio frame poses used to initialize the hand morphology mapping."""
import pinocchio as pin


class RobotWrapper:
    def __init__(self, urdf_path):
        self.model = pin.buildModelFromUrdf(urdf_path)
        self.data = self.model.createData()

    @property
    def dof(self):
        return self.model.nq

    def compute_forward_kinematics(self, qpos):
        pin.forwardKinematics(self.model, self.data, qpos)

    def get_link_index(self, name):
        index = self.model.getFrameId(name, pin.BODY)
        if index >= self.model.nframes:
            raise ValueError(f'Unknown robot body: {name}')
        return index

    def get_link_pose(self, index):
        return pin.updateFramePlacement(self.model, self.data, index).homogeneous
