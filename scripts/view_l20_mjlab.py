"""Open L20 actuator sliders with mjlab GPU physics and the native MuJoCo viewer."""
import time
from pathlib import Path
import torch

import mujoco
import mujoco.viewer
from mjlab.sim import Simulation, SimulationCfg, MujocoCfg
from teleop.mapping import CONTROLS, DISPLAY_NAMES, L20_URDF, manual_model_joints
ROOT = Path(__file__).resolve().parents[1]
MODEL = L20_URDF



def build_model(with_object=False):
    spec = mujoco.MjSpec.from_file(str(MODEL))
    joints = manual_model_joints()
    for name, joint in joints.items():
        limit = joint.find("limit")
        if limit is not None:
            spec.joint(name).range = [float(limit.get(k)) for k in ("lower", "upper")]
    for joint_name, sdk_name in CONTROLS:
        joint = joints[joint_name]
        limit = joint.find("limit")
        actuator = spec.add_actuator(name=DISPLAY_NAMES[sdk_name], target=joint_name,
            trntype=mujoco.mjtTrn.mjTRN_JOINT, ctrllimited=True,
            ctrlrange=[float(limit.get("lower")), float(limit.get("upper"))])
        actuator.set_to_position(kp=20, kv=1)
    if with_object:
        body = spec.worldbody.add_body(name="cube", pos=[0.045, 0, 0.06])
        body.add_freejoint(name="cube_free")
        body.add_geom(name="cube", type=mujoco.mjtGeom.mjGEOM_BOX,
                      size=[0.016]*3, mass=0.05, rgba=[0.85, 0.35, 0.12, 1])
    model = spec.compile()
    for i in range(model.neq):
        slave = model.joint(model.eq_obj1id[i]).name
        mimic = joints[slave].find("mimic")
        model.eq_data[i, :2] = [float(mimic.get("offset")), float(mimic.get("multiplier"))]
    # Numerical regularization for the interactive position-servo example.
    model.dof_armature[:21] = 0.01
    model.eq_solref[:, 0] = 0.002
    model.dof_damping[:21] = 0.1
    return model

def main():
    model = build_model()
    sim = Simulation(1, SimulationCfg(nconmax=256, njmax=2048, mujoco=MujocoCfg(timestep=0.001)), model=model, device="cuda:0")
    display = mujoco.MjData(model)
    with mujoco.viewer.launch_passive(model, display) as viewer:
        viewer.cam.lookat[:] = [0, 0, 0.12]
        viewer.cam.distance = 0.55
        viewer.cam.azimuth = 135
        viewer.cam.elevation = -20
        print("READY: mjlab GPU physics, native viewer. Expand Control for actuator sliders.", flush=True)
        while viewer.is_running():
            start = time.monotonic()
            with viewer.lock():
                sim.data.ctrl[0, :] = torch.as_tensor(display.ctrl, device="cuda:0", dtype=torch.float32)
            for _ in range(10):
                sim.step()
            with viewer.lock():
                display.qpos[:] = sim.data.qpos[0].cpu().numpy()
                display.qvel[:] = sim.data.qvel[0].cpu().numpy()
                mujoco.mj_forward(model, display)
            viewer.sync()
            time.sleep(max(0, 0.01 - (time.monotonic() - start)))

if __name__ == "__main__":
    main()



