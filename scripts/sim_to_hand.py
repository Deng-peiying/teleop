"""Send MuJoCo slider targets directly through teleop's new L20 SDK interface."""
import argparse
import time

import mujoco
import mujoco.viewer
import torch
from mjlab.sim import Simulation, SimulationCfg, MujocoCfg

from teleop.hand import open_hand, read_named, write_named
from teleop.mapping import MANUAL_RANGES, map_angle, raw_endpoints
from view_l20_mjlab import CONTROLS, build_model

def raw_to_ctrl(raw, limits, endpoints):
    lower, upper = limits
    raw_lower, raw_upper = endpoints
    fraction = (raw - raw_lower) / (raw_upper - raw_lower)
    return lower + fraction * (upper - lower)


def slider_targets(ctrl, limits, endpoints):
    return {name: map_angle(ctrl[i], *limits[i], *endpoints[name])
            for i, (_, name) in enumerate(CONTROLS)}


# Official legacy L20 right-hand angle table, keyed by SDK channel name.
LEGACY_RANGES = {
    "thumb_rotation": (0, 1.78), "thumb_roll": (-.297, .683),
    "thumb_root": (0, .87), "thumb_tip": (0, 1.29),
    **{f"{finger}_{axis}": limits
       for finger in ("index", "middle", "ring", "pinky")
       for axis, limits in (("abd", (-.26, .26)), ("root", (0, 1.4)), ("tip", (0, 1.08)))},
}


def main(legacy=False):
    model = build_model()
    # Pose comparison only: overlapping URDF collision meshes can lock the
    # thumb despite changing actuator targets. Keep contact physics out of
    # this hardware slider viewer; do not change the official model asset.
    model.opt.disableflags |= int(mujoco.mjtDisableBit.mjDSBL_CONTACT)
    sim = Simulation(1, SimulationCfg(nconmax=256, njmax=2048,
        mujoco=MujocoCfg(timestep=0.001)), model=model, device="cuda:0")
    display = mujoco.MjData(model)
    with open_hand() as hand:
        baseline = read_named(hand)
        if set(baseline) != {name for _, name in CONTROLS}:
            raise RuntimeError("Device channels do not match the L20 simulation")
        endpoints = {joint.name: raw_endpoints(joint)
                     for joint in hand.get_capabilities().joints}
        limits = [MANUAL_RANGES[name] for _, name in CONTROLS]
        if legacy:
            limits = [LEGACY_RANGES[name] for _, name in CONTROLS]
            endpoints = {name: (0, 255) if name.endswith("_abd") else (255, 0)
                         for _, name in CONTROLS}
        for i, (joint, name) in enumerate(CONTROLS):
            lower, upper = model.actuator_ctrlrange[i]
            display.ctrl[i] = max(lower, min(upper, raw_to_ctrl(
                baseline[name], limits[i], endpoints[name])))
            display.qpos[model.joint(joint).qposadr[0]] = display.ctrl[i]
        for i in range(model.neq):
            display.qpos[model.joint(model.eq_obj1id[i]).qposadr[0]] = (
                model.eq_data[i, 1] * display.qpos[model.joint(model.eq_obj2id[i]).qposadr[0]])
        sim.data.qpos[0, :] = torch.as_tensor(display.qpos, device="cuda:0", dtype=torch.float32)
        last_target = dict(baseline)
        previous_ctrl = display.ctrl.copy()
        with mujoco.viewer.launch_passive(model, display) as viewer:
            viewer.cam.lookat[:] = [0, 0, 0.12]
            viewer.cam.distance = .55
            print("READY: MuJoCo sliders -> teleop.hand -> new L20 SDK -> CAN.", flush=True)
            while viewer.is_running():
                start = time.monotonic()
                with viewer.lock():
                    ctrl = display.ctrl.copy()
                # The real pose can lie outside the model range in legacy mode.
                # Preserve each channel until its slider is explicitly moved.
                mapped = slider_targets(ctrl, limits, endpoints)
                target = last_target.copy()
                for i, (_, name) in enumerate(CONTROLS):
                    if ctrl[i] != previous_ctrl[i]:
                        target[name] = mapped[name]
                previous_ctrl = ctrl
                if target != last_target:
                    write_named(hand, target)
                    last_target = target
                sim.data.ctrl[0, :] = torch.as_tensor(ctrl, device="cuda:0", dtype=torch.float32)
                for _ in range(20):
                    sim.step()
                with viewer.lock():
                    display.qpos[:] = sim.data.qpos[0].cpu().numpy()
                    display.qvel[:] = sim.data.qvel[0].cpu().numpy()
                    mujoco.mj_forward(model, display)
                viewer.sync()
                time.sleep(max(0, .02 - (time.monotonic() - start)))


if __name__ == "__main__":
    try:
        parser = argparse.ArgumentParser()
        parser.add_argument("--legacy-ranges", action="store_true",
                            help="Compare using the legacy L20 right-hand raw/radian table")
        main(parser.parse_args().legacy_ranges)
    except KeyboardInterrupt:
        pass
