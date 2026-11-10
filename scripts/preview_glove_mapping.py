"""Simulation only: synthetic Wuji frames -> full mapper -> raw -> MuJoCo.

No hand/glove connection, device discovery, or CAN transmission is performed.
"""
import math
import time
from types import SimpleNamespace

import mujoco
import mujoco.viewer
import torch
from linkerbot_hand import HandSide
from linkerbot_hand.capabilities.models.l20 import declared_capabilities
from mjlab.sim import Simulation, SimulationCfg, MujocoCfg

from teleop.mapping import CONTROLS, MANUAL_RANGES
from teleop.retarget import WujiToL20
from view_l20_mjlab import build_model


PHASES = [("Neutral", 3), ("Thumb CMC: all three input angles", 6),
          ("Thumb MCP/IP coupling", 5),
          *[(f"{finger}: root, then PIP/DIP coupling", 6)
            for finger in ("Index", "Middle", "Ring", "Pinky")],
          ("Full hand", 6)]


def synthetic_frame(elapsed):
    elapsed %= sum(duration for _, duration in PHASES)
    phase = 0
    while elapsed >= PHASES[phase][1]:
        elapsed -= PHASES[phase][1]
        phase += 1
    duration = PHASES[phase][1]
    amount = math.sin(math.pi * elapsed / duration)**2
    fingers = [SimpleNamespace(angles=[0.]*5, confidence=1.) for _ in range(5)]
    if phase == 1:
        fingers[0].angles[:3] = [.45*amount, .6*amount, -.35*amount]
    elif phase == 2:
        fingers[0].angles[3:] = [.9*amount, .6*amount]
    elif 3 <= phase <= 6:
        finger = fingers[phase-2]
        local = elapsed % 3
        amount = math.sin(math.pi * local / 3)**2
        if elapsed < 3:
            finger.angles[0] = .8*amount
        else:
            finger.angles[2:4] = [1.2*amount, .8*amount]
    elif phase == 7:
        fingers[0].angles = [.45*amount, .6*amount, -.35*amount, .9*amount, .6*amount]
        for finger in fingers[1:]:
            finger.angles = [.7*amount, .12*amount, 1.2*amount, .8*amount, 0.]
    return phase, SimpleNamespace(fingers=fingers)


def main():
    mapper = WujiToL20(declared_capabilities(HandSide.RIGHT))
    model = build_model()
    sim = Simulation(1, SimulationCfg(nconmax=256, njmax=2048,
        mujoco=MujocoCfg(timestep=.001)), model=model, device="cuda:0")
    display = mujoco.MjData(model)
    with mujoco.viewer.launch_passive(model, display) as viewer:
        viewer.cam.lookat[:] = [0, 0, .12]
        viewer.cam.distance = .48
        viewer.cam.azimuth = 135
        viewer.cam.elevation = -20
        print("READY: SIMULATION ONLY. Synthetic Wuji -> 21-to-16 mapper -> raw -> MuJoCo.", flush=True)
        start = time.monotonic()
        last_phase = None
        while viewer.is_running():
            tick = time.monotonic()
            phase, frame = synthetic_frame(tick-start)
            raw = mapper.map(frame)
            ctrl = []
            for _, name in CONTROLS:
                lower, upper = MANUAL_RANGES[name]
                raw_lower, raw_upper = mapper.endpoints[name]
                ctrl.append(lower+(raw[name]-raw_lower)*(upper-lower)/(raw_upper-raw_lower))
            if phase != last_phase:
                print(PHASES[phase][0], flush=True)
                last_phase = phase
            sim.data.ctrl[0, :] = torch.as_tensor(ctrl, device="cuda:0", dtype=torch.float32)
            for _ in range(20):
                sim.step()
            with viewer.lock():
                display.ctrl[:] = ctrl
                display.qpos[:] = sim.data.qpos[0].cpu().numpy()
                display.qvel[:] = sim.data.qvel[0].cpu().numpy()
                mujoco.mj_forward(model, display)
            viewer.sync()
            time.sleep(max(0, .02-(time.monotonic()-tick)))


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
