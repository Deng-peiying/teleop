"""Independent arm, hand and arm-hand MuJoCo-to-device entry points."""
import argparse
from contextlib import ExitStack
import sys
import threading
import time

import mujoco
from mujoco import viewer as mj_viewer
import numpy as np

from .mapping import CONTROLS, MANUAL_RANGES, map_angle, raw_endpoints
from .mujoco_model import build_control_model, display_targets


def raw_to_angle(raw, limits, endpoints):
    lower, upper = limits
    raw_lower, raw_upper = endpoints
    return lower + (raw - raw_lower) / (raw_upper - raw_lower) * (upper - lower)


def hand_targets(ctrl, previous_ctrl, baseline, endpoints):
    """Preserve every real raw channel until its corresponding slider changes."""
    target = baseline.copy()
    for i, (_, channel) in enumerate(CONTROLS):
        if ctrl[i] != previous_ctrl[i]:
            target[channel] = map_angle(ctrl[i], *MANUAL_RANGES[channel], *endpoints[channel])
    return target


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("mode", choices=("arm", "hand", "arm-hand"))
    parser.add_argument("--ip", default="192.168.1.18")
    parser.add_argument("--preview", action="store_true", help="Model-only sliders, no device connections")
    parser.add_argument("--check", action="store_true", help="Compile model and print controls, no connections")
    args = parser.parse_args(argv)
    model = build_control_model(args.mode, preview=args.preview)
    if args.check:
        print(f"{args.mode}: {model.nu} controls, {model.nq} joints")
        for i in range(model.nu):
            print(model.actuator(i).name, model.actuator_ctrlrange[i])
        return 0

    data = mujoco.MjData(model)
    arm_count = 0 if args.mode == "hand" else 7
    stop_requested = threading.Event()

    def key_callback(key):
        if key == 32:
            stop_requested.set()

    with ExitStack() as stack:
        arm = hand = None
        if not args.preview and args.mode != "hand":
            from .arm import open_arm, read_arm, write_arm
            arm = stack.enter_context(open_arm(args.ip))
            data.ctrl[:7] = read_arm(arm)
        if not args.preview and args.mode != "arm":
            from .hand import open_hand, read_named, write_named
            hand = stack.enter_context(open_hand())
            baseline = read_named(hand)
            endpoints = {j.name: raw_endpoints(j) for j in hand.get_capabilities().joints}
            for i, (_, channel) in enumerate(CONTROLS):
                angle = raw_to_angle(baseline[channel], MANUAL_RANGES[channel], endpoints[channel])
                data.ctrl[arm_count + i] = np.clip(
                    angle, *model.actuator_ctrlrange[arm_count + i])
            print("hand initial raw:", baseline, flush=True)
        if arm:
            # Stop the arm before potentially waiting for the hand SDK to close.
            stack.callback(arm.rm_set_arm_stop)
        previous_ctrl = data.ctrl.copy()
        display_targets(model, data)
        model.vis.headlight.ambient[:] = 0.6
        with mj_viewer.launch_passive(model, data, key_callback=key_callback) as viewer:
            viewer.cam.lookat[:] = [0, 0, 0.12 if args.mode == "hand" else 0.55]
            viewer.cam.distance = 0.55 if args.mode == "hand" else 1.5
            print(f"READY {args.mode} {'PREVIEW' if args.preview else 'LIVE'}: "
                  "Control sliders send on change. ARM degrees; L20 radians. Close to disconnect.",
                  flush=True)
            while viewer.is_running():
                start = time.monotonic()
                viewer.sync()
                if stop_requested.is_set():
                    stop_requested.clear()
                    if arm:
                        print("arm stop:", arm.rm_set_arm_stop(), flush=True)
                        print("arm clear:", arm.rm_set_arm_delete_trajectory(), flush=True)
                with viewer.lock():
                    ctrl = data.ctrl.copy()
                    display_targets(model, data)
                if arm and not np.array_equal(ctrl[:7], previous_ctrl[:7]):
                    write_arm(arm, ctrl[:7].tolist())
                if hand:
                    target = hand_targets(ctrl[arm_count:], previous_ctrl[arm_count:], baseline, endpoints)
                    if target != baseline:
                        write_named(hand, target)
                        print("hand target raw:", target, flush=True)
                        baseline = target
                previous_ctrl = ctrl
                time.sleep(max(0, 0.02 - (time.monotonic() - start)))
    return 0


def arm_main():
    return main(["arm", *sys.argv[1:]])


def hand_main():
    return main(["hand", *sys.argv[1:]])


def arm_hand_main():
    return main(["arm-hand", *sys.argv[1:]])


if __name__ == "__main__":
    raise SystemExit(main())
