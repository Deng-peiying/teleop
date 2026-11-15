"""RM75-only MuJoCo slider debugging; --preview and --check never connect."""
import argparse
from contextlib import ExitStack
import threading
import time
import mujoco
import mujoco.viewer
import numpy as np
from .mujoco_model import build_control_model, display_targets

def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('mode',nargs='?',choices=['arm'],default='arm')
    parser.add_argument('--ip',default='192.168.1.18')
    parser.add_argument('--preview',action='store_true')
    parser.add_argument('--check',action='store_true')
    args = parser.parse_args(argv)
    model = build_control_model(preview=args.preview)
    if args.check:
        print(f'RM75: {model.nu} controls, {model.nq} joints')
        return 0
    data = mujoco.MjData(model)
    stop = threading.Event()
    def key_callback(key):
        if key == 32: stop.set()
    with ExitStack() as stack:
        robot = None
        if not args.preview:
            from .arm import open_arm, read_arm, write_arm
            robot = stack.enter_context(open_arm(args.ip))
            data.ctrl[:] = read_arm(robot)
            stack.callback(robot.rm_set_arm_stop)
        previous = data.ctrl.copy()
        display_targets(model,data)
        with mujoco.viewer.launch_passive(model,data,key_callback=key_callback) as viewer:
            viewer.cam.lookat[:] = [0,0,.55]
            viewer.cam.distance = 1.5
            print('RM75 sliders: degrees; Space stops/clears arm; close disconnects.',flush=True)
            while viewer.is_running():
                started = time.monotonic()
                viewer.sync()
                if stop.is_set():
                    stop.clear()
                    if robot:
                        robot.rm_set_arm_stop()
                        robot.rm_set_arm_delete_trajectory()
                with viewer.lock():
                    ctrl = data.ctrl.copy()
                    display_targets(model,data)
                if robot and not np.array_equal(ctrl,previous):
                    write_arm(robot,ctrl.tolist())
                previous = ctrl
                time.sleep(max(0,.02-(time.monotonic()-started)))
    return 0

def arm_main():
    return main()

if __name__ == '__main__':
    raise SystemExit(main())
