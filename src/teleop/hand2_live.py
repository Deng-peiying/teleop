"""Explicitly selected method for live simulation; paired claims use offline replay."""
import argparse
import contextlib
import json
import time
from datetime import datetime
from pathlib import Path
import mujoco
import mujoco.viewer
import numpy as np
from wuji_sdk import SdkManager
from .hand2_bench import METHODS, CONFIG, SharedSimulation, file_hash
from .hand2_methods import build_method
from .wuji_hand2_vendor import ROOT, latest_frame
from .simulation_video import SimulationVideo

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--method', choices=METHODS, default='thumb_track_b4')
    parser.add_argument('--address', default='192.168.1.100:50001')
    parser.add_argument('--user', default='operator_left')
    parser.add_argument('--config', default=str(CONFIG))
    parser.add_argument('--hz', type=float)
    parser.add_argument('--display', choices=['command', 'dynamics'], default='command')
    parser.add_argument('--duration', type=float, default=0.0)
    parser.add_argument('--headless', action='store_true')
    parser.add_argument('--label-static', action='store_true', help='F5/F6 mark 5-second static contact trials; algorithm unchanged')
    args = parser.parse_args()
    cfg = json.loads(Path(args.config).read_text())
    args.hz = args.hz or cfg['hz']
    if args.hz <= 0 or args.duration < 0:
        parser.error('Invalid rate/duration')
    dt = 1 / args.hz
    manager = SdkManager.instance()
    previous = manager.current_user()
    selected = next((u for u in manager.list_users() if u['display_name'] == args.user))
    out = ROOT / 'runs/hand2_live' / datetime.now().strftime('%Y%m%d-%H%M%S')
    out.mkdir(parents=True, exist_ok=False)
    meta = {'method': args.method, 'kind': 'live_qualitative_preview', 'config': cfg, 'profile': selected, 'hz': args.hz, 'display': args.display, 'source_sha256': {p.name: file_hash(p) for p in Path(__file__).parent.glob('hand2_*.py')}, 'warning': 'Live latency differs by method. Use paired offline results for quantitative comparison.'}
    meta['source_sha256']['thumb_relative.py'] = file_hash(Path(__file__).with_name('thumb_relative.py'))
    meta['source_sha256']['thumb_patch.py'] = file_hash(Path(__file__).with_name('thumb_patch.py'))
    for filename in ('surface_tracker.py', 'thumb_gradient.py', 'simulation_video.py'):
        meta['source_sha256'][filename] = file_hash(Path(__file__).with_name(filename))
    (out / 'manifest.json').write_text(json.dumps(meta, indent=2), encoding='utf-8')
    viewer = None
    sub = None
    label_requests = []
    segments = []
    active_label = None
    patch_requests = []
    video_requests = []
    video = None

    def label_key(key):
        if key == 298:  # GLFW F9
            video_requests.append(True)
        if args.label_static and key in (294, 295):
            label_requests.append('index_middle_together' if key == 294 else 'thumb_index_side_touch')
    try:
        manager.switch_user(selected['user_id'])
        glove = manager.connect(address=args.address, device_name='hand2_experiment_glove')
        if str(glove.hand_side().get()).lower() != 'left':
            raise ValueError('Expected left glove')
        sub = glove.hand_skeleton().subscribe()
        samples = []
        deadline = time.monotonic() + 8
        while len(samples) < 30 and time.monotonic() < deadline:
            frame = latest_frame(sub)
            if frame is not None:
                kp = np.array([j.pose.position for j in frame.joints], float)
                if kp.shape == (21, 3) and np.isfinite(kp).all():
                    samples.append(kp)
            time.sleep(0.005)
        if len(samples) < 2:
            raise RuntimeError('No valid glove skeleton')
        method = build_method(args.method, np.array(samples), cfg, dt)
        sim = SharedSimulation(args.hz)
        video = SimulationVideo(sim.model, out)
        if not args.headless:
            viewer = mujoco.viewer.launch_passive(sim.model, sim.command_data if args.display == 'command' else sim.data, key_callback=label_key)
            viewer.cam.azimuth, viewer.cam.elevation, viewer.cam.distance = (180, -20, 0.35)
            viewer.cam.lookat[:] = sim.camera_center
        start = last_frame = last_tick = time.monotonic()
        count = 0
        overruns = 0
        q = np.zeros(20)
        gap = False
        print('LIVE METHOD:', args.method, '|', args.display, '| profile', args.user, flush=True)
        print('RUN:', out, flush=True)
        with (out / 'frames.jsonl').open('w', encoding='utf-8') as log:
            while (viewer is None or viewer.is_running()) and (not args.duration or time.monotonic() - start < args.duration):
                tick = time.monotonic()
                frame = latest_frame(sub)
                if active_label is not None and tick - start >= active_label['end']:
                    print('STATIC LABEL COMPLETE:', active_label['label'], flush=True)
                    active_label = None
                if label_requests and active_label is None:
                    active_label = {'label': label_requests.pop(0), 'start': tick - start, 'end': tick - start + 5.0, 'annotation': 'Operator-triggered intended contact; not sensor-verified contact truth'}
                    segments.append(active_label.copy())
                    (out / 'segments.json').write_text(json.dumps(segments, indent=2), encoding='utf-8')
                    print('STATIC LABEL START:', active_label['label'], '5 seconds', flush=True)
                if frame is not None:
                    received_wall_s = time.time()
                    kp = np.array([j.pose.position for j in frame.joints], np.float32)
                    if gap:
                        method = build_method(args.method, np.array(samples), cfg, dt)
                        gap = False
                    if patch_requests:
                        method.patch.confirm(patch_requests[-1])
                        patch_requests.clear()
                    before = time.perf_counter()
                    try:
                        rotations = [[j.pose.orientation.x, j.pose.orientation.y, j.pose.orientation.z, j.pose.orientation.w] for j in frame.joints]
                        q = np.asarray(method.step(kp, rotations))
                    except Exception as exc:
                        failed = dict(t=tick - start, error=str(exc), keypoints_m=kp.tolist(), skeleton_orientation_xyzw=[[j.pose.orientation.x, j.pose.orientation.y, j.pose.orientation.z, j.pose.orientation.w] for j in frame.joints])
                        if hasattr(method, 'current_sdk'):
                            failed['sdk_command_rad'] = method.current_sdk.tolist()
                        (out / 'failed_frame.json').write_text(json.dumps(failed, indent=2))
                        raise
                    solve_ms = (time.perf_counter() - before) * 1000
                    last_frame = tick
                    count += 1
                    overruns += int(solve_ms > 1000 / args.hz)
                elif tick - last_frame > 0.25 and (not gap):
                    gap = True
                    print('Tracking gap; holding command, resetting solver on recovery', flush=True)
                sim.dt = min(max(tick - last_tick, dt), 0.1)
                last_tick = tick
                state, _, _, ncon = sim.step(q)
                if viewer is not None:
                    if video_requests:
                        for _ in video_requests:
                            try:
                                if video.active:
                                    video.stop()
                                    print('VIDEO SAVED:', video.path, flush=True)
                                else:
                                    video.start()
                                    print('VIDEO RECORDING:', video.path, flush=True)
                            except Exception as exc:
                                print('VIDEO ERROR:', exc, flush=True)
                        video_requests.clear()
                    label_text = ''
                    if args.method in ('thumb_pose_clearance', 'thumb_patch_clearance', 'thumb_slide_auto', 'thumb_track_a2', 'thumb_track_a3', 'thumb_track_a4', 'thumb_track_a5', 'thumb_track_b4') and hasattr(method, 'side_diagnostics'):
                        info = method.side_diagnostics
                        label_text = f"\nRelative error {info['relative_error_mm']:.1f} mm | pad normal {info['pad_normal_error_deg']:.1f} deg"
                        from .thumb_patch import mark_surfaces
                        patch = info['patch']
                        label_text += f"\nF7 confirm self-touch | F8 release / pen | patch weight {patch['strength']:.2f}\nblue candidate | amber closest | red intersect | gap {1000 * patch['nearest']['gap_m']:.1f} mm"
                        label_text = f"\nAutomatic four-finger slide | {patch['target_finger']} | weight {patch['strength']:.2f}\nblue candidate | amber closest | red intersect | gap {1000 * patch['nearest']['gap_m']:.1f} mm"
                        with viewer.lock():
                            viewer.user_scn.ngeom = 0
                            mark_surfaces(viewer.user_scn, patch['nearest'], patch['candidate'])
                    if args.label_static:
                        label_text += '\nF5: index+middle together | F6: thumb+index side (hold 5s)'
                        if active_label is not None:
                            label_text += f"\nRECORDING {active_label['label']}: {max(0, active_label['end'] - (tick - start)):.1f}s"
                    viewer.set_texts((mujoco.mjtFontScale.mjFONTSCALE_200, None, f'LIVE {args.method} | {args.display}\nprofile={args.user}\nframes={count} | solver overruns={overruns}' + label_text, 'F9: start / stop video\n' + ('REC - ' + video.path.name if video.active else 'Video idle')))
                    viewer.sync()
                    try:
                        video.capture(sim.command_data if args.display == 'command' else sim.data, viewer.cam, viewer.opt)
                    except Exception as exc:
                        print('VIDEO ERROR:', exc, flush=True)
                        video.stop()
                if frame is not None:
                    record = {'t': tick - start, 'keypoints_m': kp.tolist(), 'command_rad': q.tolist(), 'actual_rad': state.tolist(), 'solve_ms': solve_ms, 'contacts': ncon, 'received_wall_s': received_wall_s, 'output_ready_wall_s': time.time(), 'source_seq': int(frame.header.seq), 'device_timestamp_us': int(frame.header.timestamp_us), 'device_to_receive_clock_difference_ms': 1000 * (received_wall_s - frame.header.timestamp_us / 1000000.0), 'latency_note': 'Device/host clock difference is not a synchronized one-way latency measurement; actual_rad is simulation.'}
                    record['skeleton_orientation_xyzw'] = [[joint.pose.orientation.x, joint.pose.orientation.y, joint.pose.orientation.z, joint.pose.orientation.w] for joint in frame.joints]
                    if active_label is not None:
                        record['operator_label'] = active_label['label']
                    record['thumb_side'] = method.side_diagnostics
                    if hasattr(method, 'baseline_command'):
                        record['repository_baseline_command_rad'] = method.baseline_command.tolist()
                        record['contact_strength'] = method.contact_strength
                        record['contact_finger'] = method.contact_finger
                        record['contact_arc'] = method.contact_arc
                        record['contact_target_m'] = None if method.contact_target is None else method.contact_target.tolist()
                        record['solver_status'] = method.last_status
                    log.write(json.dumps(record) + '\n')
                    if count % 120 == 0:
                        log.flush()
                        print(f'{args.method}: frames={count}, overruns={overruns}', flush=True)
                time.sleep(max(0, dt - (time.monotonic() - tick)))
        if count == 0:
            raise RuntimeError('No live frames processed')
    except KeyboardInterrupt:
        pass
    finally:
        if video is not None:
            with contextlib.suppress(Exception):
                video.close()
        if viewer is not None:
            viewer.close()
        if sub is not None:
            with contextlib.suppress(Exception):
                sub.close()
        with contextlib.suppress(Exception):
            manager.disconnect_all()
        manager.switch_user(previous['user_id'])
if __name__ == '__main__':
    main()
