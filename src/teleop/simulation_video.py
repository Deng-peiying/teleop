"""On-demand MuJoCo viewport capture with asynchronous MP4 encoding."""
import json
import queue
import shutil
import subprocess
import threading
import time

import mujoco


class SimulationVideo:
    def __init__(self, model, directory, fps=20, width=960, height=720):
        self.model = model
        self.directory = directory
        self.fps = fps
        self.width = width
        self.height = height
        self.renderer = None
        self.process = None
        self.error = None
        self.clip = 0
        self.frames = 0
        self.dropped = 0

    @property
    def active(self):
        return self.process is not None

    def start(self):
        executable = shutil.which('ffmpeg')
        if executable is None:
            raise RuntimeError('Video recording requires ffmpeg. Install it with: sudo apt install ffmpeg')
        if self.renderer is None:
            self.model.vis.global_.offwidth = max(self.width, self.model.vis.global_.offwidth)
            self.model.vis.global_.offheight = max(self.height, self.model.vis.global_.offheight)
            self.renderer = mujoco.Renderer(self.model, height=self.height, width=self.width)
        self.clip += 1
        self.path = self.directory / f'video-{self.clip:03d}.mp4'
        self.encoder_log = self.path.with_suffix('.ffmpeg.log').open('wb')
        self.process = subprocess.Popen([
            executable, '-hide_banner', '-loglevel', 'error', '-y',
            '-f', 'rawvideo', '-pix_fmt', 'rgb24', '-s', f'{self.width}x{self.height}',
            '-r', str(self.fps), '-i', '-', '-an', '-c:v', 'libx264',
            '-preset', 'ultrafast', '-crf', '22', '-pix_fmt', 'yuv420p',
            '-movflags', '+faststart', str(self.path),
        ], stdin=subprocess.PIPE, stdout=subprocess.DEVNULL, stderr=self.encoder_log)
        self.pending = queue.Queue(maxsize=2)
        self.error = None
        self.frames = 0
        self.dropped = 0
        self.started = time.monotonic()
        self.last_capture = -1
        self.worker = threading.Thread(target=self._encode, daemon=True)
        self.worker.start()

    def _encode(self):
        last = None
        written = 0
        try:
            while True:
                item = self.pending.get()
                if item is None:
                    break
                index, frame = item
                while written < index:
                    self.process.stdin.write(last if last is not None else frame)
                    written += 1
                self.process.stdin.write(frame)
                written += 1
                last = frame
            self.frames = written
        except Exception as exc:
            self.error = str(exc)
        finally:
            self.process.stdin.close()

    def capture(self, data, camera, options):
        if not self.active:
            return
        if self.error:
            raise RuntimeError(self.error)
        index = int((time.monotonic() - self.started) * self.fps)
        if index == self.last_capture:
            return
        self.renderer.update_scene(data, camera=camera, scene_option=options)
        frame = self.renderer.render().tobytes()
        try:
            self.pending.put_nowait((index, frame))
        except queue.Full:
            self.dropped += 1
        self.last_capture = index

    def stop(self):
        if not self.active:
            return
        while self.worker.is_alive():
            try:
                self.pending.put(None, timeout=.1)
                break
            except queue.Full:
                pass
        self.worker.join(timeout=15)
        if self.worker.is_alive():
            self.process.kill()
            self.worker.join(timeout=5)
        code = self.process.wait(timeout=10)
        self.process = None
        self.encoder_log.close()
        self.path.with_suffix('.json').write_text(json.dumps(dict(
            fps=self.fps, width=self.width, height=self.height,
            encoded_frames=self.frames, dropped_captures=self.dropped,
            wall_duration_s=time.monotonic()-self.started, encoder_exit_code=code,
            error=self.error, view='MuJoCo model with current camera and geometry visibility',
        ), indent=2), encoding='utf-8')
        if code or self.error:
            raise RuntimeError(f'Video encoder failed; see {self.path.with_suffix(".ffmpeg.log")}')

    def close(self):
        try:
            self.stop()
        finally:
            if self.renderer is not None:
                self.renderer.close()
