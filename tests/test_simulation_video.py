"""Exercise real encoding across multiple keyboard-style recording sessions."""
import json
import shutil
import subprocess
import tempfile
import time
import unittest
from pathlib import Path

import mujoco
from teleop.simulation_video import SimulationVideo


@unittest.skipUnless(shutil.which('ffmpeg') and shutil.which('ffprobe'), 'ffmpeg tools required')
class SimulationVideoTest(unittest.TestCase):
    def test_two_clips_are_finalized_and_decodable(self):
        model = mujoco.MjModel.from_xml_string('<mujoco><worldbody><geom type="sphere" size=".1"/></worldbody></mujoco>')
        data = mujoco.MjData(model)
        mujoco.mj_forward(model, data)
        camera = mujoco.MjvCamera()
        camera.distance = 1.
        options = mujoco.MjvOption()
        with tempfile.TemporaryDirectory() as folder:
            recorder = SimulationVideo(model, Path(folder), width=320, height=240)
            try:
                for number in (1, 2):
                    recorder.start()
                    for _ in range(4):
                        recorder.capture(data, camera, options)
                        time.sleep(.06)
                    recorder.stop()
                    self.assertFalse(recorder.active)
                    path = Path(folder) / f'video-{number:03d}.mp4'
                    result = subprocess.check_output([
                        'ffprobe', '-v', 'error', '-select_streams', 'v:0',
                        '-show_entries', 'stream=width,height,nb_frames', '-of', 'json', str(path),
                    ], text=True)
                    stream = json.loads(result)['streams'][0]
                    self.assertEqual((stream['width'], stream['height']), (320, 240))
                    self.assertGreaterEqual(int(stream['nb_frames']), 4)
                    subprocess.run(['ffmpeg', '-v', 'error', '-i', str(path), '-f', 'null', '-'], check=True)
            finally:
                recorder.close()


if __name__ == '__main__':
    unittest.main()
