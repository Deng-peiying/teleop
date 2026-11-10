"""Wuji discovery and latest-frame subscription through the official SDK."""
from contextlib import contextmanager

from wuji_sdk import DeviceType, SdkManager


@contextmanager
def open_glove(sn=None):
    manager = SdkManager.instance()
    devices = [d for d in manager.scan() if d.device_type == DeviceType.WujiGlove]
    if sn:
        devices = [d for d in devices if d.sn == sn]
    if len(devices) != 1:
        raise RuntimeError(f"Expected one glove, discovered {[d.sn for d in devices]}")
    glove = manager.connect(sn=devices[0].sn, device_name="teleop_glove")
    try:
        side = glove.hand_side().get()
        if str(getattr(side, "value", side)).lower() != "right":
            raise RuntimeError(f"Expected right glove, got {glove.hand_side().get()}")
        yield glove
    finally:
        manager.disconnect(glove.device_name)


def latest_angles(subscription):
    """Drain the queued frames. None means no new frame is available."""
    frame = subscription.recv()
    if frame is None:
        return None
    for _ in range(255):
        newer = subscription.recv()
        if newer is None:
            return frame
        frame = newer
    raise RuntimeError("Glove queue cannot be drained; stop following and reconnect")

