"""L20 connection and named channel access through the official SDK."""
from contextlib import contextmanager

from linkerbot_hand import discover_hands


@contextmanager
def open_hand():
    hand = discover_hands().require_one().create_hand_device()
    try:
        hand.open()
        info = hand.get_device_info(timeout=1.0)
        if info.model != "L20" or info.side.value != "right":
            raise RuntimeError(f"Expected L20 right, discovered {info.model} {info.side}")
        yield hand
    finally:
        hand.close()


def read_named(hand):
    names = hand.get_capabilities().joint_names
    sample = hand.get_angles(timeout=1.0)
    return dict(zip(names, sample.raw, strict=True))


def write_channel(hand, channel, raw, baseline):
    """Preserve the other targets from the caller's baseline."""
    target = dict(baseline)
    target[channel] = raw
    write_named(hand, target)


def write_named(hand, target):
    """Write a complete named pose in the device's declared channel order."""
    names = hand.get_capabilities().joint_names
    if set(target) != set(names):
        raise ValueError("Target must contain exactly the device's named channels")
    hand.set_angles([target[name] for name in names]).wait(timeout=1.0)
