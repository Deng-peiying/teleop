"""Read each SDK independently, then connect them through a measured mapping."""
import argparse
import json
import math
import time

from .glove import latest_angles, open_glove
from .hand import open_hand, read_named, write_channel, write_named
from .mapping import map_angle
from .retarget import WujiToL20


def print_glove(sn):
    with open_glove(sn) as glove:
        print("Glove", glove.serial_number, glove.hand_side().get(), flush=True)
        subscription = glove.hand_joint_angles().subscribe()
        while True:
            frame = latest_angles(subscription)
            if frame is not None:
                print(json.dumps([{"finger": i, "angles": list(f.angles), "confidence": f.confidence}
                                  for i, f in enumerate(frame.fingers)]), flush=True)
            time.sleep(0.1)


def follow(args):
    with open_glove(args.sn) as glove, open_hand() as hand:
        subscription = glove.hand_joint_angles().subscribe()
        baseline = read_named(hand)
        if args.live:
            print(f"Sending {args.channel}; other targets come from initial feedback.", flush=True)
        else:
            print("Preview targets", flush=True)
        last_new = time.monotonic()
        while True:
            frame = latest_angles(subscription)
            if frame is None:
                if time.monotonic()-last_new > args.stale_seconds:
                    raise TimeoutError("No new glove frame; following ended")
                time.sleep(1/args.hz)
                continue
            last_new = time.monotonic()
            finger = frame.fingers[args.finger]
            if not math.isfinite(finger.confidence) or finger.confidence < args.min_confidence:
                raise ValueError("Glove confidence below the configured threshold")
            angle = finger.angles[args.component]
            target = map_angle(angle, args.open_angle, args.bent_angle,
                               args.raw_open, args.raw_bent)
            if args.live:
                write_channel(hand, args.channel, target, baseline)
            print(json.dumps({"glove_angle": angle, "channel": args.channel, "target_raw": target,
                              "feedback_raw": read_named(hand)[args.channel], "live": args.live}), flush=True)
            time.sleep(1/args.hz)


def follow_all(args):
    with open_glove(args.sn) as glove, open_hand() as hand:
        mapper = WujiToL20(hand.get_capabilities())
        subscription = glove.hand_joint_angles().subscribe()
        print("Sending all 16 L20 channels" if args.live else "Preview all 16 targets", flush=True)
        last_new = time.monotonic()
        while True:
            frame = latest_angles(subscription)
            if frame is None:
                if time.monotonic() - last_new > args.stale_seconds:
                    raise TimeoutError("No new glove frame; following ended")
                time.sleep(1/args.hz)
                continue
            last_new = time.monotonic()
            target = mapper.map(frame, args.min_confidence)
            if args.live:
                write_named(hand, target)
            print(json.dumps({"target_raw": target, "feedback_raw": read_named(hand),
                              "live": args.live}), flush=True)
            time.sleep(1/args.hz)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="command", required=True)
    commands.add_parser("hand", help="Read official L20 identity and 16 named channels")
    glove = commands.add_parser("glove", help="Print finger indices, angle components and confidence")
    glove.add_argument("--sn")
    follow_parser = commands.add_parser("follow", help="Preview one measured mapping; --live enables writes")
    follow_parser.add_argument("--sn")
    follow_parser.add_argument("--finger", type=int, required=True)
    follow_parser.add_argument("--component", type=int, required=True)
    follow_parser.add_argument("--channel", choices=("index_root", "index_tip", "index_abd"), required=True)
    follow_parser.add_argument("--open-angle", type=float, required=True)
    follow_parser.add_argument("--bent-angle", type=float, required=True)
    follow_parser.add_argument("--raw-open", type=int, required=True)
    follow_parser.add_argument("--raw-bent", type=int, required=True)
    follow_parser.add_argument("--hz", type=float, default=20)
    follow_parser.add_argument("--stale-seconds", type=float, default=0.5)
    follow_parser.add_argument("--min-confidence", type=float, default=0.5)
    follow_parser.add_argument("--live", action="store_true")
    full = commands.add_parser("follow-all", help="Map all 21 Wuji angles to 16 L20 targets; --live enables writes")
    full.add_argument("--sn")
    full.add_argument("--hz", type=float, default=20)
    full.add_argument("--stale-seconds", type=float, default=0.5)
    full.add_argument("--min-confidence", type=float, default=0.5)
    full.add_argument("--live", action="store_true")
    args = parser.parse_args()
    if args.command in ("follow", "follow-all"):
        if args.command == "follow" and (args.finger < 0 or args.component < 0):
            parser.error("finger and component must be nonnegative")
        if not math.isfinite(args.hz) or not 0 < args.hz <= 100:
            parser.error("hz must be within (0, 100]")
        if not math.isfinite(args.stale_seconds) or args.stale_seconds <= 0:
            parser.error("stale-seconds must be positive")
        if not math.isfinite(args.min_confidence) or not 0 <= args.min_confidence <= 1:
            parser.error("min-confidence must be within [0, 1]")
        if args.command == "follow":
            map_angle(args.open_angle, args.open_angle, args.bent_angle, args.raw_open, args.raw_bent)
    try:
        if args.command == "hand":
            with open_hand() as hand:
                print(hand.get_device_info(timeout=1.0))
                print(json.dumps(read_named(hand)), flush=True)
        elif args.command == "glove":
            print_glove(args.sn)
        elif args.command == "follow-all":
            follow_all(args)
        else:
            follow(args)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
