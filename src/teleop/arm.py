"""RM75 connection and direct latest-target position control."""
from contextlib import contextmanager


@contextmanager
def open_arm(ip="192.168.1.18"):
    from Robotic_Arm.rm_robot_interface import RoboticArm, rm_thread_mode_e

    robot = RoboticArm(rm_thread_mode_e.RM_TRIPLE_MODE_E)
    handle = robot.rm_create_robot_arm(ip, 8080)
    if handle.id < 0:
        raise RuntimeError(f"Cannot connect to RM75 at {ip}")
    try:
        print("arm stop:", robot.rm_set_arm_stop(), flush=True)
        print("arm clear trajectories:", robot.rm_set_arm_delete_trajectory(), flush=True)
        yield robot
    finally:
        try:
            print("arm stop on close:", robot.rm_set_arm_stop(), flush=True)
            print("arm clear on close:", robot.rm_set_arm_delete_trajectory(), flush=True)
        finally:
            print("arm disconnect:", robot.rm_delete_robot_arm(), flush=True)


def read_arm(robot):
    code, angles = robot.rm_get_joint_degree()
    if code:
        raise RuntimeError(f"rm_get_joint_degree returned {code}")
    return angles


def write_arm(robot, angles):
    code = robot.rm_movej_follow(list(angles))
    print("rm_movej_follow:", code, "target_deg:", list(angles), flush=True)
    if code:
        raise RuntimeError(f"rm_movej_follow returned {code}")
