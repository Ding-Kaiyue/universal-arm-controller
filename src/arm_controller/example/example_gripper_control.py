#!/usr/bin/env python3

import argparse
import os
import sys
import time


def try_import_from_workspace():
    current_dir = os.path.dirname(os.path.abspath(__file__))
    candidates = [
        os.path.join(current_dir, "..", "..", "install", "arm_controller", "lib", "arm_controller"),
        os.path.join(current_dir, "..", "..", "..", "install", "arm_controller", "lib", "arm_controller"),
    ]

    for candidate in candidates:
        candidate = os.path.abspath(candidate)
        if os.path.isdir(candidate) and candidate not in sys.path:
            sys.path.insert(0, candidate)

    import arm_controller_py as ac
    return ac


def parse_args():
    parser = argparse.ArgumentParser(
        description="ARM Controller gripper IPC Python demo"
    )
    parser.add_argument(
        "--mapping",
        default="left_gripper",
        help="Target gripper mapping, for example right_gripper or left_gripper",
    )
    parser.add_argument(
        "--open-position",
        type=int,
        default=255,
        help="Open position in raw scale [0, 255]",
    )
    parser.add_argument(
        "--close-position",
        type=int,
        default=0,
        help="Close position in raw scale [0, 255]",
    )
    parser.add_argument(
        "--velocity",
        type=int,
        default=128,
        help="Velocity in raw scale [0, 255]",
    )
    parser.add_argument(
        "--effort",
        type=int,
        default=128,
        help="Effort in raw scale [0, 255]",
    )
    parser.add_argument(
        "--gripper-type",
        type=int,
        default=-1,
        help="Gripper type: 0=OmniPicker, 1=PGC, -1=auto by mapping",
    )
    parser.add_argument(
        "--pause",
        type=float,
        default=0.8,
        help="Pause in seconds between open and close",
    )
    return parser.parse_args()


def main():
    args = parse_args()
    ac = try_import_from_workspace()

    print("===============================================================")
    print("ARM Controller Gripper IPC Python Demo")
    print("===============================================================\n")

    if not ac.IPCLifecycle.initialize():
        print("IPC initialize failed")
        raise SystemExit(1)

    gripper = ac.BasicOpsIPCInterface()

    print(
        f"mapping={args.mapping}, open={args.open_position}, close={args.close_position}, "
        f"velocity={args.velocity}, effort={args.effort}, gripper_type={args.gripper_type}"
    )

    if not gripper.gripper_control(
        args.open_position,
        args.mapping,
        args.velocity,
        args.effort,
        args.gripper_type,
    ):
        print(f"open failed: {gripper.get_last_error()}")
        raise SystemExit(1)

    time.sleep(args.pause)

    if not gripper.gripper_control(
        args.close_position,
        args.mapping,
        args.velocity,
        args.effort,
        args.gripper_type,
    ):
        print(f"close failed: {gripper.get_last_error()}")
        raise SystemExit(1)

    print(
        f"Gripper demo finished, mode={gripper.get_current_mode(args.mapping)}, "
        f"state={gripper.get_execution_state(args.mapping)}"
    )


if __name__ == "__main__":
    main()
