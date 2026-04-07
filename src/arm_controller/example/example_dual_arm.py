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
        description="ARM Controller MoveJ IPC Python demo"
    )
    parser.add_argument(
        "--mapping",
        choices=["left_arm", "right_arm", "both"],
        default="left_arm",
        help="Target arm mapping",
    )
    parser.add_argument(
        "--positions",
        nargs=6,
        type=float,
        default=[0, 0.52, 1.55, 1.55, 1.55, 2.6],
        metavar=("J1", "J2", "J3", "J4", "J5", "J6"),
        help="Target joint positions in radians",
    )
    parser.add_argument(
        "--wait",
        action="store_true",
        help="Wait until the controller reports SUCCESS or FAILED",
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=15.0,
        help="Wait timeout in seconds when --wait is enabled",
    )
    parser.add_argument(
        "--poll-interval",
        type=float,
        default=0.1,
        help="Polling interval in seconds when --wait is enabled",
    )
    return parser.parse_args()


def wait_for_completion(ac, movej, mapping, timeout_s, poll_interval_s):
    deadline = time.monotonic() + timeout_s

    while time.monotonic() < deadline:
        state = movej.get_execution_state(mapping)
        if state == ac.ExecutionState.SUCCESS:
            print(f"[{mapping}] execution succeeded")
            return True
        if state == ac.ExecutionState.FAILED:
            print(f"[{mapping}] execution failed: {movej.get_last_error()}")
            return False
        time.sleep(poll_interval_s)

    print(f"[{mapping}] wait timeout, last_state={movej.get_execution_state(mapping)}")
    return False


def send_movej(ac, mapping, positions, wait, timeout_s, poll_interval_s):
    movej = ac.MoveJIPCInterface()

    print(f"Sending MoveJ command -> {mapping}: {positions}")
    if not movej.execute(positions, mapping):
        print(f"[{mapping}] failed: {movej.get_last_error()}")
        return False

    print(
        f"[{mapping}] queued, mode={movej.get_current_mode(mapping)}, "
        f"state={movej.get_execution_state(mapping)}"
    )

    if wait:
        return wait_for_completion(ac, movej, mapping, timeout_s, poll_interval_s)
    return True


def main():
    args = parse_args()
    ac = try_import_from_workspace()

    print("===============================================================")
    print("ARM Controller MoveJ Python Demo")
    print("===============================================================\n")

    if not ac.IPCLifecycle.initialize():
        print("IPC initialize failed")
        raise SystemExit(1)

    mappings = ["left_arm", "right_arm"] if args.mapping == "both" else [args.mapping]

    success = True
    for mapping in mappings:
        ok = send_movej(
            ac,
            mapping,
            args.positions,
            args.wait,
            args.timeout,
            args.poll_interval,
        )
        success = success and ok

    ac.IPCLifecycle.shutdown()

    if not success:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
