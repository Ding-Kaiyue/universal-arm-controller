#!/usr/bin/env python3

import os
import sys
import threading
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


def move_arm(ac, mapping, velocity, duration_ms):
    joint_velocity = ac.JointVelocityIPCInterface()
    interval_s = 0.005
    end_time = time.monotonic() + duration_ms / 1000.0

    while time.monotonic() < end_time:
        if not joint_velocity.execute(velocity, mapping):
            print(f"[{mapping}] failed: {joint_velocity.get_last_error()}")
            break
        time.sleep(interval_s)

    joint_velocity.execute([0.0] * len(velocity), mapping)


def move_cartesian_arm(ac, mapping, velocity, duration_ms):
    cartesian_velocity = ac.CartesianVelocityIPCInterface()
    interval_s = 0.005
    end_time = time.monotonic() + duration_ms / 1000.0

    while time.monotonic() < end_time:
        if not cartesian_velocity.execute(velocity, mapping):
            print(f"[{mapping}] failed: {cartesian_velocity.get_last_error()}")
            break
        time.sleep(interval_s)

    cartesian_velocity.execute([0.0] * len(velocity), mapping)


def main():
    ac = try_import_from_workspace()

    print("===============================================================")
    print("ARM Controller IPC Python Demo")
    print("===============================================================\n")

    if not ac.IPCLifecycle.initialize():
        print("IPC initialize failed")
        raise SystemExit(1)

    duration_ms = 1000

    print("========== JointVelocity dual-arm demo (1 second) ==========")
    print("Sending continuous JointVelocity commands to left_arm and right_arm...")

    left_thread = threading.Thread(
        target=move_arm,
        args=(ac, "left_arm", [0.0, 0.0, 0.0, 0.0, 0.0, -0.2], duration_ms),
    )
    right_thread = threading.Thread(
        target=move_arm,
        args=(ac, "right_arm", [0.0, 0.0, 0.0, 0.0, 0.0, -0.2], duration_ms),
    )

    left_thread.start()
    right_thread.start()
    left_thread.join()
    right_thread.join()

    print("Motion complete")


if __name__ == "__main__":
    main()
