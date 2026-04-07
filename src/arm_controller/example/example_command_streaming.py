#!/usr/bin/env python3

import argparse
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


def pack_command(positions, velocities=None, efforts=None):
    if velocities is None and efforts is None:
        return list(positions)
    if velocities is None or efforts is None:
        raise ValueError("velocities and efforts must both be provided for full command mode")
    return list(positions) + list(velocities) + list(efforts)


def stream_command(ac, mapping, positions, velocities, efforts, duration_s, interval_s):
    controller = ac.CommandStreamingIPCInterface()
    packed = pack_command(positions, velocities, efforts)
    if velocities is None and efforts is None:
        stop_cmd = list(positions)
    else:
        stop_cmd = pack_command(positions, [0.0] * len(velocities), efforts)

    deadline = time.monotonic() + duration_s
    send_count = 0

    while time.monotonic() < deadline:
        if not controller.execute(packed, mapping):
            raise RuntimeError(
                f"[{mapping}] command streaming failed: {controller.get_last_error()}"
            )
        send_count += 1
        time.sleep(interval_s)

    if not controller.execute(stop_cmd, mapping):
        raise RuntimeError(
            f"[{mapping}] stop command failed: {controller.get_last_error()}"
        )

    print(
        f"[{mapping}] sent {send_count} commands, "
        f"mode={controller.get_current_mode(mapping)}, "
        f"state={controller.get_execution_state(mapping)}"
    )


def parse_args():
    parser = argparse.ArgumentParser(
        description="Test CommandStreaming mode. Position-only input is supported."
    )
    parser.add_argument(
        "--mapping",
        choices=["left_arm", "right_arm", "both"],
        default="left_arm",
        help="Target arm mapping",
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=1.0,
        help="Stream duration in seconds",
    )
    parser.add_argument(
        "--interval",
        type=float,
        default=0.005,
        help="Send interval in seconds",
    )
    parser.add_argument(
        "--positions",
        type=float,
        nargs=6,
        default=[1.36, 1.48, 0.31, 0.0, 0.47, 0.96],
        help="Joint positions in radians",
    )
    parser.add_argument(
        "--velocities",
        type=float,
        nargs=6,
        default=None,
        help="Joint velocities in radians/s (optional; omit with --efforts for position-only mode)",
    )
    parser.add_argument(
        "--efforts",
        type=float,
        nargs=6,
        default=None,
        help="Joint efforts / torques (optional; omit with --velocities for position-only mode)",
    )
    args = parser.parse_args()

    has_vel = args.velocities is not None
    has_eff = args.efforts is not None
    if has_vel != has_eff:
        parser.error("Provide both --velocities and --efforts, or provide neither for position-only mode")

    return args


def main():
    args = parse_args()
    ac = try_import_from_workspace()

    if not ac.IPCLifecycle.initialize():
        raise RuntimeError("IPC initialize failed")

    print("IPC initialize success")
    if args.velocities is None and args.efforts is None:
        print(
            f"mapping={args.mapping}, duration={args.duration}s, interval={args.interval}s, "
            f"positions={args.positions}, mode=position_only"
        )
    else:
        print(
            f"mapping={args.mapping}, duration={args.duration}s, interval={args.interval}s, "
            f"positions={args.positions}, velocities={args.velocities}, efforts={args.efforts}"
        )

    mappings = ["left_arm", "right_arm"] if args.mapping == "both" else [args.mapping]
    threads = [
        threading.Thread(
            target=stream_command,
            args=(
                ac,
                mapping,
                args.positions,
                args.velocities,
                args.efforts,
                args.duration,
                args.interval,
            ),
            daemon=False,
        )
        for mapping in mappings
    ]

    start_time = time.monotonic()
    for thread in threads:
        thread.start()
    for thread in threads:
        thread.join()
    elapsed = time.monotonic() - start_time

    print(f"CommandStreaming test finished in {elapsed:.3f}s")


if __name__ == "__main__":
    main()
