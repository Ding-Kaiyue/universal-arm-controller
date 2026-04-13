#!/usr/bin/env python3
import time
import arm_controller_ipc as acipc


def require_ok(ok: bool, component, action: str) -> None:
    if ok:
        return
    raise RuntimeError(f"{action} failed: {component.get_last_error()}")


def main() -> None:
    if not acipc.initialize_producer():
        raise RuntimeError("IPC init failed: make sure consumer node is running first")

    movej = acipc.MoveJ()

    try:
        require_ok(
            movej.execute([-1.57, -0.3236, -0.5854, 0.0, 0.5236, 0.0], "left_arm"),
            movej,
            "MoveJ",
        )

        # Poll async execution state with clearer output.
        timeout_s = 10.0
        poll_s = 0.05
        deadline = time.monotonic() + timeout_s
        last_state = None
        seen_active = False

        while time.monotonic() < deadline:
            state = movej.get_execution_state("left_arm")
            if state != last_state:
                print(f"state={state.name}")
                last_state = state

            if state in (acipc.ExecutionState.PENDING, acipc.ExecutionState.EXECUTING):
                seen_active = True

            if state in (acipc.ExecutionState.SUCCESS, acipc.ExecutionState.FAILED):
                print(f"final state={state.name}")
                return

            # Some controllers may go back to IDLE after execution without
            # leaving SUCCESS/FAILED visible long enough for coarse polling.
            if state == acipc.ExecutionState.IDLE and seen_active:
                print("final state=IDLE (execution likely finished)")
                return

            time.sleep(poll_s)

        print(f"timeout after {timeout_s:.1f}s, last_state={last_state.name if last_state else 'UNKNOWN'}")
    finally:
        # Participant side shutdown is local-context cleanup only.
        acipc.shutdown()


if __name__ == "__main__":
    main()
