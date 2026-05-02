#!/usr/bin/env python
"""Nudge one calibrated SO-100 follower joint by a small amount."""

from __future__ import annotations

import argparse
import sys
import time

from lerobot.robots.so_follower.config_so_follower import SOFollowerRobotConfig
from lerobot.robots.so_follower.so_follower import SOFollower

JOINTS = (
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
    "gripper",
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default="/dev/ttyACM0", help="Follower motor controller serial port.")
    parser.add_argument("--robot-id", default="so100_follower", help="Calibration id to load.")
    parser.add_argument("--joint", choices=JOINTS, default="shoulder_pan", help="Joint to nudge.")
    parser.add_argument(
        "--delta",
        type=float,
        default=2.0,
        help="Nudge amount. Arm joints use degrees; gripper uses percentage points.",
    )
    parser.add_argument(
        "--max-delta",
        type=float,
        default=5.0,
        help="Refuse moves with an absolute delta larger than this.",
    )
    parser.add_argument("--settle-s", type=float, default=1.0, help="Seconds to wait before reading back.")
    parser.add_argument(
        "--leave-torque-on",
        action="store_true",
        help="Keep motors holding position after the script exits.",
    )
    parser.add_argument(
        "--execute",
        action="store_true",
        help="Actually move the joint. Without this flag, only print the planned move.",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if abs(args.delta) > args.max_delta:
        print(f"Refusing delta {args.delta}; max allowed is +/-{args.max_delta}.", file=sys.stderr)
        return 2

    robot = SOFollower(
        SOFollowerRobotConfig(
            port=args.port,
            id=args.robot_id,
            disable_torque_on_disconnect=not args.leave_torque_on,
        )
    )

    try:
        robot.connect()
        positions = robot.bus.sync_read("Present_Position", JOINTS, num_retry=5)
        hold_action = {f"{joint}.pos": float(positions[joint]) for joint in JOINTS}
        key = f"{args.joint}.pos"
        current = hold_action[key]
        target = current + args.delta

        print(f"{args.joint}: current={current:.2f}, target={target:.2f}, delta={args.delta:+.2f}")
        if not args.execute:
            print("Dry run only. Re-run with --execute to send the target.")
            return 0

        robot.send_action(hold_action)
        time.sleep(0.2)

        action = hold_action.copy()
        action[key] = target
        sent = robot.send_action(action)
        print(f"sent {args.joint}: {sent[key]:.2f}")
        time.sleep(args.settle_s)

        try:
            after = float(
                robot.bus.read("Present_Position", args.joint, normalize=True, num_retry=5)
            )
            print(f"{args.joint}: after={after:.2f}")
        except Exception as exc:
            print(f"Warning: readback failed after sending target: {exc}", file=sys.stderr)
        return 0
    finally:
        if robot.is_connected:
            try:
                robot.disconnect()
            except Exception as exc:
                print(f"Warning: disconnect failed: {exc}", file=sys.stderr)


if __name__ == "__main__":
    raise SystemExit(main())
