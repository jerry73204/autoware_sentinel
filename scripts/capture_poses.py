#!/usr/bin/env python3
"""
Capture initial pose and goal pose from RViz and save them to a YAML file.

Subscribes to the same topics that Autoware's RViz adaptors use:
  - /initialpose (geometry_msgs/PoseWithCovarianceStamped) — "2D Pose Estimate" tool
  - /planning/mission_planning/goal (geometry_msgs/PoseStamped) — "2D Nav Goal" tool

Workflow:
  1. Start this script
  2. In RViz, click "2D Pose Estimate" and place the initial pose on the map
  3. In RViz, click "2D Nav Goal" and place the goal pose on the map
  4. The script saves both poses to a YAML file and exits

Usage:
    python3 scripts/capture_poses.py
    python3 scripts/capture_poses.py -o tmp/my_route.yaml

The output file is read by auto_drive.py:
    python3 scripts/auto_drive.py --poses scripts/poses.yaml
"""

import argparse
import pathlib
import sys
import yaml

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped

SCRIPT_DIR = pathlib.Path(__file__).resolve().parent
DEFAULT_OUTPUT = str(SCRIPT_DIR / "poses.yaml")


def pose_to_dict(position, orientation):
    """Convert position + orientation to a flat dict."""
    return {
        "x": float(position.x),
        "y": float(position.y),
        "z": float(position.z),
        "qx": float(orientation.x),
        "qy": float(orientation.y),
        "qz": float(orientation.z),
        "qw": float(orientation.w),
    }


def covariance_to_list(covariance):
    """Extract the 3 diagonal entries we care about (x, y, yaw)."""
    return {
        "xx": float(covariance[0]),
        "yy": float(covariance[7]),
        "yaw_yaw": float(covariance[35]),
    }


class CapturePosesNode(Node):
    """Captures initial pose and goal pose from RViz topics."""

    def __init__(self):
        super().__init__("capture_poses")
        self.initial_pose = None
        self.initial_covariance = None
        self.goal_pose = None

        # /initialpose — RViz "2D Pose Estimate"
        self.create_subscription(
            PoseWithCovarianceStamped,
            "/initialpose",
            self._on_initial_pose,
            10,
        )

        # /planning/mission_planning/goal — RViz "2D Nav Goal"
        self.create_subscription(
            PoseStamped,
            "/planning/mission_planning/goal",
            self._on_goal_pose,
            10,
        )

        # Also listen on /rviz/routing/rough_goal (Autoware RoutePanel)
        self.create_subscription(
            PoseStamped,
            "/rviz/routing/rough_goal",
            self._on_goal_pose,
            10,
        )

        self.get_logger().info("Waiting for poses from RViz...")
        self.get_logger().info("  1. Use '2D Pose Estimate' to set initial pose")
        self.get_logger().info("  2. Use '2D Nav Goal' to set goal pose")

    def _on_initial_pose(self, msg: PoseWithCovarianceStamped):
        pose = msg.pose.pose
        self.initial_pose = pose_to_dict(pose.position, pose.orientation)
        self.initial_covariance = covariance_to_list(msg.pose.covariance)
        self.get_logger().info(
            f"Initial pose captured: "
            f"({self.initial_pose['x']:.2f}, {self.initial_pose['y']:.2f}, "
            f"{self.initial_pose['z']:.2f})"
        )
        self._check_complete()

    def _on_goal_pose(self, msg: PoseStamped):
        pose = msg.pose
        self.goal_pose = pose_to_dict(pose.position, pose.orientation)
        self.get_logger().info(
            f"Goal pose captured: "
            f"({self.goal_pose['x']:.2f}, {self.goal_pose['y']:.2f}, "
            f"{self.goal_pose['z']:.2f})"
        )
        self._check_complete()

    def _check_complete(self):
        if self.initial_pose and self.goal_pose:
            self.get_logger().info("Both poses captured!")

    def is_complete(self) -> bool:
        return self.initial_pose is not None and self.goal_pose is not None

    def to_dict(self) -> dict:
        data = {
            "initial_pose": self.initial_pose,
            "goal_pose": self.goal_pose,
        }
        if self.initial_covariance:
            data["initial_covariance"] = self.initial_covariance
        return data


def main():
    parser = argparse.ArgumentParser(
        description="Capture initial and goal poses from RViz"
    )
    parser.add_argument(
        "-o", "--output",
        default=DEFAULT_OUTPUT,
        help=f"Output YAML file (default: {DEFAULT_OUTPUT})",
    )
    parser.add_argument(
        "--wait-both",
        action="store_true",
        default=True,
        help="Wait until both poses are captured before saving (default)",
    )
    args = parser.parse_args()

    rclpy.init()
    node = CapturePosesNode()

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.5)
            if node.is_complete():
                break

        if not node.is_complete():
            node.get_logger().error("Interrupted before both poses were captured")
            sys.exit(1)

        # Save to YAML
        data = node.to_dict()
        with open(args.output, "w") as f:
            yaml.dump(data, f, default_flow_style=False, sort_keys=False)

        node.get_logger().info(f"Poses saved to {args.output}")
        print(f"\nSaved to: {args.output}")
        print(f"Use with: python3 scripts/auto_drive.py --poses {args.output}")

    except KeyboardInterrupt:
        # Save whatever we have if at least one pose was captured
        if node.initial_pose or node.goal_pose:
            data = node.to_dict()
            with open(args.output, "w") as f:
                yaml.dump(data, f, default_flow_style=False, sort_keys=False)
            node.get_logger().info(
                f"Interrupted — saved partial poses to {args.output}"
            )
        else:
            node.get_logger().info("Interrupted — no poses captured")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
