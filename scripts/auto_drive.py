#!/usr/bin/env python3
"""
Autonomous driving sequence for Autoware planning simulator.

Waits for Autoware services, initializes localization, sets a route, engages
autonomous mode, and waits for the vehicle to arrive at the goal.  Uses the
Autoware AD API (v1) services and topics.

Usage:
    python3 scripts/auto_drive.py --poses scripts/poses.yaml
    python3 scripts/auto_drive.py --poses scripts/poses.yaml --timeout 180

Poses can be captured from RViz with:
    python3 scripts/capture_poses.py -o scripts/poses.yaml

Requires:
    - ROS 2 Humble with Autoware 1.5.0 message packages
    - Autoware planning simulator running (via play_launch or launch files)
    - A poses YAML file (see scripts/poses.yaml for the format)
"""

import argparse
import sys
import time

import yaml
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

from autoware_adapi_v1_msgs.msg import (
    LocalizationInitializationState,
    OperationModeState,
    RouteState,
)
from autoware_adapi_v1_msgs.srv import (
    ChangeOperationMode,
    ClearRoute,
    InitializeLocalization,
    SetRoutePoints,
)
from geometry_msgs.msg import PoseWithCovarianceStamped


_LOC_STATE_NAMES = {
    LocalizationInitializationState.UNKNOWN: "UNKNOWN",
    LocalizationInitializationState.UNINITIALIZED: "UNINITIALIZED",
    LocalizationInitializationState.INITIALIZING: "INITIALIZING",
    LocalizationInitializationState.INITIALIZED: "INITIALIZED",
}


def load_poses(path: str) -> dict:
    with open(path) as f:
        data = yaml.safe_load(f)
    if "goal_pose" not in data:
        raise ValueError(f"Missing 'goal_pose' key in {path}")
    return data


class AutoDriveNode(Node):
    """Automates the full autonomous driving sequence via Autoware AD API."""

    def __init__(self, poses: dict, timeout: float):
        super().__init__("auto_drive")
        self.poses = poses
        self.timeout = timeout

        # State
        self.localization_state = LocalizationInitializationState.UNKNOWN
        self.route_state = RouteState.UNKNOWN
        self.op_mode_state = None

        # QoS for AD API state topics (transient local = get last value on subscribe)
        state_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        # Subscribers
        self.create_subscription(
            LocalizationInitializationState,
            "/api/localization/initialization_state",
            self._on_localization_state,
            state_qos,
        )
        self.create_subscription(
            RouteState,
            "/api/routing/state",
            self._on_route_state,
            state_qos,
        )
        self.create_subscription(
            OperationModeState,
            "/api/operation_mode/state",
            self._on_op_mode_state,
            state_qos,
        )

        # Service clients
        self.init_loc_client = self.create_client(
            InitializeLocalization, "/api/localization/initialize"
        )
        self.clear_route_client = self.create_client(
            ClearRoute, "/api/routing/clear_route"
        )
        self.set_route_client = self.create_client(
            SetRoutePoints, "/api/routing/set_route_points"
        )
        self.change_to_auto_client = self.create_client(
            ChangeOperationMode, "/api/operation_mode/change_to_autonomous"
        )

        # Also publish to /initialpose as fallback
        self.initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, "/initialpose", 1
        )

    # ── State callbacks ──────────────────────────────────────────────

    def _on_localization_state(self, msg):
        self.localization_state = msg.state

    def _on_route_state(self, msg):
        self.route_state = msg.state

    def _on_op_mode_state(self, msg):
        self.op_mode_state = msg

    # ── Helpers ──────────────────────────────────────────────────────

    def _spin_for(self, seconds: float):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.1)

    def _wait_for_service(self, client, name: str, timeout: float = 60.0) -> bool:
        self.get_logger().info(f"  Waiting for {name}...")
        if client.wait_for_service(timeout_sec=timeout):
            self.get_logger().info(f"  {name} ready")
            return True
        self.get_logger().error(f"  {name} not available after {timeout}s")
        return False

    def _call_service(self, client, request, name: str, timeout: float = 60.0):
        future = client.call_async(request)
        end = time.monotonic() + timeout
        while not future.done() and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.1)
        if not future.done():
            self.get_logger().error(f"  {name} call timed out after {timeout}s")
            return None
        return future.result()

    # ── Steps ────────────────────────────────────────────────────────

    def step1_wait_for_services(self) -> bool:
        """Wait for Autoware AD API services."""
        self.get_logger().info("=== Step 1: Waiting for Autoware AD API services ===")
        ok = self._wait_for_service(
            self.clear_route_client, "/api/routing/clear_route"
        )
        ok = ok and self._wait_for_service(
            self.set_route_client, "/api/routing/set_route_points"
        )
        ok = ok and self._wait_for_service(
            self.change_to_auto_client, "/api/operation_mode/change_to_autonomous"
        )
        return ok

    def step2_initialize_localization(self) -> bool:
        """Publish initial pose and optionally call the localization service."""
        self.get_logger().info("=== Step 2: Initialize localization ===")

        initial = self.poses.get("initial_pose")
        if not initial:
            self.get_logger().info("  No initial_pose in poses file, skipping")
            return True

        cov = self.poses.get("initial_covariance", {})

        # Publish /initialpose topic
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = initial["x"]
        msg.pose.pose.position.y = initial["y"]
        msg.pose.pose.position.z = initial["z"]
        msg.pose.pose.orientation.x = initial["qx"]
        msg.pose.pose.orientation.y = initial["qy"]
        msg.pose.pose.orientation.z = initial["qz"]
        msg.pose.pose.orientation.w = initial["qw"]
        msg.pose.covariance[0] = cov.get("xx", 0.25)
        msg.pose.covariance[7] = cov.get("yy", 0.25)
        msg.pose.covariance[35] = cov.get("yaw_yaw", 0.07)
        self.initial_pose_pub.publish(msg)
        self.get_logger().info("  Published /initialpose")

        # Also call the service if available
        if self.init_loc_client.wait_for_service(timeout_sec=5.0):
            req = InitializeLocalization.Request()
            req.pose = [msg]
            resp = self._call_service(
                self.init_loc_client, req, "InitializeLocalization", timeout=10.0
            )
            if resp and resp.status.success:
                self.get_logger().info("  Localization initialized via service")
        return True

    def step3_wait_for_localization(self, timeout: float) -> bool:
        """Wait for localization to reach INITIALIZED state."""
        self.get_logger().info(
            f"=== Step 3: Waiting for localization (up to {timeout:.0f}s) ==="
        )
        end = time.monotonic() + timeout
        last_log = time.monotonic()
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.5)
            if self.localization_state == LocalizationInitializationState.INITIALIZED:
                self.get_logger().info("  Localization: INITIALIZED")
                return True
            now = time.monotonic()
            if now - last_log >= 5.0:
                name = _LOC_STATE_NAMES.get(
                    self.localization_state, str(self.localization_state)
                )
                self.get_logger().info(f"  Localization state: {name}")
                last_log = now
        self.get_logger().error(
            f"  Localization did not reach INITIALIZED within {timeout:.0f}s"
        )
        return False

    def step4_set_route(self) -> bool:
        """Clear any existing route and set a new one."""
        self.get_logger().info("=== Step 4: Setting route ===")

        # Clear first
        self.get_logger().info("  Clearing existing route...")
        self._call_service(
            self.clear_route_client, ClearRoute.Request(), "ClearRoute"
        )
        self._spin_for(2.0)

        goal = self.poses["goal_pose"]
        req = SetRoutePoints.Request()
        req.header.frame_id = "map"
        req.header.stamp = self.get_clock().now().to_msg()
        req.option.allow_goal_modification = False
        req.goal.position.x = goal["x"]
        req.goal.position.y = goal["y"]
        req.goal.position.z = goal["z"]
        req.goal.orientation.x = goal["qx"]
        req.goal.orientation.y = goal["qy"]
        req.goal.orientation.z = goal["qz"]
        req.goal.orientation.w = goal["qw"]
        req.waypoints = []

        for attempt in range(3):
            if attempt > 0:
                self.get_logger().info(f"  Retry {attempt}/2 in 5s...")
                self._spin_for(5.0)
            resp = self._call_service(
                self.set_route_client, req, "SetRoutePoints"
            )
            if resp and resp.status.success:
                self.get_logger().info("  Route set successfully")
                return True
            if resp:
                self.get_logger().warn(
                    f"  Route failed (attempt {attempt + 1}): {resp.status.message}"
                )
        self.get_logger().error("  Failed to set route after 3 attempts")
        return False

    def step5_wait_for_route(self, timeout: float = 15.0) -> bool:
        """Wait for route state to become SET."""
        self.get_logger().info(
            f"=== Step 5: Waiting for route to be planned ({timeout}s max) ==="
        )
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.5)
            if self.route_state == RouteState.SET:
                self.get_logger().info("  Route state: SET")
                return True
            if self.route_state == RouteState.ARRIVED:
                self.get_logger().info("  Route state: ARRIVED (already at goal)")
                return True
        self.get_logger().warn(
            f"  Route not SET after {timeout}s (state={self.route_state})"
        )
        return self.route_state == RouteState.SET

    def step6_engage_autonomous(self, timeout: float = 120.0) -> bool:
        """Wait for autonomous mode to become available, then engage.

        First waits for is_autonomous_mode_available = True (the trajectory
        pipeline must be producing output), then calls change_to_autonomous.
        """
        self.get_logger().info(
            f"=== Step 6: Engaging autonomous mode (up to {timeout:.0f}s) ==="
        )
        end = time.monotonic() + timeout

        # Wait for autonomous mode to be available
        self.get_logger().info("  Waiting for is_autonomous_mode_available...")
        last_log = time.monotonic()
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.5)
            if (
                self.op_mode_state is not None
                and self.op_mode_state.is_autonomous_mode_available
            ):
                self.get_logger().info("  Autonomous mode is available")
                break
            now = time.monotonic()
            if now - last_log >= 10.0:
                if self.op_mode_state is not None:
                    s = self.op_mode_state
                    self.get_logger().info(
                        f"  mode={s.mode}, auto_avail={s.is_autonomous_mode_available}, "
                        f"control={s.is_autoware_control_enabled}"
                    )
                else:
                    self.get_logger().info(
                        "  No /api/operation_mode/state received yet"
                    )
                last_log = now
        else:
            self.get_logger().error(
                f"  Autonomous mode never became available after {timeout:.0f}s"
            )
            return False

        # Engage with retry.  Use a short per-call timeout (10s) so we retry
        # frequently — zenoh-pico queryable routing through zenohd can take
        # 30-60s non-deterministically, and a single long timeout wastes the
        # entire budget on one attempt.
        req = ChangeOperationMode.Request()
        attempt = 0
        while time.monotonic() < end:
            attempt += 1
            resp = self._call_service(
                self.change_to_auto_client, req, "ChangeOperationMode",
                timeout=10.0,
            )
            if resp and resp.status.success:
                self.get_logger().info(
                    f"  Autonomous mode engaged (attempt {attempt})"
                )
                return True
            if resp:
                self.get_logger().info(
                    f"  Engage failed: {resp.status.message} — retrying..."
                )
            self._spin_for(3.0)

        self.get_logger().error(
            f"  Failed to engage autonomous mode after {timeout:.0f}s"
        )
        return False

    def step7_wait_for_arrival(self, timeout: float) -> bool:
        """Wait for route state to become ARRIVED."""
        self.get_logger().info(
            f"=== Step 7: Driving to goal (up to {timeout:.0f}s) ==="
        )
        start = time.monotonic()
        end = start + timeout
        last_log = start
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.5)
            now = time.monotonic()
            if self.route_state == RouteState.ARRIVED:
                self.get_logger().info(
                    f"  ARRIVED at goal after {now - start:.1f}s"
                )
                return True
            if now - last_log >= 10.0:
                mode = self.op_mode_state.mode if self.op_mode_state else "?"
                self.get_logger().info(
                    f"  Driving... {now - start:.0f}s elapsed, "
                    f"route_state={self.route_state}, op_mode={mode}"
                )
                last_log = now
        self.get_logger().warn(f"  Did not arrive within {timeout:.0f}s")
        return False

    # ── Main sequence ────────────────────────────────────────────────

    def run(self) -> bool:
        self.get_logger().info(f"Auto-drive starting (timeout={self.timeout}s)")
        t0 = time.monotonic()

        if not self.step1_wait_for_services():
            return False

        self.step2_initialize_localization()

        elapsed = time.monotonic() - t0
        if not self.step3_wait_for_localization(timeout=min(self.timeout - elapsed, 60)):
            return False

        if not self.step4_set_route():
            return False

        if not self.step5_wait_for_route():
            return False

        elapsed = time.monotonic() - t0
        if not self.step6_engage_autonomous(timeout=min(self.timeout - elapsed, 120)):
            return False

        elapsed = time.monotonic() - t0
        arrived = self.step7_wait_for_arrival(
            timeout=max(self.timeout - elapsed, 30.0)
        )

        total = time.monotonic() - t0
        self.get_logger().info(
            f"Auto-drive {'completed' if arrived else 'timed out'} in {total:.1f}s"
        )
        return arrived


def main():
    parser = argparse.ArgumentParser(description="Autoware autonomous driving sequence")
    parser.add_argument(
        "--poses",
        required=True,
        help="Path to YAML file with initial_pose and goal_pose",
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=180.0,
        help="Overall timeout in seconds (default: 180)",
    )
    args = parser.parse_args()

    poses = load_poses(args.poses)
    print(f"Loaded poses from {args.poses}")

    rclpy.init()
    node = AutoDriveNode(poses, args.timeout)
    try:
        arrived = node.run()
        if not arrived:
            sys.exit(1)
    except KeyboardInterrupt:
        node.get_logger().info("Interrupted")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
