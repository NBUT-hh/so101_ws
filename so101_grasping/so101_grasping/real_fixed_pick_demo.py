#!/usr/bin/env python3
"""Fixed-position pick sequence for a real SO-101 follower arm."""

from __future__ import annotations

import os
import time
import traceback

import rclpy
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive

from .fixed_pick_demo import FixedPickDemo, OBJECT_ID, PLANNING_FRAME, downward_pose


class RealFixedPickDemo(FixedPickDemo):
    """Run the validated pick sequence against the real ros2_control driver."""

    def __init__(self) -> None:
        # A direct invocation must never move physical hardware automatically.
        super().__init__(auto_start_default=False)
        self.declare_parameter("move_to_rest", True)
        self.declare_parameter("add_collision_object", True)

        self.declare_parameter("object_x", 0.24)
        self.declare_parameter("object_y", 0.0)
        self.declare_parameter("object_z", 0.01)
        self.declare_parameter("object_size_x", 0.02)
        self.declare_parameter("object_size_y", 0.02)
        self.declare_parameter("object_size_z", 0.02)

        self.declare_parameter("pregrasp_x", 0.22)
        self.declare_parameter("pregrasp_y", 0.0)
        self.declare_parameter("pregrasp_z", 0.075)
        self.declare_parameter("approach_x", 0.22)
        self.declare_parameter("approach_y", 0.0)
        self.declare_parameter("approach_z", 0.015)
        self.declare_parameter("lift_x", 0.22)
        self.declare_parameter("lift_y", 0.0)
        self.declare_parameter("lift_z", 0.07)

        self.declare_parameter("open_position", 1.50)
        self.declare_parameter("closed_position", -0.16)

    def _float_parameter(self, name: str) -> float:
        return float(self.get_parameter(name).value)

    def _positive_parameter(self, name: str) -> float:
        value = self._float_parameter(name)
        if value <= 0.0:
            raise ValueError(f"{name} must be greater than zero")
        return value

    def _position(self, prefix: str) -> tuple[float, float, float]:
        return (
            self._float_parameter(f"{prefix}_x"),
            self._float_parameter(f"{prefix}_y"),
            self._float_parameter(f"{prefix}_z"),
        )

    def publish_real_collision_object(self) -> None:
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = [
            self._positive_parameter("object_size_x"),
            self._positive_parameter("object_size_y"),
            self._positive_parameter("object_size_z"),
        ]

        pose = Pose()
        pose.position.x = self._float_parameter("object_x")
        pose.position.y = self._float_parameter("object_y")
        pose.position.z = self._float_parameter("object_z")
        pose.orientation.w = 1.0

        msg = CollisionObject()
        msg.header.frame_id = PLANNING_FRAME
        msg.id = OBJECT_ID
        msg.primitives = [primitive]
        msg.primitive_poses = [pose]
        msg.operation = CollisionObject.ADD
        for _ in range(3):
            self.collision_pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.1)
            time.sleep(0.2)
        self.get_logger().info("Added real red_box to MoveIt Planning Scene")

    def remove_collision_object(self) -> None:
        msg = CollisionObject()
        msg.header.frame_id = PLANNING_FRAME
        msg.id = OBJECT_ID
        msg.operation = CollisionObject.REMOVE
        self.collision_pub.publish(msg)
        rclpy.spin_once(self, timeout_sec=0.1)
        time.sleep(0.2)
        self.get_logger().info("Removed red_box from MoveIt world collision geometry")

    def run(self) -> bool:
        if not self.wait_for_controllers():
            return False

        self.get_logger().warn(
            "REAL HARDWARE MODE: verify the workspace and keep emergency stop ready"
        )
        if bool(self.get_parameter("move_to_rest").value):
            if not self.move_named("rest"):
                return False

        add_object = bool(self.get_parameter("add_collision_object").value)
        if add_object:
            self.publish_real_collision_object()

        pregrasp_position = self._position("pregrasp")
        approach_position = self._position("approach")
        lift_position = self._position("lift")
        self.get_logger().info(
            "Real pick targets: "
            f"pregrasp={pregrasp_position}, "
            f"approach={approach_position}, lift={lift_position}"
        )

        if not self.move_pregrasp(downward_pose(*pregrasp_position)):
            return False
        if not self.send_gripper(self._float_parameter("open_position")):
            return False
        if not self.move_pose(
            downward_pose(*approach_position), self.pilz_params, "cartesian approach"
        ):
            return False
        if not self.send_gripper(self._float_parameter("closed_position")):
            return False

        if add_object:
            self.remove_collision_object()
        if not self.move_pose(
            downward_pose(*lift_position), self.pilz_params, "cartesian lift"
        ):
            return False

        self.get_logger().info("Real fixed pick sequence succeeded")
        return True


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RealFixedPickDemo()
    exit_code = 0
    try:
        if bool(node.get_parameter("auto_start").value):
            if not node.run():
                raise RuntimeError("real fixed pick sequence failed")
        else:
            node.get_logger().warn(
                "Real arm is ready but auto_start=false; no motion will be executed"
            )
            rclpy.spin(node)
    except KeyboardInterrupt:
        exit_code = 130
    except Exception:
        traceback.print_exc()
        exit_code = 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        # Avoid the known MoveItPy/MoveItCpp destruction crash on Jazzy.
        os._exit(exit_code)


if __name__ == "__main__":
    main()
