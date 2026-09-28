#!/usr/bin/env python3
"""Fixed-position SO-101 pick sequence using MoveItPy and controller actions."""

from __future__ import annotations

import os
import time
import traceback

import rclpy
from control_msgs.action import FollowJointTrajectory, GripperCommand
from geometry_msgs.msg import Pose, PoseStamped
from moveit.core.robot_state import RobotState
from moveit.planning import MoveItPy, MultiPipelinePlanRequestParameters
from moveit_msgs.msg import CollisionObject
from rclpy.action import ActionClient
from rclpy.node import Node
from shape_msgs.msg import SolidPrimitive
from tf_transformations import quaternion_from_euler


PLANNING_FRAME = "world"
PLANNING_GROUP = "manipulator"
EE_FRAME = "gripper_frame_link"
OBJECT_ID = "red_box"
GRIPPER_ACTION = "/follower/gripper_controller/gripper_cmd"
ARM_ACTION = "/follower/arm_trajectory_controller/follow_joint_trajectory"

OBJECT_X = 0.24
OBJECT_Y = 0.0
OBJECT_Z = 0.01
OBJECT_SIZE = (0.02, 0.02, 0.02)

PREGRASP_Z = 0.075
APPROACH_Z = 0.035
LIFT_Z = 0.07
OPEN_POSITION = 1.50
CLOSED_POSITION = -0.16


def downward_pose(x: float, y: float, z: float) -> PoseStamped:
    q = quaternion_from_euler(0.0, 3.141592653589793, 0.0)
    pose = PoseStamped()
    pose.header.frame_id = PLANNING_FRAME
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = z
    pose.pose.orientation.x = q[0]
    pose.pose.orientation.y = q[1]
    pose.pose.orientation.z = q[2]
    pose.pose.orientation.w = q[3]
    return pose


class FixedPickDemo(Node):
    def __init__(self, auto_start_default: bool = True) -> None:
        super().__init__("fixed_pick_demo_driver")
        self.declare_parameter("auto_start", auto_start_default)

        self.gripper_client = ActionClient(self, GripperCommand, GRIPPER_ACTION)
        self.arm_client = ActionClient(self, FollowJointTrajectory, ARM_ACTION)
        self.collision_pub = self.create_publisher(
            CollisionObject, "/collision_object", 10
        )

        self.robot = MoveItPy(
            node_name="fixed_pick_demo",
            remappings={"joint_states": "/follower/joint_states"},
        )
        self.arm = self.robot.get_planning_component(PLANNING_GROUP)
        self.ompl_params = MultiPipelinePlanRequestParameters(
            self.robot, ["ompl_rrtc"]
        )
        self.pilz_params = MultiPipelinePlanRequestParameters(
            self.robot, ["pilz_lin"]
        )

    def wait_for_controllers(self) -> bool:
        self.get_logger().info("Waiting for arm and gripper action servers...")
        arm_ready = self.arm_client.wait_for_server(timeout_sec=30.0)
        gripper_ready = self.gripper_client.wait_for_server(timeout_sec=30.0)
        if not arm_ready:
            self.get_logger().error("Arm trajectory action server is unavailable")
        if not gripper_ready:
            self.get_logger().error("Gripper action server is unavailable")
        return arm_ready and gripper_ready

    def publish_collision_object(self) -> None:
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = list(OBJECT_SIZE)

        pose = Pose()
        pose.position.x = OBJECT_X
        pose.position.y = OBJECT_Y
        pose.position.z = OBJECT_Z
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
        self.get_logger().info("Added red_box to MoveIt Planning Scene")

    def remove_collision_object(self) -> None:
        """Stop MoveIt from treating the physically grasped object as world geometry."""
        msg = CollisionObject()
        msg.header.frame_id = PLANNING_FRAME
        msg.id = OBJECT_ID
        msg.operation = CollisionObject.REMOVE
        for _ in range(3):
            self.collision_pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.1)
            time.sleep(0.2)
        self.get_logger().info("Removed red_box from MoveIt world collision geometry")

    def send_gripper(self, position: float) -> bool:
        goal = GripperCommand.Goal()
        goal.command.position = position
        goal.command.max_effort = 10.0

        send_future = self.gripper_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=20.0)
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error("Gripper goal was rejected")
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=20.0)
        result = result_future.result()
        if result is None:
            self.get_logger().error("No gripper result received")
            return False

        self.get_logger().info(
            f"Gripper position={position:.3f} "
            f"reached={result.result.reached_goal} "
            f"stalled={result.result.stalled}"
        )
        return bool(result.result.reached_goal or result.result.stalled)

    def plan_and_execute(self, parameters, label: str) -> bool:
        self.get_logger().info(f"Planning: {label}")
        plan_result = self.arm.plan(multi_plan_parameters=parameters)
        if not plan_result:
            self.get_logger().error(f"Planning failed: {label}")
            return False
        self.get_logger().info(f"Executing: {label}")
        execution_result = self.robot.execute(plan_result.trajectory, controllers=[])
        if execution_result:
            self.get_logger().info(f"Finished: {label}")
        else:
            self.get_logger().error(f"Execution failed: {label}")
        return bool(execution_result)

    def move_named(self, name: str) -> bool:
        self.arm.set_start_state_to_current_state()
        self.arm.set_goal_state(configuration_name=name)
        return self.plan_and_execute(self.ompl_params, f"named:{name}")

    def move_pose(self, pose: PoseStamped, parameters, label: str) -> bool:
        self.arm.set_start_state_to_current_state()
        self.arm.set_goal_state(
            pose_stamped_msg=pose,
            pose_link=EE_FRAME,
        )
        return self.plan_and_execute(parameters, label)

    def move_pregrasp(self, pose: PoseStamped) -> bool:
        """Solve the pregrasp pose from the current state, then plan in joints."""
        robot_state = RobotState(self.robot.get_robot_model())
        with self.robot.get_planning_scene_monitor().read_only() as scene:
            current = scene.current_state
            robot_state.set_joint_group_positions(
                PLANNING_GROUP,
                current.get_joint_group_positions(PLANNING_GROUP),
            )
        robot_state.update()
        if not robot_state.set_from_ik(
            PLANNING_GROUP, pose.pose, EE_FRAME, 0.5
        ):
            self.get_logger().error("IK failed: pregrasp pose is unreachable")
            return False
        robot_state.update()
        self.arm.set_start_state_to_current_state()
        self.arm.set_goal_state(robot_state=robot_state)
        return self.plan_and_execute(self.ompl_params, "pregrasp")

    def run(self) -> bool:
        if not self.wait_for_controllers():
            return False

        if not self.move_named("rest"):
            return False
        # Add the object only after reaching the safe start pose. Otherwise
        # the current rest configuration can be rejected as colliding with it.
        self.publish_collision_object()

        pregrasp = downward_pose(OBJECT_X-0.02, OBJECT_Y, PREGRASP_Z)
        approach = downward_pose(OBJECT_X-0.02, OBJECT_Y, APPROACH_Z-0.02)
        lift = downward_pose(OBJECT_X-0.02, OBJECT_Y, LIFT_Z)

        if not self.move_pregrasp(pregrasp):
            return False
        if not self.send_gripper(OPEN_POSITION):
            return False
        if not self.move_pose(approach, self.pilz_params, "cartesian approach"):
            return False
        if not self.send_gripper(CLOSED_POSITION):
            return False

        self.remove_collision_object()
        if not self.move_pose(lift, self.pilz_params, "cartesian lift"):
            return False

        self.get_logger().info("Fixed pick sequence succeeded; holding lifted pose")
        return True


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FixedPickDemo()
    exit_code = 0
    try:
        if bool(node.get_parameter("auto_start").value):
            success = node.run()
            if not success:
                raise RuntimeError("fixed pick sequence failed")
        else:
            node.get_logger().info("auto_start=false; waiting without executing pick")
            rclpy.spin(node)
    except KeyboardInterrupt:
        exit_code = 130
    except Exception:
        # Keep the original traceback while avoiding MoveItPy native cleanup
        # which can segfault on Jazzy after the node has finished.
        traceback.print_exc()
        exit_code = 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        # MoveItPy/MoveItCpp can crash during Python object destruction on
        # Jazzy. DDS resources are released when this process exits.
        os._exit(exit_code)


if __name__ == "__main__":
    main()
