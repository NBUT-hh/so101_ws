import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder


def _float_argument(name: str) -> ParameterValue:
    return ParameterValue(LaunchConfiguration(name), value_type=float)


def _bool_argument(name: str) -> ParameterValue:
    return ParameterValue(LaunchConfiguration(name), value_type=bool)


def generate_launch_description():
    bringup_dir = get_package_share_directory("so101_bringup")
    moveit_dir = get_package_share_directory("so101_moveit_config")
    description_dir = get_package_share_directory("so101_description")

    xacro_path = os.path.join(description_dir, "urdf", "so101_arm.urdf.xacro")
    moveit_config = (
        MoveItConfigsBuilder("so101_arm", package_name="so101_moveit_config")
        .robot_description(
            file_path=xacro_path,
            mappings={"variant": "follower", "use_ros2_control": "false"},
        )
        .robot_description_semantic()
        .robot_description_kinematics()
        .planning_pipelines(pipelines=["ompl", "pilz_industrial_motion_planner"])
        .pilz_cartesian_limits(file_path="config/pilz_cartesian_limits.yaml")
        .joint_limits()
        .trajectory_execution(
            file_path="config/moveit_controllers.yaml",
            moveit_manage_controllers=False,
        )
        .moveit_cpp(
            file_path=os.path.join(moveit_dir, "config", "moveit_py_real_config.yaml")
        )
        .to_moveit_configs()
    )

    follower = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, "launch", "follower_split.launch.py")
        ),
        launch_arguments={
            "namespace": "follower",
            "hardware_type": "real",
            "usb_port": LaunchConfiguration("usb_port"),
            "joint_config_file": LaunchConfiguration("joint_config_file"),
            "arm_controller": "arm_trajectory_controller",
            "use_rviz": "false",
        }.items(),
    )

    move_group = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(moveit_dir, "launch", "move_group.launch.py")
        ),
        launch_arguments={
            "namespace": "follower",
            "variant": "follower",
            "use_sim_time": "false",
        }.items(),
    )

    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(moveit_dir, "launch", "moveit_rviz.launch.py")
        ),
        launch_arguments={
            "namespace": "follower",
            "variant": "follower",
            "use_sim_time": "false",
        }.items(),
        condition=IfCondition(LaunchConfiguration("use_moveit_rviz")),
    )

    grasp_node = Node(
        package="so101_grasping",
        executable="real_fixed_pick_demo",
        name="real_fixed_pick_demo",
        prefix=LaunchConfiguration("python_executable"),
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {
                "use_sim_time": False,
                "auto_start": _bool_argument("auto_start"),
                "move_to_rest": _bool_argument("move_to_rest"),
                "add_collision_object": _bool_argument("add_collision_object"),
                "object_x": _float_argument("object_x"),
                "object_y": _float_argument("object_y"),
                "object_z": _float_argument("object_z"),
                "object_size_x": _float_argument("object_size_x"),
                "object_size_y": _float_argument("object_size_y"),
                "object_size_z": _float_argument("object_size_z"),
                "pregrasp_x": _float_argument("pregrasp_x"),
                "pregrasp_y": _float_argument("pregrasp_y"),
                "pregrasp_z": _float_argument("pregrasp_z"),
                "approach_x": _float_argument("approach_x"),
                "approach_y": _float_argument("approach_y"),
                "approach_z": _float_argument("approach_z"),
                "lift_x": _float_argument("lift_x"),
                "lift_y": _float_argument("lift_y"),
                "lift_z": _float_argument("lift_z"),
                "open_position": _float_argument("open_position"),
                "closed_position": _float_argument("closed_position"),
            },
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument("usb_port", default_value="/dev/ttyACM0"),
        DeclareLaunchArgument("joint_config_file", default_value=""),
        DeclareLaunchArgument("use_moveit_rviz", default_value="true"),
        DeclareLaunchArgument(
            "python_executable",
            default_value="/home/huanghao/miniconda3/envs/so101/bin/python3",
        ),
        DeclareLaunchArgument(
            "auto_start",
            default_value="false",
            description="Set true only after checking the real workspace",
        ),
        DeclareLaunchArgument("move_to_rest", default_value="true"),
        DeclareLaunchArgument("add_collision_object", default_value="true"),
        DeclareLaunchArgument("object_x", default_value="0.24"),
        DeclareLaunchArgument("object_y", default_value="0.0"),
        DeclareLaunchArgument("object_z", default_value="0.01"),
        DeclareLaunchArgument("object_size_x", default_value="0.02"),
        DeclareLaunchArgument("object_size_y", default_value="0.02"),
        DeclareLaunchArgument("object_size_z", default_value="0.02"),
        DeclareLaunchArgument("pregrasp_x", default_value="0.22"),
        DeclareLaunchArgument("pregrasp_y", default_value="0.0"),
        DeclareLaunchArgument("pregrasp_z", default_value="0.075"),
        DeclareLaunchArgument("approach_x", default_value="0.22"),
        DeclareLaunchArgument("approach_y", default_value="0.0"),
        DeclareLaunchArgument("approach_z", default_value="0.015"),
        DeclareLaunchArgument("lift_x", default_value="0.22"),
        DeclareLaunchArgument("lift_y", default_value="0.0"),
        DeclareLaunchArgument("lift_z", default_value="0.07"),
        DeclareLaunchArgument("open_position", default_value="1.50"),
        DeclareLaunchArgument("closed_position", default_value="-0.16"),
        follower,
        move_group,
        rviz,
        TimerAction(period=5.0, actions=[grasp_node]),
    ])
