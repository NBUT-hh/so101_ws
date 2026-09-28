import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    use_rviz = LaunchConfiguration("use_rviz")
    simulation_dir = get_package_share_directory("so101_simulation")
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
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .moveit_cpp(
            file_path=os.path.join(
                moveit_dir, "config", "moveit_py_config.yaml"
            )
        )
        .to_moveit_configs()
    )

    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(simulation_dir, "launch", "simulation.launch.py")
        ),
        launch_arguments={"use_rviz": use_rviz}.items(),
    )

    grasp_node = Node(
        package="so101_grasping",
        executable="fixed_pick_demo",
        name="fixed_pick_demo",
        prefix="/home/huanghao/miniconda3/envs/so101/bin/python3",
        output="screen",
        parameters=[moveit_config.to_dict(), {"use_sim_time": True}],
    )

    return LaunchDescription([
        DeclareLaunchArgument("use_rviz", default_value="true"),
        simulation,
        grasp_node,
    ])
