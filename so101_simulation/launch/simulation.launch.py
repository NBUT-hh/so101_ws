import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    use_rviz = LaunchConfiguration("use_rviz")
    simulation_dir = get_package_share_directory("so101_simulation")
    moveit_dir = get_package_share_directory("so101_moveit_config")

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(simulation_dir, "launch", "gazebo.launch.py")
        ),
        launch_arguments={"use_sim_time": "true"}.items(),
    )
    move_group = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(moveit_dir, "launch", "move_group.launch.py")
        ),
        launch_arguments={"use_sim_time": "true"}.items(),
    )
    moveit_rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(moveit_dir, "launch", "moveit_rviz.launch.py")
        ),
        launch_arguments={"use_sim_time": "true"}.items(),
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription([
        DeclareLaunchArgument("use_rviz", default_value="true"),
        gazebo,
        move_group,
        moveit_rviz,
    ])
