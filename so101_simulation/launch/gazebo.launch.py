import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    namespace = LaunchConfiguration("namespace")
    use_sim_time = LaunchConfiguration("use_sim_time")

    description_share = FindPackageShare("so101_description")
    description_parent = os.path.dirname(
        get_package_share_directory("so101_description")
    )
    world_file = PathJoinSubstitution(
        [FindPackageShare("so101_simulation"), "worlds", "so101_pick_world.sdf"]
    )
    bridge_config = PathJoinSubstitution(
        [FindPackageShare("so101_simulation"), "config", "parameter_bridge.yaml"]
    )
    xacro_file = PathJoinSubstitution(
        [description_share, "urdf", "so101_arm.urdf.xacro"]
    )

    robot_description = ParameterValue(
        Command(
            [
                FindExecutable(name="xacro"), " ", xacro_file,
                " variant:=follower",
                " use_ros2_control:=true",
                " hardware_type:=gazebo",
                " gazebo:=true",
            ]
        ),
        value_type=str,
    )

    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace=namespace,
        parameters=[
            {"robot_description": robot_description, "use_sim_time": use_sim_time}
        ],
        output="screen",
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"]
            )
        ),
        launch_arguments={"gz_args": [world_file, " -r"]}.items(),
    )

    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=["-topic", ["/", namespace, "/robot_description"], "-name", "so101"],
        output="screen",
    )

    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        parameters=[{"config_file": bridge_config}],
        output="screen",
    )

    controller_manager = ["/", namespace, "/controller_manager"]
    spawners = [
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=[
                controller,
                "--controller-manager", controller_manager,
                "--controller-manager-timeout", "60",
            ],
            output="screen",
        )
        for controller in (
            "joint_state_broadcaster",
            "arm_trajectory_controller",
            "gripper_controller",
        )
    ]

    return LaunchDescription([
        DeclareLaunchArgument("namespace", default_value="follower"),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        SetEnvironmentVariable(
            "GZ_SIM_RESOURCE_PATH",
            [description_parent, ":", os.environ.get("GZ_SIM_RESOURCE_PATH", "")],
        ),
        rsp,
        gazebo,
        bridge,
        spawn_robot,
        TimerAction(period=3.0, actions=spawners),
    ])
