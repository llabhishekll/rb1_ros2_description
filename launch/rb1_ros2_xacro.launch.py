from os import environ
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import TimerAction
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch.substitutions import LaunchConfiguration
from launch_ros.descriptions import ParameterValue
from launch_ros.actions import Node


def generate_launch_description():
    # launch parameters
    use_sim_time = LaunchConfiguration("use_sim_time", default="True")

    # gazebo launch description
    path_gazebo = Path(get_package_share_directory("gazebo_ros"))
    path_launch = path_gazebo / "launch" / "gazebo.launch.py"

    # package rb1_ros2_description
    path_root = Path(get_package_share_directory("rb1_ros2_description"))
    path_urdf = path_root / "xacro" / "rb1_ros2_base.urdf.xacro"
    path_mesh = path_root / "meshes"

    # gazebo path
    environ["GAZEBO_MODEL_PATH"] += (
        ":" + path_root.parent.as_posix() + ":" + path_mesh.as_posix()
    )

    # parameters
    namespace, robot_name = "", "rb1_robot"

    # return launch
    return LaunchDescription(
        [
            DeclareLaunchArgument(name="use_sim_time", default_value="True"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(path_launch.as_posix()),
                launch_arguments=[
                    ("verbose", "false"),
                    ("pause", "false")
                ],
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                namespace=namespace,
                output="screen",
                emulate_tty=True,
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "frame_prefix": robot_name + "/",
                        "robot_description": ParameterValue(
                            Command(
                                [
                                    " xacro ", path_urdf.as_posix(),
                                    " robot_name:=", robot_name,
                                ]
                            ),
                            value_type=str,
                        ),
                    }
                ],
            ),
            Node(
                package="gazebo_ros",
                executable="spawn_entity.py",
                name="spawn_entity",
                output="screen",
                arguments=[
                    "-entity", robot_name,
                    "-x", "0", "-y", "0", "-z", "0",
                    "-R", "0", "-P", "0", "-Y", "0",
                    "-topic", f"{namespace}/robot_description",
                ],
            ),
            TimerAction(
                period=100.0,
                actions=[
                    Node(
                        package="controller_manager",
                        executable="spawner",
                        output="screen",
                        arguments=[
                            "rb1_base_controller",
                            "--controller-manager", "/controller_manager",
                            "--controller-manager-timeout", "60",
                        ],
                    ),
                    Node(
                        package="controller_manager",
                        executable="spawner",
                        output="screen",
                        arguments=[
                            "rb1_elevator_controllers",
                            "--controller-manager", "/controller_manager",
                            "--controller-manager-timeout", "60",
                        ],
                    ),
                    Node(
                        package="controller_manager",
                        executable="spawner",
                        output="screen",
                        arguments=[
                            "joint_state_broadcaster",
                            "--controller-manager", "/controller_manager",
                            "--controller-manager-timeout", "60",
                        ],
                    ),
                ]
            )
        ]
    )