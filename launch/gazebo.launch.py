import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory

TUBES = ["tube_red", "tube_green", "tube_blue"]


def generate_launch_description():
    pkg = get_package_share_directory("lab_teleop")
    xacro_file = os.path.join(pkg, "urdf", "panda_gz.urdf.xacro")
    world_file = os.path.join(pkg, "worlds", "lab.sdf")
    robot_description = ParameterValue(Command(["xacro ", xacro_file]), value_type=str)

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py")
        ),
        launch_arguments={"gz_args": "-r " + world_file}.items(),
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description, "use_sim_time": True}],
        output="screen",
    )

    bridge_args = ["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"]
    for t in TUBES:
        bridge_args.append(f"/model/{t}/pose@geometry_msgs/msg/PoseStamped[gz.msgs.Pose")
        bridge_args.append(f"/grasp/{t}/attach@std_msgs/msg/Empty]gz.msgs.Empty")
        bridge_args.append(f"/grasp/{t}/detach@std_msgs/msg/Empty]gz.msgs.Empty")

    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=bridge_args,
        output="screen",
    )

    spawn = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=["-topic", "robot_description", "-name", "panda"],
        output="screen",
    )

    controllers = [
        Node(package="controller_manager", executable="spawner", arguments=[name])
        for name in ["joint_state_broadcaster", "panda_arm_controller", "panda_hand_controller"]
    ]

    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        bridge,
        spawn,
        RegisterEventHandler(OnProcessExit(target_action=spawn, on_exit=controllers)),
    ])
