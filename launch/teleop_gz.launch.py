import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_param_builder import ParameterBuilder
from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("moveit_resources_panda")
        .robot_description(file_path="config/panda.urdf.xacro")
        .joint_limits(file_path="config/hard_joint_limits.yaml")
        .to_moveit_configs()
    )

    servo_params = {
        "moveit_servo": ParameterBuilder("moveit_servo")
        .yaml("config/panda_simulated_config.yaml")
        .to_dict()
    }

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("lab_teleop"), "launch", "gazebo.launch.py")
        )
    )

    servo_node = Node(
        package="moveit_servo",
        executable="servo_node",
        name="servo_node",
        parameters=[
            servo_params,
            {"update_period": 0.01, "planning_group_name": "panda_arm", "use_sim_time": True},
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
        ],
        output="screen",
    )

    hand_node = ExecuteProcess(
        cmd=["python3", os.path.expanduser("~/hand_teleop/hand_servo.py"),
             "--ros-args", "-p", "use_sim_time:=true"],
        output="screen",
    )

    return LaunchDescription([
        gazebo,
        TimerAction(period=10.0, actions=[servo_node]),
        TimerAction(period=15.0, actions=[hand_node]),
    ])
