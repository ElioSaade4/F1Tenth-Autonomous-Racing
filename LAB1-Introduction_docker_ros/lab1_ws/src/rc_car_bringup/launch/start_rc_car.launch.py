import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    ld = LaunchDescription()

    config = os.path.join(
        get_package_share_directory('rc_car_bringup'),
        'config',
        'params.yaml'
    )

    talker = Node(
        package = "lab1_pkg",
        executable = "talker",
        parameters = [config]
    )

    relay = Node(
        package = "lab1_pkg",
        executable = "relay"
    )

    ld.add_action(talker)
    ld.add_action(relay)

    return ld