import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

# 1. Launch gazebo
# 2. Launch robot_state_publisher
# 3. Spawn wheely into gazebo


def generate_launch_description():

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory(
                'ros_gz_sim'), 'launch'), '/gz_sim.launch.py']
        ),
        launch_arguments={'gz_args': 'empty.sdf'}.items()
    )

    # Specify the name of the package and path to xacro file within the package
    pkg_name = 'wheely'

    robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory(pkg_name),
                          'launch'), '/robot_state_publisher.launch.py']
        ),
        launch_arguments={'use_sim_time': 'true'}.items()
    )

    spawn_wheely = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'wheely'
        ]
    )

    # Run the node
    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        spawn_wheely
    ])
