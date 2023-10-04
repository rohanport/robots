import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro

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

    # Use xacro to process the file
    xacro_file = os.path.join(get_package_share_directory(
        pkg_name), 'description/wheely_bot.urdf.xacro')
    robot_description_raw = xacro.process_file(xacro_file).toxml()

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        # add other parameters here if required
        parameters=[{'robot_description': robot_description_raw}]
    )

    spawn_wheely = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-string', robot_description_raw,
            '-name', 'wheely'
        ]
    )

    # Run the node
    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        spawn_wheely
    ])
