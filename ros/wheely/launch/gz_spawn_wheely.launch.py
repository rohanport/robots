import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import xacro


def generate_launch_description():

    # Specify the name of the package and path to xacro file within the package
    pkg_name = 'wheely'
    file_subpath = 'description/wheely_bot.urdf.xacro'


    # Use xacro to process the file
    xacro_file = os.path.join(get_package_share_directory(pkg_name),file_subpath)
    robot_description_raw = xacro.process_file(xacro_file).toxml()


    # Spawn wheely
    spawn_wheely = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-string', robot_description_raw, 
            '-name', 'wheely', 
            '-world', 'empty'
        ]
    )


    # Run the node
    return LaunchDescription([
        spawn_wheely
    ])