from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    # Required to allow communication with external pub-sub service
    node_rosbridge_server = Node(
        package='rosbridge_server',
        executable='rosbridge_websocket',
        output='screen',
    )

    # Required to provide getTopics service 
    node_rosapi = Node(
        package='rosapi',
        executable='rosapi_node',
        output='screen',
    )

    # Run the node
    return LaunchDescription([
        node_rosbridge_server,
        node_rosapi,
    ])
