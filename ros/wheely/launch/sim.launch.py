import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

# 1. Launch gazebo
# 2. Launch robot_state_publisher
# 3. Spawn wheely into gazebo


def generate_launch_description():

    # Specify the name of the package and path to xacro file within the package
    pkg_name = 'wheely'
    world_file = os.path.join(get_package_share_directory(
        pkg_name), 'worlds', 'world.sdf')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory('ros_gz_sim'),
                          'launch', 'gz_sim.launch.py')]),
        launch_arguments=[('gz_args', [' -r -v 4 ' + world_file])])

    robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory(pkg_name),
                          'launch'), '/robot_state_publisher.launch.py']
        ),
        launch_arguments={'use_sim_time': 'true'}.items()
    )

    rosbridge_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory(pkg_name),
                          'launch'), '/rosbridge_server.launch.py']
        )
    )

    spawn_wheely = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'wheely',
            '-allow_renaming', 'true'
        ]
    )

    spawn_joint_broad_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broad"],
    )

    spawn_diff_cont_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_cont"],
    )

    spawn_sensor_position = Node(
        package="wheely",
        executable="sensor_position"
    )

    spawn_action_diff_drive = Node(
        package="wheely",
        executable="action_diff_drive"
    )

    # Bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
            '/lidar@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan', # Lidar reading from GZ
            '/model/wheely/pose@geometry_msgs/msg/TransformStamped[ignition.msgs.Pose', # Model positions from GZ
            '/diff_cont/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist', # Diff drive commands from ROS2
        ],
        output='screen'
    )

    # Run the node
    return LaunchDescription([
        bridge,
        gazebo,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_wheely,
                on_exit=[
                    spawn_diff_cont_controller, 
                    spawn_joint_broad_controller,
                    spawn_sensor_position,
                    spawn_action_diff_drive
                ],
            )
        ),
        robot_state_publisher,
        rosbridge_server,
        spawn_wheely,
    ])
