import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration

from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription , ExecuteProcess,TimerAction
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution, TextSubstitution

import xacro

def generate_launch_description():

    # Check if we're told to use sim time
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Process the URDF file
    pkg_path = os.path.join(get_package_share_directory('diffdrive_arduino'))
    xacro_file = os.path.join(pkg_path, 'description', 'robot.urdf.xacro')
    robot_description_config = xacro.process_file(xacro_file)

    # Create a robot_state_publisher node
    params = {'robot_description': robot_description_config.toxml(), 'use_sim_time': use_sim_time}
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    twist_mux_params = os.path.join(get_package_share_directory('diffdrive_arduino'),'config','twist_mux.yaml')
    twist_mux = Node(
            package="twist_mux",
            executable="twist_mux",
            parameters=[twist_mux_params, {'use_sim_time': True}],
            remappings=[('/cmd_vel_out','/diff_cont/cmd_vel_unstamped')]
        )

    slam_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource([PathJoinSubstitution([FindPackageShare('slam_toolbox'),'launch','online_async_launch.py'])]),
        launch_arguments={'slam_params_file':PathJoinSubstitution([FindPackageShare('diffdrive_arduino'),'config','mapper_params_online_async.yaml']),'use_sim_time':'true'}.items()
    )
    
    rviz_node = Node(package='rviz2',executable='rviz2',name='rviz2',output='screen',arguments=['-d',PathJoinSubstitution([FindPackageShare('diffdrive_arduino'),'config','slam_config.rviz'])])
    
    # Execute gzserver and gzclient
    gzserver_process = ExecuteProcess(
        cmd=['gzserver', os.path.join(pkg_path, 'worlds', 'my_world.world'),
             '-s', 'libgazebo_ros_init.so',
             '-s', 'libgazebo_ros_factory.so',
             '-s', 'libgazebo_ros_force_system.so'],
        output='screen'
    )

    gzclient_process = ExecuteProcess(
        cmd=['gzclient'],
        output='screen'
    )

    spawn=Node(
            package="gazebo_ros",
            executable="spawn_entity.py",
            arguments=["-topic", "robot_description", "-entity", "my_bot"],
            output="screen"
        )

    joint_state_broadcaster = TimerAction(
        period=7.0,  # Delay of 3 seconds before starting joint_state_broadcaster
        actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                output='screen',
                arguments=["joint_broad"]
            )
        ]
    )

    diff_drive_controller = TimerAction(
        period=10.0,  # Delay of 5 seconds before starting forward_position_controller
        actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                output='screen',
                arguments=["diff_cont"]
            )
        ]
    )
    
    # Launch!
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use sim time if true'
        ),
        gzserver_process,
        gzclient_process,
        rviz_node,
        node_robot_state_publisher,
        spawn,
        slam_launch,
        diff_drive_controller,
        joint_state_broadcaster,
        twist_mux
    ])

