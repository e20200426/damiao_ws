import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg_share = get_package_share_directory('ignition_robot')
    
    # Set Ignition Gazebo resource path to find meshes
    gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=pkg_share
    )
    
    # Path to the custom world file
    world_file_name = 'empty_world.world'
    world_path = os.path.join(pkg_share, 'worlds', world_file_name)
    
    # Gazebo launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')]),
        launch_arguments={'gz_args': ['-r -v4 ', world_path], 
        'on_exit_shutdown': 'true'}.items()
        # launch_arguments={'world': world_path}.items(),
    )

    # URDF file
    urdf_file = os.path.join(pkg_share, 'urdf', 'tanerb_moveit_config.urdf')
    
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()
    
    # Replace package:// URIs with absolute file:// paths for Ignition Gazebo
    robot_desc = robot_desc.replace(
        'filename="package://ignition_robot/meshes/',
        f'filename="file://{pkg_share}/meshes/'
    )

    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_desc,
            'use_sim_time': True
        }]
    )

    # Spawn robot with string parameter instead of topic
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-string', robot_desc,
            '-name', 'ignition_robot',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.1',
            '-Y', '0.0'
        ],
        output='screen'
    )
    
    # Bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
        	   '/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry',
        	   '/camera/image_raw@sensor_msgs/msg/Image@gz.msgs.Image',
                   '/camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
                   '/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
                   '/world/empty/model/ignition_robot/joint_state@sensor_msgs/msg/JointState[ignition.msgs.Model'],
        output='screen'
    )

    return LaunchDescription([
        gz_resource_path,
        gazebo,
        robot_state_publisher,
        bridge,
        spawn_entity,
    ])
