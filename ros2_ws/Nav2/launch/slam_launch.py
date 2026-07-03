import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackagePrefix

def generate_launch_description():
    current_launch_dir = os.path.dirname(os.path.realpath(__file__))
    default_world_path = os.path.normpath(os.path.join(current_launch_dir, '..', 'worlds', 'world.yaml'))
    rviz_config_path = os.path.normpath(os.path.join(current_launch_dir, '..', 'nav.rviz'))
    
    socnav_sim_node =  ExecuteProcess(
            cmd=[
                    PathJoinSubstitution([
                        FindPackagePrefix('socnav_sim'),
                        'lib',
                        'socnav_sim',
                        'simros_node'
                    ]),
                    '-g',
                    default_world_path
                ],
            output='screen',
            name='socnav_sim')
    
    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[{'use_sim_time': True}] # Set to False if running on a real robot
    )
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path]
    )


    return LaunchDescription([
        socnav_sim_node,
        slam_node,
        rviz_node
    ])