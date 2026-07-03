import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackagePrefix
from launch.conditions import IfCondition, UnlessCondition


def generate_launch_description():
    current_launch_dir = os.path.dirname(os.path.realpath(__file__))
    configured_params = os.path.join(current_launch_dir, 'nav2_config.yaml')
    default_map_path = os.path.normpath(os.path.join(current_launch_dir, '..', 'maps', 'map.yaml'))
    default_world_path = os.path.normpath(os.path.join(current_launch_dir, '..', 'worlds', 'world.yaml'))
    rviz_config_path = os.path.normpath(os.path.join(current_launch_dir, '..', 'nav.rviz'))
    
    urdf_path = os.path.normpath(os.path.join(current_launch_dir, '..', 'robot.urdf'))
    
    with open(urdf_path, 'r') as infp:
        robot_desc = infp.read()

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    use_respawn = LaunchConfiguration('use_respawn', default='false')
    use_amcl = LaunchConfiguration('use_amcl', default='false')
    odom_frame = LaunchConfiguration('odom_frame', default='odom')
    map_yaml_file = LaunchConfiguration('map')
    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]
    use_gui = LaunchConfiguration('use_gui')    
    
    socnav_sim_node = ExecuteProcess(
        condition=UnlessCondition(use_gui),
        cmd=[
                PathJoinSubstitution([
                    FindPackagePrefix('socnav_sim'),
                    'lib',
                    'socnav_sim',
                    'simros_node'
                ]),
                default_world_path
            ],
        output='screen',
        name='socnav_sim')
    
    socnav_sim_node_gui =  ExecuteProcess(
            condition=IfCondition(use_gui),
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
    

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc, 'use_sim_time': True}]
    )


    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    map_server_node = LifecycleNode(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        namespace='',
        output='screen',
        parameters=[configured_params, {'yaml_filename': map_yaml_file}],
        remappings=remappings
    )

    fake_localization_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='fake_localization',
        output='screen',
        respawn=use_respawn,
        respawn_delay=2.0,
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['0', '0', '0', '0', '0', '0', 'map', odom_frame],
        remappings=remappings,
        condition=UnlessCondition(use_amcl)
    )
    
    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        respawn=use_respawn,
        respawn_delay=2.0,
        parameters=[configured_params],
        remappings=remappings,
        condition=IfCondition(use_amcl)
    )

    controller_node = LifecycleNode(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        namespace='',
        output='screen',
        parameters=[configured_params],
        remappings=remappings
    )

    planner_node = LifecycleNode(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        namespace='',
        output='screen',
        parameters=[configured_params],
        remappings=remappings
    )


    bt_navigator_node = LifecycleNode(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        namespace='',
        output='screen',
        parameters=[configured_params],
        remappings=remappings
    )

    behaviors_node = LifecycleNode(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        namespace='',
        output='screen',
        parameters=[configured_params],
        remappings=remappings
    )


    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': True,
            'node_names': ['map_server', 'planner_server', 'controller_server', 'behavior_server', 'bt_navigator']
        }]
    )
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path]
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('use_respawn', default_value='false'),
        DeclareLaunchArgument('use_amcl', default_value='false'),
        DeclareLaunchArgument('odom_frame', default_value='odom'),
        DeclareLaunchArgument('map', default_value=default_map_path, description='Full path to map yaml file'),
        DeclareLaunchArgument('use_gui', default_value='false', description='Whether to start the simulator'),
        

        socnav_sim_node,
        socnav_sim_node_gui,
        map_server_node,
        fake_localization_node,
        amcl_node,
        controller_node,
        planner_node,
        bt_navigator_node,   
        behaviors_node,     
        lifecycle_manager,
        rviz_node,
        robot_state_publisher_node,
        joint_state_publisher_node
    ])