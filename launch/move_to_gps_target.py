# Copyright (c) 2018 Intel Corporation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""This is all-in-one launch script intended for use by nav2 developers."""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription,GroupAction, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node,PushRosNamespace
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import ReplaceString, RewrittenYaml
from launch_ros.descriptions import ComposableNode, ParameterFile
from launch_ros.actions import LoadComposableNodes, SetParameter


def generate_launch_description():
    # Get the launch directory
    #bringup_dir是nav2_tb3包的路径
    bringup_dir = get_package_share_directory('nav2_bringup')
    launch_dir = os.path.join(bringup_dir, 'launch')

    move_to_gps_target_dir=get_package_share_directory('move_to_gps_target')
    move_to_gps_target_src_dir=os.path.join(move_to_gps_target_dir,'..', '..','..','..','src','move_to_gps_target')

    # Create the launch configuration variables
    slam = LaunchConfiguration('slam')
    namespace = LaunchConfiguration('namespace')
    use_namespace = LaunchConfiguration('use_namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    autostart = LaunchConfiguration('autostart')
    use_composition = LaunchConfiguration('use_composition')
    use_respawn = LaunchConfiguration('use_respawn')
    log_level = LaunchConfiguration('log_level')
    map_yaml_file = LaunchConfiguration('map')
    # Launch configuration variables specific to simulation
    use_simulator = LaunchConfiguration('use_simulator')
    use_robot_state_pub = LaunchConfiguration('use_robot_state_pub')

    lifecycle_nodes = ['map_server']

    pose = {'x': LaunchConfiguration('x_pose', default='-2.00'),
            'y': LaunchConfiguration('y_pose', default='-0.50'),
            'z': LaunchConfiguration('z_pose', default='0.01'),
            'R': LaunchConfiguration('roll', default='0.00'),
            'P': LaunchConfiguration('pitch', default='0.00'),
            'Y': LaunchConfiguration('yaw', default='0.00')}
    robot_name = LaunchConfiguration('robot_name')
    robot_sdf = LaunchConfiguration('robot_sdf')

    # Map fully qualified names to relative ones so the node's namespace can be prepended.
    # In case of the transforms (tf), currently, there doesn't seem to be a better alternative
    # https://github.com/ros/geometry2/issues/32
    # https://github.com/ros/robot_state_publisher/pull/30
    # TODO(orduno) Substitute with `PushNodeRemapping`
    #              https://github.com/ros2/launch_ros/issues/56
    remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static')]

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key=namespace,
            param_rewrites={},
            convert_types=True,
        ),
        allow_substs=True,
    )

    # Declare the launch arguments
    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace')
    
    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(
            move_to_gps_target_dir, 'maps', 'empty_world.yaml'),
        description='Full path to map file to load')

    declare_use_namespace_cmd = DeclareLaunchArgument(
        'use_namespace',
        default_value='False',
        description='Whether to apply a namespace to the navigation stack')

    declare_slam_cmd = DeclareLaunchArgument(
        'slam',
        default_value='False',
        description='Whether run a SLAM')


    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Use simulation (Gazebo) clock if true')

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(move_to_gps_target_dir, 'params', 'nav2_params.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes')

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart', default_value='True',
        description='Automatically startup the nav2 stack')

    declare_use_composition_cmd = DeclareLaunchArgument(
        'use_composition', default_value='True',
        description='Whether to use composed bringup')

    declare_use_respawn_cmd = DeclareLaunchArgument(
        'use_respawn', default_value='False',
        description='Whether to respawn if a node crashes. Applied when composition is disabled.')


    declare_use_simulator_cmd = DeclareLaunchArgument(
        'use_simulator',
        default_value='True',
        description='Whether to start the simulator')

    declare_use_robot_state_pub_cmd = DeclareLaunchArgument(
        'use_robot_state_pub',
        default_value='True',
        description='Whether to start the robot state publisher')



    declare_robot_name_cmd = DeclareLaunchArgument(
        'robot_name',
        default_value='turtlebot3_waffle',
        description='name of the robot')



    declare_log_level_cmd = DeclareLaunchArgument(
        'log_level', default_value='info', description='log level'
    )

    bringup_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'bringup_launch.py')),
        launch_arguments={'namespace': namespace,
                          'use_namespace': use_namespace,
                          'slam': slam,
                          'map': map_yaml_file,
                          'use_sim_time': use_sim_time,
                          'params_file': params_file,
                          'autostart': autostart,
                          'use_composition': use_composition,
                          'use_respawn': use_respawn}.items())
    
    # 手动广播静态的map->odom转换
    static_tf_pub_map_to_odom_cmd = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_pub_map_odom',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(PythonExpression(['not ', slam]))  # Only launch this node if slam is 'False'
    )

    # 手动广播静态的map->base_link转换
    static_tf_pub_map_to_baseLink_cmd = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_pub_map_base_link',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'base_link'],
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # 手动广播静态的base_scan->base_footprint转换
    static_tf_pub_scan_to_footprint_cmd = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_pub_scan_footprint',
        arguments=['0', '0', '0', '0', '0', '0', 'base_scan', 'base_footprint'],
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    static_tf_pub_base_link_to_base_scan_cmd = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_pub_base_link_to_base_scan',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'base_scan'],
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )


    nav2_setup_cmd=IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'navigation_launch.py')),
        launch_arguments={'namespace': namespace,
                            'use_sim_time': use_sim_time,
                            'autostart': autostart,
                            'params_file': params_file,
                            'use_composition': use_composition,
                            'use_respawn': use_respawn}.items())

    # Specify the actions
    bringup_cmd_group = GroupAction(
        [
            PushRosNamespace(condition=IfCondition(use_namespace), namespace=namespace),
            Node(
                condition=IfCondition(use_composition),
                name='nav2_container',
                package='rclcpp_components',
                executable='component_container_isolated',
                parameters=[configured_params, {'autostart': autostart}],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings,
                output='screen',
            ),

            LoadComposableNodes(
                target_container=(namespace, '/', 'nav2_container'),
                composable_node_descriptions=[
                    ComposableNode(
                        package='nav2_map_server',
                        plugin='nav2_map_server::MapServer',
                        name='map_server',
                        parameters=[
                            configured_params,
                            {'yaml_filename': map_yaml_file},
                        ],
                        remappings=remappings,
                    ),
                    ComposableNode(
                        package='nav2_lifecycle_manager',
                        plugin='nav2_lifecycle_manager::LifecycleManager',
                        name='lifecycle_manager_localization',
                        parameters=[
                            {'autostart': autostart, 'node_names': lifecycle_nodes}
                        ],
                    ),
                ],
            ),
            # IncludeLaunchDescription(
            #     PythonLaunchDescriptionSource(
            #         os.path.join(launch_dir, 'slam_launch.py')
            #     ),
            #     condition=IfCondition(slam),
            #     launch_arguments={
            #         'namespace': namespace,
            #         'use_sim_time': use_sim_time,
            #         'autostart': autostart,
            #         'use_respawn': use_respawn,
            #         'params_file': params_file,
            #     }.items(),
            # ),
            # IncludeLaunchDescription(
            #     PythonLaunchDescriptionSource(
            #         os.path.join(launch_dir, 'localization_launch.py')
            #     ),
            #     condition=IfCondition(PythonExpression(['not ', slam])),
            #     launch_arguments={
            #         'namespace': namespace,
            #         'map': map_yaml_file,
            #         'use_sim_time': use_sim_time,
            #         'autostart': autostart,
            #         'params_file': params_file,
            #         'use_composition': use_composition,
            #         'use_respawn': use_respawn,
            #         'container_name': 'nav2_container',
            #     }.items(),
            # ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(launch_dir, 'navigation_launch.py')
                ),
                launch_arguments={
                    'namespace': namespace,
                    'use_sim_time': use_sim_time,
                    'autostart': autostart,
                    'params_file': params_file,
                    'use_composition': use_composition,
                    'use_respawn': use_respawn,
                    'container_name': 'nav2_container',
                }.items(),
            ),
        ]
    )

    clear_nav2_costmap_cmd=Node(
            package='move_to_gps_target',
            executable='regularly_clear_costmap_after_starting_nav2',
            output='screen',
            # arguments=['time', '1.0']
        )
    
    my_velocity_controller_setup_cmd=Node(
            package='move_to_gps_target',
            executable='my_velocity_controller',
            output='screen',
            # arguments=['time', '1.0']
        )
    
    apm_controller_node_setup_cmd=Node(
            package='move_to_gps_target',
            executable='apm_controller_node',
            output='screen',
            # arguments=['time', '1.0']
        )

    TF2ListenerExample_cmd=Node(
            package='myTestPockage',
            executable='TF2ListenerExample',
            output='screen',
        )



    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_namespace_cmd)
    ld.add_action(declare_slam_cmd)
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_use_composition_cmd)

    ld.add_action(declare_use_simulator_cmd)
    ld.add_action(declare_use_robot_state_pub_cmd)
    ld.add_action(declare_robot_name_cmd)
    ld.add_action(declare_use_respawn_cmd)
    ld.add_action(declare_log_level_cmd)
    # Add any conditioned actions

    # ld.add_action(apm_controller_node_setup_cmd)
    ld.add_action(static_tf_pub_map_to_odom_cmd)
    ld.add_action(static_tf_pub_scan_to_footprint_cmd)
    ld.add_action(static_tf_pub_base_link_to_base_scan_cmd)
    # ld.add_action(bringup_cmd)
    ld.add_action(bringup_cmd_group)
    # ld.add_action(nav2_setup_cmd)
    ld.add_action(clear_nav2_costmap_cmd)
    ld.add_action(my_velocity_controller_setup_cmd)
    
    # ld.add_action(TF2ListenerExample_cmd)
    return ld
