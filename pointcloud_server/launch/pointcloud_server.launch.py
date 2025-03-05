#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Launch arguments
    # Path to the default param file in this package
    default_param_file = os.path.join(
        get_package_share_directory('pointcloud_server'),
        'config',
        'pointcloud_server.yaml'
    )

    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Namespace for the pointcloud_server'
    )

    # Launch arguments
    param_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_param_file,
        description='Path to the YAML file with all parameters'
    )
    input_topic_arg = DeclareLaunchArgument(
        'input_topic',
        default_value='/velodyne/points',
        description='Input pointcloud topic'
    )
    mode_arg = DeclareLaunchArgument(
        'mode',
        default_value='localization',
        description='SLAM mode [mapping | localization]'
    )

    # Always start the global map server
    global_server_node = Node(
        namespace=LaunchConfiguration('namespace'),
        package='pointcloud_server',
        executable='pointcloud_server_node',
        name='global_pointcloud_server',
        output='screen',
        parameters=[LaunchConfiguration('params_file')],
    )

    # Only start local map server in localization mode
    local_server_node = Node(
        namespace=LaunchConfiguration('namespace'),
        package='pointcloud_server',
        executable='pointcloud_server_node',
        name='local_pointcloud_server',
        output='screen',
        parameters=[LaunchConfiguration('params_file')],
        condition=IfCondition(
            PythonExpression(["'", LaunchConfiguration('mode'), "' == 'localization'"])
        )
    )

    # Lidar filter for mapping mode (output -> /global_pointcloud_server/add)
    lidar_filter_mapping_node = Node(
        namespace=LaunchConfiguration('namespace'),
        package='pointcloud_server',
        executable='filter_node',
        name='lidar_filter',
        output='screen',
        parameters=[LaunchConfiguration('params_file')],
        remappings=[
            ('~/input', LaunchConfiguration('input_topic')),
            ('~/filtered', 'global_pointcloud_server/add')
        ],
        condition=IfCondition(
            PythonExpression(["'", LaunchConfiguration('mode'), "' == 'mapping'"])
        )
    )

    # Lidar filter for localization mode (output -> /global_pointcloud_server/label_new_points_input)
    lidar_filter_localization_node = Node(
        namespace=LaunchConfiguration('namespace'),
        package='pointcloud_server',
        executable='filter_node',
        name='lidar_filter',
        output='screen',
        parameters=[LaunchConfiguration('params_file')],
        remappings=[
            ('~/input', LaunchConfiguration('input_topic')),
            ('~/filtered', 'global_pointcloud_server/label_new_points_input')
        ],
        condition=IfCondition(
            PythonExpression(["'", LaunchConfiguration('mode'), "' == 'localization'"])
        )
    )

    # Label filter node (only in localization mode)
    label_filter_node = Node(
        namespace=LaunchConfiguration('namespace'),
        package='pointcloud_server',
        executable='filter_node',
        name='label_filter',
        output='screen',
        parameters=[LaunchConfiguration('params_file')],
        remappings=[
            ('~/input',    'global_pointcloud_server/label_new_points_output'),
            ('~/filtered', 'local_pointcloud_server/add')
        ],
        condition=IfCondition(
            PythonExpression(["'", LaunchConfiguration('mode'), "' == 'localization'"])
        )
    )

    return LaunchDescription([
        namespace_arg,
        param_file_arg,
        input_topic_arg,
        mode_arg,
        global_server_node,
        local_server_node,
        lidar_filter_mapping_node,
        lidar_filter_localization_node,
        label_filter_node
    ])
