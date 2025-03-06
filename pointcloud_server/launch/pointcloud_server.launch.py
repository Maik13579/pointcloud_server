#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Locate the default parameter file
    default_param_file = os.path.join(
        get_package_share_directory('pointcloud_server'),
        'config',
        'pointcloud_server.yaml'
    )

    # -------------------------
    # Launch Arguments
    # -------------------------
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Namespace for the pointcloud_server'
    )

    param_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_param_file,
        description='Path to the YAML file with all parameters'
    )

    input_topic_arg = DeclareLaunchArgument(
        'input_topic',
        default_value='/velodyne/points',
        description='Topic name providing raw or partially filtered pointcloud'
    )

    mode_arg = DeclareLaunchArgument(
        'mode',
        default_value='localization',
        description='Operation mode: mapping or localization'
    )

    freespace_arg = DeclareLaunchArgument(
        'freespace_detection',
        default_value='false',
        description='Enable freespace detection node'
    )

    # -------------------------
    # Common / Always-running Nodes
    # -------------------------
    global_server_node = Node(
        namespace=LaunchConfiguration('namespace'),
        package='pointcloud_server',
        executable='pointcloud_server_node',
        name='global_pointcloud_server',
        output='screen',
        parameters=[LaunchConfiguration('params_file')],
    )

    # -------------------------
    # Mapping-only Nodes
    # -------------------------
    # (Filter node output -> /global_pointcloud_server/add)
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

    # -------------------------
    # Localization-only Nodes
    # -------------------------
    # Local map server
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

    # Filter node in localization mode
    # (Output -> /global_pointcloud_server/label_new_points_input)
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

    # Label filter node (feeds into local server)
    label_filter_node = Node(
        namespace=LaunchConfiguration('namespace'),
        package='pointcloud_server',
        executable='filter_node',
        name='label_filter',
        output='screen',
        parameters=[LaunchConfiguration('params_file')],
        remappings=[
            ('~/input', 'global_pointcloud_server/label_new_points_output'),
            ('~/filtered', 'local_pointcloud_server/add')
        ],
        condition=IfCondition(
            PythonExpression(["'", LaunchConfiguration('mode'), "' == 'localization'"])
        )
    )

    # -------------------------
    # Freespace Detection Node
    # -------------------------
    freespace_node = Node(
        namespace=LaunchConfiguration('namespace'),
        package='pointcloud_server',
        executable='freespace_detection_node',
        name='freespace_detection',
        output='screen',
        parameters=[LaunchConfiguration('params_file')],
        remappings=[
            ('~/input', 'global_pointcloud_server/label_new_points_input'),
        ],
        condition=IfCondition(
            PythonExpression(["'", LaunchConfiguration('freespace_detection'), "' == 'true'"])
        )
    )

    # -------------------------
    # Combine all in LaunchDescription
    # -------------------------
    return LaunchDescription([
        # Declare all arguments
        namespace_arg,
        param_file_arg,
        input_topic_arg,
        mode_arg,
        freespace_arg,

        # Common node(s)
        global_server_node,

        # Mapping-only node(s)
        lidar_filter_mapping_node,

        # Localization-only node(s)
        local_server_node,
        lidar_filter_localization_node,
        label_filter_node,

        # Freespace detection
        freespace_node
    ])
