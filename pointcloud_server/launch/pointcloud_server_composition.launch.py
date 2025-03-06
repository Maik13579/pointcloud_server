#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory
from launch.conditions import IfCondition

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
        description='Topic for the raw/filtered input cloud'
    )

    mode_arg = DeclareLaunchArgument(
        'mode',
        default_value='localization',
        description='Choose [mapping | localization]. Each node can check this param internally.'
    )

    freespace_arg = DeclareLaunchArgument(
        'freespace_detection',
        default_value='false',
        description='Whether to enable freespace detection node logic (node checks internally).'
    )

    # -------------------------
    # Composable Nodes
    # -------------------------
    # 1) Global server node
    global_server_node = ComposableNode(
        package='pointcloud_server',
        plugin='pointcloud_server::PointcloudServerNode',
        name='global_pointcloud_server',
        namespace=LaunchConfiguration('namespace'),
        parameters=[LaunchConfiguration('params_file')],
        # No special remaps here
    )

    # 2) Filter node for mapping (but will run only if param says so)
    lidar_filter_mapping_node = ComposableNode(
        package='pointcloud_server',
        plugin='pointcloud_server::FilterNode',
        name='lidar_filter',
        namespace=LaunchConfiguration('namespace'),
        parameters=[LaunchConfiguration('params_file')],
        remappings=[
            ('~/input', LaunchConfiguration('input_topic')),
            ('~/filtered', 'global_pointcloud_server/add')
        ]
    )

    # 3) Local server, Filter nodes, Label filter (for localization)
    local_server_node = ComposableNode(
        package='pointcloud_server',
        plugin='pointcloud_server::PointcloudServerNode',
        name='local_pointcloud_server',
        namespace=LaunchConfiguration('namespace'),
        parameters=[LaunchConfiguration('params_file')]
    )

    lidar_filter_localization_node = ComposableNode(
        package='pointcloud_server',
        plugin='pointcloud_server::FilterNode',
        name='lidar_filter',
        namespace=LaunchConfiguration('namespace'),
        parameters=[LaunchConfiguration('params_file')],
        remappings=[
            ('~/input', LaunchConfiguration('input_topic')),
            ('~/filtered', 'global_pointcloud_server/label_new_points_input')
        ]
    )

    label_filter_node = ComposableNode(
        package='pointcloud_server',
        plugin='pointcloud_server::FilterNode',
        name='label_filter',
        namespace=LaunchConfiguration('namespace'),
        parameters=[LaunchConfiguration('params_file')],
        remappings=[
            ('~/input', 'global_pointcloud_server/label_new_points_output'),
            ('~/filtered', 'local_pointcloud_server/add')
        ]
    )

    # Obstacle Tracker node
    obstacle_tracker_node = ComposableNode(
        package='pointcloud_server',
        plugin='pointcloud_server::ObstacleTrackerNode',
        name='obstacle_tracker',
        namespace=LaunchConfiguration('namespace'),
        parameters=[LaunchConfiguration('params_file')],
        remappings=[
            ('~/input', 'local_pointcloud_server/map'),
            ('~/obstacle_markers', '/obstacles/markers')
        ]
    )

    # 4) Freespace detection node (enabled if param freespace_detection == "true")
    freespace_node_localization = ComposableNode(
        package='pointcloud_server',
        plugin='freespace_detection::FreespaceDetectionNode',
        name='freespace_detection',
        namespace=LaunchConfiguration('namespace'),
        parameters=[LaunchConfiguration('params_file')],
        remappings=[
            ('~/input', 'global_pointcloud_server/label_new_points_input'),
            ('~/freespace_cloud', 'global_pointcloud_server/freespace_label_input')
        ],
        condition=IfCondition(
            PythonExpression([
                "'", LaunchConfiguration('freespace_detection'),
                "' == 'true' and '", LaunchConfiguration('mode'),
                "' == 'localization'"
            ])
        )
    )

    freespace_node_mapping = ComposableNode(
        package='pointcloud_server',
        plugin='freespace_detection::FreespaceDetectionNode',
        name='freespace_detection',
        namespace=LaunchConfiguration('namespace'),
        parameters=[LaunchConfiguration('params_file')],
        remappings=[
            ('~/input', 'global_pointcloud_server/add'),
            ('~/freespace_cloud', 'global_pointcloud_server/freespace')

        ],
        condition=IfCondition(
            PythonExpression([
                "'", LaunchConfiguration('freespace_detection'),
                "' == 'true' and '", LaunchConfiguration('mode'),
                "' == 'mapping'"
            ])
        )
    )

    freespace_label_filter_node = ComposableNode(
        package='pointcloud_server',
        plugin='pointcloud_server::FilterNode',
        name='freespace_label_filter',
        namespace=LaunchConfiguration('namespace'),
        parameters=[LaunchConfiguration('params_file')],
        remappings=[
            ('~/input', 'global_pointcloud_server/freespace_label_output'),
            #('~/filtered', 'global_pointcloud_server/freespace_label_input')
        ],
        condition=IfCondition(
            PythonExpression([
                "'", LaunchConfiguration('freespace_detection'),
                "' == 'true' and '", LaunchConfiguration('mode'),
                "' == 'localization'"
            ])
        )
        
    )

    # -------------------------
    # Composable Node Containers
    # -------------------------
    localization_container = ComposableNodeContainer(
        name='pointcloud_server_container',
        namespace=LaunchConfiguration('namespace'),
        package='rclcpp_components',
        executable='component_container_mt',
        output='screen',
        # Load all nodes
        composable_node_descriptions=[
            global_server_node,
            local_server_node,
            lidar_filter_localization_node,
            label_filter_node,
            obstacle_tracker_node,  
            freespace_node_localization,
            freespace_label_filter_node
        ],
        parameters=[LaunchConfiguration('params_file')],
        condition=IfCondition(
            PythonExpression(["'", LaunchConfiguration('mode'), "' == 'localization'"])
        )
    )

    mapping_container = ComposableNodeContainer(
        name='pointcloud_server_container',
        namespace=LaunchConfiguration('namespace'),
        package='rclcpp_components',
        executable='component_container_mt',
        output='screen',
        # Load all nodes
        composable_node_descriptions=[
            global_server_node,
            lidar_filter_mapping_node,
            freespace_node_mapping
        ],
        parameters=[LaunchConfiguration('params_file')],
        condition=IfCondition(
            PythonExpression(["'", LaunchConfiguration('mode'), "' == 'mapping'"])
        )
    )

    return LaunchDescription([
        namespace_arg,
        param_file_arg,
        input_topic_arg,
        mode_arg,
        freespace_arg,
        localization_container,
        mapping_container
    ])
