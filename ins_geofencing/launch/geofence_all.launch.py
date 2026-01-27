from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # 統一參數設定
    common_params = [{'use_sim_time': True}]

    # 1. RTK TF Broadcaster (來自 ins_localization)
    rtk_tf_node = Node(
        package='ins_localization',
        executable='rtk_tf_broadcaster',
        name='rtk_tf_broadcaster',
        parameters=common_params,
        output='screen'
    )

    # 2. Zone Visualizer (來自 ins_geofencing)
    visualizer_node = Node(
        package='ins_geofencing',
        executable='zone_visualizer',
        name='zone_visualizer',
        parameters=common_params,
        output='screen'
    )

    # 3. Geofence Monitor (來自 ins_geofencing)
    monitor_node = Node(
        package='ins_geofencing',
        executable='geofence_monitor',
        name='geofence_monitor',
        parameters=common_params,
        output='screen'
    )

    return LaunchDescription([
        rtk_tf_node,
        visualizer_node,
        monitor_node
    ])