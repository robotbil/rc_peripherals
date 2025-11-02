import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    # Declare arguments
    lidar_frame = LaunchConfiguration('lidar_frame', default='lidar_frame')
    scan_raw = LaunchConfiguration('scan_raw', default='sensors/lidar/scan_raw')
    scan_topic = LaunchConfiguration('scan_topic', default='sensors/lidar/scan')

    lidar_frame_arg = DeclareLaunchArgument('lidar_frame',default_value='lidar_frame',description='TF frame ID for the lidar')
    scan_raw_arg = DeclareLaunchArgument('scan_raw',default_value='sensors/lidar/scan_raw',description='Topic name for lidar_raw scan data')
    scan_topic_arg = DeclareLaunchArgument('scan_topic',default_value='sensors/lidar/scan',description='Topic name for lidar scan data')

    # Paths
    peripherals_package_path = get_package_share_directory('rc_peripherals')
    oradar_package_path = get_package_share_directory('oradar_lidar')

    lidar_launch_path = os.path.join(oradar_package_path, 'launch/ms200_scan.launch.py')
    laser_filters_config = os.path.join(peripherals_package_path, 'config/lidar_filters_config_ms200.yaml')
    
    # Include Lidar launch file
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(lidar_launch_path),
        launch_arguments={
            'topic_name': LaunchConfiguration('scan_topic'),
            'frame_id': LaunchConfiguration('lidar_frame')
        }.items()
    )



    laser_filter_node = Node(
        package='laser_filters',
        executable='scan_to_scan_filter_chain',
        output='screen',
        parameters=[laser_filters_config],
        remappings=[('scan', scan_raw),
                    ('scan_filtered', scan_topic)]
    )
    return LaunchDescription([
        scan_topic_arg,
        scan_raw_arg,
        lidar_frame_arg,

        lidar_launch,
        laser_filter_node
    ])


if __name__ == '__main__':
    from launch import LaunchService
    ld = generate_launch_description()

    ls = LaunchService()
    ls.include_launch_description(ld)
    ls.run()

