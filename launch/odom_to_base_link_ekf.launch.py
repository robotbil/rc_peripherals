from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction, IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    peripherals_package_path = get_package_share_directory('rc_peripherals')
    ekf_config_file = os.path.join(peripherals_package_path, 'config', 'ekf.yaml')

    ekf_filter_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config_file, {'use_sim_time': LaunchConfiguration('simulation')}],
     )

    return LaunchDescription([
        ekf_filter_node
    ])

if __name__ == '__main__':
    from launch import LaunchService
    ld = generate_launch_description()

    ls = LaunchService()
    ls.include_launch_description(ld)
    ls.run()

