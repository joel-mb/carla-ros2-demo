import os

import launch
import launch_ros.actions

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    ld = launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            name='objects_definition_file',
            default_value=os.path.join(get_package_share_directory('ros2_demo'), 'objects.json')
        ),
        launch_ros.actions.Node(
            package='ros2_demo',
            executable='ros2_demo',
            name=['ros2_demo'],
            output='screen',
            on_exit=launch.actions.Shutdown(),
            parameters=[{
                    'objects_definition_file': launch.substitutions.LaunchConfiguration('objects_definition_file'),
            }]
        ),
        launch_ros.actions.Node(
            package='color_converter',
            executable='color_converter',
            name='color_converter'
        ),
        launch_ros.actions.Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', os.path.join(get_package_share_directory('ros2_demo'), 'ros2_demo.rviz')]
        ),
        launch_ros.actions.Node(
            package='rqt_publisher',
            executable='rqt_publisher',
            name='rqt_publisher'
        )
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()
