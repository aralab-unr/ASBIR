from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, TextSubstitution
from launch_ros.actions import Node
from launch.actions import TimerAction
from ament_index_python.packages import get_package_share_directory

apriltag_dir = get_package_share_directory('apriltag_ros')

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2'
        ),
        Node(
            package = "image_transport",
            executable = "republish",
            arguments = ["compressed", "raw", "--ros-args", 
                         "--remap", "/in/compressed:=/D400/color/compressed",
                         "--remap", "/out:=/D400/color/decompressed"]
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                apriltag_dir + '/launch/tag_realsense.launch.py'),
        ),
        Node(
            package = "asbir_navigation",
            executable = "AlignAprilTag",
            name = 'AlignAprilTag'
        ),
        Node(
            package = "asbir_navigation",
            executable = "AlignStructure",
            name = 'AlignStructure'
        ),
        TimerAction(
            period = 10.0,
            actions = [
                Node(
                    package='asbir_navigation',
                    executable='ModelTest',
                    name='Model'
                ),
                Node(
                    package='asbir_navigation',
                    executable='GraphTest',
                    name='Graph'
                ),
                Node(
                    package='asbir_navigation',
                    executable='BuildBestPath',
                    name='BuildPath',
                ),
                Node(
                    package='asbir_navigation',
                    executable='PathController',
                    name='PathController',
                ),
                Node(
                    package='asbir_navigation',
                    executable='PotentialField',
                    name='PotentialField',
                ),
            ]),
    ])
