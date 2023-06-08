from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

realsense2_dir = get_package_share_directory('realsense2_camera')


def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                realsense2_dir + '/launch/rs_d400_and_t265_launch.py'),
            launch_arguments={
                'enable_pointcloud': 'true',
            })
    ])