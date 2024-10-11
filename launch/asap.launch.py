from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    asap_yaml = os.path.join(get_package_share_directory('asap'), 'config', 'asap.yaml')
    print(f"Loading parameters from {asap_yaml}")

    asap = Node(package='asap', executable='asap_node', name='asap', output='screen', parameters=[asap_yaml])
    return LaunchDescription(asap)
