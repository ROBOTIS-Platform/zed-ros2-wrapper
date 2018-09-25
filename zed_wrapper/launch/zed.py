import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    # use: 'zed' for "ZED" camera - 'zedm' for "ZED mini" camera
    camera_model = 'zedm' 

    # URDF file to be loaded by Robot State Publisher
    urdf = os.path.join(get_package_share_directory('stereolabs_zed'),
                        'urdf', camera_model + '.urdf')
    
    # ZED Configurations to be loaded by ZED Node
    config_common = os.path.join(get_package_share_directory('stereolabs_zed'),
                        'config', 'common.yaml')

    config_camera = os.path.join(get_package_share_directory('stereolabs_zed'),
                        'config', camera_model + '.yaml')

    # Set LOG format
    os.environ['RCUTILS_CONSOLE_OUTPUT_FORMAT'] = '{time}: [{name}] [{severity}]\t{message}'

    return LaunchDescription([
        # Robot State Publisher
        Node(package='robot_state_publisher', 
             node_executable='robot_state_publisher',
             output='screen', 
             arguments=[urdf]),
        
        # ZED
        Node(package='stereolabs_zed', 
             node_executable='zed_wrapper_node',
             output='screen', 
             arguments=['__params:='+config_common, # Common parameters
                        '__params:='+config_camera  # Camera related parameters
                       ])
    ])
