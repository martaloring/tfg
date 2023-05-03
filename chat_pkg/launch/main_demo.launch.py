import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    openpose_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(
            'openpose_pkg'),
            'launch'),
            '/main.launch.py'])
    )
         
    nav_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(
            'nav2_pkg'),
            'launch'),
            '/main.launch.py'])
    )
            
    return LaunchDescription([
        # NAV MAIN
        nav_launch,
        
        # RVIZ
        Node(
            package = 'rviz2',
            name = 'rviz2',
            executable = 'rviz2',
            output = 'screen'),
        
        # OPENPOSE MAIN
        openpose_launch,
                    
        # POSES MANAGER
        Node(
            package = 'chat_pkg',
            name = 'poses_mng',
            executable = 'poses_mng',
            output = 'screen',
            prefix = 'xterm -e')
            
    ])
