import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
                
    return LaunchDescription([
        
        # ASR (SPEECH RECOGNITION)   
        Node(
            package = 'chat_pkg',
            name = 'asr_node',
            executable = 'asr_node',
            output = 'screen',
            prefix = 'xterm -e'),

        # CHAT BOT
        Node(
            package = 'chat_pkg',
            name = 'chat_node',
            executable = 'chat_node',
            output = 'screen',
            prefix = 'xterm -e'),
        
        # TTS (TEXT TO SPEECH)
        Node(
            package = 'chat_pkg',
            name = 'tts_node',
            executable = 'tts_node',
            output = 'screen',
            prefix = 'xterm -e')
                        
    ])
