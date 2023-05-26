import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('interaction_pkg'),
        'config',
        'interaction.yaml'
        )
                
    return LaunchDescription([
        
        # ASR (SPEECH RECOGNITION)   
        Node(
            package = 'interaction_pkg',
            name = 'asr_node',
            executable = 'asr_node',
            output = 'screen',
            prefix = 'xterm -e',
            parameters = [config]),

        # CHAT BOT
        Node(
            package = 'interaction_pkg',
            name = 'chat_node',
            executable = 'chat_node',
            output = 'screen',
            prefix = 'xterm -e',
            parameters = [config]),
            
        # TTS (TEXT TO SPEECH)
        Node(
            package = 'interaction_pkg',
            name = 'tts_node',
            executable = 'tts_node',
            output = 'screen',
            prefix = 'xterm -e',
            parameters = [config]),
            
        # INTERFACE
        Node(
            package = 'interaction_pkg',
            name = 'interface_node',
            executable = 'interface_node',
            output = 'screen',
            prefix = 'xterm -e',
            parameters = [config])
                        
    ])
