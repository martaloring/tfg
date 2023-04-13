import rclpy
import sys
from rclpy.node import Node
from rclpy.exceptions import ROSInterruptException
import whisper
import queue
import tempfile
import threading
from TTS.api import TTS
import vlc

from std_msgs.msg import String


class TTS_(Node):
    def __init__(self):
        super().__init__('tts_node')  
        self.model = "base"
        self.english = False
        self.verbose = False
        self.energy = 300
        self.dynamic_energy = False
        self.pause = 0.8
        self.save_file = False
        self.temp_dir = tempfile.mkdtemp()
        self.audio_model = whisper.load_model("base")
        self.audio_queue = queue.Queue()
        self.result_queue = queue.Queue()
        self.speak = False
        self._sub_resp = self.create_subscription(String, "/response", self.callback_chat, 1)

    def callback_chat(self, msg):
        self.speak = True
        self.response = msg.data       

    def main_loop(self):

        threading.Thread(target = rclpy.spin,args = (self,), daemon=True).start()
                
        while (rclpy.ok()):
            if(self.speak):
                self.run_tts()
                self.speak = False

    def run_tts(self):
        
        tts2 = TTS("tts_models/es/css10/vits")

        tts2.tts_to_file(text=self.response, file_path="/home/mapir/output_test.mp3")

        p = vlc.MediaPlayer("file:///home/mapir/output_test.mp3")
        p.play()

def main(args=None):
    rclpy.init(args=args)
    try:
        x = TTS_()
        x.main_loop()
    except ROSInterruptException: 
        pass

    rclpy.shutdown()

if __name__ == '__main__':
    main()