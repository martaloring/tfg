import rclpy
import sys
from rclpy.node import Node
from rclpy.exceptions import ROSInterruptException
sys.path.append('/home/marta/ros2_ws/src/chat_pkg')

from youdotcom import Chat # import all the classes
import whisper
import base64
from scipy.io.wavfile import read
import numpy as np
import pickle
import torch

class whisper_node(Node):
    def __init__(self):
        super().__init__('whisper_node')
        self.new_audio = True
        self.model = whisper.load_model("base")
        self._mp3_path = '/home/mapir/ros2_ws/src/chat_pkg/mp3_files/provincias.mp3'
        #self._sub_text = self.create_subscription(String, "/audio_data", self.callback_audio, 1)


        self.original_data = open("/home/mapir/ros2_ws/src/chat_pkg/mp3_files/provincias.mp3", 'rb').read()
        self.torch_audio = torch.from_numpy(np.frombuffer(self.original_data, np.int16).flatten().astype(np.float32) / 32768.0)

        #fs, self.original_data = read("/home/mapir/ros2_ws/src/chat_pkg/mp3_files/provincias.wav")
        #print(type(self.original_data))
        #dt = self.original_data.dtype
        #print(dt.name)
        #self.torch_audio = torch.from_numpy(self.original_data.astype(np.float32) / 32768.0)

        # self.encoded_data = base64.b64encode(self.original_data)
        # #print("encoded data:")
        # #print(self.encoded_data)
        # self.decoded_data = base64.b64decode(self.encoded_data)
        #self.final_data = np.frombuffer(self.decoded_data)
        #self.final_data = pickle.loads(self.decoded_data)

    # def callback_audio(self,msg):
    #     self.audio_data = msg
    #     self.new_audio = True
        
    def transcribe(self):
        while (rclpy.ok()):
            if(self.new_audio):
                #self.result = self.model.transcribe(self._mp3_path)
                self.result = self.model.transcribe(self.torch_audio)
                print("decoded_data:")
                print(self.result["text"])
                self.new_audio = False

def main(args=None):
    rclpy.init(args=args)
    try:
        x = whisper_node()
        x.transcribe()
    except ROSInterruptException: 
        pass

    rclpy.shutdown()

if __name__ == '__main__':
    main()