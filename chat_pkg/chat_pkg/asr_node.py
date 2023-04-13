import rclpy
import sys
from rclpy.node import Node
from rclpy.exceptions import ROSInterruptException
#sys.path.append('/home/marta/ros2_ws/src/chat_pkg')
import io
from pydub import AudioSegment
import speech_recognition as sr
import whisper
import queue
import tempfile
import os
import threading
import click
import torch
import numpy as np
from TTS.api import TTS
from std_msgs.msg import String
import vlc


class ASR(Node):
    def __init__(self):
        super().__init__('asr_node')  
        self.energy = 300
        self.dynamic_energy = False
        self.pause = 0.8
        self.audio_model = whisper.load_model("base")
        self.audio_queue = queue.Queue()
        self.result_queue = queue.Queue()
        self.send_text = False
        self._pub_text = self.create_publisher(String, "/predicted_text", 1)          

    def main_loop(self):
        
        threading.Thread(target=self.record_audio).start()
        threading.Thread(target=self.transcribe_forever).start()
        
        while (rclpy.ok()):
            print(self.result_queue.get())

    def record_audio(self):
        r = sr.Recognizer()
        r.energy_threshold = self.energy
        r.pause_threshold = self.pause
        r.dynamic_energy_threshold = self.dynamic_energy

        with sr.Microphone(sample_rate=16000) as source:
            print("Di algo!")
            while (rclpy.ok()):
                audio = r.listen(source)
                torch_audio = torch.from_numpy(np.frombuffer(audio.get_raw_data(), np.int16).flatten().astype(np.float32) / 32768.0)
                self.audio_data = torch_audio

                self.audio_queue.put_nowait(self.audio_data)
    
    def transcribe_forever(self):
        while (rclpy.ok()):
            self.audio_data = self.audio_queue.get()
            self.result = self.audio_model.transcribe(self.audio_data)
            self.predicted_text = self.result["text"]

            if(self.send_text):
                msg = String()
                msg.data = self.predicted_text
                self._pub_text.publish(msg)
        
            if(self.predicted_text == " Hola, Sancio." or self.predicted_text == " Hola, Sancho."):
                print("HAS DICHO SANCHO!!")
                tts2 = TTS("tts_models/es/css10/vits")

                tts2.tts_to_file(text="Hola, Marta.", file_path="/home/mapir/saludo.mp3")

                p = vlc.MediaPlayer("file:///home/mapir/saludo.mp3")
                p.play()

                self.send_text = True

            self.result_queue.put_nowait("Has dicho: " + self.predicted_text)

def main(args=None):
    rclpy.init(args=args)
    try:
        x = ASR()
        x.main_loop()
    except ROSInterruptException: 
        pass

    rclpy.shutdown()

if __name__ == '__main__':
    main()