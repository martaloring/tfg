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

from youdotcom import Chat


class chat_response(Node):
    def __init__(self):
        super().__init__('test_2')  
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

    def main_fun(self):
        
        threading.Thread(target=self.record_audio).start()
        threading.Thread(target=self.transcribe_forever).start()
        
        while True:
            print(self.result_queue.get())

    def record_audio(self):
        #load the speech recognizer and set the initial energy threshold and pause threshold
        r = sr.Recognizer()
        r.energy_threshold = self.energy
        r.pause_threshold = self.pause
        r.dynamic_energy_threshold = self.dynamic_energy

        with sr.Microphone(sample_rate=16000) as source:
            print("Say something!")
            while True:
                audio = r.listen(source)
                torch_audio = torch.from_numpy(np.frombuffer(audio.get_raw_data(), np.int16).flatten().astype(np.float32) / 32768.0)
                self.audio_data = torch_audio

                self.audio_queue.put_nowait(self.audio_data)
    
    def transcribe_forever(self):
        while True:
            self.audio_data = self.audio_queue.get()
            self.result = self.audio_model.transcribe(self.audio_data)
            self.predicted_text = self.result["text"]
            self.result_queue.put_nowait("You said: " + self.predicted_text)
            self.generate_response()


    def generate_response(self):
        
        self.result["text"] = "Responde en español a la siguiente pregunta: "+ self.predicted_text
        chat = Chat.send_message(message=self.result, api_key="1E33LVSKM5XSL2GFJDPBE5RMZGZSW46D3PH") # send a message to YouChat. passing the message and your api key
        print("Respuesta:")
        # you can get an api key form the site: https://api.betterapi.net/ (with is also made by me)
        print(chat['message'])  # returns the message and some other data
        self.main_fun()

def main(args=None):
    rclpy.init(args=args)
    try:
        x = chat_response()
        x.main_fun()
    except ROSInterruptException: 
        pass

    rclpy.shutdown()

if __name__ == '__main__':
    main()