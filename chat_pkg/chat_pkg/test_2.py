import rclpy
import sys
from rclpy.node import Node
from rclpy.exceptions import ROSInterruptException
sys.path.append('/home/marta/ros2_ws/src/chat_pkg')
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

from youdotcom import Chat # import all the classes
import whisper

# @click.command()
# @click.option("--model", default="base", help="Model to use", type=click.Choice(["tiny","base", "small","medium","large"]))
# @click.option("--english", default=False, help="Whether to use English model",is_flag=True, type=bool)
# @click.option("--verbose", default=False, help="Whether to print verbose output", is_flag=True,type=bool)
# @click.option("--energy", default=300, help="Energy level for mic to detect", type=int)
# @click.option("--dynamic_energy", default=False,is_flag=True, help="Flag to enable dynamic engergy", type=bool)
# @click.option("--pause", default=0.8, help="Pause time before entry ends", type=float)
# @click.option("--save_file",default=False, help="Flag to save file", is_flag=True,type=bool)

class chat_response(Node):
    def __init__(self):
        super().__init__('test_2')      
        

    def main_fun(self):
        temp_dir = tempfile.mkdtemp()
        audio_model = whisper.load_model("base")
        audio_queue = queue.Queue()
        result_queue = queue.Queue()
        threading.Thread(target=self.record_audio,
                        args=(audio_queue, temp_dir)).start()
        threading.Thread(target=self.transcribe_forever,
                        args=(audio_queue, result_queue, audio_model)).start()

        while True:
            print("Pregunta:")
            print(result_queue.get()["text"])
            self.result = result_queue.get()["text"]
            self.generate_response()

    def record_audio(self,audio_queue, temp_dir):
        #load the speech recognizer and set the initial energy threshold and pause threshold
        r = sr.Recognizer()

        with sr.Microphone(sample_rate=16000) as source:
            print("Say something!")
            i = 0
            while True:
                audio = r.listen(source)
                torch_audio = torch.from_numpy(np.frombuffer(audio.get_raw_data(), np.int16).flatten().astype(np.float32) / 32768.0)
                audio_data = torch_audio

                audio_queue.put_nowait(audio_data)
                i += 1
    
    def transcribe_forever(self, audio_queue, result_queue, audio_model):
        while True:
            audio_data = audio_queue.get()
            result = audio_model.transcribe(audio_data)
            result_queue.put_nowait(result)


    def generate_response(self):
        
        #chat = Chat.send_message(message="Responde en español a la siguiente pregunta: ¿Tiene 'whisper' de OpenIA una API gratuita?", api_key="1E33LVSKM5XSL2GFJDPBE5RMZGZSW46D3PH") # send a message to YouChat. passing the message and your api key
        chat = Chat.send_message(message=self.result, api_key="1E33LVSKM5XSL2GFJDPBE5RMZGZSW46D3PH") # send a message to YouChat. passing the message and your api key
        print("Respuesta")
        # you can get an api key form the site: https://api.betterapi.net/ (with is also made by me)
        print(chat["message"])  # returns the message and some other data

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