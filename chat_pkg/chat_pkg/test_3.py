import rclpy
import sys
from rclpy.node import Node
from rclpy.exceptions import ROSInterruptException
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
        self.model = "base"
        self.english = False
        self.verbose = False
        self.energy = 300
        self.dynamic_energy = False
        self.pause = 0.8
        self.save_file = False      

    def main_fun(self):
        self.temp_dir = tempfile.mkdtemp() if self.save_file else None
        #there are no english models for large
        if self.model != "large" and self.english:
            self.model = self.model + ".en"
        self.audio_model = whisper.load_model(self.model)
        self.audio_queue = queue.Queue()
        self.result_queue = queue.Queue()
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
            i = 0
            while True:
                #get and save audio to wav file
                audio = r.listen(source)
                if self.save_file:
                    data = io.BytesIO(audio.get_wav_data())
                    audio_clip = AudioSegment.from_file(data)
                    filename = os.path.join(self.temp_dir, f"temp{i}.wav")
                    audio_clip.export(filename, format="wav")
                    audio_data = filename
                else:
                    torch_audio = torch.from_numpy(np.frombuffer(audio.get_raw_data(), np.int16).flatten().astype(np.float32) / 32768.0)
                    audio_data = torch_audio

                self.audio_queue.put_nowait(audio_data)
                i += 1


    def transcribe_forever(self):
        while True:
            audio_data = self.audio_queue.get()
            if self.english:
                result = self.audio_model.transcribe(audio_data,language='english')
            else:
                result = self.audio_model.transcribe(audio_data)

            if not self.verbose:
                predicted_text = result["text"]
                self.result_queue.put_nowait("You said: " + predicted_text)
            else:
                self.result_queue.put_nowait(result)

            if self.save_file:
                os.remove(audio_data)


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
