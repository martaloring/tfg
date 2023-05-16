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
from std_msgs.msg import String, Bool
import vlc
import rclpy.time


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
        self.start_listening = True
        self._pub_text = self.create_publisher(String, "/predicted_text", 1)
        self._pub_start_conver = self.create_publisher(Bool, "/start_conver", 1)
        self._sub_listen = self.create_subscription(Bool, "/start_listening", self.callback_listen, 1)          

        
    def main_loop(self):     
        threading.Thread(target = rclpy.spin,args = (self,), daemon=True).start()   
        while (rclpy.ok()):
            if(self.start_listening):
                print(self.result_queue.get())

    def callback_listen(self, msg):
        self.start_listening = msg.data
        if(self.start_listening):
            print("START LISTENING")
            #restart everything
            self.audio_queue = None # vacio la cola ??
            self.audio_queue = queue.Queue()
            #self.result_queue = queue.Queue() # vacio la cola ??
            self.audio = None

            r_a_t = threading.Thread(target=self.record_audio)
            t_f_t = threading.Thread(target=self.transcribe_forever)
            r_a_t.start()
            t_f_t.start()        

        if(not self.start_listening):
            print("STOP LISTENING")
            self.source.stream.close()
    
    def record_audio(self):
        r = sr.Recognizer()
        r.energy_threshold = self.energy
        r.pause_threshold = self.pause
        r.dynamic_energy_threshold = self.dynamic_energy
        print("di algo 1")
        with sr.Microphone(sample_rate=16000) as self.source:
            print("Di algo 2")
            while (rclpy.ok()):
                if(self.start_listening):
                    self.audio = r.listen(self.source)
                    torch_audio = torch.from_numpy(np.frombuffer(self.audio.get_raw_data(), np.int16).flatten().astype(np.float32) / 32768.0)
                    self.audio_data = torch_audio
                    self.audio_queue.put_nowait(self.audio_data)

    
    def transcribe_forever(self):
        while (rclpy.ok()):
            if(self.start_listening):
                self.audio_data = self.audio_queue.get()
                self.result = self.audio_model.transcribe(self.audio_data)
                self.predicted_text = self.result["text"]
                self.detected_language = self.result["language"]

                if(self.detected_language == "es"):

                    if(self.send_text):
                        msg = String()
                        msg.data = self.predicted_text
                        self._pub_text.publish(msg)

                    self.uppercase_text = self.predicted_text.upper()
                    code1 = self.uppercase_text.find("HOLA")
                    code2 = self.uppercase_text.find("SANCHO")
                    code3 = self.uppercase_text.find("SANCIO")
                    code4 = self.uppercase_text.find("SANCHEZ")
                    code5 = self.uppercase_text.find("SÁNCHEZ")
                    code6 = self.uppercase_text.find("SONCHO")

                    if(code1 != -1 and (code2 != -1 or code3 != -1 or code4 != -1 or code5 != -1 or code6 != -1)):
                        print("HAS DICHO HOLA!!")
                        tts2 = TTS("tts_models/es/css10/vits")
                        tts2.tts_to_file(text="Hola.", file_path="/home/mapir/saludo.mp3")
                        p = vlc.MediaPlayer("file:///home/mapir/saludo.mp3")
                        p.play()

                        pub_start = Bool()
                        pub_start.data = True
                        self.send_text = True
                        self._pub_start_conver.publish(pub_start)

                    self.result_queue.put_nowait("Has dicho: " + self.predicted_text)

def main(args=None):
    rclpy.init(args=args)
    x = ASR()
    try:
        #rclpy.spin(x)
        x.main_loop()
    except ROSInterruptException: 
        pass

    rclpy.shutdown()

if __name__ == '__main__':
    main()