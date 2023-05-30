import rclpy
from rclpy.node import Node
from rclpy.exceptions import ROSInterruptException
import speech_recognition as sr
import whisper
import queue
import threading
import torch
import numpy as np
from TTS.api import TTS
from std_msgs.msg import String, Bool
import vlc
import rclpy.time


class ASR(Node):
    def __init__(self):
        super().__init__('asr_node')

        ################################### ASR PARAMETERS ################################
        self.declare_parameter('ASR.energy', 300)  
        self.energy = self.get_parameter('ASR.energy').get_parameter_value().integer_value

        self.declare_parameter('ASR.dynamic_energy', False)  
        self.dynamic_energy = self.get_parameter('ASR.energy').get_parameter_value().bool_value

        self.declare_parameter('ASR.pause', 0.8)  
        self.pause = self.get_parameter('ASR.pause').get_parameter_value().double_value

        self.declare_parameter('ASR.model', 'base')  
        self.model_param = self.get_parameter('ASR.model').get_parameter_value().string_value

        self.declare_parameter('ASR.language', 'es')  
        self.lang_param = self.get_parameter('ASR.language').get_parameter_value().string_value

        self.declare_parameter('ASR.intro_msg', 'Hola.')  
        self.intro_msg = self.get_parameter('ASR.intro_msg').get_parameter_value().string_value

        self.declare_parameter('ASR.intro_path', '/home/mapir/ros2_ws/src/interaction_pkg/mp3_files/saludo.mp3')  
        self.intro_path = self.get_parameter('ASR.intro_path').get_parameter_value().string_value

        self.declare_parameter('TTS.model_name', 'tts_models/es/css10/vits')  
        self.tts_model = self.get_parameter('TTS.model_name').get_parameter_value().string_value

        self.declare_parameter('ASR.keyword_1', 'hola')  
        self.keyword_1 = self.get_parameter('ASR.keyword_1').get_parameter_value().string_value

        self.declare_parameter('ASR.keyword_2', 'sancho')  
        self.keyword_2 = self.get_parameter('ASR.keyword_2').get_parameter_value().string_value

        # falta cambiar lo de los keywords

        ################################### TOPIC PARAMETERS ################################
        self.declare_parameter('ROSTopics.asr_output_topic', '/predicted_text')  
        self.output_topic = self.get_parameter('ROSTopics.asr_output_topic').get_parameter_value().string_value

        self.declare_parameter('ROSTopics.start_conver_topic', '/start_conver')  
        self.start_conver_topic = self.get_parameter('ROSTopics.start_conver_topic').get_parameter_value().string_value

        self.declare_parameter('ROSTopics.start_listening_topic', '/start_listening')  
        self.start_listening_topic = self.get_parameter('ROSTopics.start_listening_topic').get_parameter_value().string_value

        ################################### INITIALIZATION ################################
        self.audio_model = whisper.load_model(self.model_param)
        self.audio_queue = queue.Queue()
        self.result_queue = queue.Queue()
        self.send_text = False
        self.start_listening = True
        self._pub_text = self.create_publisher(String, self.output_topic, 1)
        self._pub_start_conver = self.create_publisher(Bool, self.start_conver_topic, 1)
        self._sub_listen = self.create_subscription(Bool, self.start_listening_topic, self.callback_listen, 1)          

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
            self.audio_queue = None
            self.audio_queue = queue.Queue()
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
        print("Inicializando...")
        with sr.Microphone(sample_rate=16000) as self.source:
            print("Escuchando...")
            while (rclpy.ok()):
                if(self.start_listening):
                    self.audio = r.listen(self.source)
                    torch_audio = torch.from_numpy(np.frombuffer(self.audio.get_raw_data(), np.int16).flatten().astype(np.float32) / 32768.0)
                    self.audio_data = torch_audio
                    self.audio_queue.put_nowait(self.audio_data)

    def find_keyword(self,text):
        uppercase_text = text.upper()
        
        keyword_1_upper = self.keyword_1.upper()
        keyword_2_upper = self.keyword_2.upper()

        code1 = uppercase_text.find(keyword_1_upper)
        code2 = uppercase_text.find(keyword_2_upper)

        # para mi caso concreto quiero añadir mas keywords
        if(self.keyword_2.upper() == "SANCHO"):
            code3 = uppercase_text.find("SANCIO")
            code4 = uppercase_text.find("SANCHEZ")
            code5 = uppercase_text.find("SÁNCHEZ")
            code6 = uppercase_text.find("SONCHO")
        else:
            code3 = -1
            code4 = -1
            code5 = -1
            code6 = -1

        if(code1 != -1 and (code2 != -1 or code3 != -1 or code4 != -1 or code5 != -1 or code6 != -1)):
            print("SOLICITUD DE INTERACCIÓN")
            tts2 = TTS(self.tts_model)
            tts2.tts_to_file(text=self.intro_msg, file_path=self.intro_path)
            p = vlc.MediaPlayer("file://"+self.intro_path)
            p.play()

            pub_start = Bool()
            pub_start.data = True
            self.send_text = True
            self._pub_start_conver.publish(pub_start)

    
    def transcribe_forever(self):
        while (rclpy.ok()):
            if(self.start_listening):
                self.audio_data = self.audio_queue.get()
                self.result = self.audio_model.transcribe(self.audio_data)
                self.predicted_text = self.result["text"]
                self.detected_language = self.result["language"]

                if(self.detected_language == self.lang_param):

                    if(self.send_text):
                        msg = String()
                        msg.data = self.predicted_text
                        self._pub_text.publish(msg)

                    self.find_keyword(self.predicted_text)

                    self.result_queue.put_nowait("Has dicho: " + self.predicted_text)

def main(args=None):
    rclpy.init(args=args)
    x = ASR()
    try:
        x.main_loop()
    except ROSInterruptException: 
        pass

    rclpy.shutdown()

if __name__ == '__main__':
    main()