import rclpy
from rclpy.node import Node
from rclpy.exceptions import ROSInterruptException
import threading
from TTS.api import TTS
import vlc
import re
from num2words import num2words
from std_msgs.msg import String, Bool


class TTS_(Node):
    def __init__(self):
        super().__init__('tts_node')  
        
        ################################### TTS PARAMETERS ################################
        self.declare_parameter('TTS.language', 'es')  
        self.lang_param = self.get_parameter('TTS.language').get_parameter_value().string_value

        self.declare_parameter('TTS.model_name', 'tts_models/es/css10/vits')  
        self.model_name = self.get_parameter('TTS.model_name').get_parameter_value().string_value

        self.declare_parameter('TTS.output_path', '/home/mapir/ros2_ws/src/interaction_pkg/mp3_files/output.mp3')  
        self.output_path = self.get_parameter('TTS.output_path').get_parameter_value().string_value

        ################################### TOPIC PARAMETERS ################################
        self.declare_parameter('ROSTopics.start_listening_topic', '/start_listening')  
        self.start_listening_topic = self.get_parameter('ROSTopics.start_listening_topic').get_parameter_value().string_value

        self.declare_parameter('ROSTopics.tts_input_topic', '/input_tts')  
        self.input_tts_topic = self.get_parameter('ROSTopics.tts_input_topic').get_parameter_value().string_value

        ################################### INITIALIZATION ################################
        self._sub_resp = self.create_subscription(String, self.input_tts_topic, self.callback_chat, 1)
        self._pub_listen = self.create_publisher(Bool, self.start_listening_topic, 1)

        self.stop_lis = Bool()
        self.stop_lis.data = False
        self.start_lis = Bool()
        self.start_lis.data = True

        self.speak = False
        self.speaking = False

        self._pub_listen.publish(self.start_lis)

    def callback_chat(self, msg):
        self.speak = True
        self.response = self.replace_numbers_with_words(msg.data)
        

    def replace_numbers_with_words(self, str_num):
        pattern = r'\d+'
        matches = re.findall(pattern,str_num)
        cont = 0
        str_word = str_num
        for match in matches:
            cont = cont + 1
            str_word = str_word.replace(match, num2words(int(match),lang=self.lang_param))

        return str_word       

    def main_loop(self):

        threading.Thread(target = rclpy.spin,args = (self,), daemon=True).start()
                
        while (rclpy.ok()):

            if(self.speaking):
                now = self.get_clock().now()
                if((now - self.past).nanoseconds*1e-9) > self.duracion_seg:         # si ya ha pasado el tiempo que dura el audio, asr vuelve a escuchar
                    self._pub_listen.publish(self.start_lis)
                    self.speaking = False

            if(self.speak):                                                     # si tenemos nuevo input, speak
                self.run_tts()
                self.speak = False

    def run_tts(self):
        
        tts2 = TTS(self.model_name)

        tts2.tts_to_file(text=self.response, file_path=self.output_path)
        
        vlc_instance = vlc.Instance('--no-xlib')  # Crear una instancia de VLC
        media = vlc_instance.media_new(self.output_path)  # Crear un objeto media para el archivo MP3
        media.parse()  # Parsear la información del archivo MP3
        self.duracion_seg = media.get_duration()/1000 # Obtener la duración en segundos
        print("LA DURACION DEL AUDIO ES:")
        print(self.duracion_seg)

        self.past = self.get_clock().now()  
        self._pub_listen.publish(self.stop_lis)
        p = vlc.MediaPlayer("file://"+self.output_path)
        p.play()

        self.speaking = True
        

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