import rclpy
import sys
from rclpy.node import Node
from rclpy.exceptions import ROSInterruptException
sys.path.append('/home/marta/ros2_ws/src/chat_pkg')

from youdotcom import Chat # import all the classes
import whisper

class chat_response(Node):
    def __init__(self):
        super().__init__('test_1')

        model = whisper.load_model("base")
        self._mp3_path = '/home/marta/ros2_ws/src/chat_pkg/mp3_files/provincias.mp3'
        self.result = model.transcribe(self._mp3_path)
        print(self.result["text"])

    def generate_response(self):
        
        #chat = Chat.send_message(message="Responde en español a la siguiente pregunta: ¿Tiene 'whisper' de OpenIA una API gratuita?", api_key="1E33LVSKM5XSL2GFJDPBE5RMZGZSW46D3PH") # send a message to YouChat. passing the message and your api key
        chat = Chat.send_message(message=self.result["text"], api_key="1E33LVSKM5XSL2GFJDPBE5RMZGZSW46D3PH") # send a message to YouChat. passing the message and your api key

        # you can get an api key form the site: https://api.betterapi.net/ (with is also made by me)
        print(chat)  # returns the message and some other data

def main(args=None):
    rclpy.init(args=args)
    try:
        x = chat_response()
        x.generate_response()
    except ROSInterruptException: 
        pass

    rclpy.shutdown()

if __name__ == '__main__':
    main()