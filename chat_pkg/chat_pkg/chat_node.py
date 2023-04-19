import rclpy
import sys
from rclpy.node import Node
from rclpy.exceptions import ROSInterruptException
import threading
from std_msgs.msg import String, Bool
from langdetect import detect

from youdotcom import Chat # import all the classes

class chat_response(Node):
    def __init__(self):
        super().__init__('chat_node')
        self.ask_chat = False
        self.in_conver = False
        self._sub_text = self.create_subscription(String, "/predicted_text", self.callback_text, 1)
        self._pub_start_conver = self.create_subscription(Bool, "/start_conver", self.callback_start, 1)      
        self._pub_response = self.create_publisher(String, "/input_tts", 1)
        self._pub_end_conver = self.create_publisher(Bool, "/end_conver", 1)


    def callback_text(self, msg):
        self.past = self.get_clock().now()
        self.ask_chat = True
        self.predicted_text = msg.data
        print("He recibido una pregunta")

    def callback_start(self, msg):
        self.past = self.get_clock().now()  
        self.in_conver = True
        print("Empezar conversacion")

    def generate_response(self):

        threading.Thread(target = rclpy.spin,args = (self,), daemon=True).start()
        
        while (rclpy.ok()):
            if(self.in_conver):
                if(self.ask_chat):
                    #chat = Chat.send_message(message="Responde en español a la siguiente pregunta: ¿Tiene 'whisper' de OpenIA una API gratuita?", api_key="1E33LVSKM5XSL2GFJDPBE5RMZGZSW46D3PH") # send a message to YouChat. passing the message and your api key
                    chat = Chat.send_message(message=self.predicted_text, api_key="1E33LVSKM5XSL2GFJDPBE5RMZGZSW46D3PH") # send a message to YouChat. passing the message and your api key

                    # you can get an api key form the site: https://api.betterapi.net/ (with is also made by me)
                    print(chat)  # returns the message and some other data

                    output = String()
                    output.data = chat['message']
                    code = output.data.find("YouBot")

                    if (detect(output.data) == 'es'):
                        print("buena respuesta")
                        self._pub_response.publish(output)

                    self.ask_chat = False
                
                now = self.get_clock().now()

                if((now - self.past).nanoseconds*1e-9) > 20:
                    self.ask_chat = False
                    msg = Bool()
                    msg.data = True
                    self._pub_end_conver.publish(msg)
                    print("end conversation!!!")
                    self.in_conver = False


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