import rclpy
from rclpy.node import Node
from rclpy.exceptions import ROSInterruptException
import threading
from std_msgs.msg import String, Bool
from langdetect import detect
from youdotcom import Chat # import all the classes

class chat_response(Node):
    def __init__(self):
        super().__init__('chat_node')
        
        ################################### CHAT PARAMETERS ################################
        self.declare_parameter('CHAT.language', 'es')  
        self.lang_param = self.get_parameter('CHAT.language').get_parameter_value().string_value

        self.declare_parameter('CHAT.betterapi_key', '1E33LVSKM5XSL2GFJDPBE5RMZGZSW46D3PH')  
        self.betterapi_key = self.get_parameter('CHAT.betterapi_key').get_parameter_value().string_value

        self.declare_parameter('CHAT.timeout_secs', 180)  
        self.timeout_secs = self.get_parameter('CHAT.timeout_secs').get_parameter_value().integer_value

        
        ################################### TOPIC PARAMETERS ################################
        self.declare_parameter('ROSTopics.asr_output_topic', '/predicted_text')  
        self.input_text_topic = self.get_parameter('ROSTopics.asr_output_topic').get_parameter_value().string_value

        self.declare_parameter('ROSTopics.start_conver_topic', '/start_conver')  
        self.start_conver_topic = self.get_parameter('ROSTopics.start_conver_topic').get_parameter_value().string_value

        self.declare_parameter('ROSTopics.end_conver_topic', '/end_conver')  
        self.end_conver_topic = self.get_parameter('ROSTopics.end_conver_topic').get_parameter_value().string_value

        self.declare_parameter('ROSTopics.tts_input_topic', '/input_tts')  
        self.input_tts_topic = self.get_parameter('ROSTopics.tts_input_topic').get_parameter_value().string_value

        ################################### INITIALIZATION ################################
        self._sub_text = self.create_subscription(String, self.input_text_topic, self.callback_text, 1)
        self._pub_start_conver = self.create_subscription(Bool, self.start_conver_topic, self.callback_start, 1)      
        self._pub_response = self.create_publisher(String, self.input_tts_topic, 1)
        self._pub_end_conver = self.create_publisher(Bool, self.end_conver_topic, 1)
        self.ask_chat = False
        self.in_conver = False


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
                    chat = Chat.send_message(message=self.predicted_text, api_key=self.betterapi_key) # send a message to YouChat. passing the message and your api key
                    # you can get an api key form the site: https://api.betterapi.net/ (with is also made by me)
                    print(chat)  # returns the message and some other data

                    output = String()
                    output.data = chat['generated_text']
                    code = output.data.find("YouBot")

                    if (detect(output.data) == self.lang_param):
                        print("VALID ANSWER")
                        self._pub_response.publish(output)

                    self.ask_chat = False
                
                now = self.get_clock().now()

                if((now - self.past).nanoseconds*1e-9) > self.timeout_secs:
                    self.ask_chat = False
                    msg = Bool()
                    msg.data = True
                    self._pub_end_conver.publish(msg)
                    print("TIME OUT. END OF CONVERSATION.")
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