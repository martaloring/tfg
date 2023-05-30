import rclpy
from rclpy.node import Node
from rclpy.exceptions import ROSInterruptException
from std_msgs.msg import String, Bool
import threading
import sys
from termcolor import colored, cprint

class chat_interface(Node):
    def __init__(self):
        super().__init__('interface_node')

        ################################### INTERFACE PARAMETERS ################################
        self.declare_parameter('INTERFACE.user_label', 'Usuario')  
        self.user_label = self.get_parameter('INTERFACE.user_label').get_parameter_value().string_value

        self.declare_parameter('INTERFACE.robot_label', 'Sancho')  
        self.robot_label = self.get_parameter('INTERFACE.robot_label').get_parameter_value().string_value

        self.declare_parameter('INTERFACE.beginning_msg', 'Inicio de la conversación')  
        self.beginning_msg = self.get_parameter('INTERFACE.beginning_msg').get_parameter_value().string_value

        self.declare_parameter('INTERFACE.end_msg', 'Fin de la conversación')  
        self.end_msg = self.get_parameter('INTERFACE.end_msg').get_parameter_value().string_value

        ################################### TOPIC PARAMETERS ################################
        self.declare_parameter('ROSTopics.asr_output_topic', '/predicted_text')  
        self.asr_output_topic = self.get_parameter('ROSTopics.asr_output_topic').get_parameter_value().string_value

        self.declare_parameter('ROSTopics.tts_input_topic', '/input_tts')  
        self.input_tts_topic = self.get_parameter('ROSTopics.tts_input_topic').get_parameter_value().string_value

        self.declare_parameter('ROSTopics.end_conver_topic', '/end_conver')  
        self.end_conver_topic = self.get_parameter('ROSTopics.end_conver_topic').get_parameter_value().string_value

        self.declare_parameter('ROSTopics.start_conver_topic', '/start_conver')  
        self.start_conver_topic = self.get_parameter('ROSTopics.start_conver_topic').get_parameter_value().string_value  

        ################################### INITIALIZATION ################################
        self._sub_asr = self.create_subscription(String, self.asr_output_topic, self.callback_asr, 1)
        self._sub_chatbot = self.create_subscription(String, self.input_tts_topic, self.callback_chatbot, 1)
        self._sub_end_conver = self.create_subscription(Bool, self.end_conver_topic, self.callback_end_conver, 1)
        self._sub_start_conver = self.create_subscription(Bool, self.start_conver_topic, self.callback_start_conver, 1)          
        

    def callback_asr(self, msg):
        print(self.user_label+':'+msg.data)

    def callback_chatbot(self, msg):
        #text = colored('Sancho: ' + msg.data, 'red', attrs=['reverse', 'blink'])
        text = colored(self.robot_label+':'+msg.data, 'light_cyan')
        print(text)
        print('\n')

    def callback_start_conver(self, msg):
        print('\n')
        print(self.beginning_msg)
        print('\n')

    def callback_end_conver(self, msg):
        print('\n')
        print(self.end_msg)

    def main_loop(self):     
        threading.Thread(target = rclpy.spin,args = (self,), daemon=True).start()   
        while (rclpy.ok()):
            loop = True
     
    
def main(args=None):
    rclpy.init(args=args)
    x = chat_interface()
    try:
        x.main_loop()
    except ROSInterruptException: 
        pass

    rclpy.shutdown()

if __name__ == '__main__':
    main()