import rclpy
from rclpy.node import Node
from rclpy.exceptions import ROSInterruptException
from std_msgs.msg import String
import threading

class chat_interface(Node):
    def __init__(self):
        super().__init__('interface_node')  
        self._sub_asr = self.create_subscription(String, "/predicted_text", self.callback_asr, 1)
        self._sub_chatbot = self.create_subscription(String, "/input_tts", self.callback_chatbot, 1)          
        

    def callback_asr(self, msg):
        print('Usuario:' + msg.data)

    def callback_chatbot(self, msg):
        print('Sancho: ' + msg.data)
        print('\n')

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