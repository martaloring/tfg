import rclpy
import sys
from rclpy.node import Node
from rclpy.exceptions import ROSInterruptException
#sys.path.append('/home/mapir/ros2_ws/src/chat_pkg')
import threading
import numpy as np
from std_msgs.msg import String
from geometry_msgs.msg import PoseArray, PoseStamped
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose

class SocialPatrol(Node):
    def __init__(self):
        super().__init__('social_patrol')

        self._pub_patrol = self.create_publisher(PoseStamped, "/filtered_pose", 1)   
        self._sub_openpose = self.create_subscription(PoseArray, "/poses_topic", self.callback_OP, 1) 
        self.var = False

    def callback_OP(self, msg):  # esto nodo calcula cual es la pose de "delante de la persona" a partir de la pose de la persona
        print("callback openpose")
        self.var = True
        #if()
        pose = PoseStamped()
        pose.pose = msg.poses[0]
        pose.header.frame_id = 'map'
        

    def main_loop(self): 
        threading.Thread(target = rclpy.spin,args = (self,), daemon=True).start()
        # start patrol action (la va a hacer javi)
        while (rclpy.ok()):
            if(self.var):
                self.var = True

def main(args=None):
    rclpy.init(args=args)
    try:
        x = SocialPatrol()
        x.main_loop()
    except ROSInterruptException: 
        pass

    rclpy.shutdown()

if __name__ == '__main__':
    main()