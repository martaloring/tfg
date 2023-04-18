import rclpy
import sys
from rclpy.node import Node
from rclpy.exceptions import ROSInterruptException
#sys.path.append('/home/mapir/ros2_ws/src/chat_pkg')
import threading
import numpy as np
from std_msgs.msg import String, Bool
from geometry_msgs.msg import PoseArray, PoseStamped, Pose
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose

class SocialPatrol(Node):
    def __init__(self):
        super().__init__('social_patrol')

        self._pub_patrol = self.create_publisher(PoseStamped, "/filtered_pose", 1)   
        self._sub_openpose = self.create_subscription(PoseArray, "/poses_topic", self.callback_OP, 1)
        self._sub_compute = self.create_publisher(Bool, "/compute_pose", self.callback_compute, 1) 
        self.send = False

    def callback_compute(self,msg):
        print("callback compute")
        self.send = True

    def callback_OP(self, msg):  # esto nodo calcula cual es la pose de "delante de la persona" a partir de la pose de la persona
        print("callback openpose")
        if(len(msg.poses) > 0 and self.send):
            self.send = False
            pose = PoseStamped()
            p_goal = self.pose_front(msg.poses[0])
            pose.pose = p_goal
            pose.header.frame_id = 'map'

    def pose_front(self, p_user):
        p_goal = Pose()
        return p_goal
        

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