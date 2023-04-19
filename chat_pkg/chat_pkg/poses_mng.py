import rclpy
import sys
from rclpy.node import Node
from rclpy.exceptions import ROSInterruptException
#sys.path.append('/home/mapir/ros2_ws/src/chat_pkg')
import threading
import numpy as np
from std_msgs.msg import String, Bool
from geometry_msgs.msg import PoseArray, PoseStamped, Pose
from nav_msgs.msg import Odometry
from tf_transformations import quaternion_from_euler
import math

class SocialPatrol(Node):
    def __init__(self):
        super().__init__('social_patrol')

        self._pub_patrol = self.create_publisher(PoseStamped, "/filtered_pose", 1)   
        self._sub_openpose = self.create_subscription(PoseArray, "/poses_topic", self.callback_OP, 1)
        self._sub_compute = self.create_subscription(Bool, "/compute_pose", self.callback_compute, 1)
        self._sub_odom = self.create_subscription(Odometry, "/pose", self.callback_odom, 1) 
        self.send = False
        self.var = True
        self.robot_pose = Pose()

    def callback_odom(self,msg):
        print("callback odom")
        self.robot_pose = msg.pose.pose

    def callback_compute(self,msg):
        print("callback compute")
        self.send = True

    def callback_OP(self, msg):
        print("callback openpose")
        if(len(msg.poses) > 0 and self.send):
                self.send = False
                pose = PoseStamped()
                p_goal = self.pose_front(msg.poses[0])
                pose.pose = p_goal
                pose.header.frame_id = 'map'
                self._pub_patrol.publish(pose)

        

    def pose_front(self, p_user):
        p_goal = Pose()
        if(self.robot_pose.position.x > p_user.position.x and self.robot_pose.position.y > p_user.position.y):
            p_goal.position.x = p_user.position.x + 0.5
            p_goal.position.y = p_user.position.y + 0.5
            quat = quaternion_from_euler(0, 0,(-3/4)*math.pi)
        elif(self.robot_pose.position.x > p_user.position.x and self.robot_pose.position.y < p_user.position.y):
            p_goal.position.x = p_user.position.x + 0.5
            p_goal.position.y = p_user.position.y - 0.5
            quat = quaternion_from_euler(0, 0,(3/4)*math.pi)
        elif(self.robot_pose.position.x < p_user.position.x and self.robot_pose.position.y < p_user.position.y):
            p_goal.position.x = p_user.position.x - 0.5
            p_goal.position.y = p_user.position.y - 0.5
            quat = quaternion_from_euler(0, 0,math.pi/2)
        elif(self.robot_pose.position.x < p_user.position.x and self.robot_pose.position.y > p_user.position.y):
            p_goal.position.x = p_user.position.x - 0.5
            p_goal.position.y = p_user.position.y + 0.5
            quat = quaternion_from_euler(0, 0,-math.pi/2)

        p_goal.orientation.x = quat[0]
        p_goal.orientation.y = quat[1]
        p_goal.orientation.z = quat[2]
        p_goal.orientation.w = quat[3]

        return p_goal
        

    def main_loop(self): 
        threading.Thread(target = rclpy.spin,args = (self,), daemon=True).start()

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