import rclpy
import sys
from rclpy.node import Node
from rclpy.exceptions import ROSInterruptException
#sys.path.append('/home/mapir/ros2_ws/src/patrol')
import threading
import numpy as np
from std_msgs.msg import String
from geometry_msgs.msg import PoseArray, PoseStamped
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from patrol.action import PatrolTimes
from action_msgs.msg import GoalStatus

class SocialPatrol(Node):
    def __init__(self):
        super().__init__('social_patrol')

        self._pub_tts = self.create_publisher(String, "/input_text", 1)
        self._sub_asr = self.create_subscription(String, "/asr_info", self.callback_ASR, 1)      
        self._sub_openpose = self.create_subscription(PoseStamped, "/filtered_pose", self.callback_OP, 1) 
        self.patrullando = False
        self.goToUser = False
        self.stopPatrol = False
        self.var = True
        self.patrol_client = ActionClient(self, PatrolTimes, '/patrol_times')
        self.navToPose_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')

    # ASR CALLBACK
    def callback_ASR(self, msg):
        print("callback ASR")

    ##########################################################################################
    # 
    #           NAVIGATE_TO_POSE ACTION, FEEDBACK CALLBACK AND RESULT CALLBACK
    # 
    # ########################################################################################        

    def callback_OP(self, msg):
        print("callback openpose")
        self.stopPatrol= True
        self.next_pose = msg
        
    def start_goToUser(self):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = self.next_pose

        self.navToPose_client.wait_for_server()
        print("navigate_to_pose action server is ready")

        self._send_goal_user = self.navToPose_client.send_goal_async(goal_msg, feedback_callback = self.callback_feedback_goToUser)
        self._send_goal_user.add_done_callback(self.callback_response_goToUser)   # callback para recibir feedback del action server 

    def callback_feedback_goToUser(self, feedback_msg):
        print("callback feedback go to user")
        feedback = feedback_msg.feedback
        self.get_logger().info('Received feedback, distance remaining: {0}'.format(feedback.distance_remaining))

    def callback_response_goToUser(self, future):
        goal_handle_op = future.result()
        if(goal_handle_op.accepted):
            self.get_logger().info('Goal (goToUser) accepted :)')
            self._get_result_op = goal_handle_op.get_result_async()
            self._get_result_op.add_done_callback(self.callback_result_goToUser)

        else:
            self.get_logger().info('Goal (goToUser) rejected :(')
            return

    def callback_result_goToUser(self, future):
        print("¿he llegado?")

        #     print("exito go to user!")
        #     pub_msg = String()
        #     pub_msg.data = "He llegado a la persona"
        #     self._pub_tts.publish(pub_msg)

    ##########################################################################################
    # 
    #            PATROL_TIMES ACTION, START, STOP, FEEDBACK CALLBACK AND RESULT CALLBACK
    # 
    # ########################################################################################
    
    # START patrol and while patrol:
    def start_patrol(self):
        print("ey")
        self.patrullando = True
        goal_msg = PatrolTimes.Goal()
        goal_msg.times = 2
        self._send_goal_patrol = self.patrol_client.send_goal_async(goal_msg)
        self._send_goal_patrol.add_done_callback(self.callback_response_patrol)

    def callback_response_patrol(self, future):
        print("callback response patrol")
        self.goal_handle_pa = future.result()
        if(self.goal_handle_pa.accepted):
            self.get_logger().info('Goal (patrol) accepted :)')

        else:
            self.get_logger().info('Goal (patrol) rejected :(')
            return

    # STOP patrol and while canceling
    def stop_patrol(self):
        print("canceling patrol...")
        self.patrullando = False
        self.cancel_goal_patrol = self.goal_handle_pa.cancel_goal_async()
        self.cancel_goal_patrol.add_done_callback(self.goal_canceled_callback)

    def goal_canceled_callback(self, future):
        print("cancel patrol callback")
        self.goToUser = True
        # cancel_handle_pa = future.result()
        # if(cancel_handle_pa.response == 0):
        #     self.get_logger().info('Cancel (patrol) accepted')

        # else:
        #     self.get_logger().info('Cancel (patrol) rejected')
        #     return

    def main_loop(self): 
        threading.Thread(target = rclpy.spin,args = (self,), daemon=True).start()
        self.start_patrol()

        while (rclpy.ok()):
            if(self.stopPatrol):
                self.stop_patrol()
                self.stopPatrol = False
            if(self.goToUser):
                self.start_goToUser()
                self.goToUser = False

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