import rclpy
import sys
from rclpy.node import Node
from rclpy.exceptions import ROSInterruptException
#sys.path.append('/home/mapir/ros2_ws/src/patrol')
import threading
import numpy as np
from std_msgs.msg import String, Bool
from geometry_msgs.msg import PoseArray, PoseStamped
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from patrol.action import PatrolTimes
from action_msgs.msg import GoalStatus
import rclpy.clock
import rclpy.duration

class SocialPatrol(Node):
    def __init__(self):
        super().__init__('social_patrol')

        self._pub_tts = self.create_publisher(String, "/input_text", 1)
        self._pub_compute = self.create_publisher(Bool, "/compute_pose", 1)
        self._sub_asr = self.create_subscription(String, "/asr_info", self.callback_ASR, 1)      
        self._sub_openpose = self.create_subscription(PoseStamped, "/filtered_pose", self.callback_OP, 1) 
        self.patrullando = False
        self.goToUser = False
        self.stopPatrol = False
        self.startPatrol = True
        self.var = True
        self.patrol_client = ActionClient(self, PatrolTimes, '/patrol_times')
        self.navToPose_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.past = self.get_clock().now()
        self.compute_pose = False

    # ASR CALLBACK
    def callback_ASR(self, msg):        # aqui mando un mensaje si ya ha terminado la conversacion y quiero volver a empezar el patrol 
        print("callback ASR")

    def callback_OP(self, msg):
        print("callback openpose")
        self.stopPatrol= True
        self.next_pose = msg

    #####################################################################################################
    # 
    #       NAVIGATE_TO_POSE ACTION //// SEND GOAL & RESPONSE GOAL & FEEDBACK CALLBACK & RESULT GOAL 
    # 
    # ###################################################################################################
        
    def start_goToUser(self):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = self.next_pose

        self.navToPose_client.wait_for_server()
        print("navigate_to_pose action server is ready")

        self._send_goal_user = self.navToPose_client.send_goal_async(goal_msg, feedback_callback = self.callback_feedback_goToUser)
        self._send_goal_user.add_done_callback(self.callback_response_goToUser)   # callback para saber si el goal se ha aceptado o no

    def callback_response_goToUser(self, future):
        goal_handle_op = future.result()
        if(goal_handle_op.accepted):
            self.get_logger().info('Goal (goToUser) accepted :)')
            self._get_result_op = goal_handle_op.get_result_async()
            self._get_result_op.add_done_callback(self.callback_result_goToUser)

        else:
            self.get_logger().info('Goal (goToUser) rejected :(')
            return

    def callback_feedback_goToUser(self, feedback_msg):
        print("callback feedback go to user")
        feedback = feedback_msg.feedback
        self.get_logger().info('Received feedback, distance remaining: {0}'.format(feedback.distance_remaining))

    def callback_result_goToUser(self, future):
        print("¿he llegado?")

        # si encuentro alguna variable que se de cuenta de que no he llegado, puedo volver a hacer (self.goTouser = True)

        print("exito go to user!")
        pub_msg = String()
        pub_msg.data = "Hola, soy Sancho el robot de servicio. Estoy disponible para lo que necesites. Si quieres hacerme alguna pregunta di: Hola, Sancho."
        self._pub_tts.publish(pub_msg)

    ##########################################################################################
    # 
    #  PATROL_TIMES ACTION //// SEND GOAL & RESULT GOAL //// CANCEL GOAL & RESULT CANCELLING
    # 
    # ########################################################################################
    
    # START patrol and while patrol:
    def start_patrol(self):
        print("start patrol")
        self.patrullando = True
        self.compute_pose = False
        goal_msg = PatrolTimes.Goal()
        goal_msg.times = 2
        self._send_goal_patrol = self.patrol_client.send_goal_async(goal_msg)
        self._send_goal_patrol.add_done_callback(self.callback_response_patrol)

    def callback_response_patrol(self, future):
        print("callback response patrol")
        self.goal_handle_pa = future.result()
        if(self.goal_handle_pa.accepted):
            self.get_logger().info('Goal (patrol) accepted :)')
            self.past = self.get_clock().now()

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
        sec = rclpy.duration.Duration(seconds=2) 
        rclpy.clock.Clock().sleep_for(sec)
        self.goToUser = True

    #####################################################################################################

    def main_loop(self): 
        threading.Thread(target = rclpy.spin,args = (self,), daemon=True).start()

        while (rclpy.ok()):
            if(self.startPatrol):
                self.start_patrol()
                self.startPatrol = False
            if(self.patrullando and not self.compute_pose):
                now = self.get_clock().now()
                if((now - self.past).nanoseconds*1e-9) > 8:
                    self.compute_pose = True
                    msg = Bool()
                    msg.data = True
                    self._pub_compute.publish(msg)
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