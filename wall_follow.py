#Import rclpy node, sensor, and action
import sys
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.action import ActionClient

#Import irobot_Create libraries  
from irobot_create_msgs.msg import HazardDetectionVector
from irobot_create_msgs.action import WallFollow
from builtin_interfaces.msg import Duration


#Write class for the action to be performed
class WallFollowActionClient(Node):
    def __init__(self):
        super().__init__('wall_follow_action_client')
        self.subscription = self.create_subscription(HazardDetectionVector, '/TonyStark/hazard_detection', self.listener_callback, qos_profile_sensor_data)
        self._action_client = ActionClient(self, WallFollow, '/TonyStark/wall_follow')

#Define what the sensor will do - detect if any of the lateral bumpers are hit         
    def listener_callback(self, msg):
        for detection in msg.detections:
            det = detection.header.frame_id
            if det != "base_link":
               print(det)
               if det == "bump_right":
                   self.send_goal(follow_side=-1)
               elif det == "bump_left":
                   self.send_goal(follow_side=1)
               elif det == "bump_front_right":
                   self.send_goal(follow_side=-1)
               elif det == "bump_front_left":
                   self.send_goal(follow_side=1)
               elif det == "bump_front_center":
                   pass
     
 
 #If the lateral bumpers are hit, the robot will perform the desired action for a maximum of 5 seconds               
    def send_goal(self, follow_side=1, max_runtime=10):
        print('Ready for Action')
         
        goal_msg = WallFollow.Goal()
        goal_msg.follow_side = follow_side
            
        goal_msg.max_runtime = Duration(sec=10, nanosec=0)
         
        self._action_client.wait_for_server()
        return self._action_client.send_goal_async(goal_msg)
         
 
def main(args=None):
    rclpy.init(args=args)
     
    action_client = WallFollowActionClient()
    try:
       rclpy.spin(action_client)
    except KeyboardInterrupt:
       print('\nAction concluded')
    finally:
       print('Done')
       rclpy.shutdown()
        
        
if __name__ == '__main__':
     main()
