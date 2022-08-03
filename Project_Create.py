import sys
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.action import ActionClient

from irobot_create_msgs.msg import HazardDetectionVecotr
from irobot_create_msgs.action import WallFollow
from bultin_interfaces.msg import Duration

from irobot_create_msgs.msg import IrIntensityVector
from irobot_create_msgs.msg import IrIntensity
from irobot_create_msgs.msg import WheelVels
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3

global max_speed = 0.15
global threshold = 1000

class WallFollowActionClient(Node):
    def __init__(self):
        super().__init__('wall_follow_action_client')
        self.subscription = self.create_subscription(HazardDetectionVector, '/Viserion/hazard_detection', self.listener_callback, qos_profile_sensor_data)
        self._action_client = ActionClient(self, WallFollow, '/Viserion/wall_follow')
        
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
               
     def send_goal(self, follow_side=1, max_runtime=20):
        print('Ready to Disinfect')
         
        goal_msg = WallFollow.Goal()
        goal_msg.follow_side = follow_side
            
        goal_msg.max_runtime = Duration(sec=20, nanosec=0)
         
        self._action_client.wait_for_server()
        return self._action_client.send_goal_async(goal_msg)
         

class Obstacles(Node):
    def __init__(self):
        super().__init__('obstacles')
        self.subscription = self.create_subscription(IrIntensityVector, '/Viserion/ir_intensity', self.listener_callback, qos_profile_sensor_data)
        self.ir = IrIntensity()

        self.wheels_publisher = self.create_publisher(Twist, '/Viserion/cmd_vel', 10)
        self.wheels = Twist()
        self.linear = Vector3()
        self.angular = Vector3()

    def listener_callback(self, msg:IrIntensityVector):
        self.SPEED(msg)

    def SPEED(self, msg):
        val = []
        message = msg.readings
        for i in range(len(message)):
            val.append(message[i].value)

        if 0 < max(val) < threshold:
            speed = max_speed/max(val)
        elif max(val) >= threshold:
            speed = 0
        else:
            speed = max_speed

        self.linear.x = float(speed)
        self.linear.y = float(speed)
        self.linear.z = float(speed)
        
        self.angular.x = float(0.0)
        self.angular.y = float(0.0)
        self.angular.z = float(0.0)
        
        self.wheels.linear = self.linear
        self.wheels.angular = self.angular
        self.wheels_publisher.publish(self.wheels)


def main(args=None):
    rclpy.init(args=args)

    command = input('What do you want the robot to do? ')

    if command == 'follow':
        action_client = WallFollowActionClient()
        try:
            rclpy.spin(action_client)
        except KeyboardInterrupt:
            print('\nFinish Disinfection')
        finally:
            print('Done')
            rclpy.shutdown()
    elif command == 'drive':
        obstacles = Obstacles()
        try:
            rclpy.spin(obstacles)
        except KeyboardInterrupt:
            print('\nFinish Disinfection')
        finally:
            print('Done')
            rclpy.shutdown()
    elif command == 'control':
        print('You want to teleop! Run this command:')
        print('ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r __ns:=/namespace')
        

if __name__ == '__main__':
     main()
