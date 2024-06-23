import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from geometry_msgs.msg import Twist
import time

class FollowObject(Node):

    def __init__(self):
        super().__init__('follow_object')

        self.subscription = self.create_subscription(
            Point,
            "/detected_object",
            self.listener_callback,
            10,
        )

        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # Set up parameters
        self.declare_parameter("rcv_timeout_secs", 1.0) # Receive object location
        self.declare_parameter("angular_chase_multiplier", 0.7)
        self.declare_parameter("forward_chase_speed", 0.1)
        self.declare_parameter("search_angular_speed", 0.5)
        self.declare_parameter("max_size_thresh", 0.1)
        self.declare_parameter("filter_value", 0.9)


        self.rcv_timeout_secs = self.get_parameter('rcv_timeout_secs').get_parameter_value().double_value
        self.angular_chase_multiplier = self.get_parameter('angular_chase_multiplier').get_parameter_value().double_value
        self.forward_chase_speed = self.get_parameter('forward_chase_speed').get_parameter_value().double_value
        self.search_angular_speed = self.get_parameter('search_angular_speed').get_parameter_value().double_value
        self.max_size_thresh = self.get_parameter('max_size_thresh').get_parameter_value().double_value
        self.filter_value = self.get_parameter('filter_value').get_parameter_value().double_value


        timer_period = 0.01  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.target_val = 0.0
        self.target_dist = 0.0
        self.lastrcvtime = time.time()
    
    def timer_callback(self):
        msg = Twist()
        if (time.time() - self.lastrcvtime < self.rcv_timeout_secs):
            self.get_logger().info(f'Horizonal target: {self.target_val} | Distance target: {self.target_dist}')

            if (self.target_dist < self.max_size_thresh): # Move forward towards target
                msg.linear.x = self.forward_chase_speed
            elif (self.target_dist > self.max_size_thresh): # Reverse very slowly
                msg.linear.x = -0.1*self.forward_chase_speed

            # If within error bound, stop searching
            msg.angular.z = -self.angular_chase_multiplier*self.target_val
            if abs(self.target_val) < 0.01:
                msg.angular.z = 0.0

        else:
            self.get_logger().info('Target lost')
            msg.angular.z = self.search_angular_speed
        self.publisher.publish(msg)


    def listener_callback(self, msg):
        '''
            Based on where the object's current location, update the target location.
            Use a filter to smooth into the target value, rather than updating it fully on the first detection.
            This results in a gradual transition towards the goal state.
        '''
        f = self.filter_value
        self.target_val = self.target_val * f + msg.x * (1-f)
        self.target_dist = self.target_dist * f + msg.z * (1-f)
        self.lastrcvtime = time.time()
        # self.get_logger().info('Received: {} {}'.format(msg.x, msg.y))

def main(args=None):
    rclpy.init(args=args)
    follow_object = FollowObject()
    rclpy.spin(follow_object)
    follow_object.destroy_node()
    rclpy.shutdown()