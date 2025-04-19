#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
import math
import random 

class TurtleTom(Node):

    def __init__(self):
        super().__init__('turtle_tom_node')
        self.get_logger().info(f"Turtle Tom node has started")

        #TOM
        self.tom_pose_ = None
        self.MAX_LINEAR_VEL_TOM = 5.0
        self.MAX_ANGULAR_VEL_TOM = 5.0

        #JERRY
        self.jerry_pose_ = None
        self.JERRY_LINEAR_FACTOR = 1.1
        self.JERRY_ANGULAR_FACTOR = 1.3

        #P constants
        self.KP_LINEAR = 1.1
        self.KP_ANGULAR = 1.7

        #boundary for jerry
        self.BOUNDARY_BUFFER = 1.5
        self.X_MIN = self.BOUNDARY_BUFFER
        self.X_MAX = 11.0 - self.BOUNDARY_BUFFER
        self.Y_MIN = self.BOUNDARY_BUFFER
        self.Y_MAX = 11.0 - self.BOUNDARY_BUFFER

        #shortest possible distance betweent tom and jerry
        self.DISTANCE_THRESHOLD = 0.8

        #create subscriber
        self.tom_pose_subscriber_ = self.create_subscription(Pose, '/tom/pose', self.tom_pose_callback, 10)
        self.jerry_pose_subscriber_ = self.create_subscription(Pose, '/jerry/pose', self.jerry_pose_callback, 10)

        #create a publisher 
        self.tom_cmd_vel_publisher_ = self.create_publisher(Twist, '/tom/cmd_vel', 10)
        self.jerry_cmd_vel_publisher_ = self.create_publisher(Twist, '/jerry/cmd_vel', 10)

        #control loop
        self.control_loop_timer_ = self.create_timer(0.05, self.control_loop_callback)

    def tom_pose_callback(self, msg):
        self.tom_pose_ = msg
        self.get_logger().info(f"Tom pose: x: {msg.x:.2f}, y: {msg.y:.2f}", throttle_duration_sec=1.0)

    def jerry_pose_callback(self, msg):
        self.jerry_pose_ = msg
        self.get_logger().info(f"Jerry pose: x: {msg.x:.2f}, y: {msg.y:.2f}", throttle_duration_sec=1.0)

    def control_loop_callback(self):
        if self.tom_pose_ is None or self.jerry_pose_ is None:
            #waiting for pose data
            self.get_logger().info(f"Waiting for pose data from tom and jerry", throttle_duration_sec=1.0)
            return
        
        #Initialize msgs
        tom_msg = Twist()
        jerry_msg = Twist()

        #distance between tom and jerry
        dx = self.jerry_pose_.x - self.tom_pose_.x
        dy = self.jerry_pose_.y - self.tom_pose_.y
        distance = math.sqrt(dx**2 + dy**2)

        #if tom is close to jerry
        if distance < self.DISTANCE_THRESHOLD:
            self.get_logger().info(f"Tom is close, JERRY RUN!")

            new_x = random.uniform(self.X_MIN, self.X_MAX)
            new_y = random.uniform(self.Y_MIN, self.Y_MAX)

            dx = new_x - self.jerry_pose_.x
            dy = new_y - self.jerry_pose_.y
            distance = math.sqrt(dx**2 + dy**2)

            desired_angle = math.atan2(dy, dx)
            angle_error = desired_angle - self.jerry_pose_.theta

            #make the angle the shortest possible
            while angle_error > math.pi:
                angle_error -= 2.0 * math.pi
            while angle_error < -math.pi:
                angle_error += 2.0 * math.pi

            #P-control for jerry
            linear_vel = self.KP_LINEAR * self.JERRY_LINEAR_FACTOR * distance
            angular_vel = self.KP_ANGULAR * self.JERRY_ANGULAR_FACTOR * angle_error

            #publish to jerry
            jerry_msg.linear.x = linear_vel
            jerry_msg.angular.z = angular_vel
            self.jerry_cmd_vel_publisher_.publish(jerry_msg)

            self.get_logger().info(f"__JERRY__: Dist: {distance:.2f}, AngErr: {angle_error:.2f}, LinVel: {linear_vel:.2f}, AngVel: {angular_vel:.2f}", throttle_duration_sec=1.0)

        else:
            desired_angle = math.atan2(dy, dx)
            angle_error = desired_angle - self.tom_pose_.theta

            #make the angle the shortest possible
            while angle_error > math.pi:
                angle_error -= 2.0 * math.pi
            while angle_error < -math.pi:
                angle_error += 2.0 * math.pi

            #P-control
            linear_vel = self.KP_LINEAR * distance
            angular_vel = self.KP_ANGULAR * angle_error

            #speed limit
            linear_vel = max(0.0, min(linear_vel, self.MAX_LINEAR_VEL_TOM))
            #clamp angle velocity
            angular_vel = max(-self.MAX_ANGULAR_VEL_TOM, min(angular_vel, self.MAX_ANGULAR_VEL_TOM))

            #publish to tom
            tom_msg.linear.x = linear_vel
            tom_msg.angular.z = angular_vel
            self.tom_cmd_vel_publisher_.publish(tom_msg)

            self.get_logger().info(f"__TOM__: Dist: {distance:.2f}, AngErr: {angle_error:.2f}, LinVel: {linear_vel:.2f}, AngVel: {angular_vel:.2f}", throttle_duration_sec=1.0)

def main(args=None):
    rclpy.init(args=args)
    tom_node = TurtleTom()
    rclpy.spin(tom_node)
    #cleanup
    tom_node.destroy_node()
    rclpy.shutdown()

if  __name__=='__main__':
    main()