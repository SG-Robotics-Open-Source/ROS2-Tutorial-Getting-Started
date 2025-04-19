import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import  math

class SimpleMover(Node):

    LINEAR_SPEED = 0.5
    ANGULAR_SPEED = 0.5
    SIDE_LENGTH = 3.0 
    TURN_ANGLE = math.pi / 2.0 #90 degrees in rad

    #duration
    move_duration_ = SIDE_LENGTH / LINEAR_SPEED
    turn_duration_ = TURN_ANGLE / ANGULAR_SPEED

    def __init__(self):
        super().__init__('simple_mover_node')
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        timer_period = .13
        self.timer = self.create_timer(timer_period, self.timer_callback)

        #state variable
        self.state_ = 0
        self.state_start_time_ = self.get_clock().now() 
        self.get_logger().info(f"Initialize square sequence. Move Duration: {self.move_duration_:.2f}s, Turn Duration: {self.turn_duration_:.2f}s") 
    
    def timer_callback(self):
        msg = Twist()
        now = self.get_clock().now()
        elapsed_time = (now - self.state_start_time_).nanoseconds / 1e9 #time in seconds

        #machine state logic
        if self.state_ == 0:
            if elapsed_time < self.move_duration_:
                msg.linear.x = self.LINEAR_SPEED
                self.get_logger().info(f"State: {self.state_}: Moving Foraward... ", throttle_duration_sec=1.0)
            else:
                self.state_ = 1
                self.state_start_time_ = now
                self.get_logger().info(f"State: {self.state_}: Turning: ")
        elif self.state_ == 1:
            if elapsed_time < self.turn_duration_:
                msg.angular.z = self.ANGULAR_SPEED
                self.get_logger().info(f"State: {self.state_}: Turning...", throttle_duration_sec=1.0)
            else:
                self.state_ = 2
                self.state_start_time_ = now
                self.get_logger().info(f"State: {self.state_}: Moving Forward:")
        elif self.state_ == 2:
            if elapsed_time < self.move_duration_:
                msg.linear.x = self.LINEAR_SPEED
                self.get_logger().info(f"State: {self.state_}: Moving Forwards...", throttle_duration_sec=1.0)
            else:
                self.state_ = 3
                self.state_start_time_ = now
                self.get_logger().info(f"State: {self.state_}: Turning:")
        elif self.state_ == 3:
            if elapsed_time < self.turn_duration_:
                msg.angular.z = self.ANGULAR_SPEED
                self.get_logger().info(f"State: {self.state_}: Turning...", throttle_duration_sec=1.0)
            else:
                self.state_ = 4
                self.state_start_time_ = now
                self.get_logger().info(f"State: {self.state_}: Moving Forward:")
        elif self.state_ == 4:
            if elapsed_time < self.move_duration_:
                msg.linear.x = self.LINEAR_SPEED
                self.get_logger().info(f"State: {self.state_}: Moving Forwards...", throttle_duration_sec=1.0)
            else:
                self.state_ = 5
                self.state_start_time_ = now
                self.get_logger().info(f"State: {self.state_}: Turning:")
        elif self.state_ == 5:
            if elapsed_time < self.turn_duration_:
                msg.angular.z = self.ANGULAR_SPEED
                self.get_logger().info(f"State: {self.state_}: Turning...", throttle_duration_sec=1.0)
            else:
                self.state_ = 6
                self.state_start_time_ = now
                self.get_logger().info(f"State: {self.state_}: Moving Forward:")
        elif self.state_ == 6:
            if elapsed_time < self.move_duration_:
                msg.linear.x = self.LINEAR_SPEED
                self.get_logger().info(f"State: {self.state_}: Moving Forward...", throttle_duration_sec=1.0)
            else:
                self.state_ = 7
                self.state_start_time_ = now
                self.get_logger().info(f"State: {self.state_}: Turning:")
        elif self.state_ == 7:
            if elapsed_time < self.turn_duration_:
                msg.angular.z = self.ANGULAR_SPEED
                self.get_logger().info(f"State: {self.state_}: Turning...", throttle_duration_sec=1.0)
            else:
                self.state_ = 8
                self.state_start_time_ = now
                self.get_logger().info(f"State: {self.state_}: Square Complete")
        elif self.state_ == 8:
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            self.get_logger().info(f"State: {self.state_}: Holding Position:", throttle_duration_sec=1.0)

        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    simple_mover = SimpleMover()

    rclpy.spin(simple_mover)

    simple_mover.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
