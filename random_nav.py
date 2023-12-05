import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import random

class VelocityCommandPublisher(Node):
    def __init__(self):
        super().__init__('velocity_command_publisher')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.current_velocity_cmd = Twist()
        self.is_robot_moving = False
        self.random_motion_duration = 0
        self.move_linear = False
        self.prev_min_distance = float('inf')  # Initialize to positive infinity
        self.prev_robot_yaw = 0.0
        self.laser_subscriber = self.create_subscription(LaserScan, '/scan', self.laser_callback, 10)
        self.turning = False

    def laser_callback(self, msg):
        # Check for frontal collision based on the laser scan data
        frontal_angle_range = range(270, 360)  # Adjust the range based on your robot's configuration

        for i in frontal_angle_range:
            if msg.ranges[i] < 0.4 and not self.turning:  # Adjust the threshold distance as needed
                self.stop_robot()
                self.get_logger().info('Collision detected!')
                return

    def timer_callback(self):
        if not self.is_robot_moving:
            # Robot is not moving, generate new velocities and duration
            self.generate_random_velocity_command()
            self.is_robot_moving = True
            self.random_motion_duration = random.uniform(2.0, 5.0)  # Adjust the range as needed
        else:
            # Robot is moving, decrease duration and stop when it reaches zero or collision is detected
            self.random_motion_duration -= 1.0
            if self.random_motion_duration <= 0:
                self.stop_robot()

    def generate_random_velocity_command(self):
        # Generate random linear and angular velocities alternatively
        if self.move_linear:
            # Linear motion
            linear_velocity = random.uniform(0.1, 0.5)  # Adjust the range as needed
            angular_velocity = 0.0
            self.move_linear = False
        else:
            # Angular motion
            linear_velocity = 0.0
            angular_velocity = random.uniform(-0.5, 0.5)
            self.turning = True
            self.move_linear = True

        # Set the velocities in the Twist message
        self.current_velocity_cmd.linear.x = linear_velocity
        self.current_velocity_cmd.angular.z = angular_velocity

        # Publish the velocity command
        self.publisher_.publish(self.current_velocity_cmd)

        self.get_logger().info(f'Published new velocity command: {self.current_velocity_cmd}')

    def stop_robot(self):
        # Stop the robot by publishing zero velocities
        zero_velocity_cmd = Twist()
        self.publisher_.publish(zero_velocity_cmd)
        self.is_robot_moving = False
        self.turning = False
        self.get_logger().info('Robot stopped')

def main(args=None):
    rclpy.init(args=args)

    velocity_command_publisher = VelocityCommandPublisher()

    try:
        rclpy.spin(velocity_command_publisher)
    except KeyboardInterrupt:
        pass

    velocity_command_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

