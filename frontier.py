import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
import random
import math
from itertools import chain

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
        self.map_subscriber = self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
        self.frontier_search_radius = 100  # Radius to search for frontiers around the robot
        self.turning = False
        self.target = None

    def laser_callback(self, msg):
        # Check for frontal collision based on the laser scan data
        frontal_angle_range = range(0, 90)  # Adjust the range based on your robot's configuration
        # frontal_angle_range += range(180, 360)  # Adjust the range based on your robot's configuration

        for i in frontal_angle_range:
        # for i in chain(range(0,90), range(180,360)):
            if msg.ranges[i] < 0.4 and not self.turning:
                self.stop_robot()
                self.get_logger().info('Collision detected!')
                self.move_linear = False
                return

    def map_callback(self, msg):
        # Process the occupancy grid map to find frontiers
        # For simplicity, assume a 2D grid where 0 represents unexplored and 100 represents occupied
        frontiers = self.find_frontiers(msg)
        
        if frontiers and not self.target:
            # Choose a random frontier to navigate towards
            random_frontier = random.choice(frontiers)
                
            self.target = random_frontier
        
            # Calculate the angle to turn towards the target position
            angle_to_target = math.atan2(random_frontier[1], random_frontier[0])

            # Set the velocities to turn towards the target position
            linear_velocity = 0.0
            angular_velocity = 1.0
            self.random_motion_duration = angle_to_target
            
            self.turning = True

            self.publish_velocity(linear_velocity, angular_velocity)

            self.get_logger().info(f'Turning towards frontier: {random_frontier}')
    
        elif not self.target:
            # No frontiers found, continue random exploration
            self.generate_random_velocity_command()

    def find_frontiers(self, map_msg):
        # Identify frontiers in the map
        # For simplicity, assume a 2D grid where 0 represents unexplored and 100 represents occupied
        frontiers = []
        for i in range(map_msg.info.width):
            for j in range(map_msg.info.height):
                if map_msg.data[i + j * map_msg.info.width] == 0 and self.is_frontier(map_msg, i, j):
                    # Convert grid coordinates to world coordinates
                    x = map_msg.info.origin.position.x + (i + 0.5) * map_msg.info.resolution
                    y = map_msg.info.origin.position.y + (j + 0.5) * map_msg.info.resolution
                    frontiers.append((x, y))
        return frontiers

    def is_frontier(self, map_msg, x, y):
        # Check if a cell is a frontier cell
        for dx in range(-1, 2):
            for dy in range(-1, 2):
                if 0 <= x + dx < map_msg.info.width and 0 <= y + dy < map_msg.info.height:
                    if map_msg.data[(x + dx) + (y + dy) * map_msg.info.width] == -1:
                        return True
        return False

    def timer_callback(self):
        if not self.is_robot_moving:
            if not self.target:
                # Robot is not moving, generate new velocities and duration
                self.generate_random_velocity_command()
                self.random_motion_duration = random.uniform(2.0, 5.0)  # Adjust the range as needed
        
            else:
                distance_to_target = math.sqrt(self.target[0]**2 + self.target[1]**2)

                # Set the velocities to turn towards the target position
                linear_velocity = 0.5
                angular_velocity = 0.0
                self.random_motion_duration = distance_to_target*2
                
                self.turning = False

                self.publish_velocity(linear_velocity, angular_velocity)

                self.get_logger().info(f'Heading towards frontier: {self.target}')

                self.target = None

        else:
            # Robot is moving, decrease duration and stop when it reaches zero or collision is detected
            self.random_motion_duration -= 1.0
            if self.random_motion_duration <= 0:
                self.stop_robot()

    def generate_random_velocity_command(self):
        self.get_logger().info(f'Random Exploration')
        # Generate random linear and angular velocities alternatively
        if self.move_linear:
            # Linear motion
            linear_velocity = random.uniform(0.1, 0.3)  # Adjust the range as needed
            angular_velocity = 0.0
            self.move_linear = False
        else:
            # Angular motion
            linear_velocity = 0.0
            angular_velocity = random.uniform(-0.5, 0.5)
            self.turning = True
            self.move_linear = True

        self.publish_velocity(linear_velocity, angular_velocity)

    def publish_velocity(self, linear_velocity, angular_velocity):
        # Set the velocities in the Twist message
        self.current_velocity_cmd.linear.x = linear_velocity
        self.current_velocity_cmd.angular.z = angular_velocity

        self.is_robot_moving = True

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
