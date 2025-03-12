#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class MovementCycle(Node):
    def __init__(self):
        super().__init__('movement_cycle')
        
        # Create the publisher for cmd_vel     msg_type, topic, quality of service
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Create a Timer to control the cyclic behavior
        self.timer = self.create_timer(0.1, self.timer_callback)  # Repeat every 0.1 seconds
        
        # Initialize the Twist message
        self.cmd_msg = Twist()
        
        # State tracking: whether the robot is moving forward or rotating
        self.state = 'move_forward'
        self.get_logger().info('Movement Cycle Node has started!')
        
        # Time to move 1 meter based on linear velocity
        self.linear_velocity = 0.1  # m/s (adjust as needed)
        self.angular_velocity = -0.5  # rad/s (rotate 90 degrees in about 2 seconds)
        
        # Target distance to move (1 meter) and angle for rotation (90 degrees)
        self.target_distance = 1.0  # meters
        self.target_angle = 1.5708  # radians (90 degrees)
        
        # Tracking the distance moved and rotation angle
        self.distance_moved = 0.0
        self.angle_rotated = 0.0

    def timer_callback(self):
        if self.state == 'move_forward':
            # Move forward
            self.cmd_msg.linear.x = self.linear_velocity  # Move forward at 0.1 m/s
            self.cmd_msg.angular.z = 0.0  # No rotation
            self.publisher.publish(self.cmd_msg)
            
            # Track the distance moved (in meters)
            self.distance_moved += self.linear_velocity * 0.1  # 0.1s per timer cycle
            
            if self.distance_moved >= self.target_distance:
                # Stop after moving 1 meter
                self.cmd_msg.linear.x = 0.0  # Stop moving forward
                self.cmd_msg.angular.z = 0.0  # No rotation
                self.publisher.publish(self.cmd_msg)
                
                self.get_logger().info(f'Moved forward {self.target_distance} meters.')
                # Change state to rotate
                self.state = 'rotate_left'
                self.distance_moved = 0.0  # Reset distance for the next side of the square

        elif self.state == 'rotate_left':
            # Rotate 90 degrees left
            self.cmd_msg.linear.x = 0.0  # No forward movement
            self.cmd_msg.angular.z = self.angular_velocity  # Rotate at 0.5 rad/s
            self.publisher.publish(self.cmd_msg)
            
            # Track the angle rotated
            self.angle_rotated += 0.5 * 0.1  # 0.1s per timer cycle
            
            if self.angle_rotated >= self.target_angle:
                # Stop after rotating 90 degrees
                self.cmd_msg.linear.x = 0.0  # No forward movement
                self.cmd_msg.angular.z = 0.0  # Stop rotation
                self.publisher.publish(self.cmd_msg)
                
                self.get_logger().info('Rotated 90 degrees left.')
                # Change state back to moving forward for the next side of the square
                self.state = 'move_forward'
                self.angle_rotated = 0.0  # Reset angle for the next rotation

def main(args=None):
    rclpy.init(args=args)
    movement_cycle = MovementCycle()
    rclpy.spin(movement_cycle)
    movement_cycle.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
