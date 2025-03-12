import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from std_msgs.msg import String
import time
from math import radians


class SpiralMotion(Node):
    def __init__(self):
        super().__init__('spiral_motion_node')
        
        # ROS 2 Publisher
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # ROS 2 Subscriber to get bearing
        self.bearing_sub = self.create_subscription(
            Vector3,
            '/rotation_vector',
            self.bearing_callback,
            10
        )
        
        # Initialize parameters
        self.current_bearing = 0.0
        self.last_bearing = 0.0
        self.turns_completed = 0
        self.total_turns = 3  # Number of full spirals to complete
        
        self.angular_speed = 0.5
        self.linear_speed = 0.05
        self.radius_increment = 0.05
        self.target_turns = self.total_turns * 360

    def bearing_callback(self, msg):
        self.current_bearing = msg.z

    def move_in_spiral(self):
        twist_msg = Twist()
        
        self.get_logger().info(f'Starting spiral motion for {self.total_turns} turns')
        
        while self.turns_completed < self.target_turns:
            twist_msg.linear.x = self.linear_speed
            twist_msg.angular.z = self.angular_speed
            
            self.publisher.publish(twist_msg)
            
            # Track turns based on bearing
            if abs(self.current_bearing - self.last_bearing) >= 360:
                self.turns_completed += 360
                self.linear_speed += self.radius_increment
                self.get_logger().info(f'Turns completed: {self.turns_completed / 360}')
                
            self.last_bearing = self.current_bearing
            time.sleep(0.1)
        
        self.stop_robot()

    def stop_robot(self):
        twist_msg = Twist()
        twist_msg.linear.x = 0.0
        twist_msg.angular.z = 0.0
        self.publisher.publish(twist_msg)
        self.get_logger().info('Spiral motion complete!')


def main(args=None):
    rclpy.init(args=args)
    spiral_motion = SpiralMotion()
    spiral_motion.move_in_spiral()
    spiral_motion.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

# Let me know if you’d like me to tweak anything — happy to refine this! 🚀
