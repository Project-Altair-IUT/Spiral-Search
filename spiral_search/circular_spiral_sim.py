import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time
from math import degrees, pi


class SpiralDriver(Node):
    def __init__(self):
        super().__init__('spiral_driver')

        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.first_spiral = True
        self.total_spirals = 6  # Number of spirals to complete
        self.current_bearing = 5
        self.current_turns = 0
        self.angular_velocity = 0.5  # Constant angular velocity
        self.linear_velocity = 0.1   # Starting linear velocity
        self.velocity_increment = 0.5  # Velocity increase per turn

        self.last_update_time = time.time()
        self.timer = self.create_timer(0.1, self.move_in_spiral)

    def move_in_spiral(self):
        dt = 0.1
        

        # Update the current bearing based on angular velocity
        self.current_bearing += degrees(self.angular_velocity * dt)
        self.current_bearing %= 360
        self.get_logger().info(f'Bearing: {self.current_bearing} degrees')
        
        if self.first_spiral and self.current_bearing > 358:
            self.current_turns += 1
            self.get_logger().info(f'Completed {self.current_turns} turns')

            # Increase the linear velocity every turn to widen the spiral
            self.linear_velocity += self.velocity_increment
            self.first_spiral = False

        # Count turns
        elif self.current_bearing >358:
            self.current_turns += 1
            self.get_logger().info(f'Completed {self.current_turns} turns')

            # Increase the linear velocity every turn to widen the spiral
            self.linear_velocity += self.velocity_increment

        # Stop when the number of turns matches the desired spirals
        if self.current_turns >= self.total_spirals:
            self.stop_robot()
            return

        # Publish twist message
        twist_msg = Twist()
        twist_msg.linear.x = self.linear_velocity
        twist_msg.angular.z = self.angular_velocity
        self.publisher.publish(twist_msg)

    def stop_robot(self):
        twist_msg = Twist()
        twist_msg.linear.x = 0.0
        twist_msg.angular.z = 0.0
        self.publisher.publish(twist_msg)
        self.get_logger().info('Finished all spirals!')


def main(args=None):
    rclpy.init(args=args)
    spiral_driver = SpiralDriver()
    rclpy.spin(spiral_driver)
    spiral_driver.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
