#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import NavSatFix
from geopy.distance import distance

class GpsUpdater(Node):
    def __init__(self):
        super().__init__('gps_updater')
        
        # Initialize the subscriber to cmd_vel and publisher for updated GPS
        self.subscription = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.publisher = self.create_publisher(NavSatFix, '/updated_gps', 10)
        
        # Initialize variables
        self.current_gps = (0.0, 0.0)
        self.bearing = 0.0
        self.last_time = self.get_clock().now()
        
        self.get_logger().info('GPS Updater Node has started!')

    def update_gps(self, linear_velocity):
        # Calculate the new GPS location based on velocity and bearing
        new_coords = distance(kilometers=linear_velocity / 1000).destination(self.current_gps, bearing=self.bearing)
        #update current gps position
        self.current_gps = (new_coords.latitude,new_coords.longitude)
        # Publish updated current GPS coordinates
        gps_msg = NavSatFix()
        gps_msg.latitude = new_coords.latitude
        gps_msg.longitude = new_coords.longitude
        self.publisher.publish(gps_msg)
        
        self.get_logger().info(f'Updated GPS: ({gps_msg.latitude}, {gps_msg.longitude})')

    def cmd_vel_callback(self, msg):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9  # Time in seconds
        
        # Extract linear velocity and calculate distance traveled
        linear_velocity = msg.linear.x * dt
        
        # Update the bearing with angular velocity
        self.bearing += msg.angular.z * dt
        self.bearing %= 360
        
        # Update GPS if there's movement
        if linear_velocity > 0:
            self.update_gps(linear_velocity)
        
        self.last_time = current_time


def main(args=None):
    rclpy.init(args=args)
    gps_updater = GpsUpdater()
    rclpy.spin(gps_updater)
    gps_updater.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
