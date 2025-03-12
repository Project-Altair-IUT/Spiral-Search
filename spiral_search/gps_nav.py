import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from geopy.distance import geodesic, distance
from math import atan2, radians, degrees, sin, cos, pi
import time
import json

class GPSNavigator(Node):
    def __init__(self):
        super().__init__('gps_navigator')
        self.subscription = self.create_subscription(
            String,
            'gps_targets_list',
            self.target_callback,
            10
        )
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        
        self.current_gps = [0.0, 0.0]
        self.current_bearing = 69.0
        self.target_gps_list = []
        self.target_index = 0
        self.last_update_time = time.time()

    def calculate_bearing(self, current_gps, next_gps):
        lat1, lon1 = radians(current_gps[0]), radians(current_gps[1])
        lat2, lon2 = radians(next_gps[0]), radians(next_gps[1])
        delta_lon = lon2 - lon1
        
        x = atan2(
            sin(delta_lon) * cos(lat2),
            cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(delta_lon)
        )
        
        bearing = degrees(x)
        if bearing < 0:
            bearing += 360
        return bearing

    def update_current_position(self, linear_velocity, angular_velocity):
        current_time = time.time()
        dt = current_time - self.last_update_time
        self.last_update_time = current_time
        
        # Update bearing
        self.current_bearing += degrees(angular_velocity) * dt
        self.current_bearing %= 360
        
        # Update position
        distance_traveled = linear_velocity * dt
        new_lat, new_lon = distance(meters=distance_traveled).destination(
            (self.current_gps[0], self.current_gps[1]),
            self.current_bearing
        ).latitude, distance(meters=distance_traveled).destination(
            (self.current_gps[0], self.current_gps[1]),
            self.current_bearing
        ).longitude
        
        self.current_gps = [new_lat, new_lon]

    def target_callback(self, msg):
        self.get_logger().info('Received GPS target list!')
        try:
            self.target_gps_list = json.loads(msg.data)
            self.navigate_to_targets()
        except json.JSONDecodeError:
            self.get_logger().error('Failed to parse GPS target list')

    def navigate_to_targets(self):
        twist_msg = Twist()
        while self.target_index < len(self.target_gps_list):
            next_gps = self.target_gps_list[self.target_index]
            distance_to_target = geodesic(self.current_gps, next_gps).meters
            self.get_logger().info(f'Distance to target {self.target_index + 1}: {distance_to_target:.2f} meters')
            if distance_to_target < 0.1:
                self.get_logger().info(f'Reached target {self.target_index + 1}')
                self.target_index += 1
                continue
            
            target_bearing = self.calculate_bearing(self.current_gps, next_gps)
            bearing_diff = (target_bearing - self.current_bearing + 360) % 360
            
            #bearing_diff > 180 means we must turn right to self correct
            if bearing_diff > 180:
                bearing_diff -= 360
            #angular.z = +0.5 means turn left, -0.5 means turn right
            if abs(bearing_diff) > 5:
                twist_msg.angular.z = 0.5 if bearing_diff > 0 else -0.5
                twist_msg.linear.x = 0.0
            else:
                #if bearing is within threshold, move forward
                twist_msg.angular.z = 0.0
                twist_msg.linear.x = 0.1
            
            self.publisher.publish(twist_msg)
            self.update_current_position(twist_msg.linear.x, twist_msg.angular.z)
            time.sleep(0.1)
        
        self.stop_robot()

    def stop_robot(self):
        twist_msg = Twist()
        twist_msg.linear.x = 0.0
        twist_msg.angular.z = 0.0
        self.publisher.publish(twist_msg)
        self.get_logger().info('Navigation complete!')


def main(args=None):
    rclpy.init(args=args)
    gps_navigator = GPSNavigator()
    rclpy.spin(gps_navigator)
    gps_navigator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
