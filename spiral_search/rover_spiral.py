import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix
from geopy.distance import geodesic, distance
from math import atan2, radians, degrees, sin, cos, pi, sqrt
import time
import json

#ros2 run spiral_search rover_spiral --ros-args -p bearing_threshold:=5 -p distance_threshold:=0.1

class RoverNavigator(Node):
    def __init__(self):
        super().__init__('gps_navigator')
        self.declare_parameter('gps_error', 2.0)
        self.declare_parameter('distance_threshold', 1.0)

        self.gps_error = self.get_parameter('gps_error').value
        self.target_dist_threshold = self.get_parameter('distance_threshold').value
        self.current_gps = [0.0, 0.0]
        self.current_bearing = 0.0
        self.target_gps_list = []
        self.target_index = 0
        self.last_update_time = time.time()

        self.subscription = self.create_subscription(
            String,
            'gps_targets_list',
            self.target_callback,
            10
        )
        self.gps_loc_sub = self.create_subscription(
            NavSatFix,
             '/gps/fix',
             self.get_current_gps_loc,
             10
        )
        self.get_bearing_sub = self.create_subscription(
            Vector3,
            '/rotation_vector',
            self.bearing_callback,
            10
        )
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
    
    
    #update current gps loc from '/gps/fix' topic
    def get_current_gps_loc(self,msg):
        self.current_gps[0] = msg.latitude
        self.current_gps[1] = msg.longitude
    
    #update self bearing from 'rotation_vector' topic
    def bearing_callback(self,msg):
        self.current_bearing = msg.z
        self.get_logger().info(f'Updated bearing: {self.current_bearing:.2f} degrees')

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

    def calculate_bearing_error(self, distance_to_target):
        if distance_to_target == 0:
            return 0
        error = degrees(atan2(self.gps_error, distance_to_target))
        return error

    def target_callback(self, msg):
        self.get_logger().info('Received GPS target list!')
        try:
            self.target_gps_list = json.loads(msg.data)
            self.get_logger().info(f'Total {len(self.target_gps_list)} targets to visit')
            self.get_logger().info('Starting Spiral Search Pattern...')
            self.navigate_to_targets()
        except json.JSONDecodeError:
            self.get_logger().error('Failed to parse GPS target list')

    def navigate_to_targets(self):
        twist_msg = Twist()
        if self.target_index < len(self.target_gps_list):
            next_gps = self.target_gps_list[self.target_index]
            distance_to_target = geodesic(self.current_gps, next_gps).meters
            self.get_logger().info(f'Distance to target {self.target_index + 1}: {distance_to_target:.2f} meters')

            dynamic_bearing_threshold = self.calculate_bearing_error(distance_to_target)
            #self.get_logger().info(f'Dynamic bearing threshold: {dynamic_bearing_threshold:.2f} degrees')

            if distance_to_target < self.target_dist_threshold:
                self.get_logger().info(f'Reached target {self.target_index + 1}')
                self.target_index += 1
                return
            
            target_bearing = self.calculate_bearing(self.current_gps, next_gps)
            bearing_diff = (target_bearing - self.current_bearing + 360) % 360
            
            if bearing_diff > 180:
                bearing_diff -= 360
            self.get_logger().info(f'Target Bearing: {target_bearing}, Current Bearing: {self.current_bearing}')
            self.get_logger().info(f'bearing diff : {abs(bearing_diff)} bearing_threshold @ {dynamic_bearing_threshold}')
            if abs(bearing_diff) > dynamic_bearing_threshold:
                twist_msg.angular.z = 0.5
                twist_msg.linear.x = 0.0
            else:
                twist_msg.angular.z = 0.0
                twist_msg.linear.x = 0.1
            
            self.publisher.publish(twist_msg)
            time.sleep(0.1)
        
        #self.stop_robot()

    def stop_robot(self):
        twist_msg = Twist()
        twist_msg.linear.x = 0.0
        twist_msg.angular.z = 0.0
        self.publisher.publish(twist_msg)
        self.get_logger().info('Navigation complete!')


def main(args=None):
    rclpy.init(args=args)
    rover_navigator = RoverNavigator()
    rclpy.spin(rover_navigator)
    rover_navigator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()