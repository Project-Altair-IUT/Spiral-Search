#!/usr/bin/env python 3
import math
import time
import rclpy
import json

from queue import Queue
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix
from geopy.geocoders import Nominatim
from geopy.distance import distance
from geopy.distance import geodesic

# subscribe to /updated_gps topic
# subscribe to /gps_targets_list topic
# publish to /cmd_vel topic

#Take `next_gps` targe from gps_targets_list, place into a queue
#Take `current_gps` from /updated_gps
# When distance between cur_gps & next_gps is within a threshold, stop,rotate_right and pop queue

# Create a queue to store the target GPS locations
targets_queue = Queue()


proximity_threshold = 0.1 #meters
first_target = True

bearing = [0.0,90.0,180.0,270.0]


# Function: Given two gps locations, returns the displacement between them
def distanceFromGps(initial_gps,current_gps):
    distance_between = geodesic(initial_gps,current_gps).km * 1000 #convert to meters
    return distance_between

def calculate_bearing(current_gps, next_gps):
    """
    Calculate the bearing from the current GPS location to the next GPS location.
    """
    lat1, lon1 = current_gps
    lat2, lon2 = next_gps

    dlon = lon2 - lon1

    x = math.sin(math.radians(dlon)) * math.cos(math.radians(lat2))
    y = math.cos(math.radians(lat1)) * math.sin(math.radians(lat2)) - (math.sin(math.radians(lat1)) * math.cos(math.radians(lat2)) * math.cos(math.radians(dlon)))

    initial_bearing = math.atan2(x, y)

    # Convert bearing from radians to degrees
    initial_bearing = math.degrees(initial_bearing)
    compass_bearing = (initial_bearing + 360) % 360

    return compass_bearing

def stop(self):
    twist_msg = Twist()
    twist_msg.linear.x = 0.0  # Stop forward movement
    twist_msg.angular.z = 0.0  # Stop rotation
    self.publisher.publish(twist_msg)
    self.get_logger().info('Rover stopped')

def rotate_right_90(self):
    self.get_logger().info('Rotating right 90 degrees...')
    
    twist_msg = Twist()
    twist_msg.angular.z = -0.5  # Rotate clockwise at 0.5 rad/s
    twist_msg.linear.x = 0.0  # No forward movement
    
   
    self.publisher.publish(twist_msg)
       

   

def rotate_to_bearing(self, target_bearing):
    """
    Rotates the rover to face the target bearing.
    """
    current_bearing = self.current_bearing # Assume this function exists and returns the current bearing
    angle_to_rotate = (target_bearing - current_bearing + 360) % 360

    if angle_to_rotate > 180:
        angle_to_rotate -= 360  # Rotate counter-clockwise if the angle is greater than 180 degrees

    self.get_logger().info(f'Rotating to bearing {target_bearing} degrees...')

    twist_msg = Twist()
    if angle_to_rotate > 0:
        twist_msg.angular.z = 0.5  # Rotate counter-clockwise at 0.5 rad/s
    else:
        twist_msg.angular.z = -0.5  # Rotate clockwise at 0.5 rad/s

    twist_msg.linear.x = 0.0  # No forward movement
    self.publisher.publish(twist_msg)
    time.sleep(abs(angle_to_rotate) * math.pi / 180)  # Rotate for the calculated time

    self.get_logger().info(f'Rotated to bearing {target_bearing} degrees.')
    self.curent_bearing = target_bearing
    stop(self)


def move_forward(self):
    """Moves the robot forward at a fixed linear velocity."""
    twist_msg = Twist()
    twist_msg.linear.x = 0.1  # Move forward at 1 m/s
    twist_msg.angular.z = 0.0  # No rotation
    self.publisher.publish(twist_msg)
    self.get_logger().info('Moving forward...')



class SpiralMotionPlanner(Node):
    def __init__(self):
        super().__init__('spiral_motion_planner')
        self.enqued = False
        self.first_target = True
        self.current_gps = (0.0,0.0)
        self.current_bearing = 0.0
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.target_gps_sub = self.create_subscription(String, '/gps_targets_list', self.target_gps_callback, 10)
        self.rover_loc_sub = self.create_subscription(NavSatFix,'/updated_gps', self.current_gps_callback, 10)
        self.prog_ctrl_sub = None
        targets_queue.put(self.current_gps)
        self.timer = self.create_timer(2.0, self.check_for_topic)
        self.timer = self.create_timer(0.1, self.move_rover)
        self.get_logger().info('Spiral Motion Simulator has started')
    
    #Enques the target GPS locations into the queue
    def target_gps_callback(self, msg):
    
      try:
        if(self.enqued == False):
           targets_list = json.loads(msg.data)
           for target in targets_list:
               targets_queue.put(target)
           self.get_logger().info('Received target GPS list')
           self.enqued = True
      
      except json.JSONDecodeError as e:
        self.get_logger().error(f'Error decoding JSON: {e}')

    #Updates the current GPS location
    def current_gps_callback(self, msg):
        self.current_gps = (msg.latitude, msg.longitude)
        self.get_logger().info(f'Updated current GPS: {self.current_gps}')


    def check_for_topic(self):
        """Check if the 'program_control' topic exists."""
        topic_names_and_types = self.get_topic_names_and_types()
        topic_names = [name for name, _ in topic_names_and_types]
        
        if 'program_control' in topic_names and self.prog_ctrl_sub is None:
            self.get_logger().info("Topic 'program_control' detected! Subscribing...")
            self.prog_ctrl_sub = self.create_subscription(
                String,
                'program_control',
                self.program_control_callback,
                10
            )

    def program_control_callback(self, msg):
        """Callback function to handle program control commands."""
        control_command = msg.data.strip()

        if control_command == "Spinner sim: STOP":
            self.get_logger().info("Stopping 'Spinner sim' node...")
            self.destroy_node()
            rclpy.shutdown()
    
    def move_rover(self):
      if targets_queue.empty():
          self.get_logger().info('No more targets in the queue. Stopping the rover...')
          stop(self)
         
      else:
          next_gps = targets_queue.queue[0]
          distance_to_target = distanceFromGps(self.current_gps, next_gps)
          self.get_logger().info(f'Distance to target: next_gps {next_gps} {distance_to_target} meters')
  
          
          move_forward(self)
          if self.current_gps == next_gps or distance_to_target < proximity_threshold:
              if not self.first_target:
                rotate_right_90 (self)
              stop(self)
              self.first_target = False
              targets_queue.get()
              self.get_logger().info('Target reached.')

def main(args=None):
    rclpy.init(args=args)
    spiral_motion_planner = SpiralMotionPlanner()
    
    executor = MultiThreadedExecutor()
    executor.add_node(spiral_motion_planner)

    try:
        executor.spin()
    finally:
        spiral_motion_planner.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()