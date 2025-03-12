# pip install geopy
import rclpy
import json
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix
from geopy.geocoders import Nominatim
from geopy.distance import distance
from geopy.distance import geodesic

#Must use signed decimal degrees for gps coords
#Function does not use the deg-min-sec gps format



# Function: Given a bearing, desired distance, and current GPS location,
# return a new GPS coordinate
def calculateTargetGPS(bearing_degrees, distance_meters, currentGPSLoc):
    new_coords = distance(kilometers=distance_meters / 1000).destination(currentGPSLoc, bearing=bearing_degrees)
    return (new_coords.latitude, new_coords.longitude)

# Function: Given two gps locations, returns the displacement between them
def distanceFromGps(initial_gps,current_gps):
    distance_between = geodesic(initial_gps,current_gps).km * 1000 #convert to meters
    return distance_between

# pip install geopy
from geopy.geocoders import Nominatim
from geopy.distance import distance
from geopy.distance import geodesic

#Must use signed decimal degrees for gps coords
#Function does not use the deg-min-sec gps format



# Function: Given a bearing, desired distance, and current GPS location,
# return a new GPS coordinate
def calculateTargetGPS(bearing_degrees, distance_meters, currentGPSLoc):
    new_coords = distance(kilometers=distance_meters / 1000).destination(currentGPSLoc, bearing=bearing_degrees)
    return (new_coords.latitude, new_coords.longitude)

# Function: Given two gps locations, returns the displacement between them
def distanceFromGps(initial_gps,current_gps):
    distance_between = geodesic(initial_gps,current_gps).km * 1000 #convert to meters
    return distance_between

# Function: Given the current gps coord, returns a list of target gps coords to produce a spiral search pattern
# The spiral search pattern is a series of concentric squares with a target gps coord at each corner
def generateTargetGPSList(currentGPS,dis_inc,limiting_dist):
    init_gps = currentGPS
    cur_gps = currentGPS
    distance_traversed = 0
    target_gps_list = []
    square_lenght = dis_inc

    i=0
    count = 1
    idx = 0
             # N, E, S, W
    bearing = [0,90,180,270]
    #Stop generating when limiting distance is reached and body is facing north
    while (distance_traversed < limiting_dist) or (bearing[i] != 0):
        next_gps = calculateTargetGPS(bearing[i],square_lenght,cur_gps)
        target_gps_list.append(next_gps)
        cur_gps = next_gps
        idx = idx + 1
        i = (i+1)%4
        count = count + 1
        if count > 2:
            count = 1
            square_lenght = square_lenght + dis_inc
        distance_traversed = distanceFromGps(init_gps,next_gps)
        print("Distance Traversed: ",distance_traversed)
    return target_gps_list


class SpiralSearch(Node):
    def __init__(self):
        super().__init__('spiral_search_generator')
        #parameters with default values
        self.declare_parameter('initial_gps', [0.0,0.0])
        self.declare_parameter('dis_inc',3.0)
        self.declare_parameter('limiting_dist',10.0)
        self.got_inital_gps = False

        self.subscribe_to_gps = self.create_subscription(
            NavSatFix,
             '/gps/fix',
             self.set_inital_gps,
             10
        )

        #get parameters
        initial_gps_param = self.get_parameter('initial_gps').value
        self.inital_gps = None
        self.dis_inc = self.get_parameter('dis_inc').value
        self.limiting_dist = self.get_parameter('limiting_dist').value

        #Create publisher object
        self.publisher = self.create_publisher(String, 'gps_targets_list', 10)
           # Timer to publish periodically
        self.timer = self.create_timer(2.0, self.publish_gps_list)
        self.get_logger().info('Spiral Search Generator Node has started!')

    #publish a formatted target_gps_list to topic `gps_targets_list`
    # publish a formatted target_gps_list to topic `gps_targets_list`
    def set_inital_gps(self, msg):
        if not self.got_inital_gps:
            self.initial_gps = (msg.latitude, msg.longitude)
            self.get_logger().info(f'got initial : {self.initial_gps}, {(msg.latitude, msg.longitude)}')
            self.got_inital_gps = True
            
    def publish_gps_list(self):
        self.target_gps_list = generateTargetGPSList(self.initial_gps,self.dis_inc,self.limiting_dist)
        formatted_targets = json.dumps(self.target_gps_list)

        #Create a msg object
        msg = String()
        msg.data = formatted_targets
        self.publisher.publish(msg)
        self.get_logger().info(f'Published target GPS list: {formatted_targets}')


#use generateTargetGPSList to output a single persistent message to a gps_target_list topic
def main(args = None):
    rclpy.init(args=args)
    spiral_search = SpiralSearch()
    rclpy.spin(spiral_search)
    spiral_search.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()






