import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from .rover_serial import connect_rover, move_left, move_right, move_forward, move_forward_left, move_forward_right, stop, printFromBoard


  #must change this values
args = {
    "bport": "/dev/ttyUSB0",
    "pport": "/dev/ttyACM2"
    }

class Rover(Node):
    def __init__(self):
        super().__init__('rover_driver')
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.drive_rover,
            10
        )
        #initialize board in constructor
        self.board = connect_rover(args["pport"])
    
    def drive_rover(self,msg):
        self.get_logger().info('Rover Driver Node has started')
        
        if(msg.linear.x> 0.0 and msg.angular.z == 0.0):
            self.get_logger().info('Cmd = F, moving forward')
            move_forward(self.board)
           #time.sleep(0.1) 
        elif (msg.linear.x == 0.0 and msg.angular.z>0.0):
            self.get_logger().info('Cmd = L, turning Left')
            move_left(self.board)
           #time.sleep(0.1) 
        elif (msg.linear.x == 0.0 and msg.angular.z<0.0):
            self.get_logger().info('Cmd = R, turning Right')
            move_right(self.board)
           #time.sleep(0.1) 
        elif (msg.linear.x == 0.0 and msg.angular.z == 0.0):
            self.get_logger().info('Cmd = S, stopping')
            stop(self.board)
           #time.sleep(0.1) 

def main(args=None):
    rclpy.init(args=args)
    rover_driver = Rover()
    rclpy.spin(rover_driver)
    rover_driver.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
