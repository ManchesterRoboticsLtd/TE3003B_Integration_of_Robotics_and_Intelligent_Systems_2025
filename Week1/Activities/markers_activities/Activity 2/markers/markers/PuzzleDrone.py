import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from visualization_msgs.msg import Marker
import transforms3d
import numpy as np

class DronePublisher(Node):

    def __init__(self):
        super().__init__('frame_publisher')

        #Drone Initial Pose
        self.intial_pos_x = 1.0
        self.intial_pos_y = 1.0
        self.intial_pos_z = 1.0
        self.intial_pos_yaw = np.pi/2
        self.intial_pos_pitch = 0.0
        self.intial_pos_roll = 0.0


        #Angular velocity for the pose change and propellers
        self.omega = 0.5
        self.omega_prop = 100.0

        #Define Transformations
        self.define_TF()
        #Define Markers
        self.define_markers()
        
        #Create Transform Boradcasters


        #Create Markers Publishers


        #Create a Timer
        timer_period = 0.01 #seconds
        self.timer = self.create_timer(timer_period, self.timer_cb)


    #Timer Callback
    def timer_cb(self):

        time = self.get_clock().now().nanoseconds/1e9


    def define_markers(self):
        
        #initialise the marker(the pose and orientation will be changed on the callback function)
        

    def define_TF(self):

        #Create Trasnform Messages



def main(args=None):
    rclpy.init(args=args)

    node = DronePublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():  # Ensure shutdown is only called once
            rclpy.shutdown()
        node.destroy_node()


if __name__ == '__main__':
    main()