import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
import transforms3d
import numpy as np

class MarkersPublisher(Node):

    def __init__(self):
        super().__init__('marker_publisher')
        self.publisher = self.create_publisher(Marker, '/marker', 10)

        #Variables to be used
        self.omega = 0.5
        self.intial_pos_x = 0.0
        self.intial_pos_y = 0.0
        self.intial_pos_z = 1.0

        #initialise the marker(the pose and orientation will be changed on the callback function)
        self.marker = Marker()
        self.marker.header.frame_id = "map"
        self.marker.header.stamp = self.get_clock().now().to_msg()
        self.marker.id = 0
        self.marker.type = Marker.CUBE
        self.marker.action = Marker.ADD
        self.marker.pose.position.x = self.intial_pos_x 
        self.marker.pose.position.y = self.intial_pos_y
        self.marker.pose.position.z = self.intial_pos_z
        self.marker.pose.orientation.x = 0.0
        self.marker.pose.orientation.y = 0.0
        self.marker.pose.orientation.z = 0.0
        self.marker.pose.orientation.w = 1.0
        self.marker.scale.x = 0.2
        self.marker.scale.y = 0.5
        self.marker.scale.z = 0.2
        self.marker.color.r = 1.0
        self.marker.color.g = 1.0
        self.marker.color.b = 0.0
        self.marker.color.a = 1.0

        timer_period = 0.1 #seconds
        self.timer = self.create_timer(timer_period, self.timer_cb)
        self.i = 0



    #Timer Callback
    def timer_cb(self):
        time = self.get_clock().now().nanoseconds/1e9

        q = transforms3d.euler.euler2quat(0, 1.57, self.omega*time)
        self.marker.header.stamp = self.get_clock().now().to_msg()
        self.marker.pose.position.x = self.intial_pos_x + 0.5*np.sin(self.omega*time)
        self.marker.pose.position.y = self.intial_pos_y + 0.5*np.cos(self.omega*time)
        self.marker.pose.position.z = self.intial_pos_z
        self.marker.pose.orientation.x = q[1]
        self.marker.pose.orientation.y = q[2]
        self.marker.pose.orientation.z = q[3]
        self.marker.pose.orientation.w = q[0]


        self.publisher.publish(self.marker)



def main(args=None):
    rclpy.init(args=args)

    node = MarkersPublisher()

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