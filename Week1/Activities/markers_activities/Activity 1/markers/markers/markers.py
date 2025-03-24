import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
import transforms3d
import numpy as np

class MarkersPublisher(Node):

    def __init__(self):
        super().__init__('marker_publisher')
        self.publisher = self.create_publisher(Marker, '/marker', 10)


        timer_period = 0.1 #seconds
        self.timer = self.create_timer(timer_period, self.timer_cb)
        self.i = 0



    #Timer Callback
    def timer_cb(self):
        time = self.get_clock().now().nanoseconds/1e9



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