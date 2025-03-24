import rclpy
from rclpy.node import Node
from tf2_ros import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped
import transforms3d

class FramePublisher(Node):

    def __init__(self):
        super().__init__('frame_publisher')

        #WRITE YOUR CODE HERE

def main(args=None):
    rclpy.init(args=args)

    node = FramePublisher()

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