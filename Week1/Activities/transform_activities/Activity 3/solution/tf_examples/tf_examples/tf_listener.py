import rclpy
from rclpy.node import Node
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import transforms3d
import numpy as np

class FrameListener(Node):

    def __init__(self):
        super().__init__('frame_listener')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        #Create a Timer
        timer_period = 0.1 #seconds
        self.timer = self.create_timer(timer_period, self.timer_cb)

        self.to_frame= 'world' 
        self.from_frame = 'moving_robot_3'

        #Variables to be used
        self.start_time = self.get_clock().now()
        self.omega = 0.1


    #Timer Callback
    def timer_cb(self):

        try:
            self.transformation = self.tf_buffer.lookup_transform(
                self.to_frame,
                self.from_frame,
                rclpy.time.Time())
        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform {self.to_frame} to {self.from_frame}: {ex}')
            return
        
        self.translation = self.transformation.transform.translation
        self.rotation = self.transformation.transform.rotation

        translation_matrix = np.array([
                [1, 0, 0, self.translation.x],
                [0, 1, 1, self.translation.y],
                [0, 0, 1, self.translation.z],
                [0, 0, 0, 1]
            ])

        _rotation_matrix = transforms3d.quaternions.quat2mat([
                self.rotation.w,
                self.rotation.x,
                self.rotation.y,
                self.rotation.z
            ])
        
        #Check https://github.com/DLu/tf_transformations/blob/main/tf_transformations/__init__.py#L48
        ZOOM_IDENTITY = [1.0, 1.0, 1.0]
        TRANSLATION_IDENTITY = [0.0, 0.0, 0.0]
        rotation_matrix=transforms3d.affines.compose(TRANSLATION_IDENTITY, _rotation_matrix, ZOOM_IDENTITY)

        transformation_matrix = np.dot(translation_matrix, rotation_matrix)

        print('Transform: ')
        print(self.transformation)
        print('Transform ation Matrix: ')
        print(transformation_matrix)


def main(args=None):
    rclpy.init(args=args)

    node = FrameListener()

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