import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
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
        self.omega_wheel = 1.0

        #Define Transformations
        self.define_TF()
        
        #Create Transform Boradcasters
        self.tf_br_chassis = StaticTransformBroadcaster(self)
        self.tf_br_caster = StaticTransformBroadcaster(self)
        self.tf_br_base = TransformBroadcaster(self)
        self.tf_br_wl = TransformBroadcaster(self)
        self.tf_br_wr = TransformBroadcaster(self)

        #Create a Timer
        timer_period = 0.01 #seconds
        self.timer = self.create_timer(timer_period, self.timer_cb)

        self.tf_br_chassis.sendTransform(self.chassis_tf)
        self.tf_br_caster.sendTransform(self.caster_tf)


    #Timer Callback
    def timer_cb(self):

        time = self.get_clock().now().nanoseconds/1e9

        self.base_tf.header.stamp = self.get_clock().now().to_msg()
        self.base_tf.transform.translation.x = self.intial_pos_x + 0.5*np.cos(self.omega*time)
        self.base_tf.transform.translation.y = self.intial_pos_y + 0.5*np.sin(self.omega*time)
        self.base_tf.transform.translation.z = 0.0
        q = transforms3d.euler.euler2quat(0, 0, self.intial_pos_yaw+self.omega*time)       
        self.base_tf.transform.rotation.x = q[1]
        self.base_tf.transform.rotation.y = q[2]
        self.base_tf.transform.rotation.z = q[3]
        self.base_tf.transform.rotation.w = q[0]

        self.wheel_l_tf.header.stamp = self.get_clock().now().to_msg()
        q_prop1 = transforms3d.euler.euler2quat(0.0, self.omega_wheel*time, 0.0)       
        self.wheel_l_tf.transform.rotation.x = q_prop1[1]
        self.wheel_l_tf.transform.rotation.y = q_prop1[2]
        self.wheel_l_tf.transform.rotation.z = q_prop1[3]
        self.wheel_l_tf.transform.rotation.w = q_prop1[0]

        self.wheel_r_tf.header.stamp = self.get_clock().now().to_msg()
        q_prop2 = transforms3d.euler.euler2quat(0.0, self.omega_wheel*time,  0.0)       
        self.wheel_r_tf.transform.rotation.x = q_prop2[1]
        self.wheel_r_tf.transform.rotation.y = q_prop2[2]
        self.wheel_r_tf.transform.rotation.z = q_prop2[3]
        self.wheel_r_tf.transform.rotation.w = q_prop2[0]

        self.tf_br_base.sendTransform(self.base_tf)
        self.tf_br_wl.sendTransform(self.wheel_l_tf)
        self.tf_br_wr.sendTransform(self.wheel_r_tf)


    def define_TF(self):

        #Create Trasnform Messages
        self.base_tf = TransformStamped()
        self.base_tf.header.stamp = self.get_clock().now().to_msg()
        self.base_tf.header.frame_id = 'world'
        self.base_tf.child_frame_id = 'base'
        self.base_tf.transform.translation.x = self.intial_pos_x
        self.base_tf.transform.translation.y = self.intial_pos_y
        self.base_tf.transform.translation.z = 0.0
        q_foot = transforms3d.euler.euler2quat(self.intial_pos_roll, self.intial_pos_pitch, self.intial_pos_yaw)       
        self.base_tf.transform.rotation.x = q_foot[1]
        self.base_tf.transform.rotation.y = q_foot[2]
        self.base_tf.transform.rotation.z = q_foot[3]
        self.base_tf.transform.rotation.w = q_foot[0]


        #Create Trasnform Messages
        self.chassis_tf = TransformStamped()
        self.chassis_tf.header.stamp = self.get_clock().now().to_msg()
        self.chassis_tf.header.frame_id = 'base'
        self.chassis_tf.child_frame_id = 'chassis'
        self.chassis_tf.transform.translation.x = 0.0
        self.chassis_tf.transform.translation.y = 0.0
        self.chassis_tf.transform.translation.z = 0.05
        self.chassis_tf.transform.rotation.x = 0.0
        self.chassis_tf.transform.rotation.y = 0.0
        self.chassis_tf.transform.rotation.z = 0.0
        self.chassis_tf.transform.rotation.w = 1.0


        self.wheel_l_tf = TransformStamped()
        #Create Trasnform Messages
        self.wheel_l_tf.header.stamp = self.get_clock().now().to_msg()
        self.wheel_l_tf.header.frame_id = 'chassis'
        self.wheel_l_tf.child_frame_id = 'wheel_l'
        self.wheel_l_tf.transform.translation.x = 0.00
        self.wheel_l_tf.transform.translation.y = -0.1
        self.wheel_l_tf.transform.translation.z = 0.0
        self.wheel_l_tf.transform.rotation.x = 0.0
        self.wheel_l_tf.transform.rotation.y = 0.0
        self.wheel_l_tf.transform.rotation.z = 0.0
        self.wheel_l_tf.transform.rotation.w = 1.0


        self.wheel_r_tf = TransformStamped()
        self.wheel_r_tf.header.stamp = self.get_clock().now().to_msg()
        self.wheel_r_tf.header.frame_id = 'chassis'
        self.wheel_r_tf.child_frame_id = 'wheel_r'
        self.wheel_r_tf.transform.translation.x = 0.00
        self.wheel_r_tf.transform.translation.y = 0.1
        self.wheel_r_tf.transform.translation.z = 0.0
        self.wheel_r_tf.transform.rotation.x = 0.0
        self.wheel_r_tf.transform.rotation.y = 0.0
        self.wheel_r_tf.transform.rotation.z = 0.0
        self.wheel_r_tf.transform.rotation.w = 1.0

        self.caster_tf = TransformStamped()
        self.caster_tf.header.stamp = self.get_clock().now().to_msg()
        self.caster_tf.header.frame_id = 'chassis'
        self.caster_tf.child_frame_id = 'caster'
        self.caster_tf.transform.translation.x = -0.095
        self.caster_tf.transform.translation.y = 0.0
        self.caster_tf.transform.translation.z = -0.03
        self.caster_tf.transform.rotation.x = 0.0
        self.caster_tf.transform.rotation.y = 0.0
        self.caster_tf.transform.rotation.z = 0.0
        self.caster_tf.transform.rotation.w = 1.0


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