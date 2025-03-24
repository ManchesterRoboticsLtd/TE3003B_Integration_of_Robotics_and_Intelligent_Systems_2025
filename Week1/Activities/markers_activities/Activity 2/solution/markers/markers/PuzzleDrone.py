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
        self.tf_br_base = TransformBroadcaster(self)
        self.tf_br_base_footprint = TransformBroadcaster(self)
        self.tf_br_prop1 = TransformBroadcaster(self)
        self.tf_br_prop2 = TransformBroadcaster(self)
        self.tf_br_prop3 = TransformBroadcaster(self)
        self.tf_br_prop4 = TransformBroadcaster(self)

        #Create Markers Publishers
        self.base_marker_pub = self.create_publisher(Marker, '/base_marker', 10)
        self.prop1_marker_pub = self.create_publisher(Marker, '/prop1_marker', 10)
        self.prop2_marker_pub = self.create_publisher(Marker, '/prop2_marker', 10)
        self.prop3_marker_pub = self.create_publisher(Marker, '/prop3_marker', 10)
        self.prop4_marker_pub = self.create_publisher(Marker, '/prop4_marker', 10)

        #Create a Timer
        timer_period = 0.01 #seconds
        self.timer = self.create_timer(timer_period, self.timer_cb)


    #Timer Callback
    def timer_cb(self):

        time = self.get_clock().now().nanoseconds/1e9

        self.base.header.stamp = self.get_clock().now().to_msg()
        self.prop1.header.stamp = self.get_clock().now().to_msg()
        self.prop2.header.stamp = self.get_clock().now().to_msg()
        self.prop3.header.stamp = self.get_clock().now().to_msg()
        self.prop4.header.stamp = self.get_clock().now().to_msg()

        #Create Trasnform Messages
        self.base_link_tf.header.stamp = self.get_clock().now().to_msg()
        self.base_link_tf.transform.translation.x = self.intial_pos_x + 0.5*np.cos(self.omega*time)
        self.base_link_tf.transform.translation.y = self.intial_pos_y + 0.5*np.sin(self.omega*time)
        self.base_link_tf.transform.translation.z = self.intial_pos_z
        q = transforms3d.euler.euler2quat(self.intial_pos_roll, self.intial_pos_pitch, self.intial_pos_yaw+self.omega*time)       
        self.base_link_tf.transform.rotation.x = q[1]
        self.base_link_tf.transform.rotation.y = q[2]
        self.base_link_tf.transform.rotation.z = q[3]
        self.base_link_tf.transform.rotation.w = q[0]

        self.base_footprint_tf.header.stamp = self.get_clock().now().to_msg()
        self.base_footprint_tf.transform.translation.x = self.intial_pos_x + 0.5*np.cos(self.omega*time)
        self.base_footprint_tf.transform.translation.y = self.intial_pos_y + 0.5*np.sin(self.omega*time)
        self.base_footprint_tf.transform.translation.z = 0.0
        q = transforms3d.euler.euler2quat(0, 0, self.intial_pos_yaw+self.omega*time)       
        self.base_footprint_tf.transform.rotation.x = q[1]
        self.base_footprint_tf.transform.rotation.y = q[2]
        self.base_footprint_tf.transform.rotation.z = q[3]
        self.base_footprint_tf.transform.rotation.w = q[0]

        self.prop_1_tf.header.stamp = self.get_clock().now().to_msg()
        q_prop1 = transforms3d.euler.euler2quat(0, 0, self.omega_prop*time)       
        self.prop_1_tf.transform.rotation.x = q_prop1[1]
        self.prop_1_tf.transform.rotation.y = q_prop1[2]
        self.prop_1_tf.transform.rotation.z = q_prop1[3]
        self.prop_1_tf.transform.rotation.w = q_prop1[0]

        self.prop_2_tf.header.stamp = self.get_clock().now().to_msg()
        q_prop2 = transforms3d.euler.euler2quat(0, 0, self.omega_prop*time)       
        self.prop_2_tf.transform.rotation.x = q_prop2[1]
        self.prop_2_tf.transform.rotation.y = q_prop2[2]
        self.prop_2_tf.transform.rotation.z = q_prop2[3]
        self.prop_2_tf.transform.rotation.w = q_prop2[0]

        self.prop_3_tf.header.stamp = self.get_clock().now().to_msg()
        q_prop3 = transforms3d.euler.euler2quat(0, 0, self.omega_prop*time)       
        self.prop_3_tf.transform.rotation.x = q_prop3[1]
        self.prop_3_tf.transform.rotation.y = q_prop3[2]
        self.prop_3_tf.transform.rotation.z = q_prop3[3]
        self.prop_3_tf.transform.rotation.w = q_prop3[0]
        
        self.prop_4_tf.header.stamp = self.get_clock().now().to_msg()
        q_prop4 = transforms3d.euler.euler2quat(0, 0, self.omega_prop*time)       
        self.prop_4_tf.transform.rotation.x = q_prop4[1]
        self.prop_4_tf.transform.rotation.y = q_prop4[2]
        self.prop_4_tf.transform.rotation.z = q_prop4[3]
        self.prop_4_tf.transform.rotation.w = q_prop4[0]

        self.tf_br_base.sendTransform(self.base_link_tf)
        self.tf_br_base_footprint.sendTransform(self.base_footprint_tf)
        self.tf_br_prop1.sendTransform(self.prop_1_tf)
        self.tf_br_prop2.sendTransform(self.prop_2_tf)
        self.tf_br_prop3.sendTransform(self.prop_3_tf)
        self.tf_br_prop4.sendTransform(self.prop_4_tf)

        self.base_marker_pub.publish(self.base)
        self.prop1_marker_pub.publish(self.prop1)
        self.prop2_marker_pub.publish(self.prop2)
        self.prop3_marker_pub.publish(self.prop3)
        self.prop4_marker_pub.publish(self.prop4)

    def define_markers(self):
        
        #initialise the marker(the pose and orientation will be changed on the callback function)
        self.base = Marker()
        self.base.header.frame_id = "base_link"
        self.base.header.stamp = self.get_clock().now().to_msg()
        self.base.id = 0
        self.base.type = Marker.MESH_RESOURCE
        self.base.mesh_resource = "package://markers/meshes/base_210mm.stl"
        self.base.action = Marker.ADD
        self.base.pose.position.x = 0.0
        self.base.pose.position.y = 0.0
        self.base.pose.position.z = -0.0205
        q_base_marker = transforms3d.euler.euler2quat(1.57, 0.0, 1.57) 
        self.base.pose.orientation.x = q_base_marker[1]
        self.base.pose.orientation.y = q_base_marker[2]
        self.base.pose.orientation.z = q_base_marker[3]
        self.base.pose.orientation.w = q_base_marker[0]
        self.base.scale.x = 1.0
        self.base.scale.y = 1.0
        self.base.scale.z = 1.0
        self.base.color.r = 1.0
        self.base.color.g = 1.0
        self.base.color.b = 0.0
        self.base.color.a = 1.0

        self.prop1 = Marker()
        self.prop1.header.frame_id = "prop_1"
        self.prop1.header.stamp = self.get_clock().now().to_msg()
        self.prop1.id = 0
        self.prop1.type = Marker.MESH_RESOURCE
        self.prop1.mesh_resource = "package://markers/meshes/propeller_ccw_puller_5in.stl"
        self.prop1.action = Marker.ADD
        self.prop1.pose.position.x = 0.0 
        self.prop1.pose.position.y = 0.0
        self.prop1.pose.position.z = -0.004
        self.prop1.pose.orientation.x = 0.0
        self.prop1.pose.orientation.y = 0.0
        self.prop1.pose.orientation.z = 0.0
        self.prop1.pose.orientation.w = 1.0
        self.prop1.scale.x = 1.0
        self.prop1.scale.y = 1.0
        self.prop1.scale.z = 1.0
        self.prop1.color.r = 1.0
        self.prop1.color.g = 1.0
        self.prop1.color.b = 0.0
        self.prop1.color.a = 1.0

        self.prop2 = Marker()
        self.prop2.header.frame_id = "prop_2"
        self.prop2.header.stamp = self.get_clock().now().to_msg()
        self.prop2.id = 0
        self.prop2.type = Marker.MESH_RESOURCE
        self.prop2.mesh_resource = "package://markers/meshes/propeller_cw_puller_5in.stl"
        self.prop2.action = Marker.ADD
        self.prop2.pose.position.x = 0.0 
        self.prop2.pose.position.y = 0.0
        self.prop2.pose.position.z = -0.004
        self.prop2.pose.orientation.x = 0.0
        self.prop2.pose.orientation.y = 0.0
        self.prop2.pose.orientation.z = 0.0
        self.prop2.pose.orientation.w = 1.0
        self.prop2.scale.x = 1.0
        self.prop2.scale.y = 1.0
        self.prop2.scale.z = 1.0
        self.prop2.color.r = 1.0
        self.prop2.color.g = 1.0
        self.prop2.color.b = 0.0
        self.prop2.color.a = 1.0

        self.prop3 = Marker()
        self.prop3.header.frame_id = "prop_3"
        self.prop3.header.stamp = self.get_clock().now().to_msg()
        self.prop3.id = 0
        self.prop3.type = Marker.MESH_RESOURCE
        self.prop3.mesh_resource = "package://markers/meshes/propeller_ccw_puller_5in.stl"
        self.prop3.action = Marker.ADD
        self.prop3.pose.position.x = 0.0 
        self.prop3.pose.position.y = 0.0
        self.prop3.pose.position.z = -0.004
        self.prop3.pose.orientation.x = 0.0
        self.prop3.pose.orientation.y = 0.0
        self.prop3.pose.orientation.z = 0.0
        self.prop3.pose.orientation.w = 1.0
        self.prop3.scale.x = 1.0
        self.prop3.scale.y = 1.0
        self.prop3.scale.z = 1.0
        self.prop3.color.r = 1.0
        self.prop3.color.g = 1.0
        self.prop3.color.b = 0.0
        self.prop3.color.a = 1.0

        self.prop4 = Marker()
        self.prop4.header.frame_id = "prop_4"
        self.prop4.header.stamp = self.get_clock().now().to_msg()
        self.prop4.id = 0
        self.prop4.type = Marker.MESH_RESOURCE
        self.prop4.mesh_resource = "package://markers/meshes/propeller_cw_puller_5in.stl"
        self.prop4.action = Marker.ADD
        self.prop4.pose.position.x = 0.0 
        self.prop4.pose.position.y = 0.0
        self.prop4.pose.position.z = -0.004
        self.prop4.pose.orientation.x = 0.0
        self.prop4.pose.orientation.y = 0.0
        self.prop4.pose.orientation.z = 0.0
        self.prop4.pose.orientation.w = 1.0
        self.prop4.scale.x = 1.0
        self.prop4.scale.y = 1.0
        self.prop4.scale.z = 1.0
        self.prop4.color.r = 1.0
        self.prop4.color.g = 1.0
        self.prop4.color.b = 0.0
        self.prop4.color.a = 1.0
        

    def define_TF(self):

        #Create Trasnform Messages
        self.base_footprint_tf = TransformStamped()
        self.base_footprint_tf.header.stamp = self.get_clock().now().to_msg()
        self.base_footprint_tf.header.frame_id = 'odom'
        self.base_footprint_tf.child_frame_id = 'base_footprint'
        self.base_footprint_tf.transform.translation.x = self.intial_pos_x
        self.base_footprint_tf.transform.translation.y = self.intial_pos_y
        self.base_footprint_tf.transform.translation.z = 0.0
        q_foot = transforms3d.euler.euler2quat(self.intial_pos_roll, self.intial_pos_pitch, self.intial_pos_yaw)       
        self.base_footprint_tf.transform.rotation.x = q_foot[1]
        self.base_footprint_tf.transform.rotation.y = q_foot[2]
        self.base_footprint_tf.transform.rotation.z = q_foot[3]
        self.base_footprint_tf.transform.rotation.w = q_foot[0]


        #Create Trasnform Messages
        self.base_link_tf = TransformStamped()
        self.base_link_tf.header.stamp = self.get_clock().now().to_msg()
        self.base_link_tf.header.frame_id = 'odom'
        self.base_link_tf.child_frame_id = 'base_link'
        self.base_link_tf.transform.translation.x = self.intial_pos_x
        self.base_link_tf.transform.translation.y = self.intial_pos_y
        self.base_link_tf.transform.translation.z = self.intial_pos_z
        q = transforms3d.euler.euler2quat(self.intial_pos_roll, self.intial_pos_pitch, self.intial_pos_yaw)       
        self.base_link_tf.transform.rotation.x = q[1]
        self.base_link_tf.transform.rotation.y = q[2]
        self.base_link_tf.transform.rotation.z = q[3]
        self.base_link_tf.transform.rotation.w = q[0]


        self.prop_1_tf = TransformStamped()
        #Create Trasnform Messages
        self.prop_1_tf.header.stamp = self.get_clock().now().to_msg()
        self.prop_1_tf.header.frame_id = 'base_link'
        self.prop_1_tf.child_frame_id = 'prop_1'
        self.prop_1_tf.transform.translation.x = 0.06717
        self.prop_1_tf.transform.translation.y = 0.082
        self.prop_1_tf.transform.translation.z = -0.0125
        q_prop1 = transforms3d.euler.euler2quat(0, 0, 0)       
        self.prop_1_tf.transform.rotation.x = q_prop1[1]
        self.prop_1_tf.transform.rotation.y = q_prop1[2]
        self.prop_1_tf.transform.rotation.z = q_prop1[3]
        self.prop_1_tf.transform.rotation.w = q_prop1[0]


        self.prop_2_tf = TransformStamped()
        self.prop_2_tf.header.stamp = self.get_clock().now().to_msg()
        self.prop_2_tf.header.frame_id = 'base_link'
        self.prop_2_tf.child_frame_id = 'prop_2'
        self.prop_2_tf.transform.translation.x = -0.06717
        self.prop_2_tf.transform.translation.y = 0.082
        self.prop_2_tf.transform.translation.z = -0.0125
        q_prop2 = transforms3d.euler.euler2quat(0, 0, 0)       
        self.prop_2_tf.transform.rotation.x = q_prop2[1]
        self.prop_2_tf.transform.rotation.y = q_prop2[2]
        self.prop_2_tf.transform.rotation.z = q_prop2[3]
        self.prop_2_tf.transform.rotation.w = q_prop2[0]

        self.prop_3_tf = TransformStamped()
        self.prop_3_tf.header.stamp = self.get_clock().now().to_msg()
        self.prop_3_tf.header.frame_id = 'base_link'
        self.prop_3_tf.child_frame_id = 'prop_3'
        self.prop_3_tf.transform.translation.x = -0.06717
        self.prop_3_tf.transform.translation.y = -0.082
        self.prop_3_tf.transform.translation.z = -0.0125
        q_prop3 = transforms3d.euler.euler2quat(0, 0, 0)       
        self.prop_3_tf.transform.rotation.x = q_prop3[1]
        self.prop_3_tf.transform.rotation.y = q_prop3[2]
        self.prop_3_tf.transform.rotation.z = q_prop3[3]
        self.prop_3_tf.transform.rotation.w = q_prop3[0]
        
        self.prop_4_tf = TransformStamped()
        self.prop_4_tf.header.stamp = self.get_clock().now().to_msg()
        self.prop_4_tf.header.frame_id = 'base_link'
        self.prop_4_tf.child_frame_id = 'prop_4'
        self.prop_4_tf.transform.translation.x = 0.06717
        self.prop_4_tf.transform.translation.y = -0.082
        self.prop_4_tf.transform.translation.z = -0.0125
        q_prop4 = transforms3d.euler.euler2quat(0, 0, 0)       
        self.prop_4_tf.transform.rotation.x = q_prop4[1]
        self.prop_4_tf.transform.rotation.y = q_prop4[2]
        self.prop_4_tf.transform.rotation.z = q_prop4[3]
        self.prop_4_tf.transform.rotation.w = q_prop4[0]


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