from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    static_transform_node = Node(
                                package='tf2_ros',
                                executable='static_transform_publisher',
                                arguments = ['--x', '2', '--y', '1', '--z', '0.0',
                                            '--yaw', '1.57', '--pitch', '0', '--roll', '0.0',
                                            '--frame-id', 'world', '--child-frame-id', 'robot_1']
                                )

    static_transform_node_2 = Node(
                                package='tf2_ros',
                                executable='static_transform_publisher',
                                arguments = ['--x', '1', '--y', '2', '--z', '0.0',
                                            '--yaw', '0.0', '--pitch', '0', '--roll', '0.0',
                                            '--frame-id', 'robot_1', '--child-frame-id', 'robot_2']
                                )
    

    rviz_node = Node(name='rviz',
                    package='rviz2',
                    executable='rviz2'
                    )
    
    rqt_tf_tree_node = Node(name='rqt_tf_tree',
                    package='rqt_tf_tree',
                    executable='rqt_tf_tree'
                    )
    
    l_d = LaunchDescription([static_transform_node, static_transform_node_2 , rqt_tf_tree_node, rviz_node])

    return l_d
