from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    
    static_transform_node = Node(
                                package='tf2_ros',
                                executable='static_transform_publisher',
                                arguments = ['--x', '2', '--y', '1', '--z', '0.0',
                                            '--yaw', '0.0', '--pitch', '0', '--roll', '0.0',
                                            '--frame-id', 'map', '--child-frame-id', 'odom']
                                )

    static_transform_node_2 = Node(
                                package='tf2_ros',
                                executable='static_transform_publisher',
                                arguments = ['--x', '0', '--y', '0', '--z', '0.0',
                                            '--yaw', '0.0', '--pitch', '0', '--roll', '0.0',
                                            '--frame-id', 'world', '--child-frame-id', 'map']
                                )
    
    
    puzzledrone_node = Node(name="puzzledrone",
                            package='markers',
                            executable='PuzzleDrone'
                            )
    
    rviz_config = os.path.join(
                            get_package_share_directory('markers'),
                            'rviz',
                            'config.rviz'
                            )

    
    rviz_node = Node(name='rviz',
                    package='rviz2',
                    executable='rviz2',
                    arguments=['-d', rviz_config]
                    )
    
    rqt_tf_tree_node = Node(name='rqt_tf_tree',
                    package='rqt_tf_tree',
                    executable='rqt_tf_tree'
                    )
    
    l_d = LaunchDescription([static_transform_node, static_transform_node_2 , rqt_tf_tree_node, rviz_node, puzzledrone_node])

    return l_d