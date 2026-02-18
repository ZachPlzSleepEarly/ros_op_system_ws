from launch_ros.actions import Node
from launch import LaunchDescription
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    slam_param = {
        "use_sim_time": True,
        "base_frame": "base_footprint",
        "odom_frame": "odom",
        "map_frame": "map"
    }

    slam_cmd = Node(
        package="slam_toolbox",
        executable="sync_slam_toolbox_node",
        parameters=[slam_param]
    )
    
    rviz_file = os.path.join(get_package_share_directory('wpr_simulation2'), 'rviz', 'slam.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_file]
    )

    ld = LaunchDescription()
    ld.add_action(slam_cmd)
    ld.add_action(rviz_node)

    return ld