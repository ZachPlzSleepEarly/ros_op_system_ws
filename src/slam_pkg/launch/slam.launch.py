from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
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
        parameters=[
            slam_param,
            {'resolution': 0.01},
        ]
    )
    
    rviz_file = os.path.join(get_package_share_directory('wpr_simulation2'), 'rviz', 'slam.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_file],
    )

    sim_pkg = get_package_share_directory('wpr_simulation2')
    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(sim_pkg, 'launch', 'robocup_home.launch.py')
        )
    )

    # 键盘控制stdin，必须新开一个terminal
    keyboard_node = Node(
        package='wpr_simulation2',
        executable='keyboard_vel_cmd',
        prefix='gnome-terminal --'
    )

    ld = LaunchDescription()
    ld.add_action(slam_cmd)
    ld.add_action(rviz_node)
    ld.add_action(sim_launch)
    ld.add_action(keyboard_node)

    return ld