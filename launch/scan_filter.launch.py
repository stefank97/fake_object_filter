from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    pkg = get_package_share_directory('fake_object_filter')
    params_file = os.path.join(pkg, 'config', 'fake_object.yaml')

    return LaunchDescription([
        Node(
            package='fake_object_filter',
            executable='object_filter_node',
            name='filter_node',
            output='screen',
            parameters=[params_file],
            arguments=['--ros-args','--log-level','info'] #For plugin logs
        ),
###Remove the comments below if you want to launch the rectangle node and don't want to use the fake_object_tool package (rviz plugin)###

        # Node(
        #     package='fake_object_filter',
        #     executable='rectangle_node',
        #     name='rectangle_node',
        #     output='screen',
        #     parameters=[params_file],
        #     arguments=['--ros-args','--log-level','info']  
        # )
    ])
