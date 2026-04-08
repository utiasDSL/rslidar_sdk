#  Copyright (c) 2026  Learning Systems and Robotics Lab,
#  Technical University of Munich (TUM)
#
#  Authors:  Haoming Zhang <haoming.zhang@tum>
#
#  Permission is hereby granted, free of charge, to any person obtaining a copy
#  of this software and associated documentation files (the "Software"), to deal
#  in the Software without restriction, including without limitation the rights
#  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
#  copies of the Software, and to permit persons to whom the Software is
#  furnished to do so, subject to the following conditions:
#
#  The above copyright notice and this permission notice shall be included in all
#  copies or substantial portions of the Software.
#
#  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
#  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
#  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
#  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
#  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
#  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
#  SOFTWARE.
#

import os

from ament_index_python.packages import get_package_share_directory
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
    LogInfo
)
from launch import LaunchDescription
from launch_ros.actions import (
    Node,
    ComposableNodeContainer,
    LoadComposableNodes
)
from launch_ros.descriptions import ComposableNode
from launch.substitutions import (
    LaunchConfiguration,
    Command,
    TextSubstitution
)

os.environ["RCUTILS_COLORIZED_OUTPUT"] = "1"

default_config = os.path.join(
    get_package_share_directory('rslidar_sdk'),
    'config',
    'rslidar_config.yaml'
)


def launch_setup(context, *args, **kwargs):
    return_array = []
    container_name = LaunchConfiguration('container_name')
    container_name_val = container_name.perform(context)
    if container_name_val == '':
        container_name_val = '/rslidar/robosense_container'
        container_exec = 'component_container_mt'
        arguments_val = ['--ros-args', '--log-level', 'info']
        container = ComposableNodeContainer(
            name='robosense_container',
            namespace='/rslidar',
            package='rclcpp_components',
            executable=container_exec,
            emulate_tty=True,
            composable_node_descriptions=[],
            arguments=arguments_val,
            output='screen',
        )
        return_array.append(container)

    rslidar_component = ComposableNode(
        package='rslidar_sdk',
        plugin='robosense::lidar::RSLiDARComponent',
        name='rslidar',
        parameters=[{
            'config_path': default_config
        }],
        extra_arguments=[{'use_intra_process_comms': True}],
    )

    full_container_name = container_name_val
    load_composable_node = LoadComposableNodes(
        target_container=full_container_name,
        composable_node_descriptions=[rslidar_component]
    )
    return_array.append(load_composable_node)
    return return_array

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'container_name',
            default_value='',
            description='The name of the container to be used to load the rslidar component. If empty (default) a new container will be created.'),
        OpaqueFunction(function=launch_setup),

    ])
