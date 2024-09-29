# Copyright 2021 Intelligent Robotics Lab
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression, PathJoinSubstitution, TextSubstitution
from launch_ros.actions import Node
from launch.actions.execute_process import ExecuteProcess
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    #tracking_dir = get_package_share_directory('br2_tracking')

    # tracking_cmd = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(os.path.join(tracking_dir, 'launch', 'tracking.launch.py')))

     # Get the install directory from the environment variable
    install_dir = os.getenv('COLCON_PREFIX_PATH').split(':')[0]

    # Define the path to your package within the install directory
    groot_path = os.path.join(install_dir, 'groot',
                    'lib',
                    'groot',
                    'Groot')

    groot_cmd = ExecuteProcess(
            cmd=[groot_path, '--autoconnect', '--mode' , 'monitor', '--publisher_port', '2666', '--server_port', '2667',],
            output='screen'
        )

    patrolling_cmd = Node(
        package='master3_bt3',
        executable='behavior_main',
        parameters=[{
          'use_sim_time': True
        }],
        remappings=[
          ('input_scan', '/scan_raw'),
          ('output_vel', '/nav_vel')
        ],
        output='screen'
    )

    ld = LaunchDescription()

    # Add any actions
#    ld.add_action(tracking_cmd)
    ld.add_action(patrolling_cmd)
    ld.add_action(groot_cmd)

    return ld
