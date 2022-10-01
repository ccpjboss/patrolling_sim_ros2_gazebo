# Copyright (c) 2018 Intel Corporation
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

"""
Example for spawning multiple robots in Gazebo.

This is an example on how to create a launch file for spawning multiple robots into Gazebo
and launch multiple instances of the navigation stack, each controlling one robot.
The robots co-exist on a shared environment and are controlled by independent nav stacks.
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, GroupAction,
                            IncludeLaunchDescription, LogInfo)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch.actions import OpaqueFunction
import configparser

initPoses = {}
package_path = get_package_share_directory('patrolling_sim_ros2')

def loadInitPoses():
    try:
        ConfigIP = configparser.ConfigParser()
        ConfigIP.read(package_path+"/params/initial_poses.txt")
        for option in ConfigIP.options("InitialPoses"):
            initPoses[option] = ConfigIP.get("InitialPoses", option)
    except:
        print("Could not load initial poses file")

def launch_setup(context, *args, **kwargs):
    # Get the launch directory
    launch_dir = os.path.join(package_path, 'launch')

    # Names and poses of the robots
    loadInitPoses()

    map_name_str = LaunchConfiguration('map_path').perform(context)
    n_robots_str = LaunchConfiguration('n_robots').perform(context)
    n_robots = int(n_robots_str)

    scenario = map_name_str+'_'+n_robots_str
    iposes = initPoses[scenario.lower()]
    iposes = iposes.replace('[','')
    iposes = iposes.replace(']','')
    iposes = iposes.split(',')

    robots = []
    for i in range(n_robots):
        robots.append({'name': ('robot'+str(i)), 'x_pose': iposes[i*2], 'y_pose': iposes[i*2+1],
        'z_pose': 0.01, 'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0})
    print(robots)

    # Simulation settings
    simulator = LaunchConfiguration('simulator')

    # On this example all robots are launched with the same settings
    # map_yaml_file = LaunchConfiguration('map_path')

    autostart = LaunchConfiguration('autostart')
    rviz_config_file = LaunchConfiguration('rviz_config')
    use_robot_state_pub = LaunchConfiguration('use_robot_state_pub')
    use_rviz = LaunchConfiguration('use_rviz')
    log_settings = LaunchConfiguration('log_settings', default='false')

    # Declare the launch arguments
    world_str = package_path +'/worlds/' + map_name_str + '.world'

    declare_simulator_cmd = DeclareLaunchArgument(
        'simulator',
        default_value='gazebo',
        description='The simulator to use (gazebo or gzserver)')

    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(package_path, 'maps','1r5', '1r5.yaml'),
        description='Full path to map file to load')

    declare_robot3_params_file_cmd = DeclareLaunchArgument(
        'robot2_params_file',
        default_value=os.path.join(package_path, 'params', 'nav2_multirobot_params_2.yaml'),
        description='Full path to the ROS2 parameters file to use for robot3 launched nodes')


    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        'rviz_config',
        default_value=os.path.join(package_path, 'rviz', 'nav2_namespaced_view.rviz'),
        description='Full path to the RVIZ config file to use.')


    # Start Gazebo with plugin providing the robot spawning service
    start_gazebo_cmd = ExecuteProcess(
        cmd=[simulator, '--verbose', '-s', 'libgazebo_ros_init.so',
                                     '-s', 'libgazebo_ros_factory.so', world_str],
        output='screen')

    # Define commands for launching the navigation instances
    nav_instances_cmds = []
    for robot in robots:
        params_file = LaunchConfiguration(f"{robot['name']}_params_file")

        group = GroupAction([
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                        os.path.join(launch_dir, 'rviz_launch.py')),
                condition=IfCondition(use_rviz),
                launch_arguments={
                                  'namespace': TextSubstitution(text=robot['name']),
                                  'use_namespace': 'True',
                                  'rviz_config': rviz_config_file}.items()),

            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(package_path,
                                                           'launch',
                                                           'stop_simulation_launch.py')),
                launch_arguments={'namespace': robot['name'],
                                  'use_namespace': 'True',
                                  'map': package_path+'/maps/'+map_name_str+'/'+map_name_str+'.yaml',
                                  'use_sim_time': 'True',
                                  'params_file': params_file,
                                  'autostart': autostart,
                                  'use_rviz': 'False',
                                  'use_simulator': 'False',
                                  'headless': 'False',
                                  'use_robot_state_pub': use_robot_state_pub,
                                  'x_pose': TextSubstitution(text=str(robot['x_pose'])),
                                  'y_pose': TextSubstitution(text=str(robot['y_pose'])),
                                  'z_pose': TextSubstitution(text=str(robot['z_pose'])),
                                  'roll': TextSubstitution(text=str(robot['roll'])),
                                  'pitch': TextSubstitution(text=str(robot['pitch'])),
                                  'yaw': TextSubstitution(text=str(robot['yaw'])),
                                  'robot_name':TextSubstitution(text=robot['name']), }.items()),

            LogInfo(
                condition=IfCondition(log_settings),
                msg=['Launching ', robot['name']]),
            LogInfo(
                condition=IfCondition(log_settings),
                msg=[robot['name'], ' map yaml: ', map_name_str]),
            LogInfo(
                condition=IfCondition(log_settings),
                msg=[robot['name'], ' params yaml: ', params_file]),
            LogInfo(
                condition=IfCondition(log_settings),
                msg=[robot['name'], ' rviz config file: ', rviz_config_file]),
            LogInfo(
                condition=IfCondition(log_settings),
                msg=[robot['name'], ' using robot state pub: ', use_robot_state_pub]),
            LogInfo(
                condition=IfCondition(log_settings),
                msg=[robot['name'], ' autostart: ', autostart])
        ])

        nav_instances_cmds.append(group)

    nav_instances_cmds.append(declare_simulator_cmd)
    nav_instances_cmds.append(declare_map_yaml_cmd)
    nav_instances_cmds.append(declare_rviz_config_file_cmd)

    # Add the actions to start gazebo, robots and simulations
    nav_instances_cmds.append(start_gazebo_cmd)

    return nav_instances_cmds

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_robot_state_pub',default_value='True',description='Whether to start the robot state publisher'),
        DeclareLaunchArgument('autostart', default_value='True',description='Automatically startup the stacks'),
        DeclareLaunchArgument('robot7_params_file',default_value=os.path.join(package_path, 'params', 'nav2_multirobot_params_7.yaml'),description='Full path to the ROS2 parameters file to use for robot3 launched nodes'),
        DeclareLaunchArgument('robot6_params_file',default_value=os.path.join(package_path, 'params', 'nav2_multirobot_params_6.yaml'),description='Full path to the ROS2 parameters file to use for robot3 launched nodes'),
        DeclareLaunchArgument('robot5_params_file',default_value=os.path.join(package_path, 'params', 'nav2_multirobot_params_5.yaml'),description='Full path to the ROS2 parameters file to use for robot3 launched nodes'),
        DeclareLaunchArgument('robot4_params_file',default_value=os.path.join(package_path, 'params', 'nav2_multirobot_params_4.yaml'),description='Full path to the ROS2 parameters file to use for robot3 launched nodes'),
        DeclareLaunchArgument('robot3_params_file',default_value=os.path.join(package_path, 'params', 'nav2_multirobot_params_3.yaml'),description='Full path to the ROS2 parameters file to use for robot3 launched nodes'),
        DeclareLaunchArgument('robot2_params_file',default_value=os.path.join(package_path, 'params', 'nav2_multirobot_params_2.yaml'),description='Full path to the ROS2 parameters file to use for robot3 launched nodes'),
        DeclareLaunchArgument('robot1_params_file',default_value=os.path.join(package_path, 'params', 'nav2_multirobot_params_1.yaml'),description='Full path to the ROS2 parameters file to use for robot2 launched nodes'),
        DeclareLaunchArgument('robot0_params_file',default_value=os.path.join(package_path, 'params', 'nav2_multirobot_params_0.yaml'),description='Full path to the ROS2 parameters file to use for robot1 launched nodes'),
        DeclareLaunchArgument('use_rviz',default_value='True',description='Whether to start RVIZ'),
        DeclareLaunchArgument('rviz_config',default_value=os.path.join(package_path, 'rviz', 'nav2_namespaced_view.rviz'),description='Full path to the RVIZ config file to use.'),
        DeclareLaunchArgument('map_path'),
        DeclareLaunchArgument('n_robots'),
        OpaqueFunction(function=launch_setup)
    ])

generate_launch_description()
