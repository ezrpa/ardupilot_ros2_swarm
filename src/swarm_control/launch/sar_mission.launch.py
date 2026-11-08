#!/usr/bin/env python3
"""Search-and-rescue mission: N SearchDrones + optional TAK bridge.

Selected by SWARM_MISSION=sar (start_swarm_controller.sh). Pair with the
Gazebo world swarm_drone_sar.sdf (missing_person model, 4 camera drones):

    SWARM_WORLD=swarm_drone_sar.sdf ./start_gazebo.sh &
    SWARM_MISSION=sar TAK_URL=udp://239.2.3.1:6969 USE_GAZEBO=1 ./start_swarm.sh 4
"""
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, LogInfo,
                            OpaqueFunction, TimerAction)
from launch_ros.actions import Node
import json
import os

from swarm_control.core import config
from swarm_control.mission.points_distributor import generate_grid_waypoints, split_serpentine

FIELD_ORIGIN_LAT = config.FIELD_ORIGIN_LAT
FIELD_ORIGIN_LON = config.FIELD_ORIGIN_LON

# Search area: SEARCH_AREA_M square centred on the field origin. Must match
# generate_missing_person.py --area. 7x7 grid -> 20 m spacing; the camera
# footprint at SEARCH_ALT is ~41 x 31 m (HFOV 1.2 rad, 4:3), so adjacent
# frames overlap and the area is fully covered.
SEARCH_AREA_M = 120.0
SEARCH_GRID = 7
SEARCH_ALT = 30.0
CONFIRM_ALT = 15.0

LIB = '/ros2_ws/install/swarm_control/lib/swarm_control/'


def launch_setup(context, *args, **kwargs):
    num_drones = int(context.launch_configurations['num_drones'])
    rmw = context.launch_configurations.get('rmw_implementation',
                                            'rmw_fastrtps_cpp')
    instances = list(range(num_drones))
    # bfs / dfs: coordinated grid search from separate launch points
    # (SPAWN_LAYOUT=corners); strips: pre-assigned serpentine (original)
    search_algo = os.environ.get('SAR_SEARCH', 'strips') or 'strips'

    wps = generate_grid_waypoints(field_size=SEARCH_AREA_M,
                                  grid_points=SEARCH_GRID, height=SEARCH_ALT)
    chunks = split_serpentine(wps, num_drones)

    actions = [LogInfo(msg=f"SAR: {num_drones} search drones, {len(wps)} "
                           f"waypoints over {SEARCH_AREA_M:.0f} m square, "
                           f"search={search_algo}")]
    for i in instances:
        node = Node(
            package='swarm_control',
            executable='search_drone_dds.py',
            name=f'drone{config.sysid(i)}',
            output='screen',
            respawn=True,          # EKF/GPS warmup races, as in mapping
            respawn_delay=5.0,
            parameters=[{
                'drone_id': config.sysid(i),
                'instance': i,
                'swarm_instances': ' '.join(str(j) for j in instances),
                'assigned_waypoints': json.dumps(chunks[i]),
                'field_origin_lat': FIELD_ORIGIN_LAT,
                'field_origin_lon': FIELD_ORIGIN_LON,
                'camera_port': config.camera_udp_port(i),
                'logs_dir': '/root/logs',
                'search_alt': SEARCH_ALT,
                'confirm_alt': CONFIRM_ALT,
                'search_algo': search_algo,
                'search_area_m': SEARCH_AREA_M,
                'search_grid': SEARCH_GRID,
            }],
            additional_env={'ROS_DOMAIN_ID': str(config.ros_domain_id(i)),
                            'RMW_IMPLEMENTATION': rmw},
        )
        actions.append(TimerAction(period=i * 5.0, actions=[node]))

    tak_urls = context.launch_configurations.get('tak_url', '').split()
    if tak_urls:
        actions.append(ExecuteProcess(
            cmd=[LIB + 'tak_bridge.py', '--mission', 'sar',
                 '--url', *tak_urls, '--logs', '/root/logs',
                 '--instances', *[str(i) for i in instances],
                 '--sprayers', '--field-half', str(SEARCH_AREA_M / 2)],
            output='screen', name='tak_bridge',
            respawn=True, respawn_delay=5.0))
        actions.append(LogInfo(msg=f"TAK bridge enabled -> {tak_urls}"))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('num_drones', default_value='4'),
        DeclareLaunchArgument('rmw_implementation',
                              default_value='rmw_fastrtps_cpp'),
        DeclareLaunchArgument('tak_url', default_value='',
                              description='Space-separated CoT sinks; empty '
                                          '= TAK bridge disabled'),
        OpaqueFunction(function=launch_setup),
    ])
