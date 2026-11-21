#!/usr/bin/env python3
"""CLE53 + 실제 크기 트랙 단독 테스트 런치.

실행: ros2 launch simulation_pkg cle53_test.launch.py [world:=track_real_night.world]
- Gazebo(track_real.world) + CLE53 스폰 + sim_simulation_sender_node 만 띄운다.
- 주최 측 YOLO/경로 노드는 Prius 시점으로 학습돼 있어 여기서는 띄우지 않는다.
- 스폰 위치는 원본 driving_ego() 좌표를 ÷3.5 하고, 뒷바퀴 기준으로 0.3m 뒤로 옮긴 추정값이다.
  Gazebo에서 출발선과 차로 중앙에 맞는지 보고 SPAWN 값을 고칠 것.
"""
import os
import re
import subprocess
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction, DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

SPAWN = dict(x=-1.03, y=-6.49, z=0.01, yaw=-0.007)  # [추정] base_link(뒷바퀴 접지 중심) 위치, 전진 방향 +x 기준 yaw


def launch_setup(context):
    subprocess.run(['killall', 'gzserver'])
    subprocess.run(['killall', 'gzclient'])
    share = get_package_share_directory('simulation_pkg')
    world_file = os.path.join(share, 'worlds', LaunchConfiguration('world').perform(context))
    model_file = os.path.join(share, 'models', 'cle53', 'model.sdf')

    return [
        # gazebo_ros_init: /clock 발행 → sender의 지연·모터 모델이 시뮬 시간으로 돈다(실시간보다 느려도 실차와 같은 시간 척도)
        ExecuteProcess(cmd=['gazebo', '--verbose', world_file, '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so',
                            '--ros-args', '--params-file', os.path.join(share, 'config', 'gazebo_params.yaml')],  # /clock 100Hz (-p는 gazebo 로그재생 옵션과 겹침)
                       output='screen'),
        TimerAction(period=5.0, actions=[
            Node(package='gazebo_ros', executable='spawn_entity.py', output='screen',
                 arguments=['-file', model_file, '-entity', 'ego_vehicle',
                            '-x', str(SPAWN['x']), '-y', str(SPAWN['y']), '-z', str(SPAWN['z']),
                            '-Y', str(SPAWN['yaw'])]),
        ]),
        Node(package='simulation_pkg', executable='sim_simulation_sender_node', output='screen',
             parameters=[os.path.join(share, 'config', 'vehicle_dynamics.yaml'), {'use_sim_time': True}]),
        # 시뮬 전용: Gazebo camera/image_sim → 역광·자동노출 → camera/image_raw(실차 카메라와 같은 토픽)
        Node(package='simulation_pkg', executable='camera_sim_node', output='screen', parameters=[sun_params(world_file)]),
    ] + viewer(context)


def sun_params(world_file):
    """월드의 sun_light 방향·색 → camera_sim_node 역광 초기값"""
    m = re.search(r'<light name="sun_light".*?</light>', open(world_file).read(), re.S)
    v = lambda t: [float(x) for x in re.search(f'<{t}>([^<]+)</{t}>', m.group(0)).group(1).split()[:3]]
    return dict(sun_dir=v('direction'), sun_diffuse=v('diffuse'))


# 카메라 창(전방/후방/위성): 주최 측 런치가 rqt로 카메라를 보여주던 것과 같은 역할. viewer:=false 로 끔
VIEWER_SCRIPT = os.path.expanduser('~/cle53_expert/cam_viewer.py')


def viewer(context):
    if LaunchConfiguration('viewer').perform(context).lower() != 'true':
        return []
    if not os.path.exists(VIEWER_SCRIPT):
        print(f'[cle53_test] 카메라 뷰어 없음, 건너뜀: {VIEWER_SCRIPT}')
        return []
    # Gazebo와 차량이 뜬 뒤(스폰 5초 후) 실행
    return [TimerAction(period=7.0, actions=[ExecuteProcess(cmd=['python3', VIEWER_SCRIPT], output='screen')])]


def generate_launch_description():
    # 시간대 선택: track_real.world(오후) / track_real_morning / _noon / _cloudy / _night .world
    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='track_real.world'),
        DeclareLaunchArgument('viewer', default_value='true', description='카메라 창(전방/후방/위성) 띄우기'),
        OpaqueFunction(function=launch_setup),
    ])
