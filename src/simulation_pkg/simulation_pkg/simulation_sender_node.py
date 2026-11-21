"""MotionCommand(조향 -7~7, 좌·우 PWM -255~255) → 차량 동역학 모델(vehicle_model.py) → Gazebo cmd_vel
(planar_move: linear.x/y = 무게중심 속도, angular.z = 요레이트). Gazebo는 받은 속도를 그대로 적용하므로 동역학은 전부 모델이 정한다.

실차와 같은 규격의 센서 토픽도 발행(시뮬에서 기록기·보정 스크립트를 그대로 시험하기 위해):
  encoder/wheel_speeds (Float32MultiArray): 바퀴 둘레 속도 m/s [앞왼, 앞오, 뒤왼, 뒤오]
  imu/data (sensor_msgs/Imu): angular_velocity.z = 요레이트, linear_acceleration.x/y = 차체 가속도(중력 제외)
  sim_drive_state (Float32MultiArray, 시뮬 전용 진단): [뒤왼 슬립, 뒤오 슬립, 조향각, 횡속도, 요레이트, 차체 슬립각, 뒤왼 하중, 뒤오 하중]
값: config/vehicle_dynamics.yaml, 실행 중 ros2 param set /simulation_sender_node <이름> <값>.
"""
import math
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSDurabilityPolicy, QoSReliabilityPolicy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32MultiArray
from interfaces_pkg.msg import MotionCommand

from simulation_pkg.config import SimulationSenderSettings as set
from simulation_pkg.vehicle_model import PARAMS, model_step, new_state, sensors

STEER = set.STEERING
DIRECT = set.DIRECTION
MAX_STEER = 0.45  # 명령 ±7 ↔ 바퀴 ±0.45rad
DT = 0.01         # 발행 주기(s, 시뮬 시간)


class MotorControlNode(Node):
    def __init__(self):
        super().__init__('simulation_sender_node')
        self.declare_parameter('sub_topic', set.MOTION_PLANNER_TOPIC)
        self.declare_parameter('pub_topic', set.GAZEBO_CONTROL_TOPIC)
        for k, v in PARAMS.items():
            self.declare_parameter(k, v)
        qos = QoSProfile(reliability=QoSReliabilityPolicy.RELIABLE, history=QoSHistoryPolicy.KEEP_LAST,
                         durability=QoSDurabilityPolicy.VOLATILE, depth=1)
        self.create_subscription(MotionCommand, self.get_parameter('sub_topic').value, self.data_callback, qos)
        self.create_subscription(Odometry, 'ego_pose', self.pose_callback, 10)
        self.publisher = self.create_publisher(Twist, self.get_parameter('pub_topic').value, qos)
        self.enc_pub = self.create_publisher(Float32MultiArray, 'encoder/wheel_speeds', 10)
        self.imu_pub = self.create_publisher(Imu, 'imu/data', 10)
        self.debug_pub = self.create_publisher(Float32MultiArray, 'sim_drive_state', 10)
        self.queue = deque()            # (수신 시각 s, (조향 rad, PWM 좌, PWM 우))
        self.cmd = (0.0, 0.0, 0.0)      # 지연이 지나 적용 중인 명령
        self.state = new_state()
        self.pose = None                # (x, y, yaw) 뒷축 중심, 바닥 마찰 배율용
        self.last_t = None
        self.create_timer(DT, self.step)

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def data_callback(self, m):
        steer = (STEER * m.steering) / 7.0 * MAX_STEER
        self.queue.append((self.now(), (steer, float(DIRECT * m.left_speed), float(DIRECT * m.right_speed))))

    def pose_callback(self, m):
        q = m.pose.pose.orientation
        self.pose = (m.pose.pose.position.x, m.pose.pose.position.y,
                     math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))

    def step(self):
        p = {k: self.get_parameter(k).value for k in PARAMS}
        t = self.now()
        # 시뮬 시간 타이머는 /clock 갱신 때만 깨므로 실제 경과 시간으로 적분(고정 DT 가정 금지)
        dt = DT if self.last_t is None else min(t - self.last_t, 0.1)
        if dt <= 0:
            return
        self.last_t = t
        while self.queue and self.queue[0][0] <= t - p['cmd_delay']:
            self.cmd = self.queue.popleft()[1]
        v, v_y, r = model_step(self.state, self.cmd, dt, p, self.pose)
        msg = Twist()
        msg.linear.x, msg.linear.y, msg.angular.z = float(v), float(v_y), float(r)
        self.publisher.publish(msg)
        wheels, yaw_rate, ax, ay = sensors(self.state, p)
        self.enc_pub.publish(Float32MultiArray(data=[float(w) for w in wheels]))
        imu = Imu()
        imu.header.stamp = self.get_clock().now().to_msg()
        imu.header.frame_id = 'imu_link'
        imu.angular_velocity.z = float(yaw_rate)
        imu.linear_acceleration.x, imu.linear_acceleration.y = float(ax), float(ay)
        self.imu_pub.publish(imu)
        s = self.state
        beta = math.atan2(v_y, v) if abs(v) > 1e-3 else 0.0
        self.debug_pub.publish(Float32MultiArray(data=[float(x) for x in (
            s['s_l'], s['s_r'], s['steer'], v_y, r, beta, s['n_l'], s['n_r'])]))

    def stop_cmd(self):
        self.publisher.publish(Twist())
        self.get_logger().error("\n\nRobot stopped\n\n")


def main(args=None):
    rclpy.init(args=args)
    node = MotorControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.stop_cmd()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
