"""MotionCommand(조향 -7~7, 속도 -255~255) → Gazebo cmd_vel(planar_move: linear.x/y = 무게중심 속도, angular.z = 요레이트).

실차 구동 모델(값은 config/vehicle_dynamics.yaml, 실행 중 ros2 param set 으로 변경 가능).
Gazebo는 받은 속도를 그대로 적용하므로(planar_move) 차량 동역학은 전부 이 모델이 정한다.
요레이트 = v·tanδ_eff/L (뒷바퀴 축 기준 자전거 모델), 무게중심 횡속도 = 요레이트·com_x (뒷바퀴 축이 옆으로 안 미끄러지게).

  공통: 명령 지연(cmd_delay) → 무부하 바퀴 속도 표(speed_table, 데드존 포함) / 조향 속도 제한(steer_rate)
  slip_enable=false: 바퀴 속도 = 차체 속도(모터 1차 지연 motor_tau 만). 기본값은 이전 동작과 같다.
  slip_enable=true (모든 슬립):
    바퀴: m_w dv_w/dt = F_motor - F_t,   F_motor = k (v_noload(PWM) - v_w)   (DC 모터 토크-속도 직선)
          k = drive_force_max / 최대 무부하 속도,  m_w = motor_tau * k  (바퀴를 들고 잰 무부하 시상수)
    차체: m dv_b/dt = F_t - c_rr m g
    접지력 F_t = mu(s) * m g * drive_frac,  s = (v_w - v_b) / max(|v_w|, |v_b|, V_EPS)
      mu(s): 0~s_peak 직선 상승 → mu_peak, 이후 s=1 에서 mu_slide 까지 직선 감소(헛돌수록 접지 감소)
    커브: 곡률 = tanδ/L / (1 + understeer_k v²), 횡가속도 한계 g*sqrt(mu_lat² - mu_long²) (마찰원)로 자름
"""
import math
from collections import deque

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSDurabilityPolicy, QoSReliabilityPolicy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray
from interfaces_pkg.msg import MotionCommand

from simulation_pkg.config import SimulationSenderSettings as set

STEER = set.STEERING
DIRECT = set.DIRECTION
MAX_STEER = 0.45  # 명령 ±7 ↔ 바퀴 ±0.45rad
DT = 0.01         # 발행 주기(s, 시뮬 시간)
H_MAX = 0.001     # 슬립 모델 하위 스텝 최대 길이(s)
G = 9.81
V_EPS = 0.05      # 슬립 분모 하한(m/s): 정지 근처 0 나눗셈 방지

PARAMS = dict(cmd_delay=0.0, motor_tau=0.0, steer_rate=0.0, understeer_k=0.0, wheelbase=0.59, com_x=0.3,
              speed_table_cmd=[-255.0, 0.0, 255.0], speed_table_mps=[-5.0, 0.0, 5.0],
              slip_enable=False, mass=27.62, drive_frac=0.5, drive_force_max=138.0,
              mu_peak=0.6, s_peak=0.15, mu_slide=0.08, c_rr=0.03, mu_lat=0.6)


def mu_of(s, p):
    a = abs(s)
    if a <= p['s_peak']:
        m = p['mu_peak'] * a / p['s_peak']
    else:
        m = p['mu_peak'] + (p['mu_slide'] - p['mu_peak']) * min(1.0, (a - p['s_peak']) / (1.0 - p['s_peak']))
    return math.copysign(m, s)


def traction(v_w, v_b, p):
    return mu_of((v_w - v_b) / max(abs(v_w), abs(v_b), V_EPS), p) * p['mass'] * G * p['drive_frac']


def model_step(s, cmd, dt, p):
    """s=dict(v_wheel, v_body, steer, mu_long) 갱신 후 (차체 속도, 유효 조향각) 반환. cmd=(조향 rad, 속도 명령)"""
    target_steer, speed_cmd = cmd
    v_nl = float(np.interp(speed_cmd, p['speed_table_cmd'], p['speed_table_mps']))
    if p['steer_rate'] > 0:
        d = p['steer_rate'] * dt
        s['steer'] += max(-d, min(d, target_steer - s['steer']))
    else:
        s['steer'] = target_steer
    if not p['slip_enable']:
        a = 1.0 if p['motor_tau'] <= 0 else min(1.0, dt / p['motor_tau'])
        s['v_wheel'] += a * (v_nl - s['v_wheel'])
        s['v_body'], s['mu_long'] = s['v_wheel'], 0.0
    else:
        k = p['drive_force_max'] / max(abs(x) for x in p['speed_table_mps'])
        m_w = max(p['motor_tau'], 1e-3) * k
        n = max(1, math.ceil(dt / H_MAX))
        h = dt / n
        for _ in range(n):
            v_w, v_b = s['v_wheel'], s['v_body']
            f_t = traction(v_w, v_b, p)
            df = max(0.0, (traction(v_w + 1e-4, v_b, p) - f_t) / 1e-4)
            # 바퀴: 암시적(접지력 기울기가 커서 명시적 적분은 발산)
            s['v_wheel'] = v_w + h * (k * (v_nl - v_w) - f_t) / m_w / (1.0 + h * (k + df) / m_w)
            f_rr = p['c_rr'] * p['mass'] * G
            if abs(v_b) < 1e-3 and abs(f_t) <= f_rr:
                s['v_body'] = 0.0   # 구름저항(정지마찰)을 못 이기면 정지 유지
            else:
                nv = v_b + h * (f_t - math.copysign(f_rr, v_b if abs(v_b) >= 1e-3 else f_t)) / p['mass']
                s['v_body'] = 0.0 if v_b * nv < 0 else nv   # 구름저항이 방향을 뒤집지 않게
            s['mu_long'] = f_t / (p['mass'] * G)
    v = s['v_body']
    kappa = math.tan(s['steer']) / p['wheelbase'] / (1.0 + p['understeer_k'] * v * v)
    if p['slip_enable'] and abs(v) > 1e-3:
        a_lat = G * math.sqrt(max(0.0, p['mu_lat'] ** 2 - s['mu_long'] ** 2))
        kappa = math.copysign(min(abs(kappa), a_lat / (v * v)), kappa)
    return v, math.atan(kappa * p['wheelbase'])


def _run(p, cmd, secs):
    s = dict(v_wheel=0.0, v_body=0.0, steer=0.0, mu_long=0.0)
    out = [model_step(s, cmd, DT, p) + (s['v_wheel'],) for _ in range(int(round(secs / DT)))]
    return s, out


# 자체 점검
_p = dict(PARAMS)
assert model_step(dict(v_wheel=0.0, v_body=0.0, steer=0.0, mu_long=0.0), (0.2, 51), DT, _p) == (1.0, 0.2)  # 기본 = 이전 매핑
_p.update(slip_enable=True, motor_tau=0.15)
_s, _o = _run(_p, (0.0, 255), 6.0)
assert _o[149][0] < 0.5 and _o[149][2] > 3.0      # 최대 PWM 출발: 1.5초에 바퀴는 헛돌고(>3 m/s) 차체는 느림
assert _o[-1][0] > 3.0                            # 그 뒤 접지가 회복되며 출발
_s, _o = _run(_p, (0.0, 51), 6.0)
assert 0.0 < (_o[-1][2] - _o[-1][0]) / _o[-1][2] < _p['s_peak']   # 정속 직진: 작은 정상 슬립
_s, _o = _run(_p, (0.45, 255), 6.0)
assert abs(_o[-1][1]) < 0.45                      # 고속 급커브: 횡 접지 한계로 유효 조향 감소


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
        self.publisher = self.create_publisher(Twist, self.get_parameter('pub_topic').value, qos)
        # 실차 엔코더·IMU와 비교용: [바퀴 속도, 차체 속도, 슬립률, 유효 조향각, 종방향 mu]
        self.debug_pub = self.create_publisher(Float32MultiArray, 'sim_drive_state', 10)
        self.queue = deque()            # (수신 시각 s, (조향 rad, 속도 명령))
        self.cmd = (0.0, 0.0)           # 지연이 지나 적용 중인 명령
        self.state = dict(v_wheel=0.0, v_body=0.0, steer=0.0, mu_long=0.0)
        self.last_t = None
        self.create_timer(DT, self.step)

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def data_callback(self, m):
        steer = (STEER * m.steering) / 7.0 * MAX_STEER
        self.queue.append((self.now(), (steer, float(DIRECT * m.left_speed))))

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
        v, steer = model_step(self.state, self.cmd, dt, p)
        msg = Twist()
        yaw_rate = v * math.tan(steer) / p['wheelbase']
        msg.linear.x = float(v)
        msg.linear.y = float(yaw_rate * p['com_x'])
        msg.angular.z = float(yaw_rate)
        self.publisher.publish(msg)
        s = self.state
        slip = (s['v_wheel'] - v) / max(abs(s['v_wheel']), abs(v), V_EPS)
        self.debug_pub.publish(Float32MultiArray(data=[float(s['v_wheel']), float(v), float(slip), float(steer), float(s['mu_long'])]))

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
