"""MotionCommand(조향 -7~7, 속도 -255~255) → Gazebo cmd_vel(planar_move: linear.x/y = 무게중심 속도, angular.z = 요레이트).

실차 구동 모델(값은 config/vehicle_dynamics.yaml, 실행 중 ros2 param set 으로 변경 가능).
Gazebo는 받은 속도를 그대로 적용하므로(planar_move) 차량 동역학은 전부 이 모델이 정한다.
slip_enable=false: 요레이트 = v·tanδ/L/(1+understeer_k v²), 무게중심 횡속도 = 요레이트·com_x (뒷바퀴 축이 옆으로 안 미끄러짐).

  공통: 명령 지연(cmd_delay) → 무부하 바퀴 속도 표(speed_table, 데드존 포함) / 조향 속도 제한(steer_rate)
  slip_enable=false: 바퀴 속도 = 차체 속도(모터 1차 지연 motor_tau 만). 기본값은 이전 동작과 같다.
  slip_enable=true (모든 슬립):
    바퀴: m_w dv_w/dt = F_motor - F_t,   F_motor = k (v_noload(PWM) - v_w)   (DC 모터 토크-속도 직선)
          k = drive_force_max / 최대 무부하 속도,  m_w = motor_tau * k  (바퀴를 들고 잰 무부하 시상수)
    차체: m dv_b/dt = F_t - c_rr m g
    접지력 F_t = mu(s) * m g * drive_frac,  s = (v_w - v_b) / max(|v_w|, |v_b|, V_EPS)
      mu(s): 0~s_peak 직선 상승 → mu_peak, 이후 s=1 에서 mu_slide 까지 직선 감소(헛돌수록 접지 감소)
    횡(동적 자전거 모델, 0.3 m/s 이상): 앞·뒤 슬립각 → 횡력(alpha_peak까지 직선, mu*N 포화)
      뒷바퀴 횡 접지 = sqrt(mu_lat² - mu_long²) (구동력 쓴 만큼 감소) → 커브 가속 시 뒤가 밀림(오버스티어)
      상태: 종속도, 무게중심 횡속도, 요레이트 → planar_move 로 그대로 전달
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
V_DYN = 0.3       # 이 속도 미만은 운동학 횡 모델(동적 모델 슬립각이 불안정)

PARAMS = dict(cmd_delay=0.0, motor_tau=0.0, steer_rate=0.0, understeer_k=0.0, wheelbase=0.59, com_x=0.3,
              speed_table_cmd=[-255.0, 0.0, 255.0], speed_table_mps=[-5.0, 0.0, 5.0],
              slip_enable=False, mass=27.62, drive_frac=0.5, drive_force_max=138.0,
              mu_peak=0.6, s_peak=0.15, mu_slide=0.08, c_rr=0.03, mu_lat=0.6,
              alpha_peak=0.1, yaw_inertia=2.4)


def mu_of(s, p):
    a = abs(s)
    if a <= p['s_peak']:
        m = p['mu_peak'] * a / p['s_peak']
    else:
        m = p['mu_peak'] + (p['mu_slide'] - p['mu_peak']) * min(1.0, (a - p['s_peak']) / (1.0 - p['s_peak']))
    return math.copysign(m, s)


def traction(v_w, v_b, p):
    return mu_of((v_w - v_b) / max(abs(v_w), abs(v_b), V_EPS), p) * p['mass'] * G * p['drive_frac']


def lat_force(alpha, mu, n_load, p):
    """횡력: 슬립각 alpha_peak 까지 직선, 이후 mu*N 에서 포화"""
    return mu * n_load * max(-1.0, min(1.0, alpha / p['alpha_peak']))


def model_step(s, cmd, dt, p):
    """s=dict(v_wheel, v_body, v_y, r, steer, mu_long) 갱신 후 (종속도, 무게중심 횡속도, 요레이트) 반환.
    cmd=(조향 rad, 속도 명령)"""
    target_steer, speed_cmd = cmd
    v_nl = float(np.interp(speed_cmd, p['speed_table_cmd'], p['speed_table_mps']))
    if p['steer_rate'] > 0:
        d = p['steer_rate'] * dt
        s['steer'] += max(-d, min(d, target_steer - s['steer']))
    else:
        s['steer'] = target_steer
    L, b = p['wheelbase'], p['com_x']
    if not p['slip_enable']:
        a = 1.0 if p['motor_tau'] <= 0 else min(1.0, dt / p['motor_tau'])
        s['v_wheel'] += a * (v_nl - s['v_wheel'])
        v = s['v_body'] = s['v_wheel']
        s['mu_long'] = 0.0
        s['r'] = v * math.tan(s['steer']) / L / (1.0 + p['understeer_k'] * v * v)
        s['v_y'] = s['r'] * b
        return v, s['v_y'], s['r']
    m, delta = p['mass'], s['steer']
    n_r = m * G * p['drive_frac']
    n_f = m * G - n_r
    k = p['drive_force_max'] / max(abs(x) for x in p['speed_table_mps'])
    m_w = max(p['motor_tau'], 1e-3) * k
    n = max(1, math.ceil(dt / H_MAX))
    h = dt / n
    for _ in range(n):
        # 종방향: 뒷바퀴(구동) 슬립
        v_w, v_b = s['v_wheel'], s['v_body']
        f_t = traction(v_w, v_b, p)
        df = max(0.0, (traction(v_w + 1e-4, v_b, p) - f_t) / 1e-4)
        s['v_wheel'] = v_w + h * (k * (v_nl - v_w) - f_t) / m_w / (1.0 + h * (k + df) / m_w)  # 암시적(기울기 큼)
        f_rr = p['c_rr'] * m * G
        if abs(v_b) < 1e-3 and abs(f_t) <= f_rr:
            s['v_body'] = 0.0   # 구름저항(정지마찰)을 못 이기면 정지 유지
        else:
            nv = v_b + h * (f_t - math.copysign(f_rr, v_b if abs(v_b) >= 1e-3 else f_t) + m * s['v_y'] * s['r']) / m
            s['v_body'] = 0.0 if v_b * nv < 0 else nv   # 구름저항이 방향을 뒤집지 않게
        s['mu_long'] = f_t / n_r
        # 횡방향: 동적 자전거 모델(앞·뒤 슬립각 → 횡력, 뒷바퀴는 구동력만큼 횡 접지 감소 = 마찰원)
        vx = s['v_body']
        if vx < V_DYN:   # 저속·후진: 운동학(슬립각 계산이 불안정)
            s['r'] = vx * math.tan(delta) / L
            s['v_y'] = s['r'] * b
            continue
        a_f = L - b
        alpha_f = delta - math.atan((s['v_y'] + a_f * s['r']) / vx)
        alpha_r = -math.atan((s['v_y'] - b * s['r']) / vx)
        f_yf = lat_force(alpha_f, p['mu_lat'], n_f, p)
        f_yr = lat_force(alpha_r, math.sqrt(max(0.0, p['mu_lat'] ** 2 - s['mu_long'] ** 2)), n_r, p)
        v_y, r = s['v_y'], s['r']
        s['v_y'] = v_y + h * ((f_yf * math.cos(delta) + f_yr) / m - vx * r)
        s['r'] = r + h * (a_f * f_yf * math.cos(delta) - b * f_yr) / p['yaw_inertia']
    return s['v_body'], s['v_y'], s['r']


def new_state():
    return dict(v_wheel=0.0, v_body=0.0, v_y=0.0, r=0.0, steer=0.0, mu_long=0.0)


def _run(p, cmd, secs, s=None):
    s = s or new_state()
    out = [model_step(s, cmd, DT, p) + (s['v_wheel'],) for _ in range(int(round(secs / DT)))]
    return s, out


def _rear_alpha(o, p):
    vx, vy, r, _ = o
    return -math.atan((vy - p['com_x'] * r) / vx)


# 자체 점검(슬립 값은 추정 시작값 기준)
_p = dict(PARAMS)
_o = model_step(new_state(), (0.2, 51), DT, _p)
assert _o[0] == 1.0 and abs(_o[2] - math.tan(0.2) / 0.59) < 1e-9 and abs(_o[1] - _o[2] * 0.3) < 1e-9  # 기본 = 이전 동작
_p.update(slip_enable=True, motor_tau=0.15)
_s, _o = _run(_p, (0.0, 255), 6.0)
assert _o[149][0] < 0.5 and _o[149][3] > 3.0      # 최대 PWM 출발: 1.5초에 바퀴는 헛돌고 차체는 느림
assert _o[-1][0] > 3.0                            # 그 뒤 접지가 회복되며 출발
_s, _o = _run(_p, (0.0, 51), 6.0)
assert 0.0 < (_o[-1][3] - _o[-1][0]) / _o[-1][3] < _p['s_peak']   # 정속 직진: 작은 정상 슬립
_s, _ = _run(_p, (0.0, 153), 5.0)
_s, _o = _run(_p, (0.3, 153), 3.0, _s)
assert _rear_alpha(_o[-1], _p) < _p['alpha_peak']                 # 고속 커브, 가속 없음: 뒷바퀴 접지 유지
_s, _ = _run(_p, (0.0, 102), 5.0)
_s, _ = _run(_p, (0.3, 102), 2.0, _s)
_s, _o = _run(_p, (0.3, 255), 2.0, _s)
assert _rear_alpha(_o[-1], _p) > _p['alpha_peak'] and math.atan2(_o[-1][1], _o[-1][0]) < 0  # 커브 중 급가속: 뒤가 밀림

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
        # 실차 엔코더·IMU와 비교용: [바퀴 속도, 종속도, 종슬립률, 조향각, 뒷바퀴 종 mu, 횡속도(무게중심), 요레이트, 차체 슬립각]
        self.debug_pub = self.create_publisher(Float32MultiArray, 'sim_drive_state', 10)
        self.queue = deque()            # (수신 시각 s, (조향 rad, 속도 명령))
        self.cmd = (0.0, 0.0)           # 지연이 지나 적용 중인 명령
        self.state = new_state()
        self.last_t = None
        self.create_timer(DT, self.step)

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def data_callback(self, m):
        steer = (STEER * m.steering) / 7.0 * MAX_STEER
        # 실차: 뒷바퀴 좌우가 각각 다른 모터. shortcut: 좌우 PWM 평균 한 바퀴로 계산, 좌우 차(전자 디퍼렌셜)로 인한 요 모멘트는 무시 — 좌우 실측이 크게 다르면 바퀴별 모델로 확장
        self.queue.append((self.now(), (steer, float(DIRECT * (m.left_speed + m.right_speed) / 2.0))))

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
        v, v_y, r = model_step(self.state, self.cmd, dt, p)
        msg = Twist()
        msg.linear.x, msg.linear.y, msg.angular.z = float(v), float(v_y), float(r)
        self.publisher.publish(msg)
        s = self.state
        slip = (s['v_wheel'] - v) / max(abs(s['v_wheel']), abs(v), V_EPS)
        beta = math.atan2(v_y, v) if abs(v) > 1e-3 else 0.0
        self.debug_pub.publish(Float32MultiArray(data=[float(x) for x in (s['v_wheel'], v, slip, s['steer'], s['mu_long'], v_y, r, beta)]))

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
