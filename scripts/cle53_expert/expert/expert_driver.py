#!/usr/bin/env python3
"""정답 주행(expert): 시뮬 실제 위치(/ego_pose, 뒷바퀴축) + 텍스처에서 뽑은 2차선 중심선으로 pure pursuit.
워크스페이스 인식/판단 코드를 쓰지 않음. 출력은 motion_planner와 같은 MotionCommand(조향 -7~7, 속도 0~255)라
DAgger 라벨로 그대로 쓴다. 시뮬 전용(실차에는 /ego_pose가 없음).
파라미터: speed(직선 속도), min_speed(급커브 속도), lookahead_k/lookahead_min, path(npy)"""
import math, os
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from nav_msgs.msg import Odometry
from interfaces_pkg.msg import MotionCommand

WHEELBASE = 0.59          # CLE53 축간거리(m)
MAX_STEER = 0.45          # 명령 ±7 ↔ ±0.45rad (simulation_sender_node.map_to_steer)
STEER_SIGN = -1           # config driving 모드 STEERING=-1: 명령 음수 = 좌회전
CMD_TO_MPS = 5.0 / 255    # simulation_sender: speed/255 * MAX_SPEED(5)


def steer_cmd(delta):
    """조향각(rad, +좌) → 정수 명령"""
    return int(max(-7, min(7, round(STEER_SIGN * delta / MAX_STEER * 7))))


assert steer_cmd(0.45) == -7 and steer_cmd(-0.45) == 7 and steer_cmd(0.0) == 0


class Expert(Node):
    def __init__(self):
        super().__init__('expert_driver')
        here = os.path.dirname(os.path.abspath(__file__))
        self.path = np.load(self.declare_parameter('path', os.path.join(here, 'lane2_center.npy')).value)
        self.speed = self.declare_parameter('speed', 100).value
        self.min_speed = self.declare_parameter('min_speed', 60).value
        self.k = self.declare_parameter('lookahead_k', 0.35).value       # 속도(m/s)당 전방주시 거리(s)
        self.ld_min = self.declare_parameter('lookahead_min', 0.45).value
        self.n = len(self.path); self.idx = None; self.pose = None; self.cur_speed = 0.0
        seg = np.diff(np.vstack([self.path, self.path[:1]]), axis=0)
        self.ds = float(np.mean(np.hypot(seg[:, 0], seg[:, 1])))
        # 곡률(헤딩 변화/거리) → 앞쪽 최대 곡률로 감속
        hd = np.unwrap(np.arctan2(seg[:, 1], seg[:, 0]))
        self.curv = np.abs(np.gradient(hd)) / self.ds
        self.pub = self.create_publisher(MotionCommand, 'topic_control_signal', QoSProfile(depth=1))
        self.create_subscription(Odometry, 'ego_pose', self.cb, 10)
        self.create_timer(0.05, self.step)

    def cb(self, m):
        p = m.pose.pose; o = p.orientation
        yaw = math.atan2(2 * (o.w * o.z + o.x * o.y), 1 - 2 * (o.y ** 2 + o.z ** 2))
        self.pose = (p.position.x, p.position.y, yaw)
        self.cur_speed = math.hypot(m.twist.twist.linear.x, m.twist.twist.linear.y)

    def step(self):
        if self.pose is None: return
        x, y, yaw = self.pose
        # 최근접점: 처음엔 전체, 이후엔 앞쪽 창에서만(트랙이 가까이 붙는 구간에서 건너뛰기 방지)
        if self.idx is None: cand = np.arange(self.n)
        else: cand = (self.idx + np.arange(-5, 60)) % self.n
        d = np.hypot(self.path[cand, 0] - x, self.path[cand, 1] - y)
        self.idx = int(cand[np.argmin(d)])
        # 속도: 앞 1.5m 안 최대 곡률로 직선 speed~min_speed 사이 보간
        ahead = (self.idx + np.arange(int(1.5 / self.ds))) % self.n
        c = float(np.max(self.curv[ahead]))
        tgt = self.speed - (self.speed - self.min_speed) * min(1.0, c / 1.0)  # 곡률 1(반경 1m) 이상이면 min_speed
        ld = max(self.ld_min, self.k * max(self.cur_speed, tgt * CMD_TO_MPS))
        tp = self.path[(self.idx + int(ld / self.ds)) % self.n]
        alpha = math.atan2(tp[1] - y, tp[0] - x) - yaw
        delta = math.atan2(2 * WHEELBASE * math.sin(alpha), ld)
        msg = MotionCommand(); msg.steering = steer_cmd(delta)
        msg.left_speed = msg.right_speed = int(tgt)
        self.pub.publish(msg)


def main():
    rclpy.init(); n = Expert()
    try: rclpy.spin(n)
    except KeyboardInterrupt: pass
    finally:
        if rclpy.ok():
            n.pub.publish(MotionCommand())  # 종료 시 정지(Ctrl+C로 컨텍스트가 이미 닫혔으면 생략)
            rclpy.shutdown()


if __name__ == '__main__':
    main()
