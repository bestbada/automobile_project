#!/usr/bin/env python3
"""/ego_pose로 반시계 바퀴 수를 센다. 트랙 중심(원점) 기준 위치각 누적(제자리 회전은 0으로 상쇄).
종료: 2바퀴 완주 / 20초간 진행 5도 미만(정체·이탈) / 역주행 -30도 / 제한시간.
결과 한 줄 JSON을 stdout에, 궤적 CSV를 argv[1]에 쓴다."""
import json, math, sys, time
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import numpy as np, os
CL = np.load(os.path.join(os.path.dirname(os.path.abspath(__file__)), 'expert', 'lane2_center.npy'))
# 규정 3.1.5 근사(타이어 폭 60mm 가정): 바퀴 ±0.22m, 차선 안쪽 가장자리 0.405m, 바깥 가장자리 0.455m
TOUCH, DEPART = 0.155, 0.205


def lat_err(x, y):
    return float(np.min(np.hypot(CL[:, 0] - x, CL[:, 1] - y)))

LAPS, STALL_S, STALL_DEG, TIMEOUT = 2, 20.0, 5.0, float(sys.argv[2]) if len(sys.argv) > 2 else 600


def unwrap_step(prev, cur):
    d = cur - prev
    return (d + math.pi) % (2 * math.pi) - math.pi


assert abs(unwrap_step(math.radians(179), math.radians(-179)) - math.radians(2)) < 1e-9


class Mon(Node):
    def __init__(self, csv):
        super().__init__('lap_monitor')
        self.csv = open(csv, 'w', buffering=1); self.csv.write('t,x,y,deg,lat\n')
        self.prev = None; self.total = 0.0; self.t0 = None; self.result = None
        self.hist = []  # (t, total)
        self.lap_times = []
        self.max_lat = 0.0; self.touches = 0; self.touching = False
        self.moving_t = None; self.last_msg = time.time(); self.start = time.time(); self.pos = (0, 0)
        self.create_subscription(Odometry, '/ego_pose', self.cb, 10)
        self.create_timer(1.0, self.watchdog)  # 메시지가 끊겨도 제한시간·정체 판정

    def watchdog(self):
        now = time.time(); deg = math.degrees(self.total)
        r = None
        if now - self.last_msg > 10: r = 'FAIL_no_pose'
        elif now - self.start > TIMEOUT: r = 'FAIL_timeout'
        elif self.moving_t is None and now - self.start > 60: r = 'FAIL_never_moved'
        if r and self.result is None:
            self.result = dict(result=r, laps=round(deg / 360, 2), t=round(now - self.start, 1),
                               lap_times=self.lap_times, x=round(self.pos[0], 2), y=round(self.pos[1], 2), max_lat=round(self.max_lat, 3), touches=self.touches)

    def cb(self, m):
        p = m.pose.pose.position; now = time.time(); self.last_msg = now; self.pos = (p.x, p.y)
        o = m.pose.pose.orientation; yaw = math.atan2(2 * (o.w * o.z + o.x * o.y), 1 - 2 * (o.y ** 2 + o.z ** 2))
        lat = max(lat_err(p.x, p.y), lat_err(p.x + 0.59 * math.cos(yaw), p.y + 0.59 * math.sin(yaw)))  # 뒷축·앞축
        self.max_lat = max(self.max_lat, lat)
        if lat > TOUCH and not self.touching: self.touches += 1
        self.touching = lat > TOUCH
        a = math.atan2(p.y, p.x)
        if self.prev is None:
            self.prev, self.t0 = a, now; return
        self.total += unwrap_step(self.prev, a); self.prev = a
        t = now - self.t0; deg = math.degrees(self.total)
        self.csv.write(f'{t:.2f},{p.x:.3f},{p.y:.3f},{deg:.1f},{lat:.3f}\n')
        if len(self.lap_times) < int(deg // 360) and deg > 0:
            self.lap_times.append(round(t, 1))
        if self.moving_t is None and math.hypot(p.x + 1.03, p.y + 6.49) > 0.2: self.moving_t = t
        self.hist.append((t, deg)); self.hist = [h for h in self.hist if t - h[0] <= STALL_S]
        r = None
        if lat > DEPART: r = 'FAIL_depart'
        elif deg >= 360 * LAPS: r = 'PASS'
        elif deg < -30: r = 'FAIL_reverse'
        elif self.moving_t is not None and t - self.moving_t > STALL_S and deg - self.hist[0][1] < STALL_DEG: r = 'FAIL_stall'
        if r and self.result is None:
            self.result = dict(result=r, laps=round(deg / 360, 2), t=round(t, 1), lap_times=self.lap_times,
                               x=round(p.x, 2), y=round(p.y, 2), max_lat=round(self.max_lat, 3), touches=self.touches)


rclpy.init(); n = Mon(sys.argv[1])
while rclpy.ok() and n.result is None:
    rclpy.spin_once(n, timeout_sec=1.0)
n.csv.close(); print(json.dumps(n.result)); rclpy.shutdown()
