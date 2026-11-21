"""CLE53 차량 동역학 모델(ROS 비의존). simulation_sender_node(시뮬)와 scripts/calibration(실측 보정)이 같이 쓴다.

입력: 조향 목표(rad, +좌), 뒷바퀴 좌·우 PWM(-255~255). 출력: 종속도 v_x, 무게중심 횡속도 v_y, 요레이트 r (차체 좌표).
값은 src/simulation_pkg/config/vehicle_dynamics.yaml.

slip_enable=false: 바퀴 = 차체(모터 1차 지연만), 운동학 자전거 모델. 이전 시뮬 동작.
slip_enable=true (모든 슬립):
  뒷바퀴 좌·우 각각: 모터(DC 토크-속도 직선) ↔ 바퀴 관성 ↔ 접지력 mu(s)·N·바닥배율
    제동: brake_mode=coast(PWM 0이면 모터 힘 0, 같은 방향 PWM은 감속 못 함, 역PWM은 역전 제동) / brake
  하중 이동: 앞뒤 = m·a_x·cg_h/L, 좌우(뒷축) = m·a_y·cg_h/track·drive_frac (직전 가속도 사용)
  종: m(dv_x/dt - v_y r) = F_xl + F_xr - 구름저항 - F_yf sinδ
  횡(동적 자전거, v_x >= V_DYN): 앞·뒤 슬립각 → 횡력(alpha_peak까지 직선, mu·N 포화)
    뒷바퀴 횡 접지 = sqrt((mu_lat·배율)² - (F_x/N)²) (마찰원) → 커브 가속 시 뒤가 밀림
    요: Iz dr/dt = a F_yf cosδ - b F_yr + (F_xr - F_xl)·track/2 (좌우 구동력 차)
  저속(v_x < V_DYN)·후진: 운동학(슬립각 계산 불안정)
  바닥 위치별 마찰: mu_scale × 덮는 패치 배율의 곱(mu_patches = [x, y, 반지름, 배율] 반복, 월드 좌표)
"""
import math

import numpy as np

G = 9.81
H_MAX = 0.001     # 슬립 모델 하위 스텝 최대 길이(s)
V_EPS = 0.05      # 슬립 분모 하한(m/s)
V_DYN = 0.3       # 이 속도 미만은 운동학 횡 모델

PARAMS = dict(cmd_delay=0.0, motor_tau=0.0, steer_rate=0.0, understeer_k=0.0, wheelbase=0.59, com_x=0.3,
              speed_table_cmd=[-255.0, 0.0, 255.0], speed_table_mps=[-5.0, 0.0, 5.0],
              slip_enable=False, mass=27.62, drive_frac=0.5, drive_force_max=138.0,
              mu_peak=0.6, s_peak=0.15, mu_slide=0.08, c_rr=0.03, mu_lat=0.6,
              alpha_peak=0.1, yaw_inertia=2.4, track=0.44, cg_h=0.25, brake_mode='coast',
              mu_scale=1.0, mu_patches=[0.0])   # [0.0] = 패치 없음(ROS 배열 파라미터는 빈 값 불가)


def new_state():
    return dict(v_x=0.0, v_y=0.0, r=0.0, w_l=0.0, w_r=0.0, steer=0.0, a_x=0.0, a_y=0.0,
                f_l=0.0, f_r=0.0, n_l=0.0, n_r=0.0, s_l=0.0, s_r=0.0)


def floor_scale(x, y, p):
    """월드 좌표 (x, y)의 마찰 배율"""
    k = p['mu_scale']
    pt = p['mu_patches']
    for i in range(0, len(pt) - 3, 4):
        if (x - pt[i]) ** 2 + (y - pt[i + 1]) ** 2 <= pt[i + 2] ** 2:
            k *= pt[i + 3]
    return k


def mu_of(s, p):
    a = abs(s)
    if a <= p['s_peak']:
        m = p['mu_peak'] * a / p['s_peak']
    else:
        m = p['mu_peak'] + (p['mu_slide'] - p['mu_peak']) * min(1.0, (a - p['s_peak']) / (1.0 - p['s_peak']))
    return math.copysign(m, s)


def traction(w, v_g, n_load, scale, p):
    """바퀴 둘레 속도 w, 그 바퀴 위치의 지면 속도 v_g → 종방향 접지력(N)"""
    return mu_of((w - v_g) / max(abs(w), abs(v_g), V_EPS), p) * n_load * scale


def lat_force(alpha, mu, n_load, p):
    """횡력: 슬립각 alpha_peak 까지 직선, 이후 mu*N 포화"""
    return mu * n_load * max(-1.0, min(1.0, alpha / p['alpha_peak']))


def motor_force(v_nl, w, k, p):
    """DC 모터 힘. coast: PWM 0이면 0, 같은 방향으로는 구동만(감속 못 함), 역방향 PWM은 역전 제동"""
    f = k * (v_nl - w)
    if p['brake_mode'] != 'coast':
        return f
    if v_nl == 0.0:
        return 0.0
    if v_nl * w >= 0 and f * v_nl < 0:   # 같은 방향인데 바퀴가 더 빠름 → 구동 안 함
        return 0.0
    return f


def wheel_positions(pose, p):
    """pose=(x, y, yaw) 뒷축 중심 → 뒤왼·뒤오·앞 바퀴 월드 좌표"""
    x, y, yaw = pose
    c, s = math.cos(yaw), math.sin(yaw)
    t2 = p['track'] / 2
    return [(x - s * t2, y + c * t2), (x + s * t2, y - c * t2), (x + c * p['wheelbase'], y + s * p['wheelbase'])]


def model_step(st, cmd, dt, p, pose=None):
    """cmd=(조향 목표 rad, PWM 좌, PWM 우). st 갱신 후 (v_x, v_y, r) 반환. pose 있으면 바닥 배율 적용"""
    target_steer, pwm_l, pwm_r = cmd
    nl = lambda c: float(np.interp(c, p['speed_table_cmd'], p['speed_table_mps']))
    v_l, v_r = nl(pwm_l), nl(pwm_r)
    if p['steer_rate'] > 0:
        d = p['steer_rate'] * dt
        st['steer'] += max(-d, min(d, target_steer - st['steer']))
    else:
        st['steer'] = target_steer
    L, b, t2, m = p['wheelbase'], p['com_x'], p['track'] / 2, p['mass']
    delta = st['steer']
    if not p['slip_enable']:
        a = 1.0 if p['motor_tau'] <= 0 else min(1.0, dt / p['motor_tau'])
        st['w_l'] += a * (v_l - st['w_l'])
        st['w_r'] += a * (v_r - st['w_r'])
        v = st['v_x'] = (st['w_l'] + st['w_r']) / 2
        st['r'] = v * math.tan(delta) / L / (1.0 + p['understeer_k'] * v * v)
        st['v_y'] = st['r'] * b
        return st['v_x'], st['v_y'], st['r']
    sc_l, sc_r, sc_f = [floor_scale(x, y, p) for x, y in wheel_positions(pose, p)] if pose else (p['mu_scale'],) * 3
    k = p['drive_force_max'] / 2 / max(abs(x) for x in p['speed_table_mps'])   # 바퀴 하나의 모터
    m_w = max(p['motor_tau'], 1e-3) * k
    a_f = L - b
    n_rear0 = m * G * p['drive_frac']
    n = max(1, math.ceil(dt / H_MAX))
    h = dt / n
    for _ in range(n):
        vx, vy, r = st['v_x'], st['v_y'], st['r']
        # 하중 이동(직전 가속도)
        dn_long = m * st['a_x'] * p['cg_h'] / L
        n_rear = min(max(n_rear0 + dn_long, 0.0), m * G)
        n_f = m * G - n_rear
        dn_lat = m * st['a_y'] * p['cg_h'] / p['track'] * p['drive_frac']
        n_l, n_r = max(n_rear / 2 - dn_lat, 0.0), max(n_rear / 2 + dn_lat, 0.0)
        f_x = []
        for key, v_nl, v_g, n_w, sc in (('w_l', v_l, vx - r * t2, n_l, sc_l), ('w_r', v_r, vx + r * t2, n_r, sc_r)):
            w = st[key]
            f = traction(w, v_g, n_w, sc, p)
            df = max(0.0, (traction(w + 1e-4, v_g, n_w, sc, p) - f) / 1e-4)
            fm = motor_force(v_nl, w, k, p)
            k_eff = k if fm != 0.0 else 0.0
            st[key] = w + h * (fm - f) / m_w / (1.0 + h * (k_eff + df) / m_w)   # 암시적(접지 기울기 큼)
            f_x.append(f)
        f_l, f_r = f_x
        st.update(f_l=f_l, f_r=f_r, n_l=n_l, n_r=n_r,
                  s_l=(st['w_l'] - (vx - r * t2)) / max(abs(st['w_l']), abs(vx - r * t2), V_EPS),
                  s_r=(st['w_r'] - (vx + r * t2)) / max(abs(st['w_r']), abs(vx + r * t2), V_EPS))
        f_rr = p['c_rr'] * m * G
        if vx >= V_DYN:
            alpha_f = delta - math.atan((vy + a_f * r) / vx)
            alpha_r = -math.atan((vy - b * r) / vx)
            f_yf = lat_force(alpha_f, p['mu_lat'] * sc_f, n_f, p)
            mu_lat_l = math.sqrt(max(0.0, (p['mu_lat'] * sc_l) ** 2 - (f_l / max(n_l, 1e-6)) ** 2))
            mu_lat_r = math.sqrt(max(0.0, (p['mu_lat'] * sc_r) ** 2 - (f_r / max(n_r, 1e-6)) ** 2))
            f_yr = lat_force(alpha_r, mu_lat_l, n_l, p) + lat_force(alpha_r, mu_lat_r, n_r, p)
            ax = (f_l + f_r - f_rr - f_yf * math.sin(delta)) / m + vy * r
            ay_dot = (f_yf * math.cos(delta) + f_yr) / m - vx * r
            r_dot = (a_f * f_yf * math.cos(delta) - b * f_yr + (f_r - f_l) * t2) / p['yaw_inertia']
            st['v_x'] = vx + h * ax
            st['v_y'] = vy + h * ay_dot
            st['r'] = r + h * r_dot
            st['a_x'], st['a_y'] = ax, ay_dot + vx * r
        else:
            fx = f_l + f_r
            if abs(vx) < 1e-3 and abs(fx) <= f_rr:
                nvx = 0.0   # 구름저항(정지마찰)을 못 이기면 정지 유지
            else:
                nvx = vx + h * (fx - math.copysign(f_rr, vx if abs(vx) >= 1e-3 else fx)) / m
                nvx = 0.0 if vx * nvx < 0 else nvx   # 구름저항이 방향을 뒤집지 않게
            st['a_x'] = (nvx - vx) / h
            st['v_x'] = nvx
            st['r'] = nvx * math.tan(delta) / L
            st['v_y'] = st['r'] * b
            st['a_y'] = nvx * st['r']
    return st['v_x'], st['v_y'], st['r']


def sensors(st, p):
    """시뮬 센서값(실차와 같은 규격): 바퀴 둘레 속도 [앞왼, 앞오, 뒤왼, 뒤오] m/s, 요레이트, 가속도 x·y (차체 좌표)
    앞바퀴는 비구동·자유 회전 → 그 바퀴 위치의 지면 속도를 바퀴 방향으로 투영"""
    vx, vy, r, d = st['v_x'], st['v_y'], st['r'], st['steer']
    b, t2, L = p['com_x'], p['track'] / 2, p['wheelbase']
    vy_f = vy + (L - b) * r      # 앞축 횡속도
    front = [(vx - r * yy) * math.cos(d) + vy_f * math.sin(d) for yy in (t2, -t2)]
    return front + [st['w_l'], st['w_r']], r, st['a_x'], st['a_y']


def run(p, cmds, dt, st=None):
    """cmds: (조향, PWM좌, PWM우) 목록을 dt 간격으로 → 각 스텝 (v_x, v_y, r, w_l, w_r) 배열"""
    st = st or new_state()
    out = []
    for c in cmds:
        out.append(model_step(st, c, dt, p) + (st['w_l'], st['w_r']))
    return st, np.array(out)


# ---------------- 자체 점검(슬립 값은 추정 시작값 기준) ----------------
def _rear_alpha(o, p):
    return -math.atan((o[1] - p['com_x'] * o[2]) / o[0])


def _hold(c, secs, dt=0.01):
    return [c] * int(round(secs / dt))


_p = dict(PARAMS)
_o = model_step(new_state(), (0.2, 51, 51), 0.01, _p)
assert _o[0] == 1.0 and abs(_o[2] - math.tan(0.2) / 0.59) < 1e-9 and abs(_o[1] - _o[2] * 0.3) < 1e-9  # 기본 = 이전 동작
_p.update(slip_enable=True, motor_tau=0.15)
_s, _o = run(_p, _hold((0.0, 255, 255), 6.0), 0.01)
assert _o[149, 0] < 0.5 and _o[149, 3] > 3.0      # 최대 PWM 출발: 1.5초에 바퀴는 헛돌고 차체는 느림
assert _o[-1, 0] > 3.0                            # 그 뒤 접지가 회복되며 출발
_s, _o = run(_p, _hold((0.0, 51, 51), 6.0), 0.01)
assert 0.0 < (_o[-1, 3] - _o[-1, 0]) / _o[-1, 3] < _p['s_peak']   # 정속 직진: 작은 정상 슬립
_v = _o[-1, 0]
_s, _o = run(_p, _hold((0.0, 0, 0), 1.0), 0.01, _s)
assert 0.0 < _o[-1, 0] < _v and _o[-1, 0] > _v - 0.4   # PWM 0: 코스트(구름저항으로 천천히 감속)
_s0, _ = run(_p, _hold((0.0, 102, 102), 4.0), 0.01)
_v = _s0['v_x']
_s, _o = run(_p, _hold((0.0, -40, -40), 0.5), 0.01, dict(_s0))
assert _o[-1, 0] < _v - 0.5 and -_p['s_peak'] < _s['s_l'] < 0   # 약한 역PWM 제동: 접지 안에서 빠른 감속
_s, _o = run(_p, _hold((0.0, -255, -255), 0.5), 0.01, dict(_s0))
assert _s['s_l'] < -1.0 and _o[-1, 0] > _v - 0.5   # 최대 역PWM: 바퀴가 거꾸로 헛돌아 제동이 오히려 약함(제동 슬립)
_s, _ = run(_p, _hold((0.0, 153, 153), 5.0), 0.01)
_s, _o = run(_p, _hold((0.3, 153, 153), 3.0), 0.01, _s)
assert _rear_alpha(_o[-1], _p) < _p['alpha_peak'] and _s['n_r'] > _s['n_l']   # 고속 좌커브: 접지 유지, 바깥(오른쪽) 하중 증가
_s, _ = run(_p, _hold((0.0, 102, 102), 5.0), 0.01)
_s, _ = run(_p, _hold((0.3, 102, 102), 2.0), 0.01, _s)
_s, _o = run(_p, _hold((0.3, 255, 255), 2.0), 0.01, _s)
assert _rear_alpha(_o[-1], _p) > _p['alpha_peak']   # 커브 중 급가속: 뒤가 밀림
_s, _ = run(_p, _hold((0.0, 102, 102), 4.0), 0.01)
_s, _o = run(_p, _hold((0.0, 60, 140), 2.0), 0.01, _s)
assert _o[-1, 2] > 0.01                             # 조향 0에서 오른쪽 PWM이 크면 좌회전 요(타이어 횡력이 잡아 작음)
_pl = dict(_p, mu_patches=[5.0, 0.0, 1.0, 0.3])
assert floor_scale(5.2, 0.1, _pl) == 0.3 and floor_scale(7.0, 0.0, _pl) == 1.0
_s1, _ = run(_p, _hold((0.0, 255, 255), 2.0), 0.01)
_s2 = new_state()
for _ in range(200):
    model_step(_s2, (0.0, 255, 255), 0.01, _pl, pose=(5.0, 0.0, 0.0))
assert _s2['v_x'] < _s1['v_x']                      # 미끄러운 바닥: 출발 더 느림
