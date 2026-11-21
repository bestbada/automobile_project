#!/usr/bin/env python3
"""실측 CSV(real_logger.py) → 차량 모델 파라미터 보정 → vehicle_dynamics.yaml 갱신본.
Gazebo 불필요: 동역학이 전부 vehicle_model.py 안에 있으므로 기록된 명령을 모델에 넣어 센서값(엔코더 4개·요레이트·가속도)을
그대로 예측하고 오차 최소화(scipy least_squares).

사용:
  calibrate.py --E1 lift.csv --E2 straight.csv --E3 circle.csv [--E4 step.csv --E5 power.csv]
               [--fixed mass=27.6,drive_frac=0.52,com_x=0.3,track=0.44,cg_h=0.25] [--out 새.yaml]
  calibrate.py --selftest      # 정답 파라미터로 가짜 실측을 만들어 되찾는지 확인
단계: E1(무부하) 속도표·모터 시상수·명령 지연 → E2(직진) 구동력·mu 곡선·구름저항 → E3~E5(커브) 횡 접지·요 관성·조향 속도
슬립률 보고: 앞바퀴(비구동) 엔코더를 IMU 가속도와 상보 필터로 합친 차체 속도 기준(뒷바퀴 엔코더와 비교)."""
import argparse, math, os, re, sys
import numpy as np
from scipy.optimize import least_squares

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.join(HERE, '..', '..')
sys.path.insert(0, os.path.join(ROOT, 'src', 'simulation_pkg', 'simulation_pkg'))
import vehicle_model as vm  # noqa: E402

YAML = os.path.join(ROOT, 'src', 'simulation_pkg', 'config', 'vehicle_dynamics.yaml')
STEER_SIGN, MAX_STEER = -1, 0.45      # sender와 같은 명령→조향각 매핑(shortcut: 직선 가정, 가변저항 실측 후 표로 교체)
DT = 0.01
CH = ['enc_fl', 'enc_fr', 'enc_rl', 'enc_rr', 'gyro_z', 'acc_x', 'acc_y']
NOISE = dict(enc=0.01, gyro_z=0.005, acc=0.05)   # 잔차 정규화(센서 잡음 수준)


def load(path):
    d = np.genfromtxt(path, delimiter=',', names=True)
    return {k: d[k] for k in d.dtype.names}


def yaml_params(path=YAML):
    """vehicle_dynamics.yaml → PARAMS 덮어쓴 dict(주석 보존 위해 PyYAML로 읽기만)"""
    import yaml
    p = dict(vm.PARAMS)
    p.update(yaml.safe_load(open(path))['simulation_sender_node']['ros__parameters'])
    return p


def commands(d, delay):
    n = len(d['t'])
    k = int(round(delay / DT))
    idx = np.clip(np.arange(n) - k, 0, n - 1)
    steer = STEER_SIGN * d['steer_cmd'][idx] / 7.0 * MAX_STEER
    return list(zip(steer, d['pwm_l'][idx].astype(float), d['pwm_r'][idx].astype(float)))


def simulate(d, p):
    st = vm.new_state()
    out = np.empty((len(d['t']), 7))
    for i, c in enumerate(commands(d, p['cmd_delay'])):
        vm.model_step(st, c, DT, p)
        w, r, ax, ay = vm.sensors(st, p)
        out[i] = w + [r, ax, ay]
    return out


def scale_of(ch):
    return NOISE['enc'] if ch.startswith('enc') else NOISE['gyro_z'] if ch == 'gyro_z' else NOISE['acc']


def residual(datas, p, chans):
    res = []
    for d in datas:
        pred = simulate(d, p)
        for ch in chans:
            e = (pred[:, CH.index(ch)] - d[ch]) / scale_of(ch)
            res.append(np.where(np.isnan(e), 0.0, e))
    return np.concatenate(res)


def fit(datas, p, names, bounds, chans, label):
    x0 = np.clip([p[k] for k in names], [b[0] for b in bounds], [b[1] for b in bounds])

    def f(x):
        q = dict(p, **dict(zip(names, x)))
        return residual(datas, q, chans)
    r0 = np.sqrt(np.mean(f(x0) ** 2))
    sol = least_squares(f, x0, bounds=([b[0] for b in bounds], [b[1] for b in bounds]),
                        x_scale=np.maximum(np.abs(x0), 1e-3), diff_step=1e-3, max_nfev=80)
    p.update(dict(zip(names, sol.x)))
    print(f'[{label}] 정규화 RMS {r0:.2f} → {np.sqrt(np.mean(sol.fun ** 2)):.2f}  ' +
          ', '.join(f'{k}={v:.4g}' for k, v in zip(names, sol.x)))
    return p


def fit_lift(d, p):
    """E1 무부하: 각 PWM 계단 마지막 1초 평균 → 속도표, 그 뒤 모터 시상수·명령 지연(PWM≠0 구간만, 코스트 감속은 모델 밖)"""
    w = (d['enc_rl'] + d['enc_rr']) / 2
    pwm = (d['pwm_l'] + d['pwm_r']) / 2
    table = {0.0: 0.0}
    edges = np.flatnonzero(np.diff(pwm) != 0) + 1
    for a, b in zip(np.r_[0, edges], np.r_[edges, len(pwm)]):
        if pwm[a] != 0 and b - a > 150:
            table[float(pwm[a])] = float(np.nanmean(w[b - 100:b]))
    ks = sorted(table)
    p.update(speed_table_cmd=[float(k) for k in ks], speed_table_mps=[table[k] for k in ks])
    print('[E1] 속도표', ', '.join(f'{k:.0f}:{table[k]:.3f}' for k in ks))
    q = dict(p, slip_enable=False)
    mask = np.r_[pwm != 0]
    best = None
    for delay in np.arange(0.0, 0.205, 0.01):
        def f(x):
            pred = simulate(d, dict(q, motor_tau=x[0], cmd_delay=delay))
            return np.concatenate([((pred[:, i] - d[c]) / NOISE['enc'])[mask] for i, c in ((2, 'enc_rl'), (3, 'enc_rr'))])
        sol = least_squares(f, [0.1], bounds=([0.005], [2.0]), max_nfev=30)
        if best is None or sol.cost < best[0]:
            best = (sol.cost, delay, sol.x[0])
    p.update(cmd_delay=float(best[1]), motor_tau=float(best[2]))
    print(f'[E1] cmd_delay={best[1]:.3f}s motor_tau={best[2]:.3f}s')
    return p


def slip_report(d, p, label):
    """앞바퀴 엔코더(비구동) + IMU 가속도 상보 필터 → 차체 속도, 뒷바퀴와 비교한 슬립률"""
    steer = STEER_SIGN * d['steer_cmd'] / 7.0 * MAX_STEER
    v_f = (d['enc_fl'] + d['enc_fr']) / 2 * np.cos(steer)     # 앞바퀴 → 차체 종속도(운동학 근사)
    v, T = 0.0, 0.3                                           # 상보 필터: IMU 적분, 앞바퀴로 0.3초 시상수 보정
    vb = np.empty_like(v_f)
    for i in range(len(v_f)):
        v += d['acc_x'][i] * DT
        if not np.isnan(v_f[i]):
            v += (v_f[i] - v) * DT / T
        vb[i] = v
    w = (d['enc_rl'] + d['enc_rr']) / 2
    s = (w - vb) / np.maximum(np.maximum(np.abs(w), np.abs(vb)), vm.V_EPS)
    moving = np.abs(w) > 0.1
    if moving.any():
        print(f'[{label}] 슬립률(뒷바퀴 vs 앞바퀴+IMU): 최대 {np.nanmax(s[moving]):.2f}, 최소 {np.nanmin(s[moving]):.2f}, '
              f'중앙값 {np.nanmedian(s[moving]):.3f}')


def write_yaml(p, out):
    s = open(YAML).read()
    for k, v in p.items():
        val = '[' + ', '.join(f'{x:.4g}' for x in v) + ']' if isinstance(v, list) else (
            str(v).lower() if isinstance(v, bool) else f'{v:.4g}' if isinstance(v, float) else str(v))
        s = re.sub(rf'^(\s+{k}:\s*)(\[[^\]]*\]|[^\s#]+)', lambda m: m.group(1) + val, s, count=1, flags=re.M)
    open(out, 'w').write(s)
    print('saved', out)


def calibrate(files, fixed):
    p = yaml_params()
    p.update(fixed)
    if 'E1' in files:
        p = fit_lift(load(files['E1']), p)
    p['slip_enable'] = True
    if 'E2' in files:
        d = load(files['E2'])
        slip_report(d, p, 'E2')
        p = fit([d], p, ['drive_force_max', 'mu_peak', 's_peak', 'mu_slide', 'c_rr'],
                [(10, 500), (0.1, 1.5), (0.02, 0.6), (0.01, 1.0), (0.0, 0.3)], CH[:4] + ['acc_x'], 'E2')
    turns = [load(files[e]) for e in ('E3', 'E4', 'E5') if e in files]
    if turns:
        names, bnds = ['mu_lat', 'alpha_peak', 'yaw_inertia'], [(0.1, 1.5), (0.01, 0.5), (0.2, 20)]
        if all(np.isnan(d['steer_meas']).all() for d in turns):
            names.append('steer_rate'); bnds.append((0.5, 30.0))
            if p['steer_rate'] <= 0:   # 0 = '즉시'라는 특별값 → 0 근처에서 불연속이라 적합 시작값으로 못 씀
                p['steer_rate'] = 5.0
        p = fit(turns, p, names, bnds, CH[:5] + ['acc_y'], 'E3~E5')
    return p


def selftest():
    """정답 파라미터(기본값과 다름)로 가짜 실측 → 보정 → 회복 확인"""
    import tempfile
    src = open(os.path.join(HERE, 'test_sequence.py')).read().split('def main')[0].replace('import rclpy', '')
    src = '\n'.join(l for l in src.splitlines() if not l.startswith('from '))
    ns = {}; exec(src, ns)
    true = yaml_params()
    true.update(cmd_delay=0.05, motor_tau=0.12, speed_table_cmd=[-255, -40, -20, 0, 20, 40, 255],
                speed_table_mps=[-3.2, -0.5, 0.0, 0.0, 0.0, 0.5, 3.3], drive_force_max=110.0, mu_peak=0.7,
                s_peak=0.12, mu_slide=0.15, c_rr=0.04, mu_lat=0.55, alpha_peak=0.12, yaw_inertia=3.0,
                steer_rate=4.0, slip_enable=True)
    rng = np.random.default_rng(0)
    tmp = tempfile.mkdtemp()
    files = {}
    for e, seq in ns['SEQ'].items():
        rows = [(s, l, r) for s, l, r, secs in seq for _ in range(int(round(secs / DT)))]
        d = {'t': np.arange(len(rows)) * DT, 'steer_cmd': np.array([r[0] for r in rows], float),
             'pwm_l': np.array([r[1] for r in rows], float), 'pwm_r': np.array([r[2] for r in rows], float)}
        pred = simulate(d, dict(true, slip_enable=(e != 'E1')))   # E1: 바퀴가 공중 → 무부하(슬립 없음)
        for i, ch in enumerate(CH):
            d[ch] = pred[:, i] + rng.normal(0, scale_of(ch), len(rows))
        d['steer_meas'] = np.full(len(rows), np.nan)
        files[e] = os.path.join(tmp, e + '.csv')
        cols = ['t', 'steer_cmd', 'pwm_l', 'pwm_r'] + CH + ['steer_meas']
        np.savetxt(files[e], np.column_stack([d[c] for c in cols]), delimiter=',', header=','.join(cols), comments='')
    got = calibrate(files, {})
    print('\n정답 vs 보정')
    worst = 0.0
    for k in ['cmd_delay', 'motor_tau', 'drive_force_max', 'mu_peak', 's_peak', 'mu_slide', 'c_rr',
              'mu_lat', 'alpha_peak', 'yaw_inertia', 'steer_rate']:
        e = abs(got[k] - true[k]) / max(abs(true[k]), 1e-6)
        worst = max(worst, e) if k not in ('s_peak', 'alpha_peak') else worst
        print(f'  {k:16s} {true[k]:8.4g} {got[k]:8.4g}  ({e * 100:.0f}%)')
    tab = np.interp([40, 120, 255, -80], got['speed_table_cmd'], got['speed_table_mps'])
    ref = np.interp([40, 120, 255, -80], true['speed_table_cmd'], true['speed_table_mps'])
    print('  속도표(40,120,255,-80)', np.round(tab, 3), '정답', np.round(ref, 3))
    # 파라미터 회복보다 중요한 것: 보정된 모델이 실측(가짜)을 재현하는가
    for e in ('E2', 'E3'):
        d = load(files[e])
        err = simulate(d, got) - simulate(d, true)
        print(f'  {e} 예측 오차 RMS: 엔코더 {np.sqrt(np.mean(err[:, :4] ** 2)):.3f} m/s, 요레이트 {np.sqrt(np.mean(err[:, 4] ** 2)):.3f} rad/s')
    assert np.allclose(tab, ref, atol=0.05), '속도표 회복 실패'
    assert abs(got['cmd_delay'] - true['cmd_delay']) <= 0.011 and abs(got['motor_tau'] / true['motor_tau'] - 1) < 0.2
    return got, true


if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    for e in ('E1', 'E2', 'E3', 'E4', 'E5'):
        ap.add_argument('--' + e)
    ap.add_argument('--fixed', default='', help='실측 고정값 key=value,...')
    ap.add_argument('--out', default=os.path.join(HERE, 'vehicle_dynamics.calibrated.yaml'))
    ap.add_argument('--selftest', action='store_true')
    a = ap.parse_args()
    if a.selftest:
        selftest()
        sys.exit(0)
    files = {e: getattr(a, e) for e in ('E1', 'E2', 'E3', 'E4', 'E5') if getattr(a, e)}
    fixed = {k: float(v) for k, v in (kv.split('=') for kv in a.fixed.split(',') if kv)}
    p = calibrate(files, fixed)
    p['slip_enable'] = True
    write_yaml(p, a.out)
