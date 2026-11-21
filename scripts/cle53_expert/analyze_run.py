#!/usr/bin/env python3
"""폐루프 기록 분석: 부호 있는 횡오차(+ = 중심선 왼쪽/트랙 안쪽), 조향, 경로 위치. 사용: analyze_run.py <rec폴더> [시작초] [끝초]"""
import json, sys, numpy as np, os
CL = np.load(os.path.join(os.path.dirname(os.path.abspath(__file__)), 'expert', 'lane2_center.npy'))
def slat(x, y):
    d = np.hypot(CL[:, 0] - x, CL[:, 1] - y); i = np.argmin(d); t = CL[(i + 1) % len(CL)] - CL[i]
    return np.sign(t[0] * (y - CL[i, 1]) - t[1] * (x - CL[i, 0])) * d[i]
t0, t1 = (float(sys.argv[2]), float(sys.argv[3])) if len(sys.argv) > 3 else (0, 1e9)
for l in open(os.path.join(sys.argv[1], 'log.jsonl')):
    r = json.loads(l)
    if r['xy'] and t0 <= r['t'] <= t1:
        print(r['i'], r['t'], r['xy'], r['yaw'], 'st', r['steer'], 'sp', r['speed'], 'lat %+.3f' % slat(*r['xy']), 'path', r['path_tail'] and r['path_tail'][:2])
