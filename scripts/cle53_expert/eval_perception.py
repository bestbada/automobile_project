#!/usr/bin/env python3
"""인식 평가: expert 주행 기록(recorder cam 모드)의 각 프레임에서
정답 = 2차선 중심선이 차량 전방 F m 지점에서 갖는 횡위치(좌+, m), 예측 = 인식 파이프라인의 같은 지점 횡위치.
사용: python3 eval_perception.py <rec폴더> [yolo|camera_perception_pkg.lane_cv] [F=1.6]"""
import sys, os, json, math
import numpy as np, cv2

HERE = os.path.dirname(os.path.abspath(__file__))
CL = np.load(os.path.join(HERE, 'expert', 'lane2_center.npy'))
# motion_planner_node와 같은 변환
CAM_X, FAR, NEAR, H, CUT, PXM, CX = 0.24, 2.5, 0.85, 576, 300, 410 / 0.866, 512


def roi_y_for(f):
    return (CAM_X + FAR - f) / (FAR - NEAR) * H - CUT


def gt_lateral(x, y, yaw, f):
    """중심선 위 점 중 차량 좌표 전방거리가 f에 가장 가까운(앞쪽, 근처) 점의 횡위치"""
    c, s = math.cos(yaw), math.sin(yaw)
    dx, dy = CL[:, 0] - x, CL[:, 1] - y
    fwd, left = dx * c + dy * s, -dx * s + dy * c
    ok = (np.abs(left) < 1.5) & (fwd > 0)
    i = np.argmin(np.abs(fwd[ok] - f))
    return float(left[ok][i])


def main():
    rec, method = sys.argv[1], (sys.argv[2] if len(sys.argv) > 2 else 'yolo')
    F = float(sys.argv[3]) if len(sys.argv) > 3 else 1.6
    sys.path.insert(0, HERE)
    if method == 'yolo':
        import replay
        def predict(img, dx=0.0, dyaw=0.0):
            r = replay.run(img)
            if r is None: return None
            pts = [(x, y) for x, y in zip(r[0], range(5, 155, 50)) if x != 0]
            if not pts: return None
            x, yr = min(pts, key=lambda p: abs(p[1] - roi_y_for(F)))
            return (CX - x) / PXM
    else:
        import importlib; m = importlib.import_module(method)
        predict = lambda img, dx=0.0, dyaw=0.0: m.lateral_at(img, F, dx, dyaw)
    errs = []; out = []; last = None
    for l in open(os.path.join(rec, 'log.jsonl')):
        r = json.loads(l)
        if not r['xy'] or r['yaw'] is None: continue
        f = os.path.join(rec, '%04d_cam.jpg' % r['i'])
        if not os.path.exists(f): continue
        gt = gt_lateral(*r['xy'], math.radians(r['yaw']), F)
        # 직전 프레임 이후 이동(기록된 실제 위치) → 검출기 예측용
        yaw = math.radians(r['yaw']); dx = dyaw = 0.0
        if last is not None:
            ddx, ddy = r['xy'][0] - last[0], r['xy'][1] - last[1]
            dx = ddx * math.cos(last[2]) + ddy * math.sin(last[2])
            dyaw = (yaw - last[2] + math.pi) % (2 * math.pi) - math.pi
        last = (r['xy'][0], r['xy'][1], yaw)
        p = predict(cv2.imread(f), dx, dyaw)
        e = None if p is None else p - gt
        out.append(dict(i=r['i'], xy=r['xy'], gt=round(gt, 3), pred=None if p is None else round(p, 3), err=None if e is None else round(e, 3)))
        errs.append(e)
    v = np.array([abs(e) for e in errs if e is not None])
    print(f'{method}: frames {len(errs)}, missing {sum(e is None for e in errs)}, '
          f'|err| median {np.median(v):.3f} p90 {np.percentile(v, 90):.3f} max {v.max():.3f}, >0.10m {np.mean(v > 0.10) * 100:.1f}%')
    json.dump(out, open(os.path.join(rec, f'eval_{method}.json'), 'w'))


if __name__ == '__main__':
    main()
