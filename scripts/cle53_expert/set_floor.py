#!/usr/bin/env python3
"""바닥 위치별 마찰 랜덤화: 2차선 중심선 근처에 마찰 패치(원) n개 → simulation_sender_node mu_patches 파라미터.
사용: set_floor.py --random <시드> [--n 6]   /  set_floor.py --clear
패치: 중심 = 중심선 위 임의 점 + 횡 ±0.5m, 반지름 0.3~1.2m, 배율 0.5~1.1 (천 매트 주름·먼지·광택 구간 흉내, [추정] 범위)
슬립 모델(slip_enable true)에서만 효과."""
import argparse, os, random, subprocess
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))


def random_patches(rng, n):
    path = np.load(os.path.join(HERE, 'expert', 'lane2_center.npy'))
    out = []
    for _ in range(n):
        x, y = path[rng.randrange(len(path))]
        out += [float(x + rng.uniform(-0.5, 0.5)), float(y + rng.uniform(-0.5, 0.5)),
                rng.uniform(0.3, 1.2), rng.uniform(0.5, 1.1)]
    return out


if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    ap.add_argument('--random', type=int)
    ap.add_argument('--n', type=int, default=6)
    ap.add_argument('--clear', action='store_true')
    a = ap.parse_args()
    v = [0.0] if a.clear or a.random is None else random_patches(random.Random(a.random), a.n)
    val = '[' + ','.join(f'{x:.3f}' for x in v) + ']'
    subprocess.run(['ros2', 'param', 'set', '/simulation_sender_node', 'mu_patches', val], check=True)
    print('mu_patches', val)
