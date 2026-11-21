#!/usr/bin/env python3
"""실행 중인 Gazebo의 조명을 바꾼다(월드 파일 수정 없음). 강화학습 에피소드마다 햇빛 방향·세기 랜덤화용.
Gazebo 자체 토픽 ~/light/modify(msgs::Light)를 gz topic -p 로 발행하므로 추가 플러그인 불필요.
조명 이름: sun_light(directional, 전역 햇빛), ceiling_0..15(point, 천장등). 창문 햇빛 조각은 모델 sun_patch_0..2(set_patch)

CLI 예: set_light.py sun_light --dir -0.8 -0.35 -0.48 --diffuse 0.9 0.9 0.85
        set_light.py ceiling_0 ceiling_1 --diffuse 0.3 0.3 0.3 --att 0.6 0.05 0.01
        set_light.py sun_light --sun 30 120 0.8    # 고도(도), 방위(도, +x에서 반시계), 세기(0~1)
        set_light.py --random [시드]                # 에피소드마다 햇빛+햇빛 조각 랜덤(범위: 아래 상수)
Python: from set_light import apply_random_sun; info = apply_random_sun(random.Random(seed))"""
import argparse, math, random, subprocess

# 랜덤 햇빛 범위(창문 위치 미상 → 방위 전체). 대회장 정보 생기면 좁힐 것
ELEV, AZIM, INTENSITY = (10, 60), (0, 360), (0.3, 1.0)        # sun_light(전역 햇빛)
# 창문 햇빛 조각(월드의 sun_patch_0..2 반투명 판): 최대 개수, 폭 m, 불투명도(밝기), 중심 범위(월드 좌표, 트랙)
N_PATCH, PATCH_W, PATCH_OPACITY = 3, (0.4, 1.5), (0.2, 0.85)
PATCH_X, PATCH_Y, PATCH_MAX_LEN = (-5.0, 5.0), (-6.6, 6.6), 4.0


def sun(elev_deg, azim_deg, intensity, color=(1.0, 0.97, 0.9)):
    """고도·방위·세기 → set_light 인자. direction은 빛이 '향하는' 방향(아래쪽 -z)"""
    e, a = math.radians(elev_deg), math.radians(azim_deg)
    return dict(direction=(-math.cos(e) * math.cos(a), -math.cos(e) * math.sin(a), -math.sin(e)),
                diffuse=tuple(c * intensity for c in color))


def random_sun(rng=random):
    """에피소드 시작 때 1회 호출. 반환: (요약 dict, sun_light 인자, [sun_patch_k 인자 dict 또는 None(숨김)])
    햇빛 조각 0~N_PATCH개. 조각은 햇빛 방위 방향으로 길쭉(고도 낮을수록 길게)"""
    e, az, i = rng.uniform(*ELEV), rng.uniform(*AZIM), rng.uniform(*INTENSITY)
    n = rng.randint(0, N_PATCH); info = dict(elev=round(e, 1), azim=round(az, 1), intensity=round(i, 2), patches=[])
    patches = []
    for k in range(N_PATCH):
        if k >= n: patches.append(None); continue
        w, op = rng.uniform(*PATCH_W), rng.uniform(*PATCH_OPACITY)
        pt = dict(x=rng.uniform(*PATCH_X), y=rng.uniform(*PATCH_Y), yaw=math.radians(az), w=w,
                  l=min(PATCH_MAX_LEN, w * (1 + 1 / math.tan(math.radians(e)))), opacity=op)
        patches.append(pt); info['patches'].append({k_: round(v, 2) for k_, v in pt.items()})
    return info, sun(e, az, i), patches


def set_patch(k, p, world='default'):
    """sun_patch_k 판 배치. p=None이면 바닥 아래로 숨김"""
    pub = lambda t, m: subprocess.run(['gz', 'topic', '-w', world, '-p', f'/gazebo/{world}/{t}', '-m', m], check=True, timeout=10)
    if p is None:
        return pub('model/modify', f'name: "sun_patch_{k}" pose {{position {{x: 0 y: 0 z: -5}} orientation {{x: 0 y: 0 z: 0 w: 1}}}}')
    pub('model/modify', f'name: "sun_patch_{k}" pose {{position {{x: {p["x"]} y: {p["y"]} z: 0.003}} '
                        f'orientation {{x: 0 y: 0 z: {math.sin(p["yaw"] / 2)} w: {math.cos(p["yaw"] / 2)}}}}}')
    pub('visual', f'name: "sun_patch_{k}::link::visual" parent_name: "sun_patch_{k}::link" '
                  f'scale {{x: {p["l"]} y: {p["w"]} z: 1}} transparency: {1 - p["opacity"]}')


def apply_random_sun(rng=random, world='default'):
    info, sun_kw, patches = random_sun(rng)
    set_light('sun_light', world, **sun_kw)
    for k, p in enumerate(patches):
        set_patch(k, p, world)
    return info


def light_msg(name, direction=None, diffuse=None, specular=None, pose=None, att=None, range_=None,
              spot=None, shadows=None):
    """msgs::Light 텍스트 포맷. 지정한 필드만 바뀐다. spot=(inner_rad, outer_rad, falloff)"""
    v3 = lambda k, v: f'{k} {{x: {v[0]} y: {v[1]} z: {v[2]}}}'
    col = lambda k, v: f'{k} {{r: {v[0]} g: {v[1]} b: {v[2]} a: 1}}'
    f = [f'name: "{name}"']
    if direction is not None: f.append(v3('direction', direction))
    if diffuse is not None: f.append(col('diffuse', diffuse))
    if specular is not None: f.append(col('specular', specular))
    if pose is not None: f.append('pose {' + v3('position', pose) + ' orientation {x: 0 y: 0 z: 0 w: 1}}')
    if att is not None:
        f += [f'attenuation_constant: {att[0]}', f'attenuation_linear: {att[1]}', f'attenuation_quadratic: {att[2]}']
    if range_ is not None: f.append(f'range: {range_}')
    if spot is not None: f += [f'spot_inner_angle: {spot[0]}', f'spot_outer_angle: {spot[1]}', f'spot_falloff: {spot[2]}']
    if shadows is not None: f.append(f'cast_shadows: {str(bool(shadows)).lower()}')
    return ' '.join(f)


assert light_msg('a', diffuse=(1, 0, 0)) == 'name: "a" diffuse {r: 1 g: 0 b: 0 a: 1}'
assert abs(sun(90, 0, 1)['direction'][2] + 1) < 1e-9
assert random_sun(random.Random(0))[0] == random_sun(random.Random(0))[0]
assert all(p is None or p['l'] <= PATCH_MAX_LEN for s_ in range(20) for p in random_sun(random.Random(s_))[2])


def set_light(name, world='default', **kw):
    subprocess.run(['gz', 'topic', '-w', world, '-p', f'/gazebo/{world}/light/modify', '-m', light_msg(name, **kw)],
                   check=True, timeout=10)


def main():
    p = argparse.ArgumentParser()
    p.add_argument('names', nargs='*')
    p.add_argument('--dir', nargs=3, type=float, dest='direction')
    p.add_argument('--sun', nargs=3, type=float, metavar=('ELEV', 'AZIM', 'I'))
    p.add_argument('--diffuse', nargs=3, type=float)
    p.add_argument('--specular', nargs=3, type=float)
    p.add_argument('--pose', nargs=3, type=float)
    p.add_argument('--att', nargs=3, type=float)
    p.add_argument('--range', type=float, dest='range_')
    p.add_argument('--spot', nargs=3, type=float)
    p.add_argument('--shadows', type=int)
    p.add_argument('--random', nargs='?', const=-1, type=int, metavar='SEED', help='랜덤 고도·방위·세기(시드 선택)')
    p.add_argument('--world', default='default')
    a = vars(p.parse_args()); names, world, s, r = a.pop('names'), a.pop('world'), a.pop('sun'), a.pop('random')
    if s: a.update(sun(*s))
    if r is not None:  # 랜덤은 햇빛 조명 전체에 적용(names 무시)
        print(apply_random_sun(random.Random(None if r < 0 else r), world)); return
    for n in names:
        set_light(n, world, **{k: v for k, v in a.items() if v is not None})


if __name__ == '__main__':
    main()
