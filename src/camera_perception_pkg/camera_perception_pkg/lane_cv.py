#!/usr/bin/env python3
"""영상처리 차선 인식(YOLO 미사용). CLE53 전방 카메라 → 미터 BEV → 바깥 실선(우측) 추적 → 2차선 중심 곡선.
좌표: 차량(뒷바퀴축) 기준 x 전방(m), y 좌측(m)."""
import math
import numpy as np
import cv2

# 카메라(model.sdf camera_link): 뒷바퀴축 기준 x 0.24, 높이 0.595, 숙임 0.26529rad, 1024x576, hfov 1.2292
CAM_X, CAM_H, PITCH, IW, IH, HFOV = 0.24, 0.595, 0.26529, 1024, 576, 1.2292
FX = IW / 2 / math.tan(HFOV / 2)
RES = 0.01                                   # BEV 1px = 1cm
X0, X1, Y0, Y1 = 0.80, 2.80, -1.60, 1.60     # BEV 범위(전방 0.8~2.8m, 좌우 ±1.6m)
LANE_OFF = 0.435                             # 바깥 실선 중심 → 2차선 중심(트랙 텍스처 실측)
START = 1.0                                  # 선 시작점 탐색 범위(BEV 아래에서 m): 근거리가 햇빛에 가려도 먼 선 사용
# 2차 계수 상한: 중심선 최소 반경 약 1.17m(lane2_center.npy) → 가장 안쪽 선 반경 약 0.74m → a=1/(2R)≈0.68
A_MAX = 0.7


def _build_map():
    xs = np.arange(X1, X0, -RES)             # 행: 위=먼 곳
    ys = np.arange(Y1, Y0, -RES)             # 열: 왼쪽=좌측(+y)
    gx, gy = np.meshgrid(xs, ys, indexing='ij')
    # 카메라 좌표(광축 전방 zc, 오른쪽 xc, 아래 yc)
    dx, dz = gx - CAM_X, -CAM_H
    zc = dx * math.cos(PITCH) - dz * math.sin(PITCH)
    yc = -(dx * math.sin(PITCH) + dz * math.cos(PITCH))
    xc = -gy
    u = (FX * xc / zc + IW / 2).astype(np.float32)
    v = (FX * yc / zc + IH / 2).astype(np.float32)
    return u, v, xs, ys


MAP_U, MAP_V, XS, YS = _build_map()
# 자체 점검: 전방 2.5m(카메라 기준) 지면 → 이미지 265행 근처 (lane_info_extractor 사다리꼴과 일치)
_r = int(round((X1 - (CAM_X + 2.5)) / RES)); assert abs(MAP_V[_r, len(YS) // 2] - 265) < 3


def bev(img):
    g = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) if img.ndim == 3 else img.copy()
    g[500:, 450:575] = 0                      # 화면 아래 라이다 원통(u 458~565, v>=508) → 무효 영역
    return cv2.remap(g, MAP_U, MAP_V, cv2.INTER_LINEAR, borderValue=0)


def line_mask(b):
    """밝은 얇은 선(폭 5cm) 강조: top-hat(주변 대비) + 적응 임계값 → 조명 변화에 강함"""
    th = cv2.morphologyEx(b, cv2.MORPH_TOPHAT, cv2.getStructuringElement(cv2.MORPH_RECT, (15, 15)))
    valid = b > 0
    # 바깥 실선은 잔디 옆이라 대비가 점선의 약 절반 → 최상위값 비례 임계값(15~40)
    t = float(np.clip(0.3 * np.percentile(th[valid], 99.5), 15, 40)) if valid.any() else 255
    m = ((th > t) & valid).astype(np.uint8)
    return cv2.morphologyEx(m, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))


def _runs(row, with_width=False):
    """한 행의 밝은 구간 중심(열 인덱스) 목록. with_width면 (중심, 폭)"""
    d = np.diff(np.r_[0, row, 0].astype(np.int16))   # uint8 그대로면 -1이 255로 넘어감
    s, e = np.where(d == 1)[0], np.where(d == -1)[0]
    return [((a + b - 1) / 2, b - a) if with_width else (a + b - 1) / 2 for a, b in zip(s, e) if 2 <= b - a <= 40]


def track_line(m, y_start, max_gap=0.35, win=0.06, start_tol=0.22):
    """가까운 곳(아래 행)부터 위로 한 행씩 선을 추적. y_start: 근거리 예상 횡위치(m). 반환: (x, y) 배열 또는 None"""
    H = m.shape[0]; col = lambda y: (Y1 - y) / RES
    pts = []; c = None; slope = 0.0; gap = 0; widths = {}
    for r in range(H - 1, -1, -1):
        rw = dict(_runs(m[r], with_width=True)); runs = list(rw)
        if c is None:                                   # 시작점: 예상 위치 근처
            if (H - 1 - r) * RES > START: return None   # 시야 하한이 약 1.09m라 0.8~(0.8+START)m에서 시작점 탐색
            cand = [u for u in runs if abs(u - col(y_start)) <= start_tol / RES]
            if cand: c = min(cand, key=lambda u: abs(u - col(y_start))); pts.append((r, c)); widths[r] = rw[c]
            continue
        pred = c + slope * (gap + 1)
        cand = [u for u in runs if abs(u - pred) <= win / RES + 2 * abs(slope) + gap * 0.3]
        if cand:
            u = min(cand, key=lambda u: abs(u - pred))
            if len(pts) >= 5:
                r0, c0 = pts[-5]; slope = 0.7 * slope + 0.3 * (u - c0) / (r0 - r)
            c = u; gap = 0; pts.append((r, c)); widths[r] = rw[u]
        else:
            gap += 1
            if gap * RES > max_gap: break
    if len(pts) < 15 or (pts[0][0] - pts[-1][0]) * RES < 0.25: return None
    p = np.array(pts, float)
    out = np.stack([X1 - p[:, 0] * RES, Y1 - p[:, 1] * RES], 1)
    # 선의 수직 폭(m) = 행 방향 폭 × cos(기울기): 차선 5cm, 횡단보도 막대 10cm
    sl = np.polyfit(out[:, 0], out[:, 1], 1)[0]
    track_line.width = float(np.median([widths[r] for r, _ in pts])) * RES / math.sqrt(1 + sl * sl)
    return out


def find_lines(m, max_gap=0.40):
    """근거리(0.8~1.8m, START)에서 시작하는 모든 선을 추적. 반환: [(근거리 횡위치, 채움률, 점들)]
    채움률 = 추적 구간 중 실제로 선이 있던 행 비율(실선 ≈1, 점선 ≈0.5)"""
    H = m.shape[0]; starts = []
    for r in range(H - 1, H - 1 - int(START / RES), -3):
        for u in _runs(m[r]):
            y = Y1 - u * RES
            if all(abs(y - q) > 0.08 for q in starts): starts.append(y)
    lines = []
    for y in starts:
        t = track_line(m, y, max_gap=max_gap, start_tol=0.04)
        if t is None: continue
        w = track_line.width
        ny = float(np.median(t[t[:, 0] < t[:, 0].min() + 0.2, 1]))
        if any(abs(ny - l[0]) < 0.1 for l in lines): continue   # 같은 선 중복
        span = (t[:, 0].max() - t[:, 0].min()) / RES + 1
        lines.append((ny, len(t) / span, t, w))
    # 횡단보도: 폭 10cm 막대가 여러 개 나란히 → 폭 8cm 초과 선이 3개 이상이면 넓은 선 제외
    # (실선 하나만 넓게 잡히는 경우(연석 띠와 붙음)는 유지)
    if sum(l[3] > 0.08 for l in lines) >= 3:
        lines = [l for l in lines if l[3] <= 0.08]
    return [l[:3] for l in lines]


def offset_curve(pts, off, a=None):
    """선 점들을 2차 다항식 y(x)로 맞춘 뒤 좌측 법선 방향으로 off(m) 평행이동. 근거리는 0.3m까지 연장.
    a: 점이 적을 때(햇빛 포화 등으로 선이 짧게 보임) 쓸 2차 계수(직전 곡선에서). 없으면 직선 피팅"""
    if len(pts) > 40: a = np.polyfit(pts[:, 0], pts[:, 1], 2)[0]
    if a is None: k = np.polyfit(pts[:, 0], pts[:, 1], 1)
    else:
        a = float(np.clip(a, -A_MAX, A_MAX))   # 트랙보다 급한 곡률은 잡음(포화 경계·그림자 섞임)
        k = np.r_[a, np.polyfit(pts[:, 0], pts[:, 1] - a * pts[:, 0] ** 2, 1)]
    xs = np.arange(0.3, pts[:, 0].max(), 0.02)
    ys = np.polyval(k, xs); s = np.polyval(np.polyder(k), xs); n = np.sqrt(1 + s * s)
    return np.stack([xs - off * s / n, ys + off / n], 1)


class LaneDetector:
    """보이는 선마다 가능한 해석(바깥 실선/중앙 점선/안쪽 실선)으로 2차선 중심 후보를 만들고,
    직전 차로 중심(1.2m 지점 횡위치)에 가장 가까운 후보를 고름. 선 간격 0.87m라 해석 간 차이가 커서 구분됨."""
    SOLID = 0.8          # 채움률 이상이면 실선
    GAP = 2 * LANE_OFF   # 선 간격 0.87m

    def __init__(self):
        self.prev = None; self.miss = 0

    def predict(self, dx, dyaw):
        """직전 중심 곡선을 차량 이동(전진 dx m, 좌회전 dyaw rad)만큼 옮겨 현재 차량 좌표로"""
        if self.prev is None: return None
        c, s = math.cos(dyaw), math.sin(dyaw)
        x, y = self.prev[:, 0] - dx, self.prev[:, 1] - 0.5 * dx * dyaw   # 원호 이동 근사
        return np.stack([c * x + s * y, -s * x + c * y], 1)

    def detect(self, img, dx=0.0, dyaw=0.0):
        """dx, dyaw: 직전 호출 이후 차량 이동(명령값 또는 오도메트리로 추정)"""
        pred = self.predict(dx, dyaw)
        if pred is not None: self.prev = pred                  # 놓친 프레임에도 예측은 계속 이동
        lines = find_lines(line_mask(bev(img)))
        cands = []   # (차로 중심 곡선, 감점)
        for ny, fill, t in lines:
            long_ = (t[:, 0].max() - t[:, 0].min()) >= 0.8
            # 선 종류 판정은 커브에서 믿기 어려움(점선 마디가 행으로 겹침) → 세 해석 모두 허용, 종류는 약한 감점만
            if long_ and fill >= self.SOLID: pens = (0.0, 0.05, 0.03)     # (바깥 실선, 점선, 안쪽 실선)
            elif long_: pens = (0.05, 0.0, 0.05)
            else: pens = (0.05, 0.04, 0.08)
            kinds = list(zip((LANE_OFF, -LANE_OFF, -LANE_OFF - self.GAP), pens))
            # 짧은 선은 직전 차로 중심을 해당 선 위치로 옮긴 곡선의 곡률을 빌림
            a = lambda off: None if self.prev is None else np.polyfit(*offset_curve(self.prev, -off).T, 2)[0]
            cands += [(offset_curve(t, off, a(off)), pen) for off, pen in kinds]
        if not cands:
            self.miss += 1; return None
        c12 = lambda c: float(c[np.argmin(np.abs(c[:, 0] - 1.2)), 1])

        def dist(c):   # 예측 곡선과 1.1~1.7m 구간 평균 횡차이(예측 없으면 차량 정면 기준)
            if self.prev is None: return abs(c12(c))
            k = (c[:, 0] >= 1.1) & (c[:, 0] <= 1.7)
            if not k.any(): k = slice(None)
            order = np.argsort(self.prev[:, 0])
            py = np.interp(c[k, 0], self.prev[order, 0], self.prev[order, 1])
            return float(np.mean(np.abs(c[k, 1] - py)))
        tol = 0.25 + 0.05 * self.miss
        # 예측과의 차이 + 종류 감점 + 차량에서 먼 해석 감점(잘못 잠긴 상태에서 복귀용)
        ok = [(d + pen + 0.05 * abs(c12(c)), c) for c, pen in cands for d in [dist(c)] if d < tol]
        if not ok:
            self.miss += 1
            if self.miss > 15: self.prev = None                   # 1.5초 넘게 못 찾으면 예측 초기화
            return None
        best = min(ok, key=lambda sc: sc[0])[1]
        self.prev = best; self.miss = 0
        return best


_det = LaneDetector()


def lateral_at(img, F, dx=0.0, dyaw=0.0):
    """평가용: 전방 F m에서 차로 중심 횡위치(좌+)"""
    c = _det.detect(img, dx, dyaw)
    if c is None: return None
    return float(c[np.argmin(np.abs(c[:, 0] - F)), 1])
