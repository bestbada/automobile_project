# 트랙 텍스처에서 2차선(바깥 차선) 중심선 추출 → 월드 좌표 폴리라인(반시계 순서) lane2_center.npy
import cv2, numpy as np
TEX = '/home/lee/automobile_project/src/simulation_pkg/models/race_track_real/materials/textures/track_real.jpg'
W, H = 15.38, 11.564          # 평면 크기(m), 모델 yaw 1.57 → wx=-my, wy=mx
OFFSET = 0.435                # 바깥 실선 중심 → 2차선 중심(텍스처 실측: -6.935 → -6.50)
img = cv2.imread(TEX, 0); h, w = img.shape; s = w / W   # px/m
white = (img > 200).astype(np.uint8)
cs, hier = cv2.findContours(white, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_NONE)
# 바깥 실선 = 면적이 가장 큰 외곽 윤곽(채우면 트랙 전체+안쪽)
outer = max((c for c, hh in zip(cs, hier[0]) if hh[3] < 0), key=cv2.contourArea)
fill = np.zeros_like(white); cv2.drawContours(fill, [outer], -1, 1, -1)
d = cv2.distanceTransform(fill, cv2.DIST_L2, 5) / s       # 바깥 윤곽(선 바깥 가장자리)에서 안쪽 거리 m
lane = ((d > OFFSET + 0.025) * 1).astype(np.uint8)        # 선 폭 50mm의 절반 보정
c2, _ = cv2.findContours(lane, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
cl = max(c2, key=cv2.contourArea)[:, 0, :].astype(float)
mx, my = cl[:, 0] / s - W / 2, H / 2 - cl[:, 1] / s
pts = np.stack([-my, mx], 1)                               # 월드 좌표
# 0.05m 간격으로 리샘플 + 반시계(면적 양수) 정렬
seg = np.r_[0, np.cumsum(np.hypot(*np.diff(pts, axis=0).T))]
u = np.arange(0, seg[-1], 0.05); pts = np.stack([np.interp(u, seg, pts[:, 0]), np.interp(u, seg, pts[:, 1])], 1)
if 0.5 * np.sum(pts[:, 0] * np.roll(pts[:, 1], -1) - np.roll(pts[:, 0], -1) * pts[:, 1]) < 0: pts = pts[::-1]
np.save('lane2_center.npy', pts)
# 자체 점검: 스폰(-1.03,-6.49)은 2차선 중앙이어야 함
dmin = np.min(np.hypot(pts[:, 0] + 1.03, pts[:, 1] + 6.49))
print('length %.2fm, points %d, spawn->centerline %.3fm' % (seg[-1], len(pts), dmin)); assert dmin < 0.08
