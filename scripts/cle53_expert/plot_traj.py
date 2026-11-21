# 궤적 CSV를 트랙 텍스처 위에 그림. 월드→모델: 트랙이 yaw 1.57 회전 → mx=wy, my=-wx
import sys, csv, matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt
img = plt.imread('/home/lee/automobile_project/src/simulation_pkg/models/race_track_real/materials/textures/track_real.jpg')
W, H = 15.38, 11.564
plt.figure(figsize=(12, 9)); plt.imshow(img, extent=[-W/2, W/2, -H/2, H/2])
for f in sys.argv[2:]:
    r = list(csv.DictReader(open(f)))
    plt.plot([float(p['y']) for p in r], [-float(p['x']) for p in r], lw=2, label=f.split('/')[-1])
plt.plot(-6.49, 1.03, 'r*', ms=15); plt.legend(); plt.savefig(sys.argv[1], dpi=60, bbox_inches='tight')
