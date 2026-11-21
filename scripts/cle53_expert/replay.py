# 저장 프레임으로 lane_info_extractor 파이프라인 오프라인 재현
import sys, cv2, numpy as np
from types import SimpleNamespace as NS
from ultralytics import YOLO
from camera_perception_pkg.lib import camera_perception_func_lib as C
m = YOLO('/home/lee/automobile_project/src/simulation_pkg/simulation_pkg/data/sim.pt')
SRC = [[390, 265], [634, 265], [834, 550], [190, 550]]

def run(img, save=None):
    r = m(img, verbose=False, conf=0.5)[0]
    dets = [NS(class_name=m.names[int(c)], mask=NS(data=[NS(x=float(x), y=float(y)) for x, y in r.masks[k].xy[0].tolist()],
                height=img.shape[0], width=img.shape[1])) for k, c in enumerate(r.boxes.cls)] if r.masks is not None else []
    if not dets: return None
    e = C.draw_edges(NS(detections=dets), cls_name='lane2', color=255)
    h, w = e.shape[:2]
    dst = [[round(w * 0.3), 0], [round(w * 0.7), 0], [round(w * 0.7), h], [round(w * 0.3), h]]
    b = C.bird_convert(e, srcmat=SRC, dstmat=dst); roi = cv2.convertScaleAbs(C.roi_rectangle_below(b, cutting_idx=300))
    g = C.dominant_gradient(roi, theta_limit=70)
    pts = [round(C.get_lane_center(roi, detection_height=y, detection_thickness=10, road_gradient=g, lane_width=410)) for y in range(5, 155, 50)]
    if save: cv2.imwrite(save, np.vstack([cv2.resize(e.astype(np.uint8), (1024, 576)), cv2.cvtColor(b.astype(np.uint8), cv2.COLOR_GRAY2BGR) if b.ndim == 2 else b][:1] + [cv2.convertScaleAbs(b)]))
    return pts, round(g, 1), [m.names[int(c)] for c in r.boxes.cls]

if __name__ == '__main__':
    for f in sys.argv[1:]:
        print(f.split('/')[-1], run(cv2.imread(f), save=f'replay_{f.split("/")[-1][:4]}.png'))
