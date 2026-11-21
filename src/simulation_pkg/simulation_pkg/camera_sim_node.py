"""시뮬 전용 카메라 후처리: camera/image_sim(Gazebo 원본) → 역광 + 자동노출 흉내 → camera/image_raw.

실차(Logitech C920, 자동노출)는 카메라 드라이버가 camera/image_raw를 직접 발행하므로 이 노드를 띄우지 않는다.
인식 노드는 시뮬·실차 모두 camera/image_raw만 구독 → 시뮬에서 만든 인식/학습 모델을 그대로 실차에 연결.

- 월드는 실제 조명의 EXPOSURE_K(0.35)배로 렌더링돼 포화가 없다(월드 파일 주석 참고).
- 역광: 햇빛 방향(sun_dir, 빛이 향하는 방향)과 차량 자세(ego_pose)로 카메라 기준 태양 각도를 구해
  태양 쪽 번짐(veiling glare) + 화면 전체 대비 저하 + 태양이 시야 근처면 고스트(렌즈 반사 원)를 더한다.
  shortcut: 지붕·벽에 의한 가림은 무시(창문 위치 미상). 대회장 창문 위치 알면 방위 제한 추가.
- 자동노출: 중앙가중 측광값이 target이 되도록 이득을 프레임마다 지수적으로 조정(포화된 출력 기준 측광).
- 번짐(bloom): 포화된 밝은 영역(창문·신호등) 주변 빛 번짐. 실차 사진(신호등 주변)에서 확인한 현상.
- 하이라이트 압축(knee): 일반 카메라 톤 곡선처럼 knee 이상 밝기를 부드럽게 눌러 햇빛 구간에서도 선 대비를 남김.
  shortcut: C920 실제 톤 곡선 미측정. 햇빛 드는 실차 영상으로 knee 값 맞출 것.
- 잡음: 출력 기준 σ≈5(실차 사진 노면 잡음 중앙값 5.0, Gazebo 잡음 약 3.4에 더함).
  보정 근거: 실차 주행 사진(skku-sample-dataset, 640x480, 188장) 하단 노면/선/잔디 밝기 중앙값 93/215/166.
  shortcut: 역광 모양·세기, 측광 방식, 반응 속도는 C920 직접 실측 없이 정한 값. 햇빛 드는 실차 영상 생기면 맞출 것.
"""
import math
import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry

# 전방 카메라(cle53 model.sdf): 1024x576, hfov 1.2292, 아래로 0.26529rad 숙임
IW, IH, HFOV, PITCH = 1024, 576, 1.2292, 0.26529
FX = IW / 2 / math.tan(HFOV / 2)
LOW = 8   # 역광은 1/8 해상도로 계산 후 확대

# 저해상도 화소별 광선(카메라 좌표: 전방, 왼쪽, 위)
_u, _v = np.meshgrid((np.arange(IW // LOW) + 0.5) * LOW, (np.arange(IH // LOW) + 0.5) * LOW)
RAYS = np.stack([np.ones_like(_u), -(_u - IW / 2) / FX, -(_v - IH / 2) / FX], -1)
RAYS /= np.linalg.norm(RAYS, axis=-1, keepdims=True)
# 중앙가중 측광 가중치
_yy, _xx = np.mgrid[0:IH, 0:IW]
METER_W = np.exp(-(((_xx - IW / 2) / (0.35 * IW)) ** 2 + ((_yy - IH / 2) / (0.35 * IH)) ** 2) / 2).astype(np.float32)
METER_W /= METER_W.sum()


def sun_in_camera(sun_dir, q):
    """햇빛 방향(빛이 향하는 방향, 월드) + 차량 자세 쿼터니언(x,y,z,w) → 카메라 좌표(전방, 왼쪽, 위)의 태양 단위벡터"""
    x, y, z, w = q
    R = np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                  [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                  [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])
    cp, sp = math.cos(PITCH), math.sin(PITCH)
    axes = R @ np.array([[cp, 0, -sp], [0, 1, 0], [sp, 0, cp]]).T   # 열: 카메라 전방·왼쪽·위(월드)
    s = -np.asarray(sun_dir, float); s /= np.linalg.norm(s) + 1e-12
    return s @ axes


def glare(sc, strength):
    """카메라 좌표 태양벡터 sc, 세기(실제 기준 0~1) → (H,W) 더할 밝기(실제 기준 0~255 단위, 255 초과 가능)"""
    if strength <= 0: return None
    ang = np.arccos(np.clip(RAYS @ sc, -1, 1))
    g = 400 * strength / (1 + (ang / 0.15) ** 2)            # 태양 쪽 번짐(각도 제곱에 반비례)
    g += 40 * strength * max(sc[0], 0) ** 2                  # 태양을 향할수록 전체 대비 저하
    if sc[0] > 0.5:                                          # 태양이 시야 근처(광축 60° 이내): 고스트
        su, sv = IW / 2 - FX * sc[1] / sc[0], IH / 2 - FX * sc[2] / sc[0]
        k = 60 * strength * ((sc[0] - 0.5) / 0.5) ** 2
        for t, r in ((0.4, 20), (0.9, 40), (1.4, 70)):     # 태양 → 화면 중심 반대편으로 늘어선 원
            gu, gv = IW / 2 - t * (su - IW / 2), IH / 2 - t * (sv - IH / 2)
            g += k * np.exp(-((_u - gu) ** 2 + (_v - gv) ** 2) / (2 * r * r))
    return cv2.resize(g.astype(np.float32), (IW, IH), interpolation=cv2.INTER_LINEAR)


def knee(x, k0):
    """k0 이하는 그대로, 위는 255로 점근(tanh). 기울기 연속"""
    return np.where(x < k0, x, k0 + (255 - k0) * np.tanh((x - k0) / (255 - k0)))


assert knee(np.float32(100), 190) == 100 and 254 < knee(np.float32(600), 190) <= 255 and 235 < knee(np.float32(255), 190) < 245
# 자체 점검: 정면 수평 차량에서 태양이 카메라 광축 방향이면 sc ≈ (1,0,0), 화면 중심 번짐 최대
_sc = sun_in_camera((-math.cos(PITCH), 0, math.sin(PITCH)), (0, 0, 0, 1))
assert np.allclose(_sc, (1, 0, 0), atol=1e-9)
_g = glare(_sc, 1.0); assert np.unravel_index(_g.argmax(), _g.shape)[1] in range(IW // 2 - LOW, IW // 2 + LOW)


class CameraSimNode(Node):
    def __init__(self):
        super().__init__('camera_sim_node')
        p = lambda n, v: self.declare_parameter(n, v)
        p('k', 0.35)                      # 월드 렌더 조명 배율(EXPOSURE_K). 초기 이득 = 1/k
        p('target', 125.0)                # 중앙가중 측광 목표. 110이면 노면·잔디가 실차보다 15~25% 어두워 올림
        p('ae_rate', 0.15)                # 프레임당 이득 조정 비율(30fps에서 약 0.2s 시정수)
        p('gain_max', 15.0)
        p('glare', 1.0)                   # 역광 세기 배율(0이면 끔)
        p('bloom', 0.5)                   # 포화 영역 번짐 세기(0이면 끔)
        p('knee', 255.0)                  # 하이라이트 압축 시작 밝기(255 이상이면 끔). 190 시험에서 이득 확인 못 해 기본 끔(night는 knee 무관하게 통과·이탈 섞임)
        p('noise', 3.7)                   # 추가 잡음 σ(출력 단위). Gazebo 잡음과 합쳐 실차 σ≈5
        p('sun_dir', [-0.5, -0.5, -0.7])  # launch가 월드의 sun_light로 덮어씀, set_light.py가 갱신
        p('sun_diffuse', [0.0, 0.0, 0.0])
        self.gain = 1 / self.get_parameter('k').value
        self.q = (0.0, 0.0, 0.0, 1.0); self.bridge = CvBridge(); self.rng = np.random.default_rng()
        self.create_subscription(Odometry, 'ego_pose', self.pose_cb, 10)
        self.pub = self.create_publisher(Image, 'camera/image_raw', qos_profile_sensor_data)
        self.create_subscription(Image, 'camera/image_sim', self.cb, qos_profile_sensor_data)

    def pose_cb(self, m):
        o = m.pose.pose.orientation; self.q = (o.x, o.y, o.z, o.w)

    def cb(self, msg):
        g = lambda n: self.get_parameter(n).value
        k = g('k')
        img = self.bridge.imgmsg_to_cv2(msg, 'bgr8').astype(np.float32)
        gl = glare(sun_in_camera(g('sun_dir'), self.q), g('glare') * max(g('sun_diffuse')) / k)
        if gl is not None:
            img += (gl * k)[..., None]                           # 실제 기준 → 렌더(k배) 단위
        out = img * self.gain
        if g('bloom') > 0:   # 255 넘는 부분(포화된 광원)을 넓게 퍼뜨려 더함
            hot = cv2.resize(np.maximum(out - 230, 0), (IW // LOW, IH // LOW), interpolation=cv2.INTER_AREA)
            out += g('bloom') * cv2.resize(cv2.GaussianBlur(hot, (0, 0), 3), (IW, IH))
        if g('knee') < 255:
            out = knee(out, g('knee')).astype(np.float32)
        if g('noise') > 0:
            out += self.rng.standard_normal(out.shape, dtype=np.float32) * g('noise')
        out = np.clip(out, 0, 255)
        meter = float((cv2.cvtColor(out, cv2.COLOR_BGR2GRAY) * METER_W).sum())
        self.gain = float(np.clip(self.gain * (g('target') / max(meter, 1.0)) ** g('ae_rate'), 0.5, g('gain_max')))
        o = self.bridge.cv2_to_imgmsg(out.astype(np.uint8), 'bgr8'); o.header = msg.header
        self.pub.publish(o)


def main(args=None):
    rclpy.init(args=args)
    node = CameraSimNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()


if __name__ == '__main__':
    main()
