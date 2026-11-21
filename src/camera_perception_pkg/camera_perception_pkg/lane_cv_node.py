"""영상처리 차선 인식 노드(YOLO 미사용): camera/image_raw → 2차선 중심 곡선 → path_planning_result.
motion_planner_node가 그대로 쓰도록 경로를 lane_info_extractor ROI 픽셀 좌표로 바꿔 발행한다.
검출 실패 프레임은 직전 경로를 차량 이동(명령값 추정)만큼 옮긴 예측을 발행(최대 약 1.5초, LaneDetector.miss 15).
motion_planner는 경로가 안 오면 직전 경로를 차량 좌표 그대로 유지해서, 급커브에서 선이 시야 밖으로 나가면
조향이 한쪽에 고정돼 이탈했음(track_real 0.6바퀴 지점)."""
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, qos_profile_sensor_data
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from interfaces_pkg.msg import PathPlanningResult, MotionCommand
from .lane_cv import LaneDetector, IW, IH

# 명령값 → 차량 이동 추정(검출기 예측용). 시뮬: 속도 cmd/255*5 m/s, 조향 cmd ±7 ↔ ±0.45rad, 음수=좌
WHEELBASE, MAX_STEER, STEER_SIGN, CMD_TO_MPS = 0.59, 0.45, -1, 5.0 / 255
# motion_planner_node.roi_to_vehicle의 역변환
CAM_X, BEV_FAR, BEV_NEAR, BEV_H, ROI_CUT, PX_PER_M, IMG_CX = 0.24, 2.5, 0.85, 576, 300, 410 / 0.866, 512


def vehicle_to_roi(fwd, left):
    return IMG_CX - left * PX_PER_M, (CAM_X + BEV_FAR - fwd) / (BEV_FAR - BEV_NEAR) * BEV_H - ROI_CUT


class LaneCvNode(Node):
    def __init__(self):
        super().__init__('lane_cv_node')
        self.bridge = CvBridge(); self.det = LaneDetector()
        # speed_scale: 실제 속도 / 명령 환산 속도. 실차는 엔코더·IMU 측정으로 보정(시뮬 1.0)
        self.speed_scale = self.declare_parameter('speed_scale', 1.0).value
        self.cmd = MotionCommand(); self.last_t = None
        self.create_subscription(MotionCommand, 'topic_control_signal', lambda m: setattr(self, 'cmd', m), QoSProfile(depth=1))
        self.pub = self.create_publisher(PathPlanningResult, 'path_planning_result', QoSProfile(depth=1))
        self.create_subscription(Image, 'camera/image_raw', self.cb, qos_profile_sensor_data)

    def cb(self, msg):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        dt = 0.0 if self.last_t is None else min(max(t - self.last_t, 0.0), 0.5); self.last_t = t
        v = (self.cmd.left_speed + self.cmd.right_speed) / 2 * CMD_TO_MPS * self.speed_scale
        delta = STEER_SIGN * self.cmd.steering / 7 * MAX_STEER
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        if img.shape[:2] != (IH, IW):   # BEV 변환은 시뮬 카메라(1024x576, model.sdf) 기준 → 다른 해상도면 결과가 틀림
            self.get_logger().error(f'카메라 해상도 {img.shape[1]}x{img.shape[0]} != {IW}x{IH}: lane_cv 카메라 상수 확인 필요',
                                    throttle_duration_sec=2.0)
            return
        ctr = self.det.detect(img, dx=v * dt, dyaw=v * math.tan(delta) / WHEELBASE * dt)
        if ctr is None:
            ctr = self.det.prev   # 검출 실패: 오도메트리로 옮긴 직전 경로(1.5초 넘으면 None)
        if ctr is None:
            return
        out = PathPlanningResult()
        # 먼 곳 → 가까운 곳 순서(기존 경로와 같은 방향)
        for fwd, left in ctr[::-1]:
            x, y = vehicle_to_roi(float(fwd), float(left))
            out.x_points.append(x); out.y_points.append(y)
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = LaneCvNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()


if __name__ == '__main__':
    main()
