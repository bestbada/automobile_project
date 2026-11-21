import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSReliabilityPolicy

import math
from interfaces_pkg.msg import PathPlanningResult, MotionCommand

#---------------Variable Setting---------------
SUB_PATH_TOPIC_NAME = "path_planning_result"
PUB_TOPIC_NAME = "topic_control_signal"


# CLE53 기하 / BEV(lane_info_extractor) → 차량 좌표 변환
WHEELBASE = 0.59          # 축간거리(m)
MAX_STEER = 0.45          # 조향 명령 ±7 ↔ 바퀴 ±0.45rad
STEER_SIGN = -1           # 명령 음수 = 좌회전 (simulation config driving STEERING=-1)
CAM_X = 0.24              # 뒷바퀴축 → 카메라 전방 거리(m)
BEV_FAR, BEV_NEAR, BEV_H = 2.5, 0.85, 576   # BEV 0행=카메라 앞 2.5m, 576행=0.85m (지면 선형)
ROI_CUT = 300             # roi_rectangle_below cutting_idx
PX_PER_M = 410 / 0.866    # BEV 차로폭 410px = 0.866m
IMG_CX = 512


def roi_to_vehicle(x_px, y_roi):
    """ROI 픽셀 → 뒷바퀴축 기준 (전방 m, 좌측 m)"""
    fwd = CAM_X + BEV_FAR - (y_roi + ROI_CUT) / BEV_H * (BEV_FAR - BEV_NEAR)
    return fwd, (IMG_CX - x_px) / PX_PER_M


def pure_pursuit_cmd(fwd, left):
    """목표점(차량 좌표)으로 pure pursuit 조향 명령(-7~7)"""
    ld = math.hypot(fwd, left)
    delta = math.atan2(2 * WHEELBASE * left / ld, ld)   # sinα = left/ld
    return int(max(-7, min(7, round(STEER_SIGN * delta / MAX_STEER * 7))))


assert pure_pursuit_cmd(1.6, 0.0) == 0 and pure_pursuit_cmd(1.6, 0.3) < 0 and pure_pursuit_cmd(1.6, -0.3) > 0
#----------------------------------------------

# 모션 플랜 발행 주기 (초) - 소수점 필요 (int형은 반영되지 않음)
TIMER = 0.1

class MotionPlanningNode(Node):
    def __init__(self):
        super().__init__('motion_planner_node')

        # 토픽 이름 설정
        self.sub_path_topic = self.declare_parameter('sub_lane_topic', SUB_PATH_TOPIC_NAME).value
        self.pub_topic = self.declare_parameter('pub_topic', PUB_TOPIC_NAME).value
        
        self.timer_period = self.declare_parameter('timer', TIMER).value

        # lookahead: 목표점까지 직선거리(m, 뒷바퀴축 기준)
        # speed: 좌/우 속도 명령(0~255)
        self.lookahead = self.declare_parameter('lookahead', 1.6).value
        self.speed = self.declare_parameter('speed', 100).value
        # min_speed: 최대 조향(±7)일 때 속도. 조향량에 비례해 speed→min_speed로 감속(기본=speed, 감속 없음)
        self.min_speed = self.declare_parameter('min_speed', self.speed).value

        # QoS 설정
        self.qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=1
        )

        # 변수 초기화
        self.path_data = None

        self.steering_command = 0
        self.left_speed_command = 0
        self.right_speed_command = 0
        

        # 서브스크라이버 설정
        self.path_sub = self.create_subscription(PathPlanningResult, self.sub_path_topic, self.path_callback, self.qos_profile)

        # 퍼블리셔 설정
        self.publisher = self.create_publisher(MotionCommand, self.pub_topic, self.qos_profile)

        # 타이머 설정
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

    def path_callback(self, msg: PathPlanningResult):
        self.path_data = list(zip(msg.x_points, msg.y_points))
        
    def timer_callback(self):

        if self.path_data is None:
            self.steering_command = 0
        else:
            # pure pursuit: 경로점 중 직선거리가 lookahead에 가장 가까운 점을 목표로 조향.
            # (원본 기울기×gain 방식은 화면 맨 아래 고정점 근처 기울기를 써서 직선에서 진동, 커브에서 조향량 부족)
            pts = [roi_to_vehicle(x, y) for x, y in self.path_data]
            fwd, left = min(pts, key=lambda p: abs(math.hypot(*p) - self.lookahead))
            self.steering_command = pure_pursuit_cmd(fwd, left)

        # 경로가 아직 없으면(YOLO 콜드스타트) 정지
        speed = 0 if self.path_data is None else round(
            self.speed - (self.speed - self.min_speed) * abs(self.steering_command) / 7)
        self.left_speed_command = speed   # 속도 (255가 최대 속도)
        self.right_speed_command = speed



        self.get_logger().info(f"steering: {self.steering_command}, " 
                               f"left_speed: {self.left_speed_command}, " 
                               f"right_speed: {self.right_speed_command}")

        # 모션 명령 메시지 생성 및 퍼블리시
        motion_command_msg = MotionCommand()
        motion_command_msg.steering = self.steering_command
        motion_command_msg.left_speed = self.left_speed_command
        motion_command_msg.right_speed = self.right_speed_command
        self.publisher.publish(motion_command_msg)

def main(args=None):
    rclpy.init(args=args)
    node = MotionPlanningNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n\nshutdown\n\n")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
