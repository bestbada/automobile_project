#!/usr/bin/env python3
"""진단용: roi_image 수신마다 카메라 원본·ROI 이미지와 lane/path/조향/위치를 저장. argv[1]=출력 폴더, argv[2]=기록 초"""
import os, sys, json, time, math
import cv2, rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from interfaces_pkg.msg import LaneInfo, PathPlanningResult, MotionCommand

OUT, DUR = sys.argv[1], float(sys.argv[2]); os.makedirs(OUT, exist_ok=True)


class Rec(Node):
    def __init__(self):
        super().__init__('diag_recorder'); self.b = CvBridge(); q = QoSProfile(depth=1)
        self.cam = self.lane = self.path = self.cmd = self.pose = None; self.i = 0; self.t0 = time.time()
        self.log = open(os.path.join(OUT, 'log.jsonl'), 'w', buffering=1)
        self.create_subscription(Image, 'camera/image_raw', lambda m: setattr(self, 'cam', m), qos_profile_sensor_data)
        self.create_subscription(LaneInfo, 'yolov8_lane_info', lambda m: setattr(self, 'lane', m), q)
        self.create_subscription(PathPlanningResult, 'path_planning_result', lambda m: setattr(self, 'path', m), q)
        self.create_subscription(MotionCommand, 'topic_control_signal', lambda m: setattr(self, 'cmd', m), q)
        self.create_subscription(Odometry, 'ego_pose', lambda m: setattr(self, 'pose', m), 10)
        self.create_subscription(Image, 'roi_image', self.roi, q)
        if len(sys.argv) > 3 and sys.argv[3] == 'cam': self.create_timer(0.1, lambda: self.cam and self.roi(None))

    def roi(self, m):
        t = time.time() - self.t0
        if t > DUR: raise SystemExit
        self.i += 1; n = f'{self.i:04d}'
        if m is not None: cv2.imwrite(f'{OUT}/{n}_roi.png', self.b.imgmsg_to_cv2(m, 'mono8'))
        if self.cam: cv2.imwrite(f'{OUT}/{n}_cam.jpg', self.b.imgmsg_to_cv2(self.cam, 'bgr8'))
        p = self.pose.pose.pose if self.pose else None
        yaw = None
        if p:
            o = p.orientation; yaw = math.degrees(math.atan2(2 * (o.w * o.z + o.x * o.y), 1 - 2 * (o.y ** 2 + o.z ** 2)))
        self.log.write(json.dumps(dict(
            i=self.i, t=round(t, 2),
            lane=[(tp.target_x, tp.target_y) for tp in self.lane.target_points] if self.lane else None,
            lane_slope=self.lane.slope if self.lane else None,
            path_tail=[(round(x), round(y)) for x, y in zip(self.path.x_points[-10::9], self.path.y_points[-10::9])] if self.path else None,
            steer=self.cmd.steering if self.cmd else None, speed=self.cmd.left_speed if self.cmd else None,
            xy=(round(p.position.x, 2), round(p.position.y, 2)) if p else None, yaw=round(yaw, 1) if yaw is not None else None)) + '\n')


rclpy.init(); n = Rec()
try: rclpy.spin(n)
except SystemExit: pass
n.log.close()
