#!/usr/bin/env python3
"""실측 실험 명령 시퀀스 발행(topic_control_signal, MotionCommand). 실차·시뮬 공용, real_logger.py와 같이 실행.
사용: test_sequence.py <실험> [--sim]       실험: E1 E2 E3 E4 E5 (내용·필요 공간은 MEASUREMENT.md)
- 모든 실험은 정지 상태에서 시작해 정지로 끝난다. Ctrl-C 하면 즉시 PWM 0 을 보내고 끝낸다.
- 조향 명령 부호는 config STEERING=-1 기준: 음수 = 좌회전."""
import sys
import rclpy
from rclpy.node import Node
from interfaces_pkg.msg import MotionCommand

HZ = 50
# (조향 명령 -7~7, PWM 좌, PWM 우, 초)
SEQ = {
    # 무부하(차 뒤를 들어 뒷바퀴가 공중): 속도표·모터 시상수·명령 지연
    'E1': [(0, 0, 0, 2)] + [x for p in (20, 40, 80, 120, 160, 200, 255, -40, -80, -160, -255)
                            for x in ((0, p, p, 3), (0, 0, 0, 3))],
    # 직진(바닥): 출발 슬립(최대·중간 PWM), 코스트 정지, 역PWM 제동 → 구동력·mu 곡선·구름저항
    'E2': [(0, 0, 0, 2), (0, 255, 255, 2.0), (0, 0, 0, 4),
           (0, 120, 120, 3.0), (0, 0, 0, 4),
           (0, 120, 120, 2.5), (0, -40, -40, 1.0), (0, 0, 0, 2),
           (0, 120, 120, 2.5), (0, -255, -255, 0.6), (0, 0, 0, 2)],
    # 원 선회(바닥, 지름 약 3m 공간): 속도 올리며 횡 접지 한계 → mu_lat, alpha_peak
    'E3': [(0, 0, 0, 2)] + [x for p in (60, 90, 120, 150, 180) for x in ((-7, p, p, 6),)] + [(-7, 0, 0, 3)]
          + [x for p in (60, 120, 180) for x in ((7, p, p, 6),)] + [(7, 0, 0, 3)],
    # 스텝 조향(직진 중 좌우로 꺾기) → 요 관성·조향 속도
    'E4': [(0, 0, 0, 2), (0, 110, 110, 2.5), (-5, 110, 110, 0.8), (5, 110, 110, 0.8), (0, 110, 110, 1.0), (0, 0, 0, 3)],
    # 커브 중 급가속(뒤 밀림)·좌우 PWM 차 → 마찰원·하중 이동 검증
    'E5': [(0, 0, 0, 2), (-5, 80, 80, 3), (-5, 255, 255, 0.8), (0, 0, 0, 3),
           (0, 100, 100, 2), (0, 60, 140, 2), (0, 0, 0, 3)],
}


def main():
    name = sys.argv[1] if len(sys.argv) > 1 else ''
    if name not in SEQ:
        sys.exit('실험 이름: ' + ' '.join(SEQ))
    rclpy.init(args=[a for a in sys.argv[2:] if a != '--sim'] or None)
    n = Node('test_sequence', parameter_overrides=[rclpy.parameter.Parameter('use_sim_time', value='--sim' in sys.argv)])
    pub = n.create_publisher(MotionCommand, 'topic_control_signal', 1)
    stop = MotionCommand(steering=0, left_speed=0, right_speed=0)
    try:
        while rclpy.ok() and n.get_clock().now().nanoseconds == 0:   # 시뮬: /clock 대기
            rclpy.spin_once(n, timeout_sec=0.1)
        total = sum(s[3] for s in SEQ[name])
        n.get_logger().info(f'{name} 시작, {total:.1f}초')
        for steer, pl, pr, secs in SEQ[name]:
            t_end = n.get_clock().now().nanoseconds * 1e-9 + secs
            msg = MotionCommand(steering=int(steer), left_speed=int(pl), right_speed=int(pr))
            while rclpy.ok() and n.get_clock().now().nanoseconds * 1e-9 < t_end:
                pub.publish(msg)
                rclpy.spin_once(n, timeout_sec=1.0 / HZ)
        n.get_logger().info(f'{name} 끝')
    finally:
        for _ in range(5):
            pub.publish(stop)
        n.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
