#!/usr/bin/env python3
"""실측 기록: 명령·엔코더 4개·IMU를 100Hz CSV로. 실차·시뮬 공용(시뮬 sender가 같은 토픽을 발행).
사용: real_logger.py <출력.csv> <초> [--sim]
토픽 규격(실차 브리지가 맞춰야 함 — 확인 필요):
  topic_control_signal (interfaces_pkg/MotionCommand)       조향 -7~7, left/right_speed PWM
  encoder/wheel_speeds (std_msgs/Float32MultiArray)          바퀴 둘레 속도 m/s [앞왼, 앞오, 뒤왼, 뒤오]
  imu/data (sensor_msgs/Imu)                                 angular_velocity.z, linear_acceleration.x/y(차체 좌표, 중력 제외)
  steer/angle (std_msgs/Float32, 선택)                       가변저항으로 잰 앞바퀴 조향각 rad(+좌)
CSV 열: t, steer_cmd, pwm_l, pwm_r, enc_fl, enc_fr, enc_rl, enc_rr, gyro_z, acc_x, acc_y, steer_meas(없으면 nan)"""
import sys
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32, Float32MultiArray
from interfaces_pkg.msg import MotionCommand

COLS = 't,steer_cmd,pwm_l,pwm_r,enc_fl,enc_fr,enc_rl,enc_rr,gyro_z,acc_x,acc_y,steer_meas'
NAN = float('nan')


def main():
    out, secs = sys.argv[1], float(sys.argv[2])
    rclpy.init()
    n = Node('real_logger', parameter_overrides=[rclpy.parameter.Parameter('use_sim_time', value='--sim' in sys.argv)])
    v = dict(cmd=(0, 0, 0), enc=(NAN,) * 4, imu=(NAN,) * 3, steer=NAN)
    n.create_subscription(MotionCommand, 'topic_control_signal',
                          lambda m: v.update(cmd=(m.steering, m.left_speed, m.right_speed)), 10)
    n.create_subscription(Float32MultiArray, 'encoder/wheel_speeds', lambda m: v.update(enc=tuple(m.data[:4])), 10)
    n.create_subscription(Imu, 'imu/data', lambda m: v.update(
        imu=(m.angular_velocity.z, m.linear_acceleration.x, m.linear_acceleration.y)), 10)
    n.create_subscription(Float32, 'steer/angle', lambda m: v.update(steer=m.data), 10)
    f = open(out, 'w')
    f.write(COLS + '\n')
    st = {}

    def tick():
        t = n.get_clock().now().nanoseconds * 1e-9
        st.setdefault('t0', t)
        row = (t - st['t0'],) + v['cmd'] + v['enc'] + v['imu'] + (v['steer'],)
        f.write(','.join(f'{x:.5f}' if isinstance(x, float) else str(x) for x in row) + '\n')
        if t - st['t0'] >= secs:
            raise SystemExit

    while rclpy.ok() and n.get_clock().now().nanoseconds == 0:   # 시뮬: /clock 대기
        rclpy.spin_once(n, timeout_sec=0.1)
    n.create_timer(0.01, tick)
    try:
        rclpy.spin(n)
    except (SystemExit, KeyboardInterrupt):
        pass
    finally:
        f.close()
        n.destroy_node()
        rclpy.try_shutdown()
    print('saved', out)


if __name__ == '__main__':
    main()
