# CLE53 실측 절차 (IMU + 엔코더 4개)

실측값은 `src/simulation_pkg/config/vehicle_dynamics.yaml` 한 곳에 들어가고, 시뮬(`simulation_sender_node`)이 그대로 쓴다.
보정은 Gazebo 없이 `calibrate.py`가 한다(동역학이 전부 `vehicle_model.py` 안에 있음).

## 0. 준비
- **실차 토픽 규격**(실차 브리지가 이 이름·형식으로 발행해야 함, 시뮬도 같은 토픽 발행):
  | 토픽 | 형식 | 내용 |
  |---|---|---|
  | `topic_control_signal` | `interfaces_pkg/MotionCommand` | 조향 -7~7(음수=좌), 좌·우 PWM |
  | `encoder/wheel_speeds` | `std_msgs/Float32MultiArray` | 바퀴 둘레 속도 m/s `[앞왼, 앞오, 뒤왼, 뒤오]` (= 각속도 × 0.095m) |
  | `imu/data` | `sensor_msgs/Imu` | `angular_velocity.z`(+좌회전), `linear_acceleration.x/y`(차체 좌표, 중력 제외, IMU 수평 장착) |
  | `steer/angle` (선택) | `std_msgs/Float32` | 가변저항으로 잰 앞바퀴 조향각 rad(+좌). 있으면 조향 속도를 직접 씀 |
- 배터리 만충전에서 시작, 각 실험 시작 전 전압 기록(저전압 영향은 별도 반복으로 확인).
- 모든 실험은 **정지 상태에서 시작해 정지로 끝난다**(보정 시 초기 상태 = 정지).
- 실행(실차, 두 터미널):
  ```
  python3 scripts/calibration/real_logger.py E2_1.csv 30          # 실험 시간보다 길게
  python3 scripts/calibration/test_sequence.py E2                 # Ctrl-C = 즉시 PWM 0
  ```
  시뮬에서 똑같이 해 보려면 둘 다 `--sim` 추가(`cle53_test.launch.py` 실행 중).

## 1. 정적 측정 (줄자·저울)
| 값 | 방법 | YAML 키 |
|---|---|---|
| 총질량 | 저울 | `mass` |
| 뒷축 하중 비율 | 앞바퀴·뒷바퀴 각각 저울에 올려 뒷축 무게/총무게 | `drive_frac` |
| 무게중심 앞뒤 위치 | `com_x = 축간거리 × (앞축 무게/총무게)` (뒷축에서 앞으로) | `com_x` |
| 무게중심 높이 | 뒷바퀴를 높이 h만큼 들고 앞축 무게 변화로 계산(또는 추정 유지) | `cg_h` |
| 축간거리, 뒷바퀴 좌우 간격 | 줄자(바퀴 접지 중심 사이) | `wheelbase`, `track` |
| 바퀴 지름 | 줄자(엔코더 → m/s 환산에도 사용) | (현재 0.19m) |

## 2. 주행 실험
| 실험 | 조건·공간 | 시퀀스(test_sequence.py) | 구하는 값 |
|---|---|---|---|
| **E1 무부하** | 차 **뒤를 들어** 뒷바퀴가 공중에서 돌게(앞바퀴는 바닥) | PWM 20,40,80,…,255, -40…-255 각 3초 + 정지 3초 | 속도표(데드존 포함) `speed_table_*`, `motor_tau`, `cmd_delay` |
| **E2 직진** | 직선 6m 이상, 실제 트랙 매트 위 | 최대 PWM 출발 → 코스트, 중간 PWM 출발 → 코스트, 약한 역PWM 제동, 최대 역PWM 제동 | `drive_force_max`, `mu_peak`, `s_peak`, `mu_slide`, `c_rr` (출발·주행·제동 슬립) |
| **E3 원 선회** | 지름 3m 이상 빈 공간 | 최대 조향 좌로 PWM 60→180, 우로 60→180 | `mu_lat`, `alpha_peak` (커브 슬립) |
| **E4 스텝 조향** | 직선 5m | 직진 중 좌 → 우 → 직진 | `yaw_inertia`, `steer_rate` |
| **E5 커브 급가속·좌우 PWM 차** | 지름 3m + 직선 4m | 커브 중 최대 PWM(뒤 밀림), 직진 중 좌60/우140 | 마찰원·하중 이동·좌우 구동 검증 |
- 각 실험 3회 반복 → 보정 결과 편차로 신뢰도 확인. 바닥 위치별 마찰 차이를 보려면 E2를 트랙 여러 구간에서 반복.

## 3. 보정
```
python3 scripts/calibration/calibrate.py --E1 E1_1.csv --E2 E2_1.csv --E3 E3_1.csv --E4 E4_1.csv --E5 E5_1.csv \
    --fixed mass=…,drive_frac=…,com_x=…,track=…,cg_h=…
# → scripts/calibration/vehicle_dynamics.calibrated.yaml (확인 후 config/vehicle_dynamics.yaml 로 복사)
python3 scripts/calibration/calibrate.py --selftest   # 보정 코드 자체 점검(가짜 실측으로 정답 회복)
```
- 출력의 "정규화 RMS"는 잔차를 센서 잡음 수준(엔코더 0.01 m/s, 요레이트 0.005 rad/s, 가속도 0.05 m/s²)으로 나눈 값. 1~3이면 잡음 수준까지 맞은 것.
- E2 출력의 "슬립률"은 앞바퀴(비구동) 엔코더를 IMU 가속도와 상보 필터로 합친 차체 속도 기준이다(앞바퀴가 IMU 적분 거리 오차를 보정).

## 미확인·주의
- 실차 엔코더·IMU 브리지(Arduino → ROS) 코드는 아직 없다. 위 토픽 규격대로 만들어야 기록기가 동작한다.
- 조향 명령 → 앞바퀴 각도는 지금 직선(±7 ↔ ±0.45rad) 가정. `steer/angle`로 실측해 표로 바꿀 것.
- 앞바퀴 속도 → 차체 속도 변환은 운동학 근사(cos δ). 큰 슬립각 선회 중에는 오차가 있다(보정은 원시 센서값을 직접 맞추므로 영향 없음, 슬립률 보고에만 영향).
