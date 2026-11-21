# 프로젝트 작업 규칙 (CLE53 자율주행)

## Gazebo 시뮬 실행 (셸에서 시험할 때 필수)
Gazebo가 시작 중이나 주행 중에 꺼지는 경우가 있다. 셸(특히 백그라운드)로 시뮬을 띄워 두면 그냥 기다리지 말고 아래를 지킨다.

- **일정 주기로 확인**: 실행 후 1분 안에 한 번, 그 뒤 2~3분마다 다음을 확인하고 사용자에게 상태를 한 줄로 알린다.
  - `pgrep -x gzserver` 가 살아 있는지
  - launch 로그에 `process has died`, `[Err]`, `Traceback`, `Service /spawn_entity unavailable` 이 없는지
  - 결과 파일(`scripts/cle53_expert/results/summary.txt`, CSV 등)이 늘어나고 있는지
  하나라도 이상하면 즉시 원인을 보고하고, 멈춘 실행은 끈다. "기다리는 중"으로 오래 두지 않는다.
- **모든 대기에 시간 제한**: launch·시험 스크립트·`ros2 topic echo` 등은 `timeout` 으로 감싼다. 무한 대기 루프 금지.
- **대기 조건에 `pgrep -f <패턴>` 금지**: 자기 명령줄에도 매칭돼 영원히 안 끝난다. PID(`kill -0 $PID`)나 결과 파일로 확인한다.
- **정리는 프로세스 그룹 전체**: launch만 죽이면 자식 노드(sender, camera_sim 등)가 고아로 남아 `/cmd_vel` 에 0을 섞는다.
  스크립트는 `setsid -w` 로 실행하고 `trap "...; kill -9 -- -\$\$" EXIT` 로 그룹 전체를 끈다.
  시험 시작 전·후에 `ps -eo pid,cmd | grep -E "simulation_sender|camera_sim|gzserver"` 로 남은 것이 없는지 확인한다.
- **동시에 두 시뮬 실행 금지**: 서로 Gazebo를 꺼서 결과가 오염된다.
- `gazebo` 명령에 `-p` 를 쓰지 않는다(로그 재생 옵션). ROS 파라미터는 `--ros-args --params-file` 로.
- 모델·월드·YAML 수정 후 `colcon build --packages-select simulation_pkg` 하고, `install/` 쪽 파일이 바뀌었는지 `cmp` 로 확인한다(가끔 안 덮어씀).
