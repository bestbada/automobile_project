#!/bin/bash
# 모방학습 정답 시연 수집: 정답 주행(expert_driver, /ego_pose 기반) + 카메라 기록을 에피소드마다 반복.
# 사용: collect_demos.sh <에피소드 수> [시작 시드=0] [speed=60] [min_speed=51]
# - 월드 5종을 돌아가며 사용, 에피소드마다 SUN_SEED=시드로 햇빛·햇빛 조각 랜덤(천장등·ambient 그대로)
# - 출력: demos/expert_<월드>_s.._g.._sun<시드>_rec/(NNNN_cam.jpg = camera/image_raw(자동노출·역광 적용),
#   log.jsonl = 10Hz 조향·속도 명령, 위치), demos/summary.txt = 에피소드 결과(PASS만 학습에 쓸 것)
# shortcut: 출발 위치는 launch의 SPAWN(T1) 고정. T3 출발·미션1/2 시연은 해당 정답 스크립트가 생기면 추가.
# 주의: 센서 위치 확정 전에는 수집하지 말 것(시점이 바뀌면 데이터 재수집).
N=${1:?에피소드 수}; S0=${2:-0}; SPEED=${3:-60}; MIN=${4:-51}
D=$(cd $(dirname "$0") && pwd)
WORLDS=(track_real track_real_morning track_real_noon track_real_cloudy track_real_night)
for i in $(seq 0 $((N - 1))); do
  s=$((S0 + i)); w=${WORLDS[$((s % ${#WORLDS[@]}))]}
  EXPERT=1 REC=150 SUN_SEED=$s OUT=$D/demos $D/run_world.sh $w.world $SPEED $MIN 150
done
