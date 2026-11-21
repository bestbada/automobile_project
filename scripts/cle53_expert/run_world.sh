#!/bin/bash
# 사용: [EXPERT=1] [REC=초] [SUN_SEED=n] [OUT=폴더] run_world.sh <world> <speed> <gain 또는 EXPERT면 min_speed> [timeout_s]
# SUN_SEED: Gazebo 뜬 뒤 주행 전에 set_light.py --random n (햇빛 방향·세기 + 햇빛 조각) 적용, 값은 $TAG.sun.txt
# CV=1: 영상처리 차선(lane_cv_node)→motion_planner(lookahead=$3) / EXPERT=1: 정답주행(expert_driver, /ego_pose 기반) / 기본: YOLO→lane→path→motion 파이프라인. REC: 진단 기록(recorder.py)
W=$1; SPEED=$2; GAIN=$3; TO=${4:-600}
D=$(cd $(dirname "$0") && pwd); ROOT=$(cd "$D/../.." && pwd); cd "$ROOT"; OUT=${OUT:-$D/results}; mkdir -p $OUT; TAG=${EXPERT:+expert_}${CV:+cv_}${SLIP:+slip_}${W%.world}_s${SPEED}_g${GAIN}${SUN_SEED:+_sun$SUN_SEED}
source /opt/ros/humble/setup.bash; source "$ROOT/install/setup.bash"
MODEL=$ROOT/src/simulation_pkg/simulation_pkg/data/sim.pt
cleanup() { pkill -f "ros2 launch simulation_pkg"; pkill -f yolov8_node; pkill -f lane_info_extractor; pkill -f path_planner_node; pkill -f motion_planner_node; pkill -f lane_cv_node; pkill -f camera_sim_node; pkill -f sim_simulation_sender; pkill -f recorder.py; pkill -f expert_driver.py; killall -q gzserver gzclient; sleep 2; killall -9 -q gzserver gzclient; for i in $(seq 30); do ss -ltn | grep -q :11345 || break; sleep 1; done; }
cleanup
ros2 launch simulation_pkg cle53_test.launch.py world:=$W viewer:=false > $OUT/$TAG.launch.log 2>&1 &
for i in $(seq 40); do timeout 3 ros2 topic echo --once /ego_pose >/dev/null 2>&1 && break; sleep 1; done
# SLIP=1: 슬립 모델 켜고 주행(값은 vehicle_dynamics.yaml의 추정값, motor_tau 0.15)
[ -n "$SLIP" ] && ros2 param set /simulation_sender_node motor_tau 0.15 > /dev/null && ros2 param set /simulation_sender_node slip_enable true > /dev/null
[ -n "$SUN_SEED" ] && python3 $D/set_light.py --random $SUN_SEED > $OUT/$TAG.sun.txt 2>&1
if [ -n "$CV" ]; then
ros2 run camera_perception_pkg lane_cv_node > $OUT/$TAG.cv.log 2>&1 &
ros2 run decision_making_pkg motion_planner_node --ros-args -p speed:=$SPEED -p lookahead:=$GAIN -p min_speed:=${MIN:-$SPEED} > $OUT/$TAG.motion.log 2>&1 &
elif [ -n "$EXPERT" ]; then
python3 $D/expert/expert_driver.py --ros-args -p speed:=$SPEED -p min_speed:=$GAIN > $OUT/$TAG.expert.log 2>&1 &
else
ros2 run camera_perception_pkg yolov8_node --ros-args -p model:=$MODEL -p device:=cuda:0 > $OUT/$TAG.yolo.log 2>&1 &
ros2 run camera_perception_pkg lane_info_extractor_node --ros-args -p show_image:=false > $OUT/$TAG.lane.log 2>&1 &
ros2 run decision_making_pkg path_planner_node > /dev/null 2>&1 &
ros2 run decision_making_pkg motion_planner_node --ros-args -p speed:=$SPEED -p lookahead:=$GAIN -p min_speed:=${MIN:-$SPEED} > $OUT/$TAG.motion.log 2>&1 &
fi
[ -n "$REC" ] && python3 $D/recorder.py $OUT/${TAG}_rec $REC ${EXPERT:+cam}${CV:+cam} > $OUT/$TAG.rec.log 2>&1 &
R=$(python3 $D/lap_monitor.py $OUT/$TAG.csv $TO)
echo "$TAG $R" | tee -a $OUT/summary.txt
cleanup
