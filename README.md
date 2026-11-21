# 2027-Gyeonggi-AutoDriving-Competition
2027 경기도 대학생 자율주행 경진대회 온라인 교육

## 1. 초기 환경설정
```
git clone https://github.com/bestbada/automobile_project.git
cd automobile_project/
sh install.sh
source ~/.bashrc
```


## 2. 의존성 설치 (rosdep)
```
cd automobile_project/
export AMENT_PREFIX_PATH=''
export CMAKE_PREFIX_PATH=''
source /opt/ros/humble/setup.bash
rosdep install -i --from-path src --rosdistro humble -y
```

## 3. 패키지 빌드
```
cd automobile_project/
source /opt/ros/humble/setup.bash
colcon build --packages-select interfaces_pkg --allow-overriding interfaces_pkg 
source install/local_setup.bash

colcon build --symlink-install --packages-select camera_perception_pkg --allow-overriding camera_perception_pkg 
source install/local_setup.bash

colcon build --symlink-install --packages-select decision_making_pkg --allow-overriding decision_making_pkg 
source install/local_setup.bash

colcon build --symlink-install --packages-select debug_pkg --allow-overriding debug_pkg 
source install/local_setup.bash

colcon build --symlink-install --packages-select simulation_pkg --allow-overriding simulation_pkg
source install/local_setup.bash
```

## 4. 시뮬레이터 실행
```
cd automobile_project/
sudo killall -9 gazebo gzserver gzclient; ros2 launch simulation_pkg cle53_test.launch.py world:=track_real_noon.world
```

환경 확인(기준 PC와 같으면 PASS 약 89초, touches 0):
```
CV=1 MIN=45 scripts/cle53_expert/run_world.sh track_real_noon.world 60 1.1 150
tail -1 scripts/cle53_expert/results/summary.txt
```
기준 버전은 requirements.txt 머리말 참고. git pull 뒤 setup.py·모델이 바뀌었으면 colcon build와 sh install.sh(~/.gazebo/models 갱신)를 다시 한다.


## 5. 과제 제출
시뮬레이션을 실행해서 차량이 한바퀴 주행하는 영상 촬영해서 eX-campus 과제 및 평가 탭에서 [온라인 과제 제출] 항목에 제출해주세요.
