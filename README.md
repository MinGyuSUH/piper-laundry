# 📌 기본 설치

본인 workspace 안에

git clone https://github.com/havy-nine/Laundry_decision.git

해당 깃/src 안에

git clone https://github.com/mcsix/xela_server_ros2.git

git clone https://bitbucket.org/traclabs/trac_ik.git

git clone https://github.com/PickNikRobotics/topic_based_ros2_control.git


# 🧩 기본 세팅


## 🔧 파이퍼 연결하기

sudo ip link set can0 type can bitrate 1000000
sudo ip link set up can0

ros2 run piper piper_single_ctrl --ros-args \
  -p can_port:=can0 \
  -p gripper_exist:=true \
  -p gripper_val_mutiple:=2

# 모터 활성화
ros2 service call /enable_srv piper_msgs/srv/Enable "enable_request: true"


## ✋ 촉각센서 연결하기

sudo dmesg | grep ttyUSB
sudo slcand -o -s8 -t hw -S 3000000 /dev/ttyUSB0
sudo ifconfig can1 up

cd mcy_ws/piper-mou/src/xela_sensor

./xela_conf -d socketcan -c can1
./xela_server

python enable.py   # raw 모드
./xela_viz
ros2 run xela_server_ros2 xela_service.py   # /xServTopic 퍼블리시


## 🐍 파이썬 파일 실행

ros2 launch piper_with_gripper_moveit wm_demo.launch.py real:=true

cd mcy_ws/piper-mou/src/piper_ros/src/piper_moveit/piper_with_gripper_moveit/src

# conda 환경
conda activate rs
python node2.py


# 📁 파일 설명 (piper_with_gripper_moveit/src)

basket.py
- 충돌 감지 포함
- move_forward: mode=9 → mode=6 변경 후 충돌 감지 추가 필요

Logic_inte_ba.py
- ba → wm 이동
- 실패 복구 포함
- k-means / FCM 클러스터링 가능

Logic_inte_wm.py
- wm → ba 이동
- sensor_callback에 clustering 주석 포함
- 필요 시 kroc.py 참고

node2.py
- Vision → PoseGoal 좌표 전달
- conda(rs) 환경 실행 필요
- 자세한 설명: https://github.com/havy-nine/Laundry_decision

pose_goal.cpp
- PoseGoal 액션 처리
- MoveIt 경로 생성 및 전송

🔗 링크 구조
link6 <-0.11-> deep <-0.03-> tcp <-0.02-> EEE
수정 위치: piper_ros/src/piper_description/urdf/piper_description.xacro

🎛 mode 설명
0 : tcp pose → MoveIt 경로 생성
1 : tcp position + ORIENTATION angle limit
6 : deep pose → IK 직접 풀어서 경로 생성
8 : tcp pose → LIN 경로 생성
9 : deep pose → LIN 경로 생성
※ 모드는 0~9까지 가능  
※ 2,3,4,5,7 사용하려면 pose_goal.cpp 127번 라인 수정 필요

TEST_stop_demo.py
- MoveIt 경로 기반 예측 전류 vs 실제 전류 비교
- 4σ 오차 초과하면 즉시 정지하는 데모 파일
