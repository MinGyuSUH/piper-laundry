# piper-laundry

📌 기본 설치

본인 workspace 안에:

git clone https://github.com/havy-nine/Laundry_decision.git


해당 깃 /src 안에:

git clone https://github.com/mcsix/xela_server_ros2.git
git clone https://bitbucket.org/traclabs/trac_ik.git
git clone https://github.com/PickNikRobotics/topic_based_ros2_control.git

🧩 기본 세팅
🔧 파이퍼 연결하기
sudo ip link set can0 type can bitrate 1000000
sudo ip link set up can0

ros2 run piper piper_single_ctrl --ros-args \
  -p can_port:=can0 \
  -p gripper_exist:=true \
  -p gripper_val_mutiple:=2

# 모터 활성화
ros2 service call /enable_srv piper_msgs/srv/Enable "enable_request: true"

✋ 촉각센서 연결하기
sudo dmesg | grep ttyUSB

sudo slcand -o -s8 -t hw -S 3000000 /dev/ttyUSB0
sudo ifconfig can1 up

cd mcy_ws/piper-mou/src/xela_sensor        # 실행 경로 이동
./xela_conf -d socketcan -c can1           # conf 설정
./xela_server                              # 서버 실행
python enable.py                           # raw 값 mode로
./xela_viz                                 # 시각화
ros2 run xela_server_ros2 xela_service.py   # /xServTopic 토픽화

🐍 파이썬 파일 실행
# MoveIt 환경 세팅
ros2 launch piper_with_gripper_moveit wm_demo.launch.py real:=true

# 실행 경로 이동
cd mcy_ws/piper-mou/src/piper_ros/src/piper_moveit/piper_with_gripper_moveit/src

# (conda 환경에서 실행)
python node2.py     # conda activate rs 필요

📁 파일 설명 (piper_with_gripper_moveit/src)
• basket.py

충돌 감지까지 되어 있습니다.
move_forward 함수에서 mode=9 → mode=6 변경 + 충돌 감지 추가 필요

• Logic_inte_ba.py

ba → wm 실행 파일

실패 복구 포함

• Logic_inte_wm.py

wm → ba 실행 파일

둘 다 k-means / FCM 가능
sensor_callback 함수에 주석 처리해둠 (cluster)
conda 실행 필요
안되면 kroc.py의 sensor_callback 참고

• node2.py

요청 받아서 좌표 보내주는 파일

conda(rs) 환경에서 실행

상세 내용:
https://github.com/havy-nine/Laundry_decision

• pose_goal.cpp

PoseGoal 액션으로 moveit 명령 전달

🔗 링크 구조
link6 <-0.11-> deep <-0.03-> tcp <-0.02-> EEE


해당 경로에서 수정 가능:

piper_ros/src/piper_description/urdf/piper_description.xacro

🎛 mode 별 설명
mode	설명
0	tcp 기준 pose 명령 → moveit 경로 생성
1	tcp 기준 position + 현재 orientation + 각도 제한
6	deep 기준 pose 명령 → 직접 IK 풀어서 경로 생성
8	tcp 기준 pose 명령 → LIN 경로 생성
9	deep 기준 pose 명령 → LIN 경로 생성

모드는 0~9까지 가능

2,3,4,5,7 사용하거나 확장하려면 127번 line 수정 필요

🧪 TEST_stop_demo.py

MoveIt 경로 기준

예측 전류 vs 실제 전류 비교

오차가 넘으면 즉시 정지
