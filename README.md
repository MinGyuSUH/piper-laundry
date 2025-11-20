📌 기본 설치

본인 workspace 안에

git clone https://github.com/havy-nine/Laundry_decision.git


해당 깃/src 안에

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
ros2 launch piper_with_gripper_moveit wm_demo.launch.py real:=true   # MoveIt 환경 세팅

cd mcy_ws/piper-mou/src/piper_ros/src/piper_moveit/piper_with_gripper_moveit/src   # 실행 경로 이동

# conda 환경에서 실행
python node2.py

📁 파일 설명 (piper_with_gripper_moveit/src)
basket.py
- 충돌 감지까지 되어 있음
- move_forward 함수:
    mode=9 → mode=6 변경
    + 충돌 감지 기능 추가 필요

Logic_inte_ba.py
- ba → wm 이동 로직
- 실패 복구 기능 포함

Logic_inte_wm.py
- wm → ba 이동 로직
- k-means / FCM 클러스터링 모두 가능
- sensor_callback 내부 주석 참고 (cluster)


둘 다 conda 환경에서 실행해야 함
필요 시 kroc.py 의 sensor_callback 참고

node2.py
- Vision → PoseGoal 좌표 명령 노드
- conda(rs) 환경에서 실행
- 자세한 구조: https://github.com/havy-nine/Laundry_decision

pose_goal.cpp
- PoseGoal 액션 정의
- MoveIt trajectory 생성/전달

🔗 링크 구조
link6 <-0.11-> deep <-0.03-> tcp <-0.02-> EEE


수정 경로:

piper_ros/src/piper_description/urdf/piper_description.xacro

🎛 mode 별 설명
mode	설명
0	tcp 기준 pose → moveit 경로 생성
1	tcp 기준 position + orientation 기반 각도 제한
6	deep 기준 pose → 직접 IK 풀어서 경로 생성
8	tcp 기준 pose → LIN 경로 생성
9	deep 기준 pose → LIN 경로 생성
- 모드는 0~9까지 지원
- 2,3,4,5,7 사용하려면 127번 line 수정 필요

🧪 TEST_stop_demo.py
- MoveIt 경로 기반 토크/전류 예측
- 예측 vs 실제 비교
- 오차가 4σ 넘으면 즉시 정지
