# Piper-Laundry



Camera (node2.py)   
  ↓  TargetPose (Action)   
basket / Logic_inte_ba / Logic_inte_wm  (Python)   
  ↓  Pose + Mode (PoseGoal Action Client)   
pose_goal.cpp  (PoseGoal Action Server)   
  ↓  JointTrajectory / FollowJointTrajectory   
ROS2 Controller → Robot   



## 추가 설치
```bash
git clone https://github.com/havy-nine/Laundry_decision.git
git clone https://github.com/mcsix/xela_server_ros2.git
git clone https://bitbucket.org/traclabs/trac_ik.git # 일부 파일 include 부분에서 hpp -> h 로 수정해야함
git clone https://github.com/PickNikRobotics/topic_based_ros2_control.git
```
https://xela.lat-d5.com/ 에서 software 최신 버전 설치 (xela_sensor)

촉각 센서 설치는 https://github.com/MinGyuSUH/tactile_xela 를 참고하시면 됩니다.

---

## 기본 세팅

### 파이퍼 연결하기
```bash
sudo ip link set can0 type can bitrate 1000000
sudo ip link set up can0
ros2 run piper piper_single_ctrl --ros-args -p can_port:=can0 -p gripper_exist:=true -p gripper_val_mutiple:=2

ros2 service call /enable_srv piper_msgs/srv/Enable "enable_request: true" # 모터 활성화
```
---

### 촉각센서 연결하기
```bash
sudo dmesg | grep ttyUSB #명령 결과로 나오는 숫자를 아래에 입력 ex)ttyUSB20
sudo slcand -o -s8 -t hw -S 3000000 /dev/ttyUSB0 
sudo ifconfig can1 up

cd mcy_ws/piper-mou/src/xela_sensor # 실행 경로 이동
./xela_conf -d socketcan -c can1 # conf 설정
./xela_server # 서버 실행

python enable.py # raw 값 mode로
./xela_viz # 시각화
ros2 run xela_server_ros2 xela_service.py # /xServTopic 토픽화
```
---

### 파이썬 파일 실행
```bash
ros2 launch piper_with_gripper_moveit wm_demo.launch.py real:=true # moveit 환경 세팅

cd mcy_ws/piper-mou/src/piper_ros/src/piper_moveit/piper_with_gripper_moveit/src # 실행 경로 이동
python node2.py # conda 환경에서 실행 ( conda activate rs )
```
---

## 📂 파일 설명 (piper_with_gripper_moveit/src)

자세한 설명은 https://github.com/MinGyuSUH/piper-laundry/blob/main/piper_ros/README.md 를 참고하시면 됩니다.<br><br>

**basket.py** : Logic_inte_ba.py에 바구니 및 실패 복구 시 충돌 감지가 추가되어 있습니다.<br><br>

**Logic_inte_ba.py** : 실패 복구를 포함한 ba에서 wm으로 가는 실행 파일입니다.

**Logic_inte_wm.py** : wm에서 ba로 가는 실행 파일입니다.<br><br>

**node2.py** : 요청 받아서 좌표 보내주는 파일입니다. (rs) conda에서 실행해야합니다. ( conda activate rs )

더 자세한 건 https://github.com/havy-nine/Laundry_decision 에서 확인할 수 있습니다.<br><br>

**planning_scene.cpp** : 여기서 바구니 위치 등 moveit 환경을 수정할 수 있습니다. <br><br>

**pose_goal.cpp** : PoseGoal 액션으로 주고 받으며 moveit 명령을 전달합니다.<br><br>

**TEST_stop_demo.py** : moveit에서 생성한 경로로 예측한 전류랑 실제 전류랑 비교해서 넘어가면 멈추는 데모 파일 입니다.


