# 📌 기본 설치

본인 workspace 안에

```bash
git clone https://github.com/havy-nine/Laundry_decision.git
```

해당 깃/src 안에

```bash
git clone https://github.com/mcsix/xela_server_ros2.git
git clone https://bitbucket.org/traclabs/trac_ik.git
git clone https://github.com/PickNikRobotics/topic_based_ros2_control.git
```


# 🧩 기본 세팅

## 🔧 파이퍼 연결하기

```bash
sudo ip link set can0 type can bitrate 1000000
sudo ip link set up can0

ros2 run piper piper_single_ctrl --ros-args \
  -p can_port:=can0 \
  -p gripper_exist:=true \
  -p gripper_val_mutiple:=2

# 모터 활성화
ros2 service call /enable_srv piper_msgs/srv/Enable "enable_request: true"
```


## ✋ 촉각센서 연결하기

```bash
sudo dmesg | grep ttyUSB
sudo slcand -o -s8 -t hw -S 3000000 /dev/ttyUSB0
sudo ifconfig can1 up

cd mcy_ws/piper-mou/src/xela_sensor

./xela_conf -d socketcan -c can1
./xela_server
python enable.py      # raw 값 mode
./xela_viz
ros2 run xela_server_ros2 xela_service.py    # /xServTopic 토픽화
```


## 🐍 파이썬 파일 실행

```bash
ros2 launch piper_with_gripper_moveit wm_demo.launch.py real:=true

cd mcy_ws/piper-mou/src/piper_ros/src/piper_moveit/piper_with_gripper_moveit/src

# conda 환경
conda activate rs
python node2.py
```


---

# 📁 파일 설명 (piper_with_gripper_moveit/src)

### basket.py
- move_forward 함수에서 **mode=9 → mode=6으로 변경**
- 충돌 감지 추가 필요

### Logic_inte_ba.py
- 실패 복구를 포함한 **ba → wm 이동 로직**

### Logic_inte_wm.py
- **wm → ba 이동 로직**
- sensor_callback()에 clustering 주석 존재  
- 필요 시 `kroc.py` 참고  
- k-means / FCM 모두 가능 (conda 실행 필요)

### node2.py
- 요청 받아서 좌표 보내주는 파일  
- conda(rs)에서 실행  
- 자세한 내용: https://github.com/havy-nine/Laundry_decision

### pose_goal.cpp
- PoseGoal 액션으로 MoveIt 명령 전달


---

# 🔗 링크 구조

현재 링크 구조는 다음과 같습니다:

```
link6 <-0.11-> deep <-0.03-> tcp <-0.02-> EEE
```

링크 수정은 다음 경로에서 가능합니다:

```
깃/src/piper_ros/src/piper_description/urdf/piper_description.xacro
```


---

# ⚙️ mode 별 설명

```
0 : tcp기준으로 pose 명령 받아서 moveit으로 경로 생성
1 : tcp기준으로 position 과 현재 ORIENTATION으로 부터 각도 제한
6 : deep기준으로 pose 명령 받아서 직접 IK 풀어서 경로 생성
8 : tcp기준으로 pose 명령 받아서 LIN 으로 경로 생성
9 : deep기준으로 pose 명령 받아서 LIN 으로 경로 생성
```

---

## 📑 mode 표 (가독성 강화)

| mode | 기준 | 설명 |
|------|------|------|
| **0** | tcp | pose 명령 → MoveIt 경로 생성 |
| **1** | tcp | position + ORIENTATION 각도 제한 |
| **6** | deep | IK 직접 풀어서 경로 생성 |
| **8** | tcp | LIN(직선) 경로 생성 |
| **9** | deep | LIN(직선) 경로 생성 |

- 모드는 **9번까지 가능**
- 2,3,4,5,7 사용하려면 **pose_goal.cpp 127번 라인 수정**


---

# 🧪 TEST_stop_demo.py

- MoveIt에서 생성한 경로 기반  
- **예측 전류 vs 실제 전류 비교**
- threshold 넘으면 자동 정지  
