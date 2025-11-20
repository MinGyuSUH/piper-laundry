 **기본 설치**

 본인 workspace 안에

 git clone https://github.com/havy-nine/Laundry_decision.git

 해당 깃/src 안에

 git clone https://github.com/mcsix/xela_server_ros2.git

 git clone https://bitbucket.org/traclabs/trac_ik.git

 git clone https://github.com/PickNikRobotics/topic_based_ros2_control.git


 **기본 세팅**

 파이퍼 연결하기

 sudo ip link set can0 type can bitrate 1000000

 sudo ip link set up can0

 ros2 run piper piper_single_ctrl --ros-args -p can_port:=can0 -p gripper_exist:=true -p gripper_val_mutiple:=2

 ros2 service call /enable_srv piper_msgs/srv/Enable "enable_request: true"   #모터 활성화


 촉각센서 연결하기

 sudo dmesg | grep ttyUSB

 sudo slcand -o -s8 -t hw -S 3000000 /dev/ttyUSB0

 sudo ifconfig can1 up

 cd mcy_ws/piper-mou/src/xela_sensor    # 실행 경로 이동

 ./xela_conf -d socketcan -c can1       # conf 설정

 ./xela_server                          # 서버 실행

 python enable.py                       # raw 값 mode로

 ./xela_viz                             # 시각화

 ros2 run xela_server_ros2 xela_service.py   # /xServTopic 토픽화


 파이썬 파일 실행

 ros2 launch piper_with_gripper_moveit wm_demo.launch.py real:=true   # moveit 환경 세팅

 cd mcy_ws/piper-mou/src/piper_ros/src/piper_moveit/piper_with_gripper_moveit/src   # 실행 경로 이동

 python node2.py     # conda 환경에서 실행 (conda activate rs)



 **파일 설명 (piper_with_gripper_moveit/src)**

 move_forward 함수에서 mode=9를 mode=6으로 바꾸고 충돌 감지를 추가해야 합니다.   (basket.py)

 Logic_inte_ba.py : 실패 복구를 포함한 ba에서 wm으로 가는 실행 파일입니다.

 Logic_inte_wm.py : wm에서 ba로 가는 실행 파일입니다.

 -> 둘 다 k-means fcm도 가능하고 sensor_callback 함수에 주석 처리 해놨습니다. (cluster) conda에서 실행해야 합니다.
    혹시 안되면 kroc.py 의 sensor_callback 함수 참고하시면 됩니다.

 node2.py : 요청 받아서 좌표 보내주는 파일입니다. (rs) conda에서 실행해야합니다.
            더 자세한 건 https://github.com/havy-nine/Laundry_decision 에서 확인할 수 있습니다.

 pose_goal.cpp : PoseGoal 액션으로 주고 받으며 moveit 명령을 전달합니다.

 현재 링크 구조 : link6 <-0.11-> deep <-0.03-> tcp <-0.02-> EEE
 링크 구조는 해당 깃/src/piper_ros/src/piper_description/urdf/piper_description.xacro 에서 수정할 수 있습니다.

 mode 별 설명입니다.
 0 : tcp기준으로 pose 명령 받아서 moveit으로 경로 생성
 1 : tcp기준으로 position 과 현재 ORIENTATION으로 부터 각도 제한
 6 : deep기준으로 pose 명령 받아서 직접 IK 풀어서 경로 생성
 8 : tcp기준으로 pose 명령 받아서 LIN 으로 경로 생성
 9 : deep기준으로 pose 명령 받아서 LIN 으로 경로 생성

 모드는 9번까지 가능하도록 되어있습니다.
 2,3,4,5,7을 쓰고 더 쓰려면 pose_goal.cpp 127번 line을 수정해야 합니다.

 TEST_stop_demo.py :
 moveit에서 생성한 경로로 예측한 전류랑 실제 전류랑 비교해서 넘어가면 멈추는 데모 파일 입니다.
