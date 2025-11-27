
### 🔹 pose_goal.cpp
**: PoseGoal 액션으로 주고 받으며 moveit 명령을 전달합니다.**  


현재 링크 구조  
link6 <-0.11-> deep <-0.03-> tcp <-0.02-> EEE 

링크 구조는 piper_ros/src/piper_description/urdf/piper_description.xacro 에서 수정할 수 있습니다.

mode 별 설명입니다.

| Mode | 설명 |
|------|------|
| 0 | tcp기준으로 pose 명령 받아서 moveit으로 경로 생성 |
| 1 | tcp기준으로 position 과 현재 ORIENTATION으로 부터 각도 제한 |
| 6 | deep기준으로 pose 명령 받아서 직접 IK 풀어서 경로 생성 |
| 7 | tcp기준으로 pose 명령 받아서 직접 IK 풀어서 경로 생성 |
| 8 | tcp기준으로 pose 명령 받아서 LIN 으로 경로 생성 |
| 9 | deep기준으로 pose 명령 받아서 LIN 으로 경로 생성 |

모드는 9번까지 가능하도록 되어있습니다. 2,3,4,5를 쓰고 더 쓰려면 127번 line을 수정해야 합니다.<br><br>

---

### 🔹 Logic_inte_ba.py
**: 실패 복구를 포함한 ba에서 wm으로 가는 실행 파일입니다.**  


#### 0) 센서 리셋 타이밍 (중요)
잡기/접촉 판단은 **누적 벡터 기반**이라, 시도 시작 전에 잔상 제거가 필요합니다.  
각 grasp/approach 시도 전에 아래를 **반드시 호출**하세요.

- `reset_sensor_data()`  
  : 누적 taxel 벡터(`cum_x/y/z`)와 판단 플래그를 초기화

#### 1) 기본 조인트 이동 함수
- `goal(q_8dof, gripper_action='keep')`  
  : **조인트 space 이동 함수**  
  - 앞 **6축(arm)** → MoveIt FollowJointTrajectory  
  - **7~8축(gripper)** → 별도 Gripper 액션 서버 제어

**gripper_action 옵션**

| 값 | 기능 |
|-----|--------|
| `'keep'` | 그리퍼 상태 유지 (기본값) |
| `'open'` | `open_gripper()` 호출 → 그리퍼 오픈 |
| `'close'` | `close_gripper()` 호출 → 그리퍼 클로즈 |


#### 2) TargetPose 수신 함수 (node2.py와 통신)
아래 함수들은 **TargetPose 액션 서버(node2.py)** 에게 target을 요청해서 받아오는 역할입니다.

- `target_get_bg()`  
  : 배경/기준 프레임 저장용 요청 (`mode="failure_bg"`).  
  grasp 전에 background를 갱신해 segmentation 안정화에 사용됩니다.

- `target_get_ba()`  
  : basket 영역 기준 target pose 요청 (`mode="basket"`).  
  → EE pose(`/end_pose`)와 ee2cam 캘리브레이션을 이용해 base 좌표로 변환합니다.

- `target_get_wm()`  
  : wm 영역 기준 target pose 요청 (`mode="failure"`).  
  → 동일하게 base 좌표로 변환 후 실패복구에 사용합니다.


#### 3) PoseGoal 모드 매핑
- `send_pose_ba()` → `mode = 1`  
  : position + RPY orientation constraint (tcp 기준)

- `send_pose_wm()` → `mode = 0`  
**: 실패 복구를 포함한 ba에서 wm으로로
  : position + RPY orientation 고정 (tcp 기준)

- `move_forward()` → `mode = 9`  
  : RPY orientation 고정 LIN 직선 이동(DEEP 기준)


### 🔹 Logic_inte_wm.py
**위의 Logic_inte_ba.py와 함수 이름 빼고 동일**  

#### 2) TargetPose 수신 함수

- `target_get()`  
  : wm 영역 기준 target pose 요청 (`mode="wm"`).  

#### 3) PoseGoal 모드 매핑
- `send_pose()` → `mode = 1`  
  : position + RPY orientation constraint (tcp 기준)

- `move_forward()` → `mode = 9`  
  : RPY orientation 고정 LIN 직선 이동(DEEP 기준)<br><br>
  

**→ 둘 다 k-means, fcm도 가능하고 sensor_callback 함수에 주석 처리 해놨습니다.** (cluster) conda 에서 실행해야 합니다. ( conda activate cluster )

#### 접촉/정렬 판단 방식 선택 (Heuristic / KMeans / FCM)
판단 결과는 **아래 flag 중 무엇을 보느냐**로 방식이 결정됩니다.

- **Heuristic (기본)**
  - 파지(정렬) : `z_aligned`
  - 슬립/접촉 : `contacted`

- **KMeans**
  - 파지(정렬) : `z_aligned_k`
  - 슬립/접촉 : `contacted_kk` 

- **FCM**
  - 파지(정렬) : `z_aligned_f`
  - 슬립/접촉 : `contacted_ff`


self._log_state_table("grasp-check") 함수를 활용하면 3가지 방식을 쉽게 비교할 수 있습니다.

혹시 안되면 kroc.py 의 sensor_callback 함수 참고하시면 됩니다.

---

### 🔹 node2.py
**더 자세한 건 (https://github.com/havy-nine/Laundry_decision) 에서 확인할 수 있습니다.**  


---


### 🔹 TEST_stop_demo.py
**: moveit에서 생성한 경로로 예측한 전류랑 실제 전류랑 비교해서 넘어가면 멈추는 데모 파일 입니다.**  


- `/end_pose`를 받아 target pose 갱신
- pose_goal(mode 8 → mode 6) 반복 이동
- 100Hz 예측 토크 계산 + 실 Effort 비교
- 4σ 이상 오차 발생 시 즉시 정지(cancel + override)
- `q` = 강제 정지 / `s` = 재시작
- 실시간 토크 플롯 시각화 포함

---
