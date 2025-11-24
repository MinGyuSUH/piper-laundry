
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
| 8 | tcp기준으로 pose 명령 받아서 LIN 으로 경로 생성 |
| 9 | deep기준으로 pose 명령 받아서 LIN 으로 경로 생성 |

모드는 9번까지 가능하도록 되어있습니다. 2,3,4,5,7을 쓰고 더 쓰려면 127번 line을 수정해야 합니다.<br><br>

---

### 🔹 Logic_inte_ba.py
**: 실패 복구를 포함한 ba에서 wm으로 가는 실행 파일입니다.**  


#### 0) 센서 리셋 타이밍 (중요)
잡기/접촉 판단은 **누적 벡터 기반**이라, 시도 시작 전에 잔상 제거가 필요합니다.  
각 grasp/approach 시도 전에 아래를 **반드시 호출**하세요.

- `reset_sensor_data()`  
  : 누적 taxel 벡터(`cum_x/y/z`)와 판단 플래그를 초기화


#### 1) PoseGoal 모드 매핑
- `send_pose_ba()` → `mode = 1`  
  : position + RPY orientation constraint (tcp 기준)

- `send_pose_wm()` → `mode = 0`  
**: 실패 복구를 포함한 ba에서 wm으구
  : position + RPY orientation 고정 (tcp 기준)

- `move_forward()` → `mode = 9`  
  : RPY orientation 고정 LIN 직선 이동(DEEP 기준)


#### 2) 접촉/정렬 판단 방식 선택 (Heuristic / KMeans / FCM)
판단 결과는 **아래 flag 중 무엇을 보느냐**로 방식이 결정됩니다.

- **Heuristic (기본)**
  - 파지(정렬) : `z_aligned`
  - 슬립/접촉 : `contacted`

- **KMeans**
  - 파지(정렬) : `z_aligned_k`
  - 슬립/접촉 : `contacted_kk`  *(strong contact 누적 플래그)*

- **FCM**
  - 파지(정렬) : `z_aligned_f`
  - 슬립/접촉 : `contacted_ff`  *(soft contact 누적 플래그)*



### 🔹 Logic_inte_wm.py
**위와 거의 동일**  

#### 1) PoseGoal 모드 매핑
- `send_pose()` → `mode = 1`  
  : position + RPY orientation constraint (tcp 기준)
  

→ 둘 다 k-means fcm도 가능하고 sensor_callback 함수에 주석 처리 해놨습니다. (cluster) conda 에서 실행해야 합니다. ( conda activate cluster )

혹시 안되면 kroc.py 의 sensor_callback 함수 참고하시면 됩니다.

---

### 🔹 node2.py
**더 자세한 건 (https://github.com/havy-nine/Laundry_decision) 에서 확인할 수 있습니다.**  


---


### 🔹 TEST_stop_demo.py
**설명:**  
(여기에 설명을 넣을 예정)

---
