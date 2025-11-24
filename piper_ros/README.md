
### 🔹 pose_goal.cpp
**설명 : PoseGoal 액션으로 주고 받으며 moveit 명령을 전달합니다.**  


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


### 🔹 basket.py
**설명:**  


---

### 🔹 Logic_inte_ba.py
**설명:**  
(여기에 설명을 넣을 예정)

---

### 🔹 Logic_inte_wm.py
**설명:**  
(여기에 설명을 넣을 예정)

---

### 🔹 node2.py
**설명:**  
(여기에 설명을 넣을 예정)

---


### 🔹 TEST_stop_demo.py
**설명:**  
(여기에 설명을 넣을 예정)

---
