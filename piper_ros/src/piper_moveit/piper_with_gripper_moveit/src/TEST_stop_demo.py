#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import time
import threading
import sys
from collections import deque
from pathlib import Path

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import (
    QoSProfile,
    QoSReliabilityPolicy,
    QoSHistoryPolicy,
    QoSDurabilityPolicy,
)

from sensor_msgs.msg import JointState
from control_msgs.msg import JointTrajectoryControllerState
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Pose, PoseStamped

from piper_with_gripper_moveit.action import PoseGoal

import pinocchio as pin
import matplotlib.pyplot as plt
import matplotlib.animation as animation


# ========= 사용자 설정 =========
URDF_OR_XACRO = "/home/dyros/mcy_ws/piper-mou/src/piper_ros/src/piper_description/urdf/piper_description.xacro"

# URDF 상 조인트 이름 (예측/Pinocchio용, 8개라고 가정)
URDF_JOINT_NAMES = [f"joint{i+1}" for i in range(8)]

# 실제 제어용 조인트 (FollowJointTrajectory, PoseGoal 등에서 사용하는 6개)
CTRL_JOINT_NAMES = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']

# 토크 플롯에 사용할 joint index (URDF_JOINT_NAMES 기준 index)
DISPLAY_IDX = [3, 4]  # joint4, joint5

HISTORY_LEN = 1000
Y_LIM = (0, 3)
GRAVITY = np.array([0.0, 0.0, -9.81])
PRED_RATE_HZ = 100.0  # 예측 토크 계산 주기(Hz)

SIGMA = 0.0856        # 토크 표준편차 (N·m)
FOUR_SIGMA = 4.0 * SIGMA

# A, B 행렬 (예측 토크 보정용)
A = np.array([
    [0.336031, -0.046468,  0.258372,  0.180103, -0.022022, -1.702219],
    [0.11016,  -0.00126,  -0.015487,  1.4827,    0.00906,   0.558599],
    [-0.009071, -0.001866, -0.026797, -0.026905,  1.37266,   0.943661]
])
B = np.array([0.24018,  0.031745, 0.161793])


def build_pin_model(urdf_or_xacro: str):
    p = Path(urdf_or_xacro).expanduser().resolve()
    if p.suffix == ".xacro":
        import xacro
        xml = xacro.process_file(str(p)).toxml()
        mdl = pin.buildModelFromXML(xml)
    elif p.suffix == ".urdf":
        mdl = pin.buildModelFromUrdf(str(p))
    else:
        raise ValueError(f"Unsupported robot description: {p}")
    mdl.gravity.linear = np.array(GRAVITY, dtype=float)
    return mdl


class PredictControlSafetyNode(Node):
    """
    1번 + 2번 + 3번 기능을 한 노드로 합친 클래스.

    - /arm_controller/state + /joint_states → Pinocchio RNEA로 예측 토크, 실 effort 비교, 플롯
    - /end_pose 구독 → pose_goal 액션으로 반복 모션 (2번 방식)
    - 예측-실제 오차가 4σ를 넘으면 → 3번처럼 FollowJointTrajectory override로 즉시 정지
    - 키보드: q → 강제 정지, s → 다시 모션 재시작
    """

    def __init__(self):
        super().__init__("predict_control_safety_node")

        # ---------------- Pinocchio 관련 ----------------
        self.model = build_pin_model(URDF_OR_XACRO)
        self.data = self.model.createData()
        self.nj = len(URDF_JOINT_NAMES)

        # desired 상태 버퍼
        self.qd = np.zeros(self.nj)
        self.qdotd = np.zeros(self.nj)
        self.qddotd = np.zeros(self.nj)
        self.have_desired = False

        # effort 및 joint state 버퍼
        self.effort = np.zeros(self.nj)
        self.effort_last_valid = np.zeros(self.nj)
        self.last_js_msg = None  # JointState 전체 메시지 저장 (정지 override 시 사용)

        # 플롯용 히스토리
        self.tau_pred_hist = {j: deque(maxlen=HISTORY_LEN) for j in DISPLAY_IDX}
        self.tau_real_hist = {j: deque(maxlen=HISTORY_LEN) for j in DISPLAY_IDX}

        # 4σ 영역 fill_between 핸들 저장
        self.fill_4sigma = {j: None for j in DISPLAY_IDX}


        # ---------------- QoS 설정 ----------------
        self.qos_tr = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5
        )
        self.qos_rel = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=50
        )

        # ---------------- ROS 통신 설정 ----------------
        # 1) 컨트롤러 상태 (예측 계산용 desired)
        self.create_subscription(
            JointTrajectoryControllerState,
            "/arm_controller/state",
            self.state_cb,
            self.qos_rel,
        )

        # 2) joint state (effort + override 정지용)
        self.create_subscription(
            JointState,
            "/joint_states",
            self.joint_cb,
            self.qos_rel,
        )

        # 3) end-effector pose (2번 모션용)
        self.ee_pose = None
        self.create_subscription(Pose, "/end_pose", self.ee_pose_cb, 10)

        # pose_goal 액션 클라이언트 (모션용)
        self.pose_client = ActionClient(self, PoseGoal, "pose_goal")

        # 즉시 정지를 위한 FollowJointTrajectory 액션 클라이언트
        self.stop_client = ActionClient(self, FollowJointTrajectory,
                                        "/arm_controller/follow_joint_trajectory")

        # 액션 서버 준비 대기
        servers_ready = True
        if not self.pose_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("pose_goal 액션 서버가 준비되지 않았습니다.")
            servers_ready = False

        if not self.stop_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("/arm_controller/follow_joint_trajectory 서버가 준비되지 않았습니다.")
            servers_ready = False

        if not servers_ready:
            raise RuntimeError("필수 액션 서버를 찾을 수 없습니다.")

        # ---------------- 제어 상태 ----------------
        self._lock = threading.Lock()
        self.paused = False  # true면 모션 중단 상태
        self.active_pose_goal_handle = None  # 현재 실행중인 pose_goal 핸들
        self.resume_event = threading.Event()  # s키로 재시작 시 깨우는용

        # ---------------- 플롯 설정 ----------------
        self.fig, self.ax = plt.subplots()
        colors = ["g", "b"]
        self.lines_pred, self.lines_act = {}, {}
        for i, j in enumerate(DISPLAY_IDX):
            c = colors[i % len(colors)]
            name = URDF_JOINT_NAMES[j]
            self.lines_pred[j] = self.ax.plot([], [], color=c, linestyle="-",
                                              label=f"{name} pred")[0]
            self.lines_act[j] = self.ax.plot([], [], color=c, linestyle="--",
                                             label=f"{name} actual")[0]

        self.ax.set_ylim(*Y_LIM)
        self.ax.set_xlim(0, HISTORY_LEN)
        self.ax.set_title("Torque: predicted (solid) vs actual (dashed)")
        self.ax.set_xlabel("samples")
        self.ax.set_ylabel("N·m")
        self.ax.legend(loc="upper left", ncol=2)

        # 애니메이션 (matplotlib)
        self.ani = animation.FuncAnimation(self.fig, self.update_plot, interval=80)

        # ---------------- 타이머: 예측 토크 계산 + 안전검사 ----------------
        self.create_timer(1.0 / PRED_RATE_HZ, self.tick)

        # ---------------- 키보드 입력 스레드 ----------------
        threading.Thread(target=self._keyboard_loop, daemon=True).start()

        self.get_logger().info(
            "통합 노드 준비 완료.\n"
            "- 2번처럼 pose_goal로 모션 실행\n"
            "- 1번처럼 예측 토크 vs effort 플롯\n"
            "- 4σ 초과 시 3번처럼 즉시 정지\n"
            "- 키보드: q=정지, s=재시작"
        )

    # ---------- 콜백들 ----------

    def state_cb(self, msg: JointTrajectoryControllerState):
        """
        /arm_controller/state 에서 desired q, qd, qdd 읽어오는 콜백 (1번 코드 기반)
        """
        if msg.joint_names:
            jmap = {n: i for i, n in enumerate(msg.joint_names)}
        else:
            jmap = {n: i for i, n in enumerate(URDF_JOINT_NAMES)}

        def fill_from_point(dst, point_field):
            if hasattr(msg, point_field):
                pt = getattr(msg, point_field)
                for j, n in enumerate(URDF_JOINT_NAMES):
                    if n in jmap:
                        k = jmap[n]
                        if len(pt.positions) > k:
                            dst["q"][j] = pt.positions[k]
                        if len(pt.velocities) > k:
                            dst["qd"][j] = pt.velocities[k]
                        acc_ok = len(pt.accelerations) > k
                        a_val = (pt.accelerations[k] if acc_ok else 0.0)
                        dst["qdd"][j] = a_val if math.isfinite(a_val) else 0.0

        pack = {
            "q": np.zeros(self.nj),
            "qd": np.zeros(self.nj),
            "qdd": np.zeros(self.nj),
        }
        fill_from_point(pack, "desired")
        self.qd = pack["q"]
        self.qdotd = pack["qd"]
        self.qddotd = pack["qdd"]
        self.have_desired = True

    def joint_cb(self, msg: JointState):
        """
        /joint_states 에서 effort와 현재 joint 상태 저장 (1번 + 3번 기능)
        """
        self.last_js_msg = msg
        idx_map = {n: i for i, n in enumerate(msg.name)}
        for j, n in enumerate(URDF_JOINT_NAMES):
            if n in idx_map:
                k = idx_map[n]
                if len(msg.effort) > k:
                    val = msg.effort[k]
                    if math.isfinite(val):
                        self.effort[j] = val
                        self.effort_last_valid[j] = val
                    else:
                        self.effort[j] = self.effort_last_valid[j]

    def ee_pose_cb(self, msg: Pose):
        """
        /end_pose 로부터 EE pose를 받아 저장 (2번 코드 기반)
        """
        pose_msg = Pose()
        pose_msg.position.x = msg.position.x
        pose_msg.position.y = msg.position.y
        pose_msg.position.z = msg.position.z
        pose_msg.orientation.x = msg.orientation.x
        pose_msg.orientation.y = msg.orientation.y
        pose_msg.orientation.z = msg.orientation.z
        pose_msg.orientation.w = msg.orientation.w
        self.ee_pose = pose_msg

    # ---------- 예측 토크 계산 + 안전 체크 ----------

    def tick(self):
        """
        주기적으로 Pinocchio RNEA로 예측 토크 계산,
        보정(A@pred + B) 후 실제 effort와 비교.
        4σ 초과 시 3번처럼 정지 요청.
        """
        if not self.have_desired:
            return

        try:
            tau_pred = pin.rnea(self.model, self.data,
                                self.qd, self.qdotd, self.qddotd)
            tau_pred = np.abs(tau_pred)  # 원 코드와 동일하게 절댓값
        except Exception as e:
            self.get_logger().warn(f"pinocchio rnea failed: {e}")
            return

        tau_real = self.effort.copy()

        # A @ pred + B 보정 (앞의 6개 조인트만 사용)
        tau_pre = A @ tau_pred[0:6] + B

        # 히스토리 업데이트 및 4σ 체크
        violation_detected = False
        for j in DISPLAY_IDX:
            # tau_pre는 0~2 index 사용 (joint4,5,6에 해당한다고 가정)
            pred_val = float(tau_pre[j - 2])
            real_val = float(tau_real[j])

            self.tau_pred_hist[j].append(pred_val)
            self.tau_real_hist[j].append(real_val)

            err = abs(pred_val - real_val)
            if err > FOUR_SIGMA:
                violation_detected = True
                self.get_logger().warn(
                    f"[4σ 위반] joint {URDF_JOINT_NAMES[j]}: "
                    f"|pred-real|={err:.4f} > 4σ={FOUR_SIGMA:.4f}"
                )
                break

        # 4σ 이상이면 즉시 정지 요청 (3번 스타일)
        if violation_detected:
            self._request_stop("4σ torque violation")

    # ---------- 플롯 갱신 (matplotlib Animation) ----------

        # ---------- 플롯 갱신 (matplotlib Animation) ----------
    def update_plot(self, _frame):
        """
        - 예측/실제 토크 라인 업데이트
        - 각 joint에 대해 ±4σ 연한 색 밴드 시각화
        """
        for j in DISPLAY_IDX:
            yp = np.array(self.tau_pred_hist[j])
            yr = np.array(self.tau_real_hist[j])

            if len(yp) == 0:
                continue

            x = np.arange(len(yp))

            # 라인 업데이트
            self.lines_pred[j].set_data(x, yp)
            self.lines_act[j].set_data(x, yr)

            # 기존 4σ 영역 제거 (중첩 방지)
            if self.fill_4sigma[j] is not None:
                try:
                    self.fill_4sigma[j].remove()
                except ValueError:
                    # 이미 지워졌거나 축이 리셋된 경우 무시
                    pass

            # 조인트별 색상 (원래 코드 느낌 살리기)
            if j == 3:      # joint4
                band_color = "LimeGreen"
            elif j == 4:    # joint5
                band_color = "deepskyblue"
            else:
                band_color = "lightgray"

            # ±4σ 연한 밴드
            self.fill_4sigma[j] = self.ax.fill_between(
                x,
                yp - FOUR_SIGMA,
                yp + FOUR_SIGMA,
                color=band_color,
                alpha=0.25,
            )

        # x축 스크롤
        max_len = max(
            (len(self.tau_pred_hist[j]) for j in DISPLAY_IDX),
            default=0
        )
        self.ax.set_xlim(max(0, max_len - HISTORY_LEN), max_len)

        return list(self.lines_pred.values()) + list(self.lines_act.values())


    # ---------- 키보드 / 정지 / 재시작 ----------

    def _keyboard_loop(self):
        """
        키보드 입력 스레드:
        - q: 즉시 정지
        - s: 재시작
        """
        self.get_logger().info("키보드: q=즉시 정지, s=재시작")
        try:
            for line in sys.stdin:
                key = line.strip().lower()
                if key == 'q':
                    self.get_logger().warn("입력 q → 즉시 정지 요청")
                    self._request_stop("keyboard 'q'")
                elif key == 's':
                    self.get_logger().info("입력 s → 재시작 요청")
                    self._request_resume()
        except Exception as e:
            self.get_logger().error(f"keyboard thread error: {e!r}")

    def _request_stop(self, reason: str):
        """
        정지 요청: paused 플래그 세우고, 별도 스레드에서
        - pose_goal cancel
        - FollowJointTrajectory override 전송
        """
        with self._lock:
            if self.paused:
                return
            self.paused = True

        self.get_logger().warn(f"로봇 정지 요청: {reason}")
        # 실제 정지 시퀀스는 별도 스레드에서 수행 (ROS 콜백 막지 않기 위해)
        threading.Thread(target=self._do_stop_sequence, daemon=True).start()

    def _do_stop_sequence(self):
        # 1) pose_goal cancel
        with self._lock:
            gh = self.active_pose_goal_handle

        if gh is not None:
            try:
                gh.cancel_goal_async()
                self.get_logger().info("현재 pose_goal cancel 요청 전송")
            except Exception as e:
                self.get_logger().warn(f"pose_goal cancel 실패/무시: {e!r}")

        # 2) override trajectory 전송 (3번 코드와 유사)
        self._send_stop_override()

    def _request_resume(self):
        """
        s키로 재시작 요청: paused 해제 후 resume_event set.
        실행 루프(execute_sequence)가 이 이벤트를 보고 다시 움직임.
        """
        with self._lock:
            if not self.paused:
                self.get_logger().warn("이미 동작 중이거나 정지 상태가 아님 → 재시작 무시")
                return
            self.paused = False

        self.get_logger().info("모션 재시작 플래그 설정 (s 입력)")
        self.resume_event.set()

    def _get_current_joint_positions(self, timeout=1.0):
        """
        최신 /joint_states에서 CTRL_JOINT_NAMES 순서의 q_now를 얻는다.
        """
        end = time.time() + timeout
        while self.last_js_msg is None and time.time() < end and rclpy.ok():
            time.sleep(0.02)

        js = self.last_js_msg
        if js is None:
            return None

        name_to_pos = {n: v for n, v in zip(js.name, js.position)}
        try:
            q_now = [float(name_to_pos[n]) for n in CTRL_JOINT_NAMES]
        except KeyError as e:
            self.get_logger().error(f"stop override: 누락 관절 {e} → 실패")
            return None
        return q_now

    def _send_stop_override(self):
        """
        3번 코드의 _send_stop_override를 통합한 부분:
        - 현재 자세 q_now를 0.01s, 0.02s 두 포인트로 보내면서 속도 0
        - FollowJointTrajectory 액션으로 전송
        """
        q_now = self._get_current_joint_positions(timeout=1.0)
        if q_now is None:
            self.get_logger().error("stop override: joint_states 없음 → 정지 명령 전송 실패")
            return

        jt = JointTrajectory()
        jt.joint_names = CTRL_JOINT_NAMES
        jt.header.stamp = self.get_clock().now().to_msg()

        p1 = JointTrajectoryPoint()
        p1.positions = q_now
        p1.velocities = [0.0] * len(CTRL_JOINT_NAMES)
        p1.time_from_start = Duration(sec=0, nanosec=10_000_000)  # 0.01s

        p2 = JointTrajectoryPoint()
        p2.positions = q_now
        p2.velocities = [0.0] * len(CTRL_JOINT_NAMES)
        p2.time_from_start = Duration(sec=0, nanosec=20_000_000)  # 0.02s

        jt.points = [p1, p2]
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = jt

        self.get_logger().info("stop override trajectory 전송")
        future = self.stop_client.send_goal_async(goal)

        def goal_resp_cb(fut):
            gh = fut.result()
            if not gh or not gh.accepted:
                self.get_logger().error("stop override goal 거부됨")
                return

            def result_cb(rfut):
                wrapped = rfut.result()
                self.get_logger().info(
                    f"stop override 완료: status={wrapped.status} "
                    f'err={getattr(wrapped.result, "error_code", "N/A")} '
                    f'msg="{getattr(wrapped.result, "error_string", "")}"'
                )

            gh.get_result_async().add_done_callback(result_cb)

        future.add_done_callback(goal_resp_cb)

    # ---------- 모션 실행 (2번 execute_sequence 통합) ----------

    def _wait_for_ee_pose(self, timeout=5.0):
        end = time.time() + timeout
        while rclpy.ok() and self.ee_pose is None and time.time() < end:
            time.sleep(0.05)
        return self.ee_pose is not None

    def start_motion_loop(self):
        t = threading.Thread(target=self.execute_sequence, daemon=True)
        t.start()

    def execute_sequence(self):
        """
        2번의 execute_sequence를 통합.
        - 처음 /end_pose 를 한 번 받아 target으로 저장
        - paused가 아니면 mode 8 → mode 6 순서로 pose_goal 실행 반복
        - 정지 상태(paused=True)면 resume_event를 기다림
        """
        self.get_logger().info("모션 시퀀스 시작 (2번 스타일)")

        if not self._wait_for_ee_pose(timeout=5.0):
            self.get_logger().error("/end_pose 초기값을 못 받았습니다. 퍼블리셔/토픽/타입/QoS 확인!")
            return

        target = self.ee_pose
        self.get_logger().info("초기 target EE pose 설정 완료")

        while rclpy.ok():
            # 정지 상태면 재시작까지 대기
            with self._lock:
                paused = self.paused
            if paused:
                self.get_logger().info("정지 상태 → s 입력으로 재시작 대기 중...")
                self.resume_event.wait()
                self.resume_event.clear()
                continue

            # target pose를 PoseStamped로 포장
            pose_stamped = PoseStamped()
            pose_stamped.pose = target
            pose_stamped.header.frame_id = "base_link"
            pose_stamped.header.stamp = self.get_clock().now().to_msg()

            # 1) MODE 8 : move_forward
            self.get_logger().info("Step: MODE 8 (move_forward)")
            ok = self._send_pose_goal(pose_stamped, mode=8)
            if not ok:
                self.get_logger().warn("MODE 8 실패 → 잠시 후 재시도")
                time.sleep(0.5)
                continue

            # 중간에 정지 요청 들어왔으면 다음 루프에서 처리
            with self._lock:
                if self.paused:
                    continue

            # 2) MODE 6 : send_pose
            self.get_logger().info("Step: MODE 6 (send_pose)")
            ok = self._send_pose_goal(pose_stamped, mode=6)
            if not ok:
                self.get_logger().warn("MODE 6 실패 → 잠시 후 재시도")
                time.sleep(0.5)
                continue

            # 계속 반복
            # (원하면 중간에 sleep 추가)
            # time.sleep(0.1)

    def _send_pose_goal(self, pose_stamped: PoseStamped, mode: int) -> bool:
        """
        2번의 send_pose / move_forward 를 통합한 함수.
        - mode 값만 다르게 주어 pose_goal 사용.
        - send_goal_async + threading.Event로 완료까지 동기적으로 기다리되,
          실제 스핀은 메인 rclpy.spin() 스레드에서 처리.
        """
        with self._lock:
            if self.paused:
                self.get_logger().info("paused 상태 → pose_goal 전송 생략")
                return False

        goal_msg = PoseGoal.Goal()
        goal_msg.target_pose = pose_stamped
        goal_msg.mode = mode

        done_event = threading.Event()
        result_success = {"ok": False}

        def goal_response_cb(fut):
            try:
                goal_handle = fut.result()
            except Exception as e:
                self.get_logger().error(f"pose_goal 전송 예외: {e!r}")
                done_event.set()
                return

            if not goal_handle or not goal_handle.accepted:
                self.get_logger().error("pose_goal이 거부되었습니다.")
                done_event.set()
                return

            self.get_logger().info(f"pose_goal(mode={mode}) 명령 전송 완료.")
            with self._lock:
                self.active_pose_goal_handle = goal_handle

            def result_cb(res_fut):
                try:
                    res = res_fut.result().result
                    if res.success:
                        self.get_logger().info(f"✅ pose_goal(mode={mode}) 성공: {res.message}")
                        result_success["ok"] = True
                    else:
                        self.get_logger().warn(f"⚠️ pose_goal(mode={mode}) 실패: {res.message}")
                finally:
                    with self._lock:
                        self.active_pose_goal_handle = None
                    done_event.set()

            goal_handle.get_result_async().add_done_callback(result_cb)

        send_future = self.pose_client.send_goal_async(goal_msg)
        send_future.add_done_callback(goal_response_cb)

        # 여기서 기다리는 동안 실제 콜백 처리는 spin 스레드에서 동작
        while not done_event.is_set() and rclpy.ok():
            done_event.wait(0.1)

        return result_success["ok"]


# ================================================================= #
# ================================ main ============================ #
# ================================================================= #

def main(args=None):
    rclpy.init(args=args)

    try:
        node = PredictControlSafetyNode()
    except RuntimeError as e:
        print(f"노드 초기화 중 오류 발생: {e}")
        rclpy.shutdown()
        return

    # ROS 콜백용 스레드
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    # 2번처럼 모션 시퀀스 실행 (별도 스레드)
    node.start_motion_loop()

    # 1번처럼 플롯 표시 (메인 스레드 블로킹)
    try:
        plt.show()
    finally:
        node.get_logger().info("플롯 종료, 노드 정리 중...")
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()