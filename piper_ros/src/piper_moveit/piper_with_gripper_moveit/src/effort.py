#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math, time, threading
from collections import deque
from pathlib import Path
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

from sensor_msgs.msg import JointState
from control_msgs.msg import JointTrajectoryControllerState

import pinocchio as pin
import matplotlib.pyplot as plt
import matplotlib.animation as animation

# ========= 사용자 설정 =========
URDF_OR_XACRO = "/home/dyros/mcy_ws/piper-mou/src/piper_ros/src/piper_description/urdf/piper_description.xacro"
JOINT_NAMES = [f"joint{i+1}" for i in range(8)]  # 기대 조인트 순서 (state와 다르면 자동 정렬)
DISPLAY_IDX =[3, 4] #[2, 3, 4]                           # joint3,4,5만 그림
HISTORY_LEN = 1000
Y_LIM = (0, 3)
GRAVITY = np.array([0.0, 0.0, -9.81])
PRED_RATE_HZ = 100.0  # 예측 토크 계산 주기(Hz)
# =============================


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


class PlotNode(Node):
    def __init__(self):
        super().__init__("predict_vs_effort_plotter")

        # Pinocchio
        self.model = build_pin_model(URDF_OR_XACRO)
        self.data  = self.model.createData()
        self.nj = len(JOINT_NAMES)

        # 상태 버퍼
        self.qd = np.zeros(self.nj)
        self.qdotd = np.zeros(self.nj)
        self.qddotd = np.zeros(self.nj)
        self.have_desired = False

        self.effort = np.zeros(self.nj)
        self.effort_last_valid = np.zeros(self.nj)

        # 표시 히스토리
        self.tau_pred_hist = {j: deque(maxlen=HISTORY_LEN) for j in DISPLAY_IDX}
        self.tau_real_hist = {j: deque(maxlen=HISTORY_LEN) for j in DISPLAY_IDX}

        # QoS
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

        # 구독
        # 1) 컨트롤러 상태(실시간 desired/actual)
        self.create_subscription(JointTrajectoryControllerState, "/arm_controller/state", self.state_cb, self.qos_rel)
        # 3) 실제 effort
        self.create_subscription(JointState, "/joint_states", self.joint_cb, self.qos_rel)

        # 예측 토크 계산 타이머
        self.create_timer(1.0 / PRED_RATE_HZ, self.tick)

        # 플롯
        self.fig, self.ax = plt.subplots()
        colors = ["g", "b"] #["r", "g", "b"]
        self.lines_pred, self.lines_act = {}, {}
        for i, j in enumerate(DISPLAY_IDX):
            c = colors[i % 3]
            name = JOINT_NAMES[j]
            self.lines_pred[j] = self.ax.plot([], [], color=c, linestyle="-",  label=f"{name} pred")[0]
            self.lines_act[j]  = self.ax.plot([], [], color=c, linestyle="--", label=f"{name} actual")[0]
        self.ax.set_ylim(*Y_LIM)
        self.ax.set_xlim(0, HISTORY_LEN)
        self.ax.set_title("Torque: predicted(solid, from desired) vs actual(dashed, /joint_states.effort)")
        self.ax.set_xlabel("samples")
        self.ax.set_ylabel("N·m")
        self.ax.legend(loc="upper left", ncol=2)
        self.ani = animation.FuncAnimation(self.fig, self.update_plot, interval=80)

        self.get_logger().info("Ready: waiting for /arm_controller/state and /joint_states")

    # ---------- 콜백 ----------
    def state_cb(self, msg: JointTrajectoryControllerState):
        jmap = {n:i for i,n in enumerate(msg.joint_names)} if msg.joint_names else {n:i for i,n in enumerate(JOINT_NAMES)}

        def fill_from_point(dst, arr, point_field):
            if hasattr(msg, point_field):
                pt = getattr(msg, point_field)
                for j, n in enumerate(JOINT_NAMES):
                    if n in jmap:
                        k = jmap[n]
                        if len(getattr(pt, "positions")) > k:
                            dst["q"][j] = pt.positions[k]
                        if len(getattr(pt, "velocities")) > k:
                            dst["qd"][j] = pt.velocities[k]
                        # accelerations가 비어있거나 NaN이면 0
                        acc_ok = len(getattr(pt, "accelerations")) > k
                        a_val = (pt.accelerations[k] if acc_ok else 0.0)
                        dst["qdd"][j] = a_val if math.isfinite(a_val) else 0.0

        pack = {"q": np.zeros(self.nj), "qd": np.zeros(self.nj), "qdd": np.zeros(self.nj)}
        fill_from_point(pack, JOINT_NAMES, "desired")
        self.qd, self.qdotd, self.qddotd = pack["q"], pack["qd"], pack["qdd"]
        self.have_desired = True


    def joint_cb(self, msg: JointState):
        idx_map = {n:i for i,n in enumerate(msg.name)}
        for j, n in enumerate(JOINT_NAMES):
            if n in idx_map:
                k = idx_map[n]
                if len(msg.effort) > k:
                    val = msg.effort[k]
                    if math.isfinite(val):
                        self.effort[j] = val
                        self.effort_last_valid[j] = val
                    else:
                        self.effort[j] = self.effort_last_valid[j]


    # ---------- 주기 계산 ----------
    def tick(self):
        if not self.have_desired:
            return
        try:
            tau_pred = pin.rnea(self.model, self.data, self.qd, self.qdotd, self.qddotd)
            tau_pred = abs(tau_pred)
        except Exception as e:
            self.get_logger().warn(f"pinocchio rnea failed: {e}")
            return

        tau_real = self.effort.copy()

        # # ===== CSV 로그 저장 =====
        # import csv, os, time
        # log_path = os.path.expanduser("./piper_torque_log.csv")
        # write_header = not os.path.exists(log_path)

        # # pred: 6개, real: 3,4,5번(joint3~5)
        # row = (
        #     [time.time()]
        #     + list(tau_pred[:6])
        #     + list(tau_real[:6])
        #     + list(self.qd[:6])        # q
        #     + list(self.qdotd[:6])     # qdot
        #     + list(self.qddotd[:6])    # qddot
        # )

        # with open(log_path, "a", newline="") as f:
        #     w = csv.writer(f)
        #     if write_header:
        #         header = (
        #             ["t"]
        #             + [f"tau_pred{i+1}" for i in range(6)]
        #             + [f"tau_real{i+1}" for i in range(6)]
        #             + [f"q{i+1}" for i in range(6)]
        #             + [f"qd{i+1}" for i in range(6)]
        #             + [f"qdd{i+1}" for i in range(6)]
        #         )
        #         w.writerow(header)
        #     w.writerow(row)
        # # =========================

        # # ======================= A @ pred + B =======================================
        A = [[ 0.336031, -0.046468,  0.258372,  0.180103, -0.022022, -1.702219],
        [ 0.11016,  -0.00126,  -0.015487,  1.4827,    0.00906,   0.558599],
        [-0.009071, -0.001866, -0.026797, -0.026905,  1.37266,   0.943661]]

        B = [0.24018,  0.031745, 0.161793]

        tau_pre = A @ tau_pred[0:6] + B     

        for j in DISPLAY_IDX:
            self.tau_pred_hist[j].append(float(tau_pre[j-2]))
            self.tau_real_hist[j].append(float(tau_real[j]))


    # ---------- 플롯 갱신 ----------
    def update_plot(self, _frame):
        for j in DISPLAY_IDX:
            yp = list(self.tau_pred_hist[j])
            yr = list(self.tau_real_hist[j])
            x = list(range(len(yp)))
            self.lines_pred[j].set_data(x, yp)
            self.lines_act[j].set_data(x, yr)


        # --- joint4 ±3σ, ±4σ 영역 시각화 ---
        j = 4 - 1  # joint5 index (0-based)
        yp = np.array(self.tau_pred_hist[j])
        x = np.arange(len(yp))

        sigma = 0.0856  # 표준편차 (N·m)
        band3 = 3 * sigma
        band4 = 4 * sigma

        # 기존 영역 제거 (중첩 방지)
        if hasattr(self, "fill_band1") and self.fill_band1:
            self.fill_band1.remove()
        if hasattr(self, "fill_band4") and self.fill_band2:
            self.fill_band2.remove()

        # ±4σ (연한 연두색)
        self.fill_band2 = self.ax.fill_between(
            x, yp - band4, yp + band4, color="GreenYellow", alpha=0.25, label="±4σ"
        )

        # ±3σ (조금 더 진한 연두)
        self.fill_band1 = self.ax.fill_between(
            x, yp - band3, yp + band3, color="LimeGreen", alpha=0.4, label="±3σ"
        )

        # --- joint5 ±3σ, ±4σ 영역 시각화 ---
        j = 5 - 1  # joint5 index (0-based)
        yp = np.array(self.tau_pred_hist[j])
        x = np.arange(len(yp))

        # 기존 영역 제거 (중첩 방지)
        if hasattr(self, "fill_band3") and self.fill_band3:
            self.fill_band3.remove()
        if hasattr(self, "fill_band4") and self.fill_band4:
            self.fill_band4.remove()

        # ±4σ (연한 하늘색)
        self.fill_band4 = self.ax.fill_between(
            x, yp - band4, yp + band4, color="skyblue", alpha=0.25, label="±4σ"
        )

        # ±3σ (조금 더 진한 파랑)
        self.fill_band3 = self.ax.fill_between(
            x, yp - band3, yp + band3, color="deepskyblue", alpha=0.4, label="±3σ"
        )


        max_len = max((len(self.tau_pred_hist[j]) for j in DISPLAY_IDX), default=0)
        self.ax.set_xlim(max(0, max_len - HISTORY_LEN), max_len)
        return list(self.lines_pred.values()) + list(self.lines_act.values())


def main():
    rclpy.init()
    node = PlotNode()
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()
    plt.show()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
