#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import math
import sys
import threading

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, HistoryPolicy
from builtin_interfaces.msg import Duration
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
from action_msgs.msg import GoalStatus

ARM_FTJ_NS = '/arm_controller/follow_joint_trajectory'
JOINT_NAMES = ['joint1','joint2','joint3','joint4','joint5','joint6']

BOUNCE_JOINT = 1
AMP_DEG = 20.0
UP_TIME, DOWN_TIME, BACK_TIME = 1.0, 1.0, 1.0
ADD_VEL = True

BEST_EFFORT_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    durability=QoSDurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)

class BounceToJTC(Node):
    def __init__(self):
        super().__init__('bounce_to_jtc_client')
        self.cli = ActionClient(self, FollowJointTrajectory, ARM_FTJ_NS)

        self._lock = threading.Lock()
        self.state = "idle"          # idle | active | stopping | paused
        self.paused = False
        self.active_goal_handle = None
        self.last_js = None

        self.sub = self.create_subscription(
            JointState, '/joint_states', self._js_cb, BEST_EFFORT_QOS
        )

        if not self.cli.wait_for_server(timeout_sec=5.0):
            raise RuntimeError(f'no action server: {ARM_FTJ_NS}')

        self.get_logger().info("키보드: q=즉시정지, s=재시작")
        threading.Thread(target=self._keyboard_loop, daemon=True).start()

        self._send_once()

    # ========== 콜백 ==========
    def _js_cb(self, msg: JointState):
        self.last_js = msg

    def _keyboard_loop(self):
        try:
            for line in sys.stdin:
                key = line.strip().lower()
                if key == 'q':
                    self.get_logger().warn("입력 q → 즉시 정지")
                    self._request_stop()
                elif key == 's':
                    self.get_logger().info("입력 s → 재시작")
                    self._request_resume()
        except Exception as e:
            self.get_logger().error(f"keyboard thread error: {e}")


    # ========== 정지/재개 ==========
    def _request_stop(self):
        with self._lock:
            self.paused = True
            gh = self.active_goal_handle

        # 1) 현재 goal 취소 (GoalHandle 에 cancel!)
        try:
            if gh is not None:
                gh.cancel_goal_async()   # <<< 여기!
        except Exception as e:
            self.get_logger().warn(f"cancel 실패/무시: {e}")

        # 2) 덮어쓰기 stop-goal 즉시 전송
        self._send_stop_override()


    def _request_resume(self):
        with self._lock:
            if self.state != "paused":
                self.get_logger().warn(f"지금 상태={self.state} → 재시작 보류")
                return
            self.paused = False
        self._send_once()

    # 핵심: 덮어쓰기 stop-goal (현재자세 2포인트, 0.01/0.02s, 속도 0)
    def _send_stop_override(self):
        js = self._wait_one_js(timeout=1.0)
        if js is None:
            self.get_logger().error("stop override: /joint_states 없음 → paused 진입")
            with self._lock:
                self.state = "paused"
            return

        name_to_pos = {n: v for n, v in zip(js.name, js.position)}
        try:
            q_now = [float(name_to_pos[n]) for n in JOINT_NAMES]
        except KeyError as e:
            self.get_logger().error(f"stop override: 누락 관절 {e} → paused 진입")
            with self._lock:
                self.state = "paused"
            return

        jt = JointTrajectory()
        jt.joint_names = JOINT_NAMES
        jt.header.stamp = self.get_clock().now().to_msg()  # ★ 즉시 시작

        p1 = JointTrajectoryPoint()
        p1.positions = q_now
        p1.velocities = [0.0] * len(JOINT_NAMES)
        p1.time_from_start = Duration(sec=0, nanosec=10_000_000)   # 0.01s

        p2 = JointTrajectoryPoint()
        p2.positions = q_now
        p2.velocities = [0.0] * len(JOINT_NAMES)
        p2.time_from_start = Duration(sec=0, nanosec=20_000_000)   # 0.02s

        jt.points = [p1, p2]

        goal = FollowJointTrajectory.Goal(trajectory=jt)
        with self._lock:
            self.state = "stopping"

        # send with overrides: 새 goal 자체가 이전 goal을 덮어씀(컨트롤러 세팅에 따라)
        self.cli.send_goal_async(goal).add_done_callback(self._stop_goal_resp)

    def _stop_goal_resp(self, fut):
        gh = fut.result()
        if not gh or not gh.accepted:
            self.get_logger().error("stop override goal 거부됨 → 그래도 paused로 전환")
            with self._lock:
                self.state = "paused"
            return
        gh.get_result_async().add_done_callback(self._stop_goal_result)

    def _stop_goal_result(self, fut):
        wrapped = fut.result()
        self.get_logger().info(
            f"stop override 완료: status={wrapped.status} "
            f'err={getattr(wrapped.result,"error_code","N/A")} '
            f'msg="{getattr(wrapped.result,"error_string","")}"'
        )
        with self._lock:
            self.state = "paused"
            self.active_goal_handle = None

    # ========== 왕복 goal ==========
    def _send_once(self):
        with self._lock:
            if self.paused or self.state == "stopping":
                self.get_logger().info("paused/stopping → 전송 보류")
                return

        js = self._wait_one_js(timeout=5.0)
        if js is None:
            self.get_logger().error('joint_states timeout')
            return

        name_to_pos = {n: v for n, v in zip(js.name, js.position)}
        try:
            q0 = [float(name_to_pos[n]) for n in JOINT_NAMES]
        except KeyError as e:
            self.get_logger().error(f'missing joint in /joint_states: {e}')
            return

        amp = math.radians(AMP_DEG)
        q_up = q0[:]; q_up[BOUNCE_JOINT] += amp
        q_dn = q0[:]; q_dn[BOUNCE_JOINT] -= amp

        jt = JointTrajectory()
        jt.joint_names = JOINT_NAMES
        jt.header.stamp = self.get_clock().now().to_msg()  # ★ 즉시 시작

        def mkpt(pos, tfs, vel=None):
            p = JointTrajectoryPoint()
            p.positions = pos
            if vel is not None:
                p.velocities = vel
            p.time_from_start = Duration(sec=int(tfs), nanosec=int((tfs-int(tfs))*1e9))
            return p

        v_up = [0.0]*6; v_dn = [0.0]*6; v_bk = [0.0]*6
        if ADD_VEL:
            v_up[BOUNCE_JOINT] =  (amp)/max(UP_TIME,1e-6)
            v_dn[BOUNCE_JOINT] = (-2*amp)/max(DOWN_TIME,1e-6)
            v_bk[BOUNCE_JOINT] =  (amp)/max(BACK_TIME,1e-6)

        t = 0.0
        jt.points += [
            mkpt(q0, t, [0.0]*6 if ADD_VEL else None),
            mkpt(q_up, t:=t+UP_TIME,  v_up if ADD_VEL else None),
            mkpt(q_dn, t:=t+DOWN_TIME, v_dn if ADD_VEL else None),
            mkpt(q0,  t:=t+BACK_TIME, v_bk if ADD_VEL else None),
        ]

        goal = FollowJointTrajectory.Goal(trajectory=jt)
        self.get_logger().info(f'send JT → joints={jt.joint_names}, points={len(jt.points)}')

        # 표준 async 전송
        send_future = self.cli.send_goal_async(goal)
        send_future.add_done_callback(self._bounce_goal_resp)

    def _bounce_goal_resp(self, fut):
        gh = fut.result()
        with self._lock:
            self.active_goal_handle = gh if gh and gh.accepted else None

        if not gh or not gh.accepted:
            self.get_logger().error('왕복 goal 거부됨')
            with self._lock:
                self.state = "idle"
            return

        with self._lock:
            self.state = "active"
        gh.get_result_async().add_done_callback(self._bounce_result_cb)

    def _bounce_result_cb(self, fut):
        wrapped = fut.result()
        res = wrapped.result
        self.get_logger().info(
            f'왕복 결과: code={getattr(res, "error_code", "N/A")} msg="{getattr(res,"error_string","")}"'
        )
        with self._lock:
            self.active_goal_handle = None

        with self._lock:
            if self.state in ("stopping","paused"):
                self.get_logger().info("정지/일시정지 상태 → 다음 사이클 전송 안 함")
                return

        # 다음 사이클 계속
        self._send_once()

    # ========== 유틸 ==========
    def _wait_one_js(self, timeout=5.0):
        end = self.get_clock().now().nanoseconds + int(timeout*1e9)
        while rclpy.ok() and self.last_js is None:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.get_clock().now().nanoseconds > end:
                break
        return self.last_js


def main():
    rclpy.init()
    node = None
    try:
        node = BounceToJTC()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
