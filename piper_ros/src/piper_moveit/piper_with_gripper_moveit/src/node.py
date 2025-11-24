#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os, sys, json, subprocess
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from geometry_msgs.msg import Pose
from std_msgs.msg import String
from piper_with_gripper_moveit.action import TargetPose

# ====== 경로/옵션 기본값 ======
RUN_LAUNDRY_DIR = "/home/cheon/Documents/gd_0808"
RUN_LAUNDRY_PY  = os.path.join(RUN_LAUNDRY_DIR, "run_laundry.py")

RUN_FAILURE_DIR = "/home/cheon/Documents/run_failure"
RUN_FAILURE_PY  = os.path.join(RUN_FAILURE_DIR, "run_failure.py")

CKPT_PATH       = "/home/cheon/Documents/run_failure/dataset_rgb_sam2/unet_best.pt"
INTR_JSON       = "/home/cheon/Documents/run_failure/intrinsics_depth_640x480.json"

# run_failure 기본 검출 옵션
FAILURE_ARGS = [
    "--size", "384",
    "--zmax", "0.5",
    "--thr", "0.45",
    "--diff_mode", "blend",
    "--vote", "--vote_k", "10",
    "--vote_min_area", "150",
    "--vote_min_dt", "3",
    "--vote_require_curr",
    "--stale_sec", "1.0",
    "--save_dir", "/home/cheon/Documents/run_failure/_dbg",
    "--save_best_only", "--prefix", "failure"
]

def run_json(cmd, cwd, timeout=660.0):
    """외부 프로세스 실행 후 마지막 줄 JSON 파싱"""
    out = subprocess.check_output(
        cmd, cwd=cwd, stderr=subprocess.STDOUT, timeout=timeout
    ).decode("utf-8").strip()
    last = out.split("\n")[-1].strip()
    return json.loads(last)

class TargetPoseServer(Node):
    def __init__(self):
        super().__init__('target_pose_server')
        self._action_server = ActionServer(
            self, TargetPose, 'target_pose', self.execute_callback
        )

    async def execute_callback(self, goal_handle):
        self.get_logger().info('target_pose 요청 받음')

        if not goal_handle.request.start:
            self.get_logger().warn('start=False로 요청됨')
            goal_handle.abort()
            return TargetPose.Result(success=False)

        mode = goal_handle.request.mode  # "wm" | "basket" | "failure_bg" | "failure"
        self.get_logger().info(f"세탁물 감지 실행: mode={mode}")

        try:
            # ---------- 세탁기/바구니 기존 파이프라인 ----------
            if mode in ("wm", "basket"):
                cmd = ["python3", RUN_LAUNDRY_PY, "--mode", mode, "--json"]
                result = run_json(cmd, cwd=RUN_LAUNDRY_DIR)
                self.get_logger().info(f"[{mode}] result={result}")

                centers = result.get("centers", [])
                if not result.get("success", False) or len(centers) == 0:
                    self.get_logger().warn("의류 미검출")
                    goal_handle.abort()
                    return TargetPose.Result(success=False)

                center = centers[0]
                target_pose = Pose()
                target_pose.position.x = float(center[0])
                target_pose.position.y = float(center[1])
                target_pose.position.z = float(center[2])
                target_pose.orientation.w = 1.0

                out = TargetPose.Result()
                out.success = True
                out.target_pose = target_pose
                self.get_logger().info(f"[{mode}] 좌표 반환: x={center[0]:.3f}, y={center[1]:.3f}, z={center[2]:.3f}")
                goal_handle.succeed()
                return out

            # ---------- 실패 시나리오: 배경 저장 ----------
            elif mode == "failure_bg":
                # 배경만 저장 → 성공 여부만 리턴 (centers 없음)
                cmd = [
                    "python3", RUN_FAILURE_PY,
                    "--mode", "failure_bg",
                    "--json",
                    "--ckpt", CKPT_PATH,          # run_failure.py가 ckpt를 요구하므로 통일되게 전달
                    "--intr_json", INTR_JSON,
                ]
                result = run_json(cmd, cwd=RUN_FAILURE_DIR)
                self.get_logger().info(f"[failure_bg] result={result}")

                if not result.get("success", False):
                    self.get_logger().warn("배경 저장 실패")
                    goal_handle.abort()
                    return TargetPose.Result(success=False)

                # 배경 저장은 Pose 반환 불필요 → dummy pose
                dummy = Pose()
                dummy.orientation.w = 1.0
                out = TargetPose.Result()
                out.success = True
                out.target_pose = dummy
                self.get_logger().info("[failure_bg] 배경 저장 완료")
                goal_handle.succeed()
                return out

            # ---------- 실패 시나리오: 배경 기반 감지 ----------
            elif mode == "failure":
                cmd = [
                    "python3", RUN_FAILURE_PY,
                    "--mode", "failure",
                    "--json",
                    "--ckpt", CKPT_PATH,
                    "--intr_json", INTR_JSON,
                    *FAILURE_ARGS,
                ]
                result = run_json(cmd, cwd=RUN_FAILURE_DIR)
                self.get_logger().info(f"[failure] result={result}")

                centers = result.get("centers", [])
                if not result.get("success", False) or len(centers) == 0:
                    self.get_logger().warn("의류 미검출")
                    goal_handle.abort()
                    return TargetPose.Result(success=False)

                center = centers[0]
                target_pose = Pose()
                target_pose.position.x = float(center[0])
                target_pose.position.y = float(center[1])
                target_pose.position.z = float(center[2])
                target_pose.orientation.w = 1.0

                out = TargetPose.Result()
                out.success = True
                out.target_pose = target_pose
                self.get_logger().info(f"[failure] 좌표 반환: x={center[0]:.3f}, y={center[1]:.3f}, z={center[2]:.3f}")
                goal_handle.succeed()
                return out

            else:
                self.get_logger().error(f"알 수 없는 mode='{mode}'")
                goal_handle.abort()
                return TargetPose.Result(success=False)

        except subprocess.TimeoutExpired:
            self.get_logger().error("감지 프로세스 타임아웃")
            goal_handle.abort()
            return TargetPose.Result(success=False)

        except subprocess.CalledProcessError as e:
            # 외부 스크립트 stderr 포함 로그
            try:
                msg = e.output.decode()
            except Exception:
                msg = str(e)
            self.get_logger().error(f"감지 중 예외 발생:\n{msg}")
            goal_handle.abort()
            return TargetPose.Result(success=False)

        except json.JSONDecodeError as e:
            self.get_logger().error(f"JSON 파싱 실패: {e}")
            goal_handle.abort()
            return TargetPose.Result(success=False)

def main(args=None):
    rclpy.init(args=args)
    node = TargetPoseServer()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
