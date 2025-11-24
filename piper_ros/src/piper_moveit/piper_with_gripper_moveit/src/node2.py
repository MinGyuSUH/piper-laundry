"""
ROS2 Action 노드: 외부 감지 파이프라인 라우팅
- mode == "wm"       → 기존 run_laundry.py 를 그대로 호출 ( --mode wm --json )
- mode == "basket"   → run_basket.py --mode basket --ckpt ... --intr_json ... --json
- mode == "failure"  → run_failure.py --mode failure --ckpt ... --intr_json ... --json
- mode == "failure_bg" → run_failure.py --mode failure_bg --ckpt ... --intr_json ... --json

{"success": bool, "centers": [[x,y,z], ...]}


"""

import os
import sys
import json
import subprocess
from typing import List

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from geometry_msgs.msg import Pose
from piper_with_gripper_moveit.action import TargetPose

# ====== 경로/옵션 기본값 ======
PYTHON_BIN = sys.executable or "python3"
TIMEOUT_SEC = 660.0

RUN_LAUNDRY_DIR = "/home/dyros/mcy_ws/Laundry_decision/run_laundry"
RUN_LAUNDRY_PY  = os.path.join(RUN_LAUNDRY_DIR, "run_laundry.py")

RUN_FAILURE_DIR = "/home/dyros/mcy_ws/Laundry_decision/run_bf"
RUN_FAILURE_PY  = os.path.join(RUN_FAILURE_DIR, "run_failure.py")

RUN_BASKET_DIR  = "/home/dyros/mcy_ws/Laundry_decision/run_bf"
RUN_BASKET_PY   = os.path.join(RUN_BASKET_DIR, "run_basket.py")

# 공용 ckpt / 내재 파라미터(필요 시 도메인별로 나눠도 됨)
CKPT_PATH_F = "/home/dyros/mcy_ws/Laundry_decision/run_bf/dataset_rgb_sam2/unet_best_min_wo.pt"
CKPT_PATH_B = "/home/dyros/mcy_ws/Laundry_decision/run_bf/dataset_rgb_sam2/unet_best_multi.pt"
INTR_JSON  = "/home/dyros/mcy_ws/Laundry_decision/run_bf/intrinsics_color_640x480.json"

# 사용가능
FAILURE_ARGS = [
    # "--size", "384",
    # "--zmax", "0.5",
    # "--thr", "0.45",
    # "--diff_mode", "blend",
    # "--vote", "--vote_k", "10", "--vote_min_area", "150",
    # "--vote_min_dt", "3", "--vote_require_curr",
    # "--stale_sec", "1.0",
    "--save_dir", "/home/dyros/mcy_ws/Laundry_decision/run_bf/_dbg",
    "--save_best_only", "--prefix"
]


def _last_json_line(text: str) -> dict:
    """stdout에서 마지막 JSON 라인을 파싱.
    - 뒤에서부터 '{' 로 시작하고 '}' 로 끝나는 라인을 찾아 파싱
    """
    lines = [ln.strip() for ln in (text or "").splitlines() if ln.strip()]
    for ln in reversed(lines):
        if ln.startswith("{") and ln.endswith("}"):
            try:
                return json.loads(ln)
            except Exception:
                pass
    # 마지막 라인 한 번 더 시도
    if lines:
        try:
            return json.loads(lines[-1])
        except Exception:
            pass
    raise json.JSONDecodeError("no json line found", doc=(text[-500:] if text else ""), pos=0)


def run_json(cmd: List[str], cwd: str, timeout: float = TIMEOUT_SEC) -> dict:
    """외부 프로세스 실행 후 JSON 결과를 파싱해서 반환."""
    print("[node] exec:", " ".join(cmd))
    proc = subprocess.run(
        cmd,
        cwd=cwd,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        timeout=timeout,
        text=True,
    )
    
    
    out = proc.stdout or ""
    print("[node] output tail:\n" + "\n".join(out.splitlines()[-20:]))
    if proc.returncode != 0:
        raise subprocess.CalledProcessError(proc.returncode, cmd, output=out)
    return _last_json_line(out)


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

        mode = goal_handle.request.mode  # "wm" | "basket" | "failure" | "failure_bg"
        self.get_logger().info(f"세탁물 감지 실행: mode={mode}")

        try:
            # ---------- 세탁기(기존 파이프라인) ----------
            if mode == "wm":
                if not os.path.isfile(RUN_LAUNDRY_PY):
                    self.get_logger().error(f"run_laundry.py 없음: {RUN_LAUNDRY_PY}")
                    goal_handle.abort()
                    return TargetPose.Result(success=False)
                cmd = [PYTHON_BIN, RUN_LAUNDRY_PY, "--mode", "wm", "--json"]
                result = run_json(cmd, cwd=RUN_LAUNDRY_DIR)
                
                #self.get_logger().info(f"[wm] result={result}")
            # ---------- 바구니 파이프라인 ----------
            elif mode == "basket":
                if not os.path.isfile(RUN_BASKET_PY):
                    self.get_logger().error(f"run_basket.py 없음: {RUN_BASKET_PY}")
                    goal_handle.abort()
                    return TargetPose.Result(success=False)
                cmd = [
                    PYTHON_BIN, RUN_BASKET_PY,
                    "--mode", "basket",
                    "--ckpt", CKPT_PATH_B,
                    "--intr_json", INTR_JSON,
                    "--json",
                    "--diff_mode", "bilateral",
                    *FAILURE_ARGS, 
                    mode
                ]
                result = run_json(cmd, cwd=RUN_BASKET_DIR)
                
                self.get_logger().info(f"[basket] result={result}")
            # ---------- 실패(배경 기반) 파이프라인 ----------
            elif mode == "failure":
                if not os.path.isfile(RUN_FAILURE_PY):
                    self.get_logger().error(f"run_failure.py 없음: {RUN_FAILURE_PY}")
                    goal_handle.abort()
                    return TargetPose.Result(success=False)
                cmd = [
                    PYTHON_BIN, RUN_FAILURE_PY,
                    "--mode", "failure",
                    "--ckpt", CKPT_PATH_F,
                    "--intr_json", INTR_JSON,
                    "--json",
                    *FAILURE_ARGS,
                    mode
                ]
                result = run_json(cmd, cwd=RUN_FAILURE_DIR)
                self.get_logger().info(f"[failure] result={result}")
            # ---------- 실패: 배경 저장 ----------
            elif mode == "failure_bg":
                if not os.path.isfile(RUN_FAILURE_PY):
                    self.get_logger().error(f"run_failure.py 없음: {RUN_FAILURE_PY}")
                    goal_handle.abort()
                    return TargetPose.Result(success=False)
                cmd = [
                    PYTHON_BIN, RUN_FAILURE_PY,
                    "--mode", "failure_bg",
                    "--ckpt", CKPT_PATH_F,
                    "--intr_json", INTR_JSON,
                    "--json",
                ]
                result = run_json(cmd, cwd=RUN_FAILURE_DIR)
                self.get_logger().info(f"[failure_bg] result={result}")
            else:
                self.get_logger().error(f"알 수 없는 mode='{mode}'")
                goal_handle.abort()
                return TargetPose.Result(success=False)

            # ---------- 결과 처리 공통 ----------
            # debg dyd
            # self.get_logger().info(f"[{mode}] result={result}")
            centers = result.get("centers") or []
            

            print("********************************************************************")
            print(len(centers))
            print("********************************************************************")
            

            ok = bool(result.get("success", False)) and len(centers) > 0
            if not ok:
                self.get_logger().warn("의류 미검출")
                goal_handle.abort()
                return TargetPose.Result(success=False)
            
            cx, cy, cz = map(float, centers[0])
            target_pose = Pose()
            target_pose.position.x = cx
            target_pose.position.y = cy
            target_pose.position.z = cz
            target_pose.orientation.w = 1.0
            
            section_col = None
            
            if mode == "failure":
                colu = result.get("occupied_cells")
                
                col1 = colu[0]
                try:
                    col2 = colu[1]
                except:
                    col2 = None
                        
            out = TargetPose.Result()
            out.success = True
            out.target_pose = target_pose
            # if mode == "failure":
            #     out.message = result.get('section_cols')
                
            # Result에 'col' 필드가 있으면 사용, 없으면 message로 전달
            if mode == "failure" and colu is not None:
                if hasattr(out, "col"):
                    out.col = col1
                    try:
                        out.col2 = float(col2)
                    except:
                        print("col2 is None")
                # elif hasattr(out, "message"):
                #     out.message = f"col={section_col}"
                else:
                    # 필드가 전혀 없으면 로그만 남김
                    self.get_logger().warn("Result에 col/message 필드가 없어 col 값을 담지 못했습니다.")

            self.get_logger().info(f"[{mode}] 좌표 반환: x={cx:.3f}, y={cy:.3f}, z={cz:.3f}")
            if section_col is not None:
                self.get_logger().info(f"[{mode}] section_col={section_col}")
            goal_handle.succeed()
            return out

        except subprocess.TimeoutExpired:
            self.get_logger().error("감지 프로세스 타임아웃")
            goal_handle.abort()
            return TargetPose.Result(success=False)

        except subprocess.CalledProcessError as e:
            msg = e.output if isinstance(e.output, str) else (e.output or b"").decode(errors='ignore')
            self.get_logger().error(f"감지 중 예외 발생(exit {e.returncode}):\n{msg}")
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
