import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from geometry_msgs.msg import Pose

#target_pose = Pose.target_pose


import os, sys, subprocess, json
from std_msgs.msg import String

# 외부 패키지 경로 수동 추가
piper_pkg_path = "/home/cheon/Documents/dyros/piper-mou/piper_ros/src/piper_moveit"
if piper_pkg_path not in sys.path:
    sys.path.append(piper_pkg_path)

from piper_with_gripper_moveit.action import TargetPose

class TargetPoseServer(Node):
    def __init__(self):
        super().__init__('target_pose_server')

        self._action_server = ActionServer(
            self,
            TargetPose,
            'target_pose',
            self.execute_callback
        )

    async def execute_callback(self, goal_handle):
        self.get_logger().info('target_pose 요청 받음')

        if not goal_handle.request.start:
            self.get_logger().warn('start=False로 요청됨')
            goal_handle.abort()
            return TargetPose.Result(success=False)

        mode = goal_handle.request.mode  # "wm" 또는 "basket"

        self.get_logger().info(f"세탁물 감지 실행: mode={mode}")

        try:
            result_json = subprocess.check_output(
                ['python3', 'run_laundry_detector.py', '--mode', mode, '--json'],
                stderr=subprocess.STDOUT,
                timeout=10.0
            ).decode('utf-8').strip().split('\n')[-1]  # 마지막 줄만 JSON으로 사용

            result_data = json.loads(result_json)
            centers = result_data.get("centers", [])

            if not result_data.get("success", False) or len(centers) == 0:
                self.get_logger().warn("의류 미검출")
                goal_handle.abort()
                return TargetPose.Result(success=False)

            # 첫 번째 중심 좌표 사용
            center = centers[0]
            target_pose = Pose()
            target_pose.position.x = center[0]
            target_pose.position.y = center[1]
            target_pose.position.z = center[2]
            target_pose.orientation.w = 1.0  # 기본 회전 없음

            result = TargetPose.Result()
            result.success = True
            result.target_pose = target_pose

            self.get_logger().info(f"좌표 반환: x={center[0]:.3f}, y={center[1]:.3f}, z={center[2]:.3f}")
            goal_handle.succeed()
            return result

        except subprocess.TimeoutExpired:
            self.get_logger().error("감지 프로세스 타임아웃")
            goal_handle.abort()
            return TargetPose.Result(success=False)

        except Exception as e:
            self.get_logger().error(f"감지 중 예외 발생: {e}")
            goal_handle.abort()
            return TargetPose.Result(success=False)

def main(args=None):
    rclpy.init(args=args)
    node = TargetPoseServer()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
