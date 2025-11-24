import rclpy
import time
from rclpy.node import Node
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose, PoseStamped
from std_msgs.msg import Bool
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint, JointTrajectory
from builtin_interfaces.msg import Duration
from xela_server_ros2.msg import SensStream
import numpy as np
import threading
from piper_with_gripper_moveit.action import PoseGoal
from piper_with_gripper_moveit.action import TargetPose
import sys
from scipy.spatial.transform import Rotation as R
from rclpy.clock import Clock
import os
from collections import deque


class ControlTower(Node):

    def __init__(self):
        super().__init__('control_tower_node')

        self.ee_pose = None

        self.create_subscription(Pose, '/end_pose', self.ee_pose_callback, 10)
        self._arm_client = ActionClient(self, FollowJointTrajectory, '/moveit_action/arm_controller/follow_joint_trajectory')
        # self._sdk_client = ActionClient(self, FollowJointTrajectory, '/arm_controller/follow_joint_trajectory')
        self._pose_client = ActionClient(self, PoseGoal, 'pose_goal')

        # 서버가 뜰 때까지 기다리는 로직을 __init__에 추가하면 더 안정적입니다.
        servers_ready = True
        if not self._arm_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Arm action server not available.")
            servers_ready = False

        # if not self._sdk_client.wait_for_server(timeout_sec=5.0):
        #     self.get_logger().error("Arm action server not available.")
        #     servers_ready = False

        if not self._pose_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Pose Goal action server not available.")
            servers_ready = False

        if not servers_ready:
            self.destroy_node()
            rclpy.shutdown()
            raise RuntimeError("필수 액션 서버를 찾을 수 없습니다.")


        self.arm_joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']

    def ee_pose_callback(self, msg:Pose):

        pos = msg.position
        ori = msg.orientation

        pose_msg = Pose()
        pose_msg.position.x = pos.x
        pose_msg.position.y = pos.y
        pose_msg.position.z = pos.z
        pose_msg.orientation.x = ori.x
        pose_msg.orientation.y = ori.y
        pose_msg.orientation.z = ori.z
        pose_msg.orientation.w = ori.w

        self.ee_pose = pose_msg
    
    def _wait_for_ee_pose(self, timeout=5.0):
        end = self.get_clock().now().nanoseconds + int(timeout*1e9)
        while rclpy.ok() and self.ee_pose is None:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.get_clock().now().nanoseconds > end:
                return False
        return True

    def send_pose(self, pose: PoseStamped): ## mode 6
        goal_msg = PoseGoal.Goal()
        goal_msg.target_pose = pose

        goal_msg.mode = 6 # rpy

        send_goal_future = self._pose_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future)
        goal_handle = send_goal_future.result()

        if not goal_handle or not goal_handle.accepted:
            self.get_logger().error("pose_goal이 거부되었습니다.")
            return False
        self.get_logger().info("pose_goal 명령 전송 완료.")

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result().result

        if result.success:
            self.get_logger().info(f"✅ pose 명령 성공: {result.message}")
            return True
        else:
            self.get_logger().warn(f"⚠️ pose 명령 실패: {result.message}")
            # return False

    def move_forward(self, pose: PoseStamped): ## mode 7

        goal_msg = PoseGoal.Goal()
        goal_msg.target_pose = pose

        goal_msg.mode = 8 # pose2

        send_goal_future = self._pose_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future)
        try:
            goal_handle = send_goal_future.result()
        except Exception as e:
            self.get_logger().error(f"pose_goal 전송 예외: {e!r}")
            return False

        if not goal_handle or not goal_handle.accepted:
            self.get_logger().error("pose_goal이 거부되었습니다.")
            return False
        self.get_logger().info("pose_goal 명령 전송 완료.")

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result().result

        if result.success:
            self.get_logger().info(f"✅ pose 명령 성공: {result.message}")
            return True
        else:
            self.get_logger().warn(f"⚠️ pose 명령 실패: {result.message}")
            return False
        
    def MOVE_DEEP(self, pose: PoseStamped): ## mode 8
        goal_msg = PoseGoal.Goal()
        goal_msg.target_pose = pose

        goal_msg.mode = 9 # pose2

        send_goal_future = self._pose_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future)
        try:
            goal_handle = send_goal_future.result()
        except Exception as e:
            self.get_logger().error(f"pose_goal 전송 예외: {e!r}")
            return False

        if not goal_handle or not goal_handle.accepted:
            self.get_logger().error("pose_goal이 거부되었습니다.")
            return False
        self.get_logger().info("pose_goal 명령 전송 완료.")

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result().result

        if result.success:
            self.get_logger().info(f"✅ pose 명령 성공: {result.message}")
            return True
        else:
            self.get_logger().warn(f"⚠️ pose 명령 실패: {result.message}")
            return False
    
    def execute_sequence(self):

        self.get_logger().info("Step 0: 문 여는 중")
        # time.sleep(5)
        count = 0

        while rclpy.ok():

            if not self._wait_for_ee_pose(timeout=5.0):
                self.get_logger().error("/end_pose 초기값을 못 받았습니다. 퍼블리셔/토픽/타입/QoS 확인!")
                return
            
            if count ==0:
                target = self.ee_pose
                self.get_logger().info("target set")
                count += 1
            
            pose_stamped = PoseStamped()
            pose_stamped.pose = target
            pose_stamped.header.frame_id = 'base_link'
            pose_stamped.header.stamp = Clock().now().to_msg()

            self.get_logger().info("Step 1: MODE 8") ## tcp
            self.move_forward(pose_stamped)

            self.get_logger().info("Step 1: MODE 6") # DEEP
            self.send_pose(pose_stamped)
            
            # self.get_logger().info("Step 1: MODE 9") ## DEEP
            # self.MOVE_DEEP(pose_stamped)

            # self.get_logger().info("Step 1: MODE 6")
            # self.send_pose(pose_stamped)

            continue


# ================================================================= #
# ===================== 핵심 변경 부분: main 함수 =================== #
# ================================================================= #

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = ControlTower()
    except RuntimeError as e:
        print(f"노드 초기화 중 오류 발생: {e}")
        return

    # MultiThreadedExecutor는 여전히 사용합니다.
    # 콜백들이 블로킹 함수(spin_until_future_complete)에 의해 중단되지 않도록 하기 위함입니다.
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)

    # 메인 로직을 실행할 작업 스레드
    sequence_thread = threading.Thread(target=node.execute_sequence, daemon=True)
    sequence_thread.start()

    # =================================================================================
    # 중요: 메인 스레드에서는 executor.spin()을 호출하지 않습니다!
    # 대신, 작업 스레드가 끝날 때까지 (또는 Ctrl+C가 눌릴 때까지) 기다리기만 합니다.
    # 모든 ROS 메시지 처리는 sequence_thread 내부의 rclpy.spin_until_future_complete가
    # 전담하게 됩니다.
    # =================================================================================
    try:
        # 작업 스레드가 살아있는 동안 메인 스레드는 여기서 대기합니다.
        sequence_thread.join()
    except KeyboardInterrupt:
        node.get_logger().info('사용자에 의해 중단 요청됨.')
    finally:
        node.get_logger().info('노드 종료 중...')
        # Executor와 노드를 깔끔하게 종료합니다.
        executor.shutdown()
        # 노드가 이미 파괴되었을 수 있으므로 확인 후 파괴
        if rclpy.ok():
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()


    
