import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import time

class GripperTestNode(Node):
    def __init__(self):
        super().__init__('gripper_test_node')
        self.client = ActionClient(self, FollowJointTrajectory, '/gripper_controller/follow_joint_trajectory')
        self.joint_names = ['joint7', 'joint8']

    def send_gripper_command(self, joint7, joint8, duration_sec=1.0):
        if not self.client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Gripper action server not available')
            return False

        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = self.joint_names

        point = JointTrajectoryPoint()
        point.positions = [joint7, joint8]
        point.time_from_start = Duration(sec=int(duration_sec), nanosec=0)
        goal_msg.trajectory.points.append(point)
        goal_msg.trajectory.header.stamp = self.get_clock().now().to_msg()

        self.get_logger().info(f"Sending gripper command: {joint7}, {joint8}")
        future = self.client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected')
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result()

        if result.status == 4:
            self.get_logger().error("Goal was aborted.")
            return False

        self.get_logger().info('Gripper command complete')
        return True


def main():
    rclpy.init()
    node = GripperTestNode()

    try:
        # 열기 테스트
        node.send_gripper_command(0.035, -0.035)
        time.sleep(2.0)

        # 닫기 테스트
        node.send_gripper_command(0.0, -0.0)
        time.sleep(2.0)

        # 다시 열기 테스트
        node.send_gripper_command(0.035, -0.035)
        time.sleep(2.0)

    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()