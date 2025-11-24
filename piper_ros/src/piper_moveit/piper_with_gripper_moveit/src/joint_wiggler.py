# import rclpy
# import math
# from rclpy.node import Node
# from rclpy.clock import Clock

# from sensor_msgs.msg import JointState
# from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
# from builtin_interfaces.msg import Duration


# class JointWigglerNode(Node):

#     def __init__(self):
#         super().__init__('joint_wiggler_node')

#         # --- 사용자가 수정해야 할 파라미터 ---
#         self.declare_parameter('target_joint', 'joint3')
#         self.declare_parameter('amplitude', 0.1)
#         self.declare_parameter('frequency', 0.5)
#         self.declare_parameter('controller_name', '/arm_controller')

#         # 로봇의 전체 관절 이름
#         self.joint_names = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
        
#         # 파라미터 가져오기
#         self.target_joint = self.get_parameter('target_joint').get_parameter_value().string_value
#         self.amplitude = self.get_parameter('amplitude').get_parameter_value().double_value
#         self.wiggle_frequency = self.get_parameter('frequency').get_parameter_value().double_value
#         self.controller_name = self.get_parameter('controller_name').get_parameter_value().string_value
        
#         # 현재 및 초기 관절 상태를 저장할 변수
#         self.initial_positions = None # [수정됨] 움직임의 기준이 되는 초기 위치
#         self.current_joint_positions = None # [수정됨] 실시간으로 업데이트되는 현재 위치
#         self.target_joint_index = self.joint_names.index(self.target_joint)
#         self.start_time = self.get_clock().now()

#         # 현재 관절 상태 구독자
#         self.create_subscription(JointState, '/joint_states', self.joint_states_callback, 10)

#         # 관절 제어 명령 발행자
#         self.publisher = self.create_publisher(JointTrajectory, f'{self.controller_name}/joint_trajectory', 10)
        
#         # 0.01초 (100Hz) 간격으로 타이머 콜백 실행
#         self.create_timer(0.01, self.timer_callback)

#         self.get_logger().info(f"✅ Joint Wiggler 시작. 대상 관절: '{self.target_joint}', 진폭: {self.amplitude} rad")

#     def joint_states_callback(self, msg: JointState):
#         """
#         [수정됨] /joint_states 토픽을 구독하여 현재 관절 상태를 계속 업데이트합니다.
#         """
#         # 관절 이름 순서에 맞게 현재 위치 값을 정렬하여 리스트로 만듭니다.
#         current_positions_temp = []
#         for name in self.joint_names:
#             try:
#                 idx = msg.name.index(name)
#                 current_positions_temp.append(msg.position[idx])
#             except ValueError:
#                 # 아직 모든 관절 정보가 도착하지 않았을 수 있으므로 함수 종료
#                 return
        
#         # 완성된 리스트를 클래스 변수에 저장
#         self.current_joint_positions = current_positions_temp

#         # 만약 초기 위치가 아직 설정되지 않았다면, 첫 수신값을 초기 위치로 지정합니다.
#         if self.initial_positions is None:
#             self.initial_positions = self.current_joint_positions
#             self.get_logger().info(f"초기 관절 위치 수신 완료: {self.initial_positions}")

#     def timer_callback(self):
#         """0.01초마다 호출되어 목표 관절 위치를 계산하고 메시지를 전송합니다."""
#         # 초기 위치와 현재 위치가 모두 수신되었는지 확인
#         if self.initial_positions is None or self.current_joint_positions is None:
#             self.get_logger().warn("아직 관절 상태 정보를 수신하지 못했습니다. 대기 중...")
#             return

#         # 사인파(sin wave)를 이용해 부드러운 움직임 생성
#         elapsed_time = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
#         offset = self.amplitude * math.sin(2 * math.pi * self.wiggle_frequency * elapsed_time)

#         # 목표 위치는 드리프트를 방지하기 위해 '초기 위치'를 기준으로 계산
#         target_positions = list(self.initial_positions)
#         target_positions[self.target_joint_index] += offset

#         # [수정됨] 현재 값과 목표 값을 함께 로그로 출력
#         # 소수점 4자리까지만 보이도록 포매팅
#         current_pos_str = [f"{p:.4f}" for p in self.current_joint_positions]
#         target_pos_str = [f"{p:.4f}" for p in target_positions]
#         self.get_logger().info(f"현재값: {current_pos_str} -> 목표값: {target_pos_str}")

#         # 전송할 메시지 생성
#         traj_msg = JointTrajectory()
#         traj_msg.header.stamp = self.get_clock().now().to_msg()
#         traj_msg.joint_names = self.joint_names

#         point = JointTrajectoryPoint()
#         point.positions = [float(p) for p in target_positions]
#         point.time_from_start = Duration(sec=0, nanosec=50000000) # 0.05초

#         traj_msg.points.append(point)
#         self.publisher.publish(traj_msg)


# def main(args=None):
#     rclpy.init(args=args)
#     node = JointWigglerNode()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()

# if __name__ == '__main__':
#     main()
import rclpy
import pinocchio as pin
import numpy as np
import sys
import os
import math
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from ament_index_python.packages import get_package_share_directory

class JointWigglerWithJacobian(Node):
    def __init__(self, urdf_path):
        super().__init__('joint_wiggler_with_jacobian_node')

        # [Pinocchio 통합] URDF 파일로부터 로봇 모델 빌드
        try:
            self.model = pin.buildModelFromUrdf(urdf_path)
            self.data = self.model.createData()
            self.get_logger().info(f"✅ URDF 로드 성공: {urdf_path}")
        except Exception as e:
            self.get_logger().error(f"❌ URDF 파일 로드 실패: {e}")
            sys.exit(1)

        # Wiggler 파라미터 설정
        self.declare_parameter('target_joint', 'joint3')
        self.declare_parameter('amplitude', 0.1)
        self.declare_parameter('frequency', 0.5)
        self.target_joint = self.get_parameter('target_joint').get_parameter_value().string_value
        self.amplitude = self.get_parameter('amplitude').get_parameter_value().double_value
        self.wiggle_frequency = self.get_parameter('frequency').get_parameter_value().double_value
        
        # [수정] Pinocchio 계산에 사용할 전체 관절 목록 (URDF 기준, 8개)
        # self.pinocchio_joint_names = [j.name for j in self.model.joints if 'joint' in j.name]
        self.pinocchio_joint_names = [name for name in self.model.names if 'joint' in name]
        # [수정] 컨트롤러에 명령을 보낼 때 사용할 팔 관절 목록 (6개)
        self.controller_joint_names = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
        self.get_logger().info(f"Pinocchio 모델 관절 (총 {len(self.pinocchio_joint_names)}개): {self.pinocchio_joint_names}")
        self.get_logger().info(f"컨트롤러 대상 관절 (총 {len(self.controller_joint_names)}개): {self.controller_joint_names}")
        
        self.target_joint_index_in_full_list = self.pinocchio_joint_names.index(self.target_joint)
        self.ee_frame_name = "link7"
        self.ee_frame_id = self.model.getFrameId(self.ee_frame_name)

        # 상태 변수 초기화
        self.initial_full_positions = None
        self.current_full_positions = None
        self.start_time = self.get_clock().now()

        # ROS 통신 설정
        self.create_subscription(JointState, '/joint_states', self.joint_states_callback, 10)
        self.publisher = self.create_publisher(JointTrajectory, '/arm_controller/joint_trajectory', 10)
        self.create_timer(0.01, self.timer_callback)

    def joint_states_callback(self, msg: JointState):
        # [수정] Pinocchio가 사용하는 전체 관절 목록(8개) 기준으로 현재 위치를 저장
        current_positions_temp = []
        try:
            for name in self.pinocchio_joint_names:
                idx = msg.name.index(name)
                current_positions_temp.append(msg.position[idx])
        except (ValueError, IndexError):
            return
        
        self.current_full_positions = np.array(current_positions_temp)

        if self.initial_full_positions is None:
            self.initial_full_positions = self.current_full_positions
            self.get_logger().info(f"초기 전체 관절 위치 수신 완료: {self.initial_full_positions}")

    def timer_callback(self):
        if self.initial_full_positions is None:
            self.get_logger().warn("관절 상태 정보 수신 대기 중...", throttle_duration_sec=2)
            return

        # 1. 목표 위치 계산 (전체 8개 관절 기준)
        elapsed_time = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        offset = self.amplitude * math.sin(2 * math.pi * self.wiggle_frequency * elapsed_time)
        target_full_positions = self.initial_full_positions.copy()
        target_full_positions[self.target_joint_index_in_full_list] += offset

        # 2. 자코비안 계산 (전체 8개 관절 값 사용)
        pin.computeJointJacobians(self.model, self.data, self.current_full_positions)
        jacobian_current = pin.getFrameJacobian(self.model, self.data, self.ee_frame_id, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)

        pin.computeJointJacobians(self.model, self.data, target_full_positions)
        jacobian_target = pin.getFrameJacobian(self.model, self.data, self.ee_frame_id, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)
        
        np.set_printoptions(precision=3, suppress=True)
        log_msg = f"\n--- 현재 값의 Jacobian ---\n{jacobian_current}\n--- 목표 값의 Jacobian ---\n{jacobian_target}"
        self.get_logger().info(log_msg, throttle_duration_sec=0.5)

        # 3. 목표 위치로 이동 명령 전송 (팔 6개 관절만 필터링)
        # [수정] 컨트롤러가 이해할 수 있도록 6개 관절 값만 추출
        target_controller_positions = []
        for name in self.controller_joint_names:
            idx = self.pinocchio_joint_names.index(name)
            target_controller_positions.append(target_full_positions[idx])

        traj_msg = JointTrajectory()
        traj_msg.header.stamp = self.get_clock().now().to_msg()
        traj_msg.joint_names = self.controller_joint_names # 6개 관절 이름 전송
        point = JointTrajectoryPoint()
        point.positions = target_controller_positions # 6개 관절 위치 전송
        point.time_from_start = Duration(sec=0, nanosec=50000000)
        traj_msg.points.append(point)
        self.publisher.publish(traj_msg)

def main(args=None):
    if len(sys.argv) < 2:
        print("사용법: python3 your_script.py <urdf_파일_경로>")
        try:
            pkg_path = get_package_share_directory('piper_description')
            urdf_path_auto = os.path.join(pkg_path, 'urdf', 'piper_description.urdf')
            print(f"자동으로 찾은 경로: {urdf_path_auto}")
            sys.argv.append(urdf_path_auto)
        except Exception:
            return
    urdf_path = sys.argv[1]

    rclpy.init(args=args)
    node = JointWigglerWithJacobian(urdf_path)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()