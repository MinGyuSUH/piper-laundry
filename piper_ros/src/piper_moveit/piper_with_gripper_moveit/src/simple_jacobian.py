import rclpy
import pinocchio as pin
import numpy as np
import sys
import os
from rclpy.node import Node
from sensor_msgs.msg import JointState
from ament_index_python.packages import get_package_share_directory

class SimpleJacobianCalculator(Node):
    def __init__(self, urdf_path):
        super().__init__('simple_jacobian_calculator')

        # 1. URDF 파일로부터 로봇 모델 빌드
        try:
            self.model = pin.buildModelFromUrdf(urdf_path)
            self.data = self.model.createData()
            self.get_logger().info(f"✅ URDF 로드 성공: {urdf_path}")
        except Exception as e:
            self.get_logger().error(f"❌ URDF 파일 로드 실패: {e}")
            sys.exit(1)

        # 로봇의 관절 이름 (URDF에 정의된 순서와 같아야 함)
        # self.joint_names = [j.name for j in self.model.joints if 'joint' in j.name]
        self.joint_names = [name for name in self.model.names if 'joint' in name]
        self.get_logger().info(f"감지된 관절: {self.joint_names}")
        
        # End-Effector 프레임(링크)의 ID 찾기
        self.ee_frame_name = "link7"  # 사용자의 로봇 End-Effector 링크 이름으로 변경!
        if not self.model.existFrame(self.ee_frame_name):
            self.get_logger().error(f"'{self.ee_frame_name}' 프레임을 URDF에서 찾을 수 없습니다.")
            sys.exit(1)
        self.ee_frame_id = self.model.getFrameId(self.ee_frame_name)

        # 현재 관절 상태를 받기 위한 구독자(Subscriber)
        self.create_subscription(JointState, '/joint_states', self.joint_states_callback, 10)

    def joint_states_callback(self, msg: JointState):
        """/joint_states 토픽을 받아 자코비안을 계산하고 출력합니다."""
        
        # URDF 모델 순서에 맞게 현재 관절 위치(q)를 정렬
        q = []
        try:
            for name in self.joint_names:
                idx = msg.name.index(name)
                q.append(msg.position[idx])
        except ValueError:
            return # 모든 관절 정보가 아직 도착하지 않음

        q_np = np.array(q)

        # Pinocchio를 사용하여 자코비안 계산
        # computeJointJacobians는 모든 조인트에 대한 자코비안을 계산
        pin.computeJointJacobians(self.model, self.data, q_np)
        # getFrameJacobian은 특정 프레임(End-Effector)에 대한 자코비안을 추출
        # pin.ReferenceFrame.LOCAL_WORLD_ALIGNED : 월드 좌표계 기준 자코비안
        jacobian_matrix = pin.getFrameJacobian(self.model, self.data, self.ee_frame_id, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)

        # 결과 출력 (0.5초에 한 번씩)
        np.set_printoptions(precision=4, suppress=True)
        log_msg = f"\n--- Jacobian (at q={q_np.round(4)}) ---\n{jacobian_matrix}"
        self.get_logger().info(log_msg, throttle_duration_sec=0.5)

def main(args=None):
    if len(sys.argv) < 2:
        print("사용법: python3 simple_jacobian.py <urdf_파일_경로>")
        # 자동으로 URDF 경로를 찾아주는 예시
        try:
            pkg_path = get_package_share_directory('piper_description')
            urdf_path_auto = os.path.join(pkg_path, 'urdf', 'piper.urdf')
            print(f"자동으로 찾은 경로: {urdf_path_auto}")
            sys.argv.append(urdf_path_auto)
        except Exception:
            return

    urdf_path = sys.argv[1]
    if not os.path.exists(urdf_path):
        print(f"오류: URDF 파일을 찾을 수 없습니다 - {urdf_path}")
        return

    rclpy.init(args=args)
    node = SimpleJacobianCalculator(urdf_path)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()