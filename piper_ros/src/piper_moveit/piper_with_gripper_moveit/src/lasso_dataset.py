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
from piper_with_gripper_moveit.action import TargetPose
from piper_with_gripper_moveit.action import PoseGoal
import sys
from scipy.spatial.transform import Rotation as R
from rclpy.clock import Clock
import os
import joblib
from collections import deque

# SCALER_FILE = 'scaler_kmeansf.pkl'
# KMEANS_MODEL_FILE = 'kmeans_modelf.pkl'

# SCALER_FILE = 'scaler_model.pkl'
# FCM_CENTERS_FILE = 'fcm_centers.npy'

# FCM_M = 1.5
# WINDOW_SIZE = 5 


class ControlTower(Node):

    def __init__(self):
        super().__init__('control_tower_node')

        # try:
        #     script_dir = os.path.dirname(os.path.abspath(__file__))
        #     self.scaler = joblib.load(os.path.join(script_dir, SCALER_FILE))
        #     self.kmeans = joblib.load(os.path.join(script_dir, KMEANS_MODEL_FILE))
        #     self.n_clusters = self.kmeans.n_clusters
        #     self.get_logger().info(f"✅ Models loaded.")
        # except Exception as e:
        #     self.get_logger().error(f"❌ Model loading failed: {e}")
        #     self.destroy_node()
        #     return

        # try:
        #     script_dir = os.path.dirname(os.path.abspath(__file__))
        #     self.scaler = joblib.load(os.path.join(script_dir, SCALER_FILE))
        #     self.fcm_centers = np.load(os.path.join(script_dir, FCM_CENTERS_FILE))
        #     self.n_clusters = self.fcm_centers.shape[0]
        #     self.get_logger().info(f"✅ Models loaded. Final centers shape: {self.fcm_centers.shape}")
        # except Exception as e:
        #     self.get_logger().error(f"❌ Model loading failed: {e}")
        #     self.destroy_node()
        #     return

        self.ee_pose = None
        self.prev_x = None
        self.prev_y = None
        self.prev_z = None
        self.data_buffer = deque(maxlen=5)
        self.cum_x = [0.0] * 16
        self.cum_y = [0.0] * 16
        self.cum_z = [0.0] * 16

        self.cluster_label = None # cluster_color 대신 label을 직접 저장
        self.z_aligned = False
        self.contacted = False

        self.sensor_update_count = 0

        self.subscription = self.create_subscription(
            SensStream,
            '/xServTopic',
            self.sensor_callback,
            10
        )
        self.create_subscription(Pose, '/end_pose', self.ee_pose_callback, 10)
        self.clik_publisher = self.create_publisher(Bool, '/mode_',1)

        self._arm_client = ActionClient(self, FollowJointTrajectory, '/moveit_action/arm_controller/follow_joint_trajectory')
        self._gripper_client = ActionClient(self, FollowJointTrajectory, '/gripper_controller/follow_joint_trajectory')
        self._pose_client = ActionClient(self, PoseGoal, 'pose_goal')

        # 서버가 뜰 때까지 기다리는 로직을 __init__에 추가하면 더 안정적입니다.
        servers_ready = True
        if not self._arm_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Arm action server not available.")
            servers_ready = False
        if not self._gripper_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Gripper action server not available.")
            servers_ready = False
        if not self._pose_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Pose Goal action server not available.")
            servers_ready = False

        if not servers_ready:
            self.destroy_node()
            rclpy.shutdown()
            raise RuntimeError("필수 액션 서버를 찾을 수 없습니다.")


        self.arm_joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
        self.gripper_joint_names = ['joint7', 'joint8']

        self.ba_init = [-1.6379, 1.5724, -1.2721, -0.0566, 1.2838, 0.2908, 0.0, 0.0]
        self.wm_init = [1.5640, 0.4538, -1.5319, -0.0488, 1.2797, 0.3169, 0.0, 0.0]
        self.ba_end = [-1.5527, 2.6613, -1.5133, -0.2642, 0.6119, 0.3951, 0.0, 0.0]
        self.wm_end = [1.571, 2.146, -2.268, 0.000, 0.594, 0.000, 0.0, 0.0]
        self.wm_mid = [1.574251224, 0.336651756, -1.159398016, -0.185900708, 1.287576528, -1.357858404, 0.0, 0.0]
        self.home = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.f = [1.574251224, 0.336651756, -1.159398016, -0.185900708, 1.287576528, -0.1, 0.0, 0.0] # ft
        self.ft = [1.5533, 0.1396, -0.4014, 0.0349, 0.3665, 0.1745, 0.0, 0.0]

        self.ff = [1.6406, 0.9948, -1.3090, -0.1571, 0.8203, 0.1571, 0.0, 0.0] # fs 

    def ee_pose_callback(self, msg):
        pos = msg.position
        ori = msg.orientation
        r = R.from_quat([ori.x, ori.y, ori.z, ori.w])
        T = np.eye(4)
        T[:3, :3] = r.as_matrix()
        T[:3, 3] = [pos.x, pos.y, pos.z]
        self.ee_pose = T

    # 사용자님의 원본 _send_goal 함수
    def _send_goal(self, client, joint_names, joint_values, wait_for_result=True, timeout_sec=10.0):
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = joint_names
        goal_msg.trajectory.header.stamp = self.get_clock().now().to_msg()
        point = JointTrajectoryPoint()
        point.positions = [float(x) for x in joint_values]

        if set(joint_names) == set(self.gripper_joint_names):
            point.time_from_start = Duration(sec=2, nanosec=0)
        else:
            point.time_from_start = Duration(sec=5, nanosec=0)

        goal_msg.trajectory.points.append(point)

        # wait_for_server는 __init__에서 이미 확인했으므로 여기서는 생략 가능
        # if not client.wait_for_server(timeout_sec=timeout_sec):
        #     self.get_logger().error(f"Action server '{client._action_name}' not available.")
        #     return False

        send_goal_future = client.send_goal_async(goal_msg)
        # 여기서 spin은 작업 스레드에서만 독점적으로 사용됩니다.
        rclpy.spin_until_future_complete(self, send_goal_future)

        goal_handle = send_goal_future.result()
        if not goal_handle or not goal_handle.accepted:
            self.get_logger().warn(f"Goal for '{client._action_name}' rejected.")
            # return False
        if not wait_for_result:
            return True

        get_result_future = goal_handle.get_result_async()
        # 여기서 spin은 작업 스레드에서만 독점적으로 사용됩니다.
        rclpy.spin_until_future_complete(self, get_result_future)

        result = get_result_future.result()
        if not result:
            self.get_logger().error("결과를 받지 못했습니다.")
            # return False

        if result.status == GoalStatus.STATUS_SUCCEEDED:
            return True
        else:
            self.get_logger().warn(f"Action '{client._action_name}' failed with status: {result.status}")
            # return False

    def send_joint_command(self, joint_values_8dof):
        self.get_logger().info(f"명령 수신: {joint_values_8dof}")
        arm_target = joint_values_8dof[0:6]
        gripper_target = joint_values_8dof[6:8]

        arm_success = self._send_goal(self._arm_client, self.arm_joint_names, arm_target, wait_for_result=True)
        if not arm_success:
            self.get_logger().error("팔 이동 실패. 다음 동작을 취소합니다.")
            # return False

        gripper_success = self._send_goal(self._gripper_client, self.gripper_joint_names, gripper_target, wait_for_result=True)
        if not gripper_success:
            self.get_logger().error("그리퍼 이동 실패.")
            # return False

        self.get_logger().info(f"명령 완료: {joint_values_8dof}")
        return True

    def goal(self, joint_values_8dof, gripper_action='keep'):
        self.get_logger().info(f"[Goal 명령] Pose: {joint_values_8dof}, Gripper: {gripper_action}")
        arm_target = joint_values_8dof[:6]

        success = self._send_goal(self._arm_client, self.arm_joint_names, arm_target, wait_for_result=True)
        if not success:
            self.get_logger().error("  >> 팔 이동 실패")
            # return False

        if gripper_action == 'open':
            return self.open_gripper()
        elif gripper_action == 'close':
            return self.close_gripper()
        
        return True

    def send_gripper_command(self, joint7, joint8, wait_for_result=True):
        gripper_target = [joint7, joint8]
        return self._send_goal(
            self._gripper_client,
            self.gripper_joint_names,
            gripper_target,
            wait_for_result=wait_for_result
        )
    
    def open_gripper(self):
        self.get_logger().info("그리퍼 오픈")
        return self.send_gripper_command(0.035, -0.035, wait_for_result=True)

    def open_gripper2(self):
        self.get_logger().info("그리퍼 오픈")
        return self.send_gripper_command(0.03, -0.03, wait_for_result=True)

    def close_gripper(self):
        self.get_logger().info("그리퍼 클로즈")
        return self.send_gripper_command(0.0, -0.0, wait_for_result=True)

    def send_pose_wm(self, pose: PoseStamped):
        goal_msg = PoseGoal.Goal()
        goal_msg.target_pose = pose

        goal_msg.mode = 6 # rpy

        goal_msg.roll_tol_rad = 1.57             # 'tol_roll'/'tol_pitch' -> 'rp_tol_rad'
        goal_msg.pitch_tol_rad = 1.57/4
        goal_msg.yaw_tol_rad = 1.57/4

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

    def send_pose_ba(self, pose: PoseStamped):
        goal_msg = PoseGoal.Goal()
        self.get_logger().info(f"pose in send_pose_ba: {pose}")
        goal_msg.target_pose = pose

        # --- 여기가 수정된 부분 ---
        # .action 파일에 정의된 이름과 정확히 일치시켜야 합니다.
        goal_msg.mode = 6 # rpy
        goal_msg.roll_tol_rad = 1.57/2             # 'tol_roll'/'tol_pitch' -> 'rp_tol_rad'
        goal_msg.pitch_tol_rad = 1.57/2
        goal_msg.yaw_tol_rad = 6.28              # 'tol_yaw' -> 'yaw_tol_rad'
        
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

    def move_forward(self, pose: PoseStamped):
        # clik_mode = Bool()
        # clik_mode.data = True
        # self.clik_publisher.publish(clik_mode)
        goal_msg = PoseGoal.Goal()
        goal_msg.target_pose = pose

        goal_msg.mode = 7 # pose2

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
            # clik_mode.data = False
            # self.clik_publisher.publish(clik_mode)
            return True
        else:
            self.get_logger().warn(f"⚠️ pose 명령 실패: {result.message}")
            return False
        

    def MOVE_DEEP(self, pose: PoseStamped):
        # clik_mode = Bool()
        # clik_mode.data = True
        # self.clik_publisher.publish(clik_mode)
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
            # clik_mode.data = False
            # self.clik_publisher.publish(clik_mode)
            return True
        else:
            self.get_logger().warn(f"⚠️ pose 명령 실패: {result.message}")
            return False



    def reset_sensor_data(self):
        self.prev_x, self.prev_y, self.prev_z = None, None, None
        self.cum_x, self.cum_y, self.cum_z = [0.0] * 16, [0.0] * 16, [0.0] * 16
        self.z_aligned, self.contacted = False, False
        self.cluster_label = None
        self.get_logger().info("센서 기준값 및 상태 초기화 완료.")

    def classify_realtime_data(self, current_window):
        window_scaled = self.scaler.transform(current_window.reshape(1, -1))
        dist = np.linalg.norm(self.fcm_centers - window_scaled, axis=1)
        dist[dist < 1e-9] = 1e-9
        power = 2. / (FCM_M - 1)
        inv_dist_power = 1.0 / (dist ** power)
        u = inv_dist_power / np.sum(inv_dist_power)
        return np.argmax(u)

    def sensor_callback(self, msg):
        try:
            x_vals = [t.x for t in msg.sensors[0].taxels]
            y_vals = [t.y for t in msg.sensors[0].taxels]
            z_vals = [t.z for t in msg.sensors[0].taxels]

            if self.prev_x is not None:
                dx = [c - p for c, p in zip(x_vals, self.prev_x)]
                dy = [c - p for c, p in zip(y_vals, self.prev_y)]
                dz = [c - p for c, p in zip(z_vals, self.prev_z)]

                for i in range(16):
                    self.cum_x[i] += dx[i]
                    self.cum_y[i] += dy[i]
                    self.cum_z[i] += dz[i]

                vecs = np.array([self.cum_x, self.cum_y, self.cum_z]).T
                norms = np.linalg.norm(vecs, axis=1)
                max_norm = np.max(norms)
                if max_norm == 0:
                    max_norm = 1e-6
                scaled_vecs = vecs / max_norm
                weights = norms
                weights[weights == 0] = 1e-6

                mean_vec_biased = np.sum(vecs, axis=0) / np.sum(weights)
                current_features = np.concatenate([scaled_vecs.flatten(), mean_vec_biased])

                norm = np.linalg.norm(mean_vec_biased)
                z_axis = np.array([0, 0, 1])
                cos_theta = np.dot(mean_vec_biased, z_axis) / (norm + 1e-8)
                theta_deg = np.degrees(np.arccos(np.clip(cos_theta, -1.0, 1.0)))

                if norm < 0.55:
                    arrow_color = 'black'
                    self.z_aligned = False
                else:
                    if theta_deg < 45:
                        arrow_color = 'green'
                        self.z_aligned = True
                        
                    elif theta_deg <= 135:
                        arrow_color = 'red'
                        if norm < 0.7:
                            self.contacted = False
                            self.z_aligned = False
                        else:
                            self.contacted = True
                            self.z_aligned = False

                    else:
                        arrow_color = 'black'
                        self.z_aligned = False

                # self.data_buffer.append(
                #     np.array(current_features)
                # )

                # if len(self.data_buffer) == WINDOW_SIZE:

                #     window_mat = np.asarray(self.data_buffer, dtype=float)  # shape: (W, 4)

                #     window_vec = window_mat.reshape(1, -1)

                #     # 분류
                #     cluster = self.classify_realtime_data(window_vec)

                #     # 시각화 색상 (가드)
                #     colors = ['green', 'white', 'red', 'blue', 'gray']
                #     cluster_color = colors[cluster]

                # try:
                #     vec3_scaled = self.scaler.transform(mean_vec_biased.reshape(1, -1))
                #     self.cluster_label = int(self.kmeans.predict(vec3_scaled)[0])

                #     colors = ['green', 'black', 'red', 'black']
                #     cluster_color = colors[self.cluster_label]


                    # if self.cluster_label == 0:
                    #     self.z_aligned = True
                    # else:
                    #     self.z_aligned = False
                    # if self.cluster_label == 1:
                    #     self.contacted = True

                        # if cluster_color == "green": 
                        #     self.z_aligned = True
                        # else:
                        #     self.z_aligned = False

                        # if cluster_color == "red" or cluster_color == "blue" :
                        #      self.contacted = True

                # except Exception as e:
                #     self.get_logger().error(f"KMeans/Scaler prediction error: {e}")
                #     self.cluster_label = None

            self.prev_x, self.prev_y, self.prev_z = x_vals, y_vals, z_vals
        except Exception as e:
            self.get_logger().error(f"Sensor callback error: {e}")

    def close_door(self):
        pass

    def open_door(self):
        pass

    def transform_pose_with_offset(self, pose: Pose, HT: np.ndarray):
        # 1. position 추출
        x = pose.position.x
        y = pose.position.y
        z = pose.position.z

        # 2. 동차좌표 벡터 생성
        point_vec = np.array([x, y, z, 1.0])  # 4x1 벡터

        # 3. 동차변환행렬 곱
        transformed = HT @ point_vec  # (4x4) @ (4x1) → (4x1)

        # 4. 새로운 Pose로 반환
        new_pose = Pose()
        new_pose.position.x = transformed[0]
        new_pose.position.y = transformed[1]
        new_pose.position.z = transformed[2]

        # orientation은 그대로 유지하거나 필요시 수정
        new_pose.orientation = pose.orientation

        return new_pose

    ## CLIK
    def get_pre_grasp_pose(self, target_pose_stamped: PoseStamped, offset_z: float):
        """
        주어진 목표 자세를 기준으로 End-Effector의 Z축 방향으로 오프셋을 적용한
        사전 접근 자세(pre-grasp pose)를 계산합니다.

        Args:
            target_pose_stamped (PoseStamped): 원본 목표 자세
            offset_z (float): Z축 방향으로의 오프셋 거리 (미터 단위).
                              뒤로 물러나려면 음수 값을 사용합니다 (예: -0.02).

        Returns:
            PoseStamped: 계산된 사전 접근 자세
        """
        self.get_logger().info(f"{offset_z * 100:.1f}cm 후퇴한 사전 접근 자세 계산 중...")

        # 1. 원본 Pose 정보 추출
        original_position = target_pose_stamped.pose.position
        original_orientation = target_pose_stamped.pose.orientation
        self.get_logger().info(f"original positoin: {original_position}")
        self.get_logger().info(f"original_orientation: {original_orientation}")

        # 2. Scipy를 사용하여 쿼터니언을 회전 행렬로 변환
        orientation_quat = [
            original_orientation.x,
            original_orientation.y,
            original_orientation.z,
            original_orientation.w
        ]
        rotation_matrix = R.from_quat(orientation_quat).as_matrix()

        # 3. 회전 행렬에서 Z축 방향 벡터 추출 (base_link 기준)
        z_axis_vector = rotation_matrix[:, 2]

        # 4. 오프셋 벡터 계산
        offset_vector = z_axis_vector * offset_z
        self.get_logger().info(f"z_axis_vector: {z_axis_vector}")
        self.get_logger().info(f"offset_z: {offset_z}")
        self.get_logger().info(f"offset_vector: {offset_vector}")

        # 5. 새로운 위치 계산
        pre_grasp_position = Pose().position
        pre_grasp_position.x = original_position.x + offset_vector[0]
        pre_grasp_position.y = original_position.y + offset_vector[1]
        pre_grasp_position.z = original_position.z + offset_vector[2]

        # 6. 새로운 PoseStamped 메시지 생성 (방향은 그대로 유지)
        pre_grasp_pose_stamped = PoseStamped()
        pre_grasp_pose_stamped.header = target_pose_stamped.header
        pre_grasp_pose_stamped.pose.position = pre_grasp_position
        pre_grasp_pose_stamped.pose.orientation = original_orientation

        return pre_grasp_pose_stamped
    ## CLIK
    
    def target_get_wm(self, mode="failure", first=True): ### basket, wm, mid
        # 액션 클라이언트가 처음이면 생성
        if not hasattr(self, 'action_client'):
            self.action_client = ActionClient(self, TargetPose, 'target_pose')

        # 서버 대기
        if not self.action_client.wait_for_server(timeout_sec=3.0):
            self.get_logger().error('target_pose 액션 서버를 찾을 수 없습니다.')
            return None

        # Goal 메시지 생성
        goal_msg = TargetPose.Goal()
        goal_msg.start = True  # 요청 트리거
        goal_msg.mode = mode

        # Goal 보내기
        self.get_logger().info('target_pose 요청 전송')
        future = self.action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)

        if not future.done():
            self.get_logger().error('Goal 요청 실패')
            return None

        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Goal이 액션 서버에서 거부됨')
            return None

        # 결과 대기
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result().result

        if result.success:
            self.get_logger().info(f"받은 EA: {result.target_pose}") # EA
            self.get_logger().info(f"현재 Pose: {self.ee_pose}") # EE

        
            R = np.array([
                [-0.07403140416428758,  0.9970253999896115, -0.02144068499412377],
                [-0.997213265121594,   -0.07381230005954229,  0.010837353249242954],
                [0.009222530183886607,  0.022183239967900933,  0.9997113827508088]
            ])

            t = np.array([[-0.04615831961252831],
                        [ 0.035761448900299364],
                        [ 0.06690399326489311]]) ######## 원래 07077399326489311 ############

            ee2cam = np.vstack((np.hstack((R, t)), [0, 0, 0, 1]))

            base2cam = self.ee_pose @ ee2cam ## O2C
            

            pose = self.transform_pose_with_offset(result.target_pose, base2cam) ## O2t
            self.get_logger().info(f"변환한 bA: {pose}") # bA

            pose_stamped = PoseStamped()
            pose_stamped.pose = pose
            ## 2cm 뒤로 이동 ##
            pose_stamped.pose.position.y += 0.02
            ## orientation 고정 ##
            # pose_stamped.pose.orientation.x = 0.4170340460031675
            # pose_stamped.pose.orientation.y = -0.5958716700924536
            # pose_stamped.pose.orientation.z = -0.5408718132224749
            # pose_stamped.pose.orientation.w = -0.4224656659622063


            pose_stamped.header.frame_id = 'base_link'
            pose_stamped.header.stamp = Clock().now().to_msg()

            # while not self.goal(self.ft, 'keep'):
            #     self.get_logger().warn("  >> 실패 → 재시도 중")

            return pose_stamped
        else:

            return None

    def target_get_ba(self, mode="basket"): ### basket, wm, mid
        # 액션 클라이언트가 처음이면 생성
        if not hasattr(self, 'action_client'):
            self.action_client = ActionClient(self, TargetPose, 'target_pose')

        # 서버 대기
        if not self.action_client.wait_for_server(timeout_sec=3.0):
            self.get_logger().error('target_pose 액션 서버를 찾을 수 없습니다.')
            return None

        # Goal 메시지 생성
        goal_msg = TargetPose.Goal()
        goal_msg.start = True  # 요청 트리거
        goal_msg.mode = mode

        # Goal 보내기
        self.get_logger().info('target_pose 요청 전송')
        future = self.action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)

        if not future.done():
            self.get_logger().error('Goal 요청 실패')
            return None

        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Goal이 액션 서버에서 거부됨')
            return None

        # 결과 대기
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result().result

        if result.success:
            self.get_logger().info(f"받은 EA: {result.target_pose}") # EA
            self.get_logger().info(f"현재 Pose: {self.ee_pose}") # EE

        
            R = np.array([
                [-0.07403140416428758,  0.9970253999896115, -0.02144068499412377],
                [-0.997213265121594,   -0.07381230005954229,  0.010837353249242954],
                [0.009222530183886607,  0.022183239967900933,  0.9997113827508088]
            ])

            t = np.array([[-0.04615831961252831],
                        [ 0.035761448900299364],
                        [ 0.06590399326489311]]) ######## 원래 07077399326489311 ############

            ee2cam = np.vstack((np.hstack((R, t)), [0, 0, 0, 1]))

            base2cam = self.ee_pose @ ee2cam ## O2C
            

            pose = self.transform_pose_with_offset(result.target_pose, base2cam) ## O2t
            self.get_logger().info(f"변환한 bA: {pose}") # bA

            pose_stamped = PoseStamped()
            pose_stamped.pose = pose
            ## clik check
            # pose_stamped.pose.position.x = -0.02
            # pose_stamped.pose.position.y = -0.29
            # pose_stamped.pose.position.z = 0.28
            # 
            pose_stamped.header.frame_id = 'base_link'
            pose_stamped.header.stamp = Clock().now().to_msg()

            self.get_logger().info(f"target_get_ba: (pose_stampled) {pose_stamped}")

            return pose_stamped
        else:
            self.get_logger().warn("결과 없음 -> 문 닫기 실행")
            self.close_door()
            while not self.goal(self.home, 'keep'):
                self.get_logger().warn("  >> 실패 → 재시도 중")
            rclpy.shutdown()
            sys.exit(0)
            return None

    def target_get2(self, mode = "failure_bg"):
        # 액션 클라이언트가 처음이면 생성
        if not hasattr(self, 'action_client'):
            self.action_client = ActionClient(self, TargetPose, 'target_pose')

        # 서버 대기
        if not self.action_client.wait_for_server(timeout_sec=3.0):
            self.get_logger().error('target_pose 액션 서버를 찾을 수 없습니다.')
            return None

        # Goal 메시지 생성
        goal_msg = TargetPose.Goal()
        goal_msg.start = True  # 요청 트리거
        goal_msg.mode = mode

        # Goal 보내기
        self.get_logger().info('target_pose 요청 전송')
        future = self.action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)

        if not future.done():
            self.get_logger().error('Goal 요청 실패')
            return None

        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Goal이 액션 서버에서 거부됨')
            return None

        # 결과 대기
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result().result

        if result.success:
            self.get_logger().info(f"success")
            return True
        else:
            self.get_logger().warn("None")
            return None
    

    def overcome(self):

        while True:

            while not self.goal(self.wm_mid, 'keep'):
                self.get_logger().warn("  >> 실패 → 재시도 중")

            self.get_logger().info("Step 2: 잡기 시도 중...")
            target_wm = self.target_get_wm()  # 감지 및 잡기 시도
            self.reset_sensor_data()

            if target_wm == None:
                self.get_logger().info("결과 없음")
                self.get_logger().info("THE END")
                break

            else:
                self.get_logger().info("err")
                break


            # for i in range(5):
            #     if self.send_pose_wm(target_wm): ## 모드 1
            #         break
            #     self.get_logger().warn(f"  >> pose 명령 실패 ({i+1}/5) → 재시도 중")
            # else:  
            #     self.get_logger().warn("  >> pose 명령 5회 연속 실패 → 새 target 요청")
            #     continue  # while True 처음으로

            # for i in range(5):
            #     if self.MOVE_DEEP(target_wm): ## 모드 8
            #         break
            #     self.get_logger().warn(f"  >> pose 명령 실패 ({i+1}/5) → 재시도 중")
            # else:  
            #     self.get_logger().warn("  >> pose 명령 5회 연속 실패 → 새 target 요청")
            #     continue  # while True 처음으로

            # self.close_gripper()

            # while not self.goal(self.ff, 'keep'):
            #     self.get_logger().warn("  >> 실패 → 재시도 중")

            # self.get_logger().info("🔍 Z-축 정렬 감지")

            # # self.z_aligned = True ####
            # # self.get_logger().warn(self.cum_x, self.cum_y, self.cum_z) 
            # if self.z_aligned:
            #     self.get_logger().info("✅ 잡기 성공")
            #     break
            # else:
            #     self.get_logger().warn("❌ 잡기 실패 →  재시도")
            #     while not self.open_gripper():
            #         self.get_logger().warn("  >> 그리퍼 오픈 실패 → 재시도 중") 
            #     while not self.goal(self.wm_mid, 'keep'):
            #         self.get_logger().warn("  >> 실패 → 재시도 중")

        # while not self.goal(self.ff, 'keep'):
        #     self.get_logger().warn("  >> 실패 → 재시도 중")

    
    def execute_sequence(self):


        self.get_logger().info("Step 0: 문 여는 중")
        self.open_door()

        while rclpy.ok():

            while not self.goal(self.wm_mid, 'keep'): # 사진
                    self.get_logger().warn("  >> 실패 → 재시도 중")
            while not self.goal(self.ba_init, 'keep'): # 사진
                    self.get_logger().warn("  >> 실패 → 재시도 중")
            while not self.goal(self.wm_init, 'keep'): # 사진
                    self.get_logger().warn("  >> 실패 → 재시도 중")
            while not self.goal(self.ba_end, 'keep'): # 사진
                    self.get_logger().warn("  >> 실패 → 재시도 중")
            while not self.goal(self.f, 'keep'): # 사진
                    self.get_logger().warn("  >> 실패 → 재시도 중")
            while not self.goal([-0.9599, 1.6755, -0.8378, -0.8901, -1.2217, 1.4312, 0.0, 0.0], 'keep'): # 사진
                    self.get_logger().warn("  >> 실패 → 재시도 중")
            while not self.goal(self.ft, 'keep'): # 사진
                    self.get_logger().warn("  >> 실패 → 재시도 중")
            while not self.goal([0.5411, 1.3788, -1.5708, 1.6232, -0.6458, -0.9774, 0.0, 0.0], 'keep'): # 사진
                    self.get_logger().warn("  >> 실패 → 재시도 중")
            while not self.goal(self.ff, 'keep'): # 사진
                    self.get_logger().warn("  >> 실패 → 재시도 중")
            while not self.goal([0.3491, 1.4661, -2.4958, -0.5585, 0.9774, 0.0698, 0.0, 0.0], 'keep'): # 사진
                    self.get_logger().warn("  >> 실패 → 재시도 중")
            while not self.goal([-1.0821, 1.5184, -0.8203, -1.0297, -1.2217, 0.6109, 0.0, 0.0], 'keep'): # 사진
                    self.get_logger().warn("  >> 실패 → 재시도 중")
            
            
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