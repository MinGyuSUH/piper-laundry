import rclpy
import time
from rclpy.node import Node
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose, PoseStamped
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
from collections import deque

########################## 비지도 클러스터링 ####################################################

# import joblib

# EPS = 1e-6

# # fcm.joblib 언피클용 최소 클래스(이 이름이 없으면 로드 에러)
# class FCMModel:
#     def __init__(self, prep=None, centers=None, m: float = 2.0) -> None:
#         self.prep = prep
#         self.centers_ = centers
#         self.m = float(m)

# def _np64(x):
#     return np.require(np.asarray(x, dtype=np.float64), dtype=np.float64, requirements=["C"])

# def _extract_imputer_scaler(prep):
#     """ColumnTransformer('num' 파이프라인)에서 imputer/scaler 파라미터 추출"""
#     num = prep.named_transformers_["num"]
#     imputer = num.named_steps["impute"]
#     scaler  = num.named_steps["scale"]
#     imp = _np64(imputer.statistics_)     # (3,)
#     mean = _np64(scaler.mean_)           # (3,)
#     scale = _np64(scaler.scale_)         # (3,)
#     scale[scale == 0.0] = 1.0
#     return imp, mean, scale

# def _preprocess_mb(mb_vec3, imp_stats, mean, scale):
#     """mb(3,) → impute+standardize → (1,3) float64"""
#     X = _np64(mb_vec3).reshape(1, 3)
#     if np.isnan(X).any():
#         X = np.where(np.isnan(X), imp_stats.reshape(1, 3), X)
#     Xs = (X - mean.reshape(1, 3)) / scale.reshape(1, 3)
#     return _np64(Xs)

# def _fcm_membership(Xs, centers, m):
#     """Xs: (n,3), centers: (C,3) → U: (C,n)"""
#     Xs = _np64(Xs); centers = _np64(centers)
#     d = np.linalg.norm(Xs[:, None, :] - centers[None, :, :], axis=2)
#     d = np.maximum(d, EPS)
#     power = 2.0 / (m - 1.0)
#     U = 1.0 / ((d[:, :, None] / d[:, None, :]) ** power).sum(axis=2)
#     return U.T

##############################################################################################


class ControlTower(Node):

    def __init__(self):
        super().__init__('control_tower_node')

        self.ee_pose = None
        self.prev_x = None
        self.prev_y = None
        self.prev_z = None
        self.cum_x = [0.0] * 16
        self.cum_y = [0.0] * 16
        self.cum_z = [0.0] * 16

        self.z_aligned = False
        self.contacted = False

        self.z_aligned_k = False
        self.contacted_k = False
        self.contacted_kk = False
        self.z_aligned_f = False
        self.contacted_f = False
        self.contacted_ff = False

        self.norm = None
        self.theta = None

########################## 비지도 클러스터링 ####################################################

        # # === KMeans/FCM 모델 로드 및 전처리 파라미터 준비 ===
        # self.kmeans_available = False
        # self.fcm_available = False

        # self.kmeans = None
        # self.fcm_centers = None
        # self.fcm_m = 2.0

        # self.imp_stats = None
        # self.scaler_mean = None
        # self.scaler_scale = None

        # self.minmax_path = "/home/dyros/mcy_ws/piper-mou/src/piper_ros/src/piper_moveit/piper_with_gripper_moveit/src/taxel_minmax_norm.npy"
        # self.gmin, self.grng = self._load_minmax(self.minmax_path)
        # self.get_logger().info(f"MinMax loaded: {self.minmax_path}")

        # # 1) kmeans.joblib (sklearn Pipeline: prep + kmeans)
        # try:
        #     km_art = joblib.load("/home/dyros/mcy_ws/piper-mou/src/piper_ros/src/piper_moveit/piper_with_gripper_moveit/src/kmeans_norm.joblib")
        #     if hasattr(km_art, "named_steps") and "prep" in km_art.named_steps and "kmeans" in km_art.named_steps:
        #         prep = km_art.named_steps["prep"]
        #         self.kmeans = km_art.named_steps["kmeans"]
        #         self.imp_stats, self.scaler_mean, self.scaler_scale = _extract_imputer_scaler(prep)
        #         self.kmeans_available = True
        #         self.get_logger().info("Loaded kmeans.joblib")
        # except Exception as e:
        #     self.get_logger().warn(f"kmeans.joblib load failed: {e}")

        # # 2) fcm.joblib (dict 또는 FCMModel)
        # try:
        #     fc_art = joblib.load("/home/dyros/mcy_ws/piper-mou/src/piper_ros/src/piper_moveit/piper_with_gripper_moveit/src/fcm_norm.joblib")
        #     if isinstance(fc_art, dict) and fc_art.get("type", "") == "fcm":
        #         prep = fc_art["prep"]
        #         self.imp_stats, self.scaler_mean, self.scaler_scale = _extract_imputer_scaler(prep)
        #         self.fcm_centers = _np64(fc_art["centers"])  # (C,3) 표준화 공간
        #         self.fcm_m = float(fc_art.get("m", 2.0))
        #         self.fcm_available = True
        #         self.get_logger().info("Loaded fcm.joblib (dict)")
        #     elif isinstance(fc_art, FCMModel):
        #         prep = fc_art.prep
        #         self.imp_stats, self.scaler_mean, self.scaler_scale = _extract_imputer_scaler(prep)
        #         self.fcm_centers = _np64(fc_art.centers_)
        #         self.fcm_m = float(fc_art.m)
        #         self.fcm_available = True
        #         self.get_logger().info("Loaded fcm.joblib (FCMModel)")
        # except Exception as e:
        #     self.get_logger().warn(f"fcm.joblib load failed: {e}")

#######################################################################################################################################################


        self.subscription = self.create_subscription(
            SensStream,
            '/xServTopic',
            self.sensor_callback,
            10
        )
        self.create_subscription(Pose, '/end_pose', self.ee_pose_callback, 10)

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
        self.f = [1.574251224, 0.336651756, -1.159398016, -0.185900708, 1.287576528, -0.1, 0.0, 0.0]

        self.ff = [1.5533, 0.1396, -0.4014, 0.0349, 0.3665, 0.1745, 0.0, 0.0]
        self.h = [-1.7977, 1.5184, -2.1468, 0.6109, 0.4887, -0.4014, 0, 0]

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

    def send_pose(self, pose: PoseStamped):
        goal_msg = PoseGoal.Goal()
        goal_msg.target_pose = pose

        # --- 여기가 수정된 부분 ---
        # .action 파일에 정의된 이름과 정확히 일치시켜야 합니다.
        goal_msg.mode = 1 # rpy
        goal_msg.roll_tol_rad = 6.28            # 'tol_roll'/'tol_pitch' -> 'rp_tol_rad'
        goal_msg.pitch_tol_rad = 1.57/2
        goal_msg.yaw_tol_rad = 1.57/2          # 'tol_yaw' -> 'yaw_tol_rad'
        
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
        goal_msg = PoseGoal.Goal()
        goal_msg.target_pose = pose

        goal_msg.mode = 9

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


    def reset_sensor_data(self):
        self.prev_x, self.prev_y, self.prev_z = None, None, None
        self.cum_x, self.cum_y, self.cum_z = [0.0] * 16, [0.0] * 16, [0.0] * 16
        self.z_aligned, self.contacted = False, False

        self.z_aligned_k, self.contacted_k = False, False
        self.contacted_ff, self.contacted_kk = False, False
        self.z_aligned_f, self.contacted_f = False, False

        self.get_logger().info("센서 기준값 및 상태 초기화 완료.")

    def sensor_callback(self, msg):
        try:
            x_vals = [t.x for t in msg.sensors[0].taxels]
            y_vals = [t.y for t in msg.sensors[0].taxels]
            z_vals = [t.z for t in msg.sensors[0].taxels]

            x_vals, y_vals, z_vals = self._normalize_taxels(x_vals, y_vals, z_vals)  # 모두 float64

            if self.prev_x is None:
                self.prev_x, self.prev_y, self.prev_z = x_vals, y_vals, z_vals
                # 첫 프레임엔 예측/표시 안 함
                self.z_aligned = self.contacted = False
                self.z_aligned_k = self.contacted_k = False
                self.z_aligned_f = self.contacted_f = False
                self.contacted_ff = self.contacted_kk = False
                # print("처음입니다.")
                return

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

            self.norm = norm
            self.theta = theta_deg

            if norm < 0.55:
                self.z_aligned = False
            else:
                if theta_deg < 70.0:
                    self.z_aligned = True
                    
                elif theta_deg <= 110.0:
                    self.contacted = True
                    self.z_aligned = False
                else:
                    self.z_aligned = False

########################## 비지도 클러스터링 ####################################################

            # # --- KMeans/FCM --- COLOR_MAP = {0: "green", 1: "red", 2: "gray"}
            # LABEL_STATE_MAP_k = {0: (False, True), 1: (True, False), 2: (False, False)}    
            # LABEL_STATE_MAP_f = {0: (False, True), 1: (False, False), 2: (True, False)}          

            # # --- KMeans ---
            # if getattr(self, "kmeans_available", False) and self.kmeans is not None and self.imp_stats is not None:
            #     try:
            #         Xs_k = _preprocess_mb(mean_vec_biased, self.imp_stats, self.scaler_mean, self.scaler_scale)
            #         k_label = int(self.kmeans.predict(Xs_k)[0])
            #         za_k, ct_k = LABEL_STATE_MAP_k.get(k_label, (False, False))
            #         self.z_aligned_k = bool(za_k)
            #         self.contacted_k = bool(ct_k)
            #         if self.contacted_k == True:
            #             # print("kkkkkkkkkkkkkkkkkk")
            #             self.contacted_kk = True

            #     except Exception as e:
            #         self.get_logger().warn(f"KMeans predict skipped: {e}")
            #         # 이전 값 유지

            # # --- FCM ---
            # if getattr(self, "fcm_available", False) and self.fcm_centers is not None and self.imp_stats is not None:
            #     try:
            #         Xs_f = _preprocess_mb(mean_vec_biased, self.imp_stats, self.scaler_mean, self.scaler_scale)
            #         U = _fcm_membership(Xs_f, self.fcm_centers, self.fcm_m)  # (C,1)
            #         f_label = int(np.argmax(U[:, 0]))
            #         za_f, ct_f = LABEL_STATE_MAP_f.get(f_label, (False, False))
            #         self.z_aligned_f = bool(za_f)
            #         self.contacted_f = bool(ct_f)
            #         if self.contacted_f == True:
            #             # print("ffffffffffffffff")
            #             self.contacted_ff = True

            #     except Exception as e:
            #         self.get_logger().warn(f"FCM predict skipped: {e}")
            #         # 이전 값 유지

   #############################################################################################################################             

            # prev 갱신 (항상 마지막에)
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


    def target_get(self, mode="wm"): ### basket, wm, mid
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
            pose_stamped.header.frame_id = 'base_link'
            pose_stamped.header.stamp = Clock().now().to_msg()

            return pose_stamped
        else:

            return None

    def execute_sequence(self):
        self.get_logger().info("Step 0: 문 여는 중")
        self.open_door()
        self.open_gripper()

        while rclpy.ok():
            while True:
                self.get_logger().info("Step 1: 이니셜 포즈(wm_init)로 이동 중")
                while not self.goal(self.wm_init, 'keep'):
                    self.get_logger().warn("  >> 실패 → 재시도 중")
                self.reset_sensor_data()

                self.get_logger().info("Step 2: 잡기 시도 중...")

                target_wm = self.target_get(mode="wm")  # 감지

                for i in range(5):
                    if self.send_pose(target_wm): ## 모드 1
                        break
                    self.get_logger().warn(f"  >> pose 명령 실패 ({i+1}/5) → 재시도 중")
                else:  
                    self.get_logger().warn("  >> pose 명령 5회 연속 실패 → 새 target 요청")
                    continue  # while True 처음으로

                for i in range(5):
                    if self.move_forward(target_wm): ## 8
                        break
                    self.get_logger().warn(f"  >> pose 명령 실패 ({i+1}/5) → 재시도 중")
                else:  
                    self.get_logger().warn("  >> pose 명령 5회 연속 실패 → 새 target 요청")
                    continue  # while True 처음으로

                self.close_gripper()

                while not self.goal(self.wm_init, 'keep'):
                    self.get_logger().warn("  >> 실패 → 재시도 중")

                self.get_logger().info("🔍 Z-축 정렬 감지")
                # self.z_aligned = True ####

                if self.z_aligned:
                    self.get_logger().info("✅ 잡기 성공")
                    break
                else:
                    self.get_logger().warn("❌ 잡기 실패 →  재시도")
                    while not self.open_gripper():
                        self.get_logger().warn("  >> 그리퍼 오픈 실패 → 재시도 중") 

            self.get_logger().info("Step 3: 엔드 포즈(ba_end)로 이동 중")

            while not self.goal(self.ba_init, 'keep'):
                self.get_logger().warn("  >> 실패 → 재시도 중")
            while not self.goal(self.ba_end, 'keep'):
                self.get_logger().warn("  >> 실패 → 재시도 중")

            # while not self.goal(self.h, 'keep'):
            #     self.get_logger().warn("  >> 실패 → 재시도 중")

            self.open_gripper()
            while not self.goal(self.ba_init, 'keep'):
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
