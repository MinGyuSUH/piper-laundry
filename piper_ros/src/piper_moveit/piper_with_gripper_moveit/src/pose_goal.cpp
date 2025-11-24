#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/msg/constraints.hpp>
#include <moveit_msgs/msg/orientation_constraint.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <thread>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <geometry_msgs/msg/twist_stamped.hpp>

#include <trajectory_msgs/msg/joint_trajectory.hpp> // CLIK
#include <Eigen/Dense> // CLIK
#include "std_msgs/msg/float64_multi_array.hpp" // CLIK
#include "piper_with_gripper_moveit/action/pose_goal.hpp"

#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <moveit_msgs/srv/get_position_ik.hpp>                      // /compute_ik
#include <moveit/robot_state/conversions.h>                         // robotState <-> msg 변환
#include <moveit/planning_scene/planning_scene.h>                   // PlanningScene (self-collision check)

#include <moveit/kinematic_constraints/utils.h>

// includes 상단
#include <std_msgs/msg/bool.hpp>
#include <atomic>
#include <chrono>

using namespace std::chrono_literals;

// class PoseOrCartesianServer 멤버에 추가
std::atomic<bool> stop_requested_{false};
rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr coll_sub_;

using ActionT    = piper_with_gripper_moveit::action::PoseGoal;
using GoalHandle = rclcpp_action::ServerGoalHandle<ActionT>;


class PoseOrCartesianServer : public rclcpp::Node {
public:
  PoseOrCartesianServer(const rclcpp::NodeOptions& opt)
  : Node("pose_goal_server", opt), logger_(get_logger())
  {
    move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
      std::shared_ptr<rclcpp::Node>(this), "arm");

    base_frame_ = "base_link";
    ee_link_    = "tcp";
    EEE_ = "EEE";
    DEEP_ = "DEEP";

    move_group_->setPoseReferenceFrame(base_frame_);
    move_group_->setEndEffectorLink(ee_link_);

    velocity_publisher_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
        "/arm_controller/joint_trajectory", 10); // CLIK

    server_ = rclcpp_action::create_server<ActionT>(
      this, "pose_goal",
      std::bind(&PoseOrCartesianServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&PoseOrCartesianServer::handle_cancel, this, std::placeholders::_1),
      std::bind(&PoseOrCartesianServer::handle_accept, this, std::placeholders::_1)
    );

    RCLCPP_INFO(logger_, "Action server ready: /pose_goal");
     // ========== CHANGED: Jacobian 출력을 위한 스레드 생성 ==========
    // 생성자가 다른 작업을 막지 않도록 별도의 스레드에서 실행합니다.
    std::thread([this]() {
        RCLCPP_INFO(logger_, "Waiting for current robot state to print initial Jacobian...");
        
        moveit::core::RobotStatePtr current_state;
        int attempts = 0;
        // 최대 10초간, 또는 유효한 로봇 상태를 받을 때까지 기다립니다.
        while (attempts < 100) {
            current_state = move_group_->getCurrentState(1.0); // 1초간 상태 대기
            if (current_state) {
                break; // 성공적으로 상태를 받으면 루프 탈출
            }
            RCLCPP_WARN(logger_, "Could not get current robot state, retrying...");
            attempts++;
        }

        if (!current_state) {
            RCLCPP_ERROR(logger_, "Failed to get robot state after multiple attempts. Cannot print Jacobian.");
            return;
        }
        
        RCLCPP_INFO(logger_, "Successfully received robot state. Calculating Jacobian...");

        const auto* joint_model_group = move_group_->getRobotModel()->getJointModelGroup(move_group_->getName());
        
        Eigen::MatrixXd jacobian;
        current_state->getJacobian(joint_model_group, 
                                   current_state->getLinkModel(ee_link_),
                                   Eigen::Vector3d(0,0,0),
                                   jacobian);

        // 터미널에서 쉽게 찾을 수 있도록 눈에 띄게 출력
        std::stringstream ss;
        ss << "\n\n"
           << "========================================================\n"
           << " Jacobian Matrix at Initial State (for link '" << ee_link_ << "')\n"
           << "========================================================\n"
           << jacobian
           << "\n========================================================\n";
        
        RCLCPP_INFO_STREAM(this->get_logger(), ss.str());

    }).detach(); // 스레드를 분리하여 백그라운드에서 실행
  }

private:
  rclcpp::Logger logger_;
  moveit::planning_interface::MoveGroupInterfacePtr move_group_;
  rclcpp_action::Server<ActionT>::SharedPtr server_;
  std::string base_frame_, ee_link_, EEE_, DEEP_;
  Eigen::VectorXd previous_error_ = Eigen::VectorXd::Zero(6);

  // rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr velocity_publisher_; // CLIK
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr velocity_publisher_; // CLIK

  // -------- 액션 콜백 --------
  rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID&,
                                          std::shared_ptr<const ActionT::Goal> goal) {
    if (goal->mode < 0 || goal->mode > 9) return rclcpp_action::GoalResponse::REJECT;
    // 모드 늘리면 여기 수정해야함 //
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandle>&) {
    move_group_->stop();
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accept(const std::shared_ptr<GoalHandle>& gh) {
    std::thread([this, gh]{ execute(gh); }).detach();
  }

  void execute(const std::shared_ptr<GoalHandle>& gh) {
    auto goal   = gh->get_goal();
    auto result = std::make_shared<ActionT::Result>();
    bool ok = false;
    std::string msg;

    //////////////////////////////////////////// POSE  ///////////////////////////////////////////////////////

    if (goal->mode == 0) {               

      move_group_->setPlanningPipelineId("");  // 빈 문자열 -> 기본 파이프라인 사용
      move_group_->setPlannerId("");           // 기본 플래너(설정/rviz) 사용

      move_group_->clearPoseTargets();        
      move_group_->clearPathConstraints();    
      move_group_->setStartStateToCurrentState();

      move_group_->setGoalPositionTolerance(0.01);     // 1 cm
      move_group_->setGoalOrientationTolerance(0.1);  // ~0.57°

      move_group_->setPoseTarget(goal->target_pose.pose, ee_link_);
      // move_group_->setPoseTarget(goal->target_pose.pose, EEE_);
      ok = plan_and_exec();  //debug
      msg = ok ? "SUCCESS(M0)" : "FAILED(M0)"; //debug
    } 

    ////////////////////////////////////  POSITION + ORIENTATION CONSTRAIN  ///////////////////////////////////////////////////

    else if (goal->mode == 1) {        

      move_group_->clearPoseTargets();        
      move_group_->clearPathConstraints();    
      move_group_->setStartStateToCurrentState();

      move_group_->setPoseReferenceFrame(base_frame_);  
      move_group_->setEndEffectorLink(ee_link_);      // OR EEE_

      move_group_->setPlanningPipelineId("");  
      move_group_->setPlannerId("");           

      // 🔎 디버깅 출력
      RCLCPP_INFO(this->get_logger(),
                  "[DEBUG] pipeline=%s, planner=%s",
                  move_group_->getPlanningPipelineId().c_str(),
                  move_group_->getPlannerId().c_str());

      auto cs = make_rpy_constraint(goal->roll_tol_rad, goal->pitch_tol_rad, goal->yaw_tol_rad);
      move_group_->setPathConstraints(cs);

      move_group_->setPositionTarget(
        goal->target_pose.pose.position.x,
        goal->target_pose.pose.position.y,
        goal->target_pose.pose.position.z,
        ee_link_); // OR EEE_
        
      ok = plan_and_exec(); // debug
      msg = ok ? "SUCCESS(M1)" : "FAILED(M1)"; //debug

      move_group_->clearPathConstraints();
    } 
    
    ////////////////////////////////////  pure IK solver like DEEP  ///////////////////////////////////////////////////////

    else if (goal->mode == 6) {        // pure IK + self-collision check + JTC
      RCLCPP_WARN(logger_, "[MODE 6] IK only, no planning (self-collision only)");

      // 1️⃣ 목표 Pose 받기
      geometry_msgs::msg::PoseStamped target_pose = goal->target_pose;

      // 2️⃣ compute_ik 서비스 클라이언트 준비
      auto ik_client = this->create_client<moveit_msgs::srv::GetPositionIK>("/compute_ik");
      if (!ik_client->wait_for_service(std::chrono::seconds(2))) {
          RCLCPP_ERROR(logger_, "IK service not available");
          ok = false;
          msg = "IK service unavailable";
      } else {
          // 3️⃣ 요청 구성
          auto req = std::make_shared<moveit_msgs::srv::GetPositionIK::Request>();

          moveit::core::RobotState cur = *move_group_->getCurrentState();
          moveit::core::robotStateToRobotStateMsg(cur, req->ik_request.robot_state);

          req->ik_request.group_name       = move_group_->getName();
          req->ik_request.ik_link_name     = DEEP_; // 있어도 되고 없어도 OK
          req->ik_request.avoid_collisions = false;                              // ✅ 환경/자가충돌 무시하고 IK 해
          req->ik_request.pose_stamped     = goal->target_pose;
          req->ik_request.timeout          = rclcpp::Duration::from_seconds(0.2);

          auto resp = ik_client->async_send_request(req).get();
          if (!resp || resp->error_code.val != resp->error_code.SUCCESS) {
              RCLCPP_ERROR(logger_, "IK failed, code=%d", resp ? resp->error_code.val : -999);
              ok = false;
              msg = "IK failed";
          } else {
              // 4️⃣ 자가충돌만 검사
              const auto& model = move_group_->getRobotModel();
              planning_scene::PlanningScene ps(model);
              moveit::core::RobotState sol(model);
              moveit::core::robotStateMsgToRobotState(resp->solution, sol);
              bool self_ok = !ps.isStateColliding(sol, "arm");

              if (!self_ok) {
                  RCLCPP_ERROR(logger_, "Self-collision detected! abort.");
                  ok = false;
                  msg = "Self-collision";
              } else {
                  using FTJ = control_msgs::action::FollowJointTrajectory;

                  auto jtc_client = rclcpp_action::create_client<FTJ>(
                      this->shared_from_this(), "/arm_controller/follow_joint_trajectory");

                  if (!jtc_client->wait_for_action_server(std::chrono::seconds(2))) {
                    RCLCPP_ERROR(logger_, "JTC server not available");
                    ok = false;
                    msg = "JTC unavailable";
                  } else {
                    // 1) IK 해에서 관절값 추출
                    const auto* jmg = move_group_->getRobotModel()->getJointModelGroup(move_group_->getName());
                    std::vector<double> q;
                    sol.copyJointGroupPositions(jmg, q);

                    // 2) 목표 한 점짜리 JT 구성
                    FTJ::Goal goal_msg;
                    goal_msg.trajectory.joint_names = move_group_->getJointNames();

                    trajectory_msgs::msg::JointTrajectoryPoint p;
                    p.positions = q;

                    // 권장: builtin_interfaces::msg::Duration 로 세팅
                    builtin_interfaces::msg::Duration tfs;
                    tfs.sec = 3;
                    tfs.nanosec = 0;
                    p.time_from_start = tfs;

                    goal_msg.trajectory.points.push_back(p);

                    // 3) 전송 및 응답
                    auto gh = jtc_client->async_send_goal(goal_msg).get();
                    if (!gh) {  // ← nullptr이면 거부
                      RCLCPP_ERROR(logger_, "JTC goal rejected (nullptr handle)");
                      ok = false;
                      msg = "JTC rejected";
                    } else {
                      // 결과 받기: client->async_get_result(handle)
                      auto wrapped = jtc_client->async_get_result(gh).get();
                      ok = (wrapped.code == rclcpp_action::ResultCode::SUCCEEDED) &&
                          wrapped.result &&
                          (wrapped.result->error_code == FTJ::Result::SUCCESSFUL);
                      msg = ok ? "SUCCESS(IK→JTC)" : "FAILED(JTC exec)";
                    }
                  }
              }
          }
      }
    }

    ///////////////////////////////////////////// LIN tcp ////////////////////////////////////////////////////////////////////////

    else if (goal->mode == 8) {        // now pose + none err

      move_group_->clearPoseTargets();        
      move_group_->clearPathConstraints();    
      move_group_->setStartStateToCurrentState();

      move_group_->setPlanningPipelineId("pilz_industrial_motion_planner");
      move_group_->setPlannerId("LIN");  // LIN / PTP / CIRC

      move_group_->setGoalPositionTolerance(0.01);     // 1 cm
      move_group_->setGoalOrientationTolerance(0.1);  // ~0.57°

      // 🔎 디버깅 출력
      RCLCPP_INFO(this->get_logger(),
                  "[DEBUG] pipeline=%s, planner=%s",
                  move_group_->getPlanningPipelineId().c_str(),
                  move_group_->getPlannerId().c_str());
            
      // 2) 시작 상태 속도 0 보장
      {
        moveit::core::RobotState start = *move_group_->getCurrentState();
        std::vector<double> zeros(start.getVariableCount(), 0.0);
        start.setVariableVelocities(zeros.data());
        // (가끔 가속도도 요구되면)
        start.setVariableAccelerations(zeros.data());
        move_group_->setStartState(start);
      }

      // 3) 현재 EE 자세 유지 + 위치만 변경
      const auto cur = move_group_->getCurrentPose(ee_link_).pose;
      geometry_msgs::msg::Pose target = cur;
      target.position = goal->target_pose.pose.position;
      // target.position.y = target.position.y + 0.05 ;

      // 4) 제약(Constraints) 직접 생성 (frame/링크 명확)
      const std::string ref_frame = move_group_->getPlanningFrame(); // 보통 base/world
      // ref_frame = base_frame_;
      move_group_->setPoseReferenceFrame(base_frame_); //
      move_group_->setEndEffectorLink(ee_link_); //    ee_link_    = "tcp";


      geometry_msgs::msg::PoseStamped ps;
      ps.header.frame_id = base_frame_; //
      ps.header.stamp = this->now();
      ps.pose = target;

      move_group_->setPoseTarget(ps, ee_link_); 

      ok = plan_and_exec(); //debug
      msg = ok ? "SUCCESS(lin)" : "FAILED(lin)"; //debug

      move_group_->clearPathConstraints();

    }

    ///////////////////////////////////////////// LIN DEEP ////////////////////////////////////////////////////////////////////////

    else if (goal->mode == 9) {        // now pose + none err +++ more deep

      move_group_->clearPoseTargets();        
      move_group_->clearPathConstraints();    
      move_group_->setStartStateToCurrentState();

      move_group_->setPlanningPipelineId("pilz_industrial_motion_planner");
      move_group_->setPlannerId("LIN");  // LIN / PTP / CIRC

      move_group_->setGoalPositionTolerance(0.01);     // 1 cm
      move_group_->setGoalOrientationTolerance(0.1);  // ~0.57°

      // 🔎 디버깅 출력
      RCLCPP_INFO(this->get_logger(),
                  "[DEBUG] pipeline=%s, planner=%s",
                  move_group_->getPlanningPipelineId().c_str(),
                  move_group_->getPlannerId().c_str());
            
      // 2) 시작 상태 속도 0 보장
      {
        moveit::core::RobotState start = *move_group_->getCurrentState();
        std::vector<double> zeros(start.getVariableCount(), 0.0);
        start.setVariableVelocities(zeros.data());
        // (가끔 가속도도 요구되면)
        start.setVariableAccelerations(zeros.data());
        move_group_->setStartState(start);
      }

      // 3) 현재 EE 자세 유지 + 위치만 변경
      const auto cur = move_group_->getCurrentPose(DEEP_).pose;
      geometry_msgs::msg::Pose target = cur;
      target.position = goal->target_pose.pose.position;
      // target.position.y = target.position.y + 0.05 ;

      // 4) 제약(Constraints) 직접 생성 (frame/링크 명확)
      const std::string ref_frame = move_group_->getPlanningFrame(); // 보통 base/world
      // ref_frame = base_frame_;
      move_group_->setPoseReferenceFrame(base_frame_); //
      move_group_->setEndEffectorLink(DEEP_); //    DEEP_    = "DEEP";


      geometry_msgs::msg::PoseStamped ps;
      ps.header.frame_id = base_frame_; //
      ps.header.stamp = this->now();
      ps.pose = target;

      move_group_->setPoseTarget(ps, DEEP_); 

      ok = plan_and_exec(); //debug
      msg = ok ? "SUCCESS(lin)" : "FAILED(lin)"; //debug

      move_group_->clearPathConstraints();

    }

 
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //  ZZAM TONG  ////  ZZAM TONG  ////  ZZAM TONG  ////  ZZAM TONG  ////  ZZAM TONG  ////  ZZAM TONG  ////  ZZAM TONG  ////  ZZAM TONG  
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    else if (goal->mode == 13) {

      move_group_->clearPoseTargets();        
      move_group_->clearPathConstraints();    
      move_group_->setStartStateToCurrentState();

      move_group_->setPlanningPipelineId("pilz_industrial_motion_planner");
      move_group_->setPlannerId("LIN");  // LIN / PTP / CIRC

      // 🔎 디버깅 출력
      RCLCPP_INFO(this->get_logger(),
                  "[DEBUG] pipeline=%s, planner=%s",
                  move_group_->getPlanningPipelineId().c_str(),
                  move_group_->getPlannerId().c_str());

      // //////////////// 어떤 조인트가 문제인지 확인용 //
      // {
      //   const auto cur_state_ptr = move_group_->getCurrentState();
      //   if (!cur_state_ptr) {
      //     RCLCPP_ERROR(this->get_logger(), "[START-BOUNDS] current state is nullptr");
      //   } else {
      //     moveit::core::RobotState start = *cur_state_ptr;
      //     // 로봇 전체의 모든 변수 확인
      //     const auto& vars = start.getVariableNames();
      //     for (const auto& var : vars) {
      //       double v = start.getVariablePosition(var);
      //       const auto& b = start.getRobotModel()->getVariableBounds(var);

      //       if (b.position_bounded_) {
      //         if (v < b.min_position_ || v > b.max_position_) {
      //           RCLCPP_WARN(this->get_logger(),
      //                       "[START-BOUNDS] %s = %.6f (min=%.6f, max=%.6f)",
      //                       var.c_str(), v, b.min_position_, b.max_position_);
      //         }
      //       }
      //     }
      //   }
      // }

      /////////////////////////////// 포지션만 ////
      // move_group_->setPositionTarget(
      //   goal->target_pose.pose.position.x,
      //   goal->target_pose.pose.position.y,
      //   goal->target_pose.pose.position.z,
      //   ee_link_);
      // ok = plan_and_exec();
      // msg = ok ? "SUCCESS(M3)" : "FAILED(M3)";
            

      // 2) 시작 상태 속도 0 보장
      {
        moveit::core::RobotState start = *move_group_->getCurrentState();
        std::vector<double> zeros(start.getVariableCount(), 0.0);
        start.setVariableVelocities(zeros.data());
        // (가끔 가속도도 요구되면)
        start.setVariableAccelerations(zeros.data());
        move_group_->setStartState(start);
      }

      // 3) 현재 EE 자세 유지 + 위치만 변경
      const auto cur = move_group_->getCurrentPose(ee_link_).pose;
      geometry_msgs::msg::Pose target = cur;
      target.position = goal->target_pose.pose.position;

      // 4) 제약(Constraints) 직접 생성 (frame/링크 명확)
      const std::string ref_frame = move_group_->getPlanningFrame(); // 보통 base/world
      move_group_->setPoseReferenceFrame(ref_frame);
      move_group_->setEndEffectorLink(ee_link_);

      geometry_msgs::msg::PoseStamped ps;
      ps.header.frame_id = ref_frame;
      ps.header.stamp = this->now();
      ps.pose = target;

      move_group_->setPoseTarget(ps, ee_link_);
      // ok = plan_and_exec();
      // // CLIK
      // if (ok) {
      //       msg = "SUCCESS(M3-Pilz)";
      //   } else {
      //       //  Pilz 실패 시 CLIK로 자동 전환 ===
      //       RCLCPP_WARN(logger_, "[Mode 3] Pilz 직선 경로 실패! CLIK 제어로 전환합니다.");
            
      //       ok = execute_clik_fallback(goal->target_pose.pose); // CLIK 실행
      //       msg = ok ? "SUCCESS(M3-CLIK)" : "FAILED(M3-CLIK)";
      //       //  Pilz 실패 시 CLIK로 자동 전환 ===
      //   }

      RCLCPP_WARN(logger_, "[Mode 3] Pilz 직선 경로 실패! CLIK 제어로 전환합니다.");      
      ok = execute_clik_fallback(goal->target_pose.pose); // CLIK 실행
      msg = ok ? "SUCCESS(M3-CLIK)" : "FAILED(M3-CLIK)";

      // result->success = ok;
      // result->message = msg;
      // if (ok) gh->succeed(result); else gh->abort(result);
      // CLIK


    /////////////////////////////////////// RPY_CONSTRAINT ee_link 안쪽 

    } else if (goal->mode == 14) {        

      move_group_->clearPoseTargets();        
      move_group_->clearPathConstraints();    
      move_group_->setStartStateToCurrentState();

      move_group_->setPoseReferenceFrame(base_frame_);  
      move_group_->setEndEffectorLink(ee_link_);      // EEE_

      move_group_->setPlanningPipelineId("ompl");  
      move_group_->setPlannerId("");           

      // 🔎 디버깅 출력
      RCLCPP_INFO(this->get_logger(),
                  "[DEBUG] pipeline=%s, planner=%s",
                  move_group_->getPlanningPipelineId().c_str(),
                  move_group_->getPlannerId().c_str());

      auto cs = make_rpy_constraint(goal->roll_tol_rad, goal->pitch_tol_rad, goal->yaw_tol_rad);
      move_group_->setPathConstraints(cs);

      move_group_->setPositionTarget(
        goal->target_pose.pose.position.x,
        goal->target_pose.pose.position.y,
        goal->target_pose.pose.position.z,
        ee_link_); // EEE_
        
      // ok = plan_and_exec(); // debug
      // msg = ok ? "SUCCESS(M1)" : "FAILED(M1)"; //debug

      move_group_->clearPathConstraints();

    /////////////////////////////////////// RPY EE 

    } else if (goal->mode == 15) {        

      move_group_->clearPoseTargets();        
      move_group_->clearPathConstraints();    
      move_group_->setStartStateToCurrentState();

      move_group_->setPoseReferenceFrame(base_frame_);  
      move_group_->setEndEffectorLink(EEE_);      

      move_group_->setPlanningPipelineId("ompl");  
      move_group_->setPlannerId("");           

      // 🔎 디버깅 출력
      RCLCPP_INFO(this->get_logger(),
                  "[DEBUG] pipeline=%s, planner=%s",
                  move_group_->getPlanningPipelineId().c_str(),
                  move_group_->getPlannerId().c_str());

      auto cs = make_rpy_constraint(goal->roll_tol_rad, goal->pitch_tol_rad, goal->yaw_tol_rad);
      move_group_->setPathConstraints(cs);

      move_group_->setPositionTarget(
        goal->target_pose.pose.position.x,
        goal->target_pose.pose.position.y,
        goal->target_pose.pose.position.z,
        EEE_); 
        
      // ok = plan_and_exec(); //debug
      // msg = ok ? "SUCCESS(M2)" : "FAILED(M2)"; //debug

      move_group_->clearPathConstraints();

    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


    result->success = ok;
    result->message = msg;
    if (ok) gh->succeed(result); else gh->abort(result);

  }
        
  bool plan_and_exec() {
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    auto ok = move_group_->plan(plan);
    if (ok != moveit::core::MoveItErrorCode::SUCCESS) return false;
    auto ec = move_group_->execute(plan);
    return (ec == moveit::core::MoveItErrorCode::SUCCESS);
  }

  // --- ⬇️ CLIK FALLBACK을 위해 추가된 함수 ---

  // 자세 오차(6D 벡터) 계산 함수
  Eigen::VectorXd calculate_pose_error(const geometry_msgs::msg::Pose& goal_pose, const geometry_msgs::msg::Pose& current_pose)
  {
      Eigen::VectorXd error(6);

      // 위치 오차
      error[0] = goal_pose.position.x - current_pose.position.x;
      error[1] = goal_pose.position.y - current_pose.position.y;
      error[2] = goal_pose.position.z - current_pose.position.z;

      // 방향 오차 (쿼터니언 사용)
      Eigen::Quaterniond q_goal(goal_pose.orientation.w, goal_pose.orientation.x, goal_pose.orientation.y, goal_pose.orientation.z);
      Eigen::Quaterniond q_curr(current_pose.orientation.w, current_pose.orientation.x, current_pose.orientation.y, current_pose.orientation.z);
      
      if (q_goal.dot(q_curr) < 0.0) {
          q_curr.coeffs() *= -1.0;
      }
      
      Eigen::Quaterniond q_error = q_goal * q_curr.inverse();
      Eigen::AngleAxisd angle_axis(q_error);
      error.tail(3) = angle_axis.axis() * angle_axis.angle();
      error.tail(3) = Eigen::Vector3d::Zero(); // clik orientation error -> 0
      return error;
  }

  // CLIK 제어 루프 실행 함수
  bool execute_clik_fallback(const geometry_msgs::msg::Pose& goal_pose)
  {
      rclcpp::Rate loop_rate(200); // 250Hz 제어 루프
      double dt = 1.0/200.0; // loop time range 

      const double Kp = 40; // P 제어 게인
      const double Kd = 12.0; // D gain 
      const double success_threshold = 0.002; // 목표 도달 오차 (2mm)
      previous_error_.setZero();
      std::cout << "excute lick fallback" << std::endl;

      for (int i=0; i < 1000; ++i) { // 최대 2초간 시도 (250Hz * 2s)
          if (!rclcpp::ok()) return false;

          auto current_pose = move_group_->getCurrentPose(ee_link_).pose;
          Eigen::VectorXd error = calculate_pose_error(goal_pose, current_pose);
          RCLCPP_INFO(logger_, "Error (Pos, Ori): [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f]",
              error[0], error[1], error[2], error[3], error[4], error[5]);

          Eigen::VectorXd derivative_error = (error-previous_error_) / dt;
          previous_error_ = error;

          Eigen::VectorXd desired_twist = - (Kp * error + Kd*derivative_error);

          if (error.norm() < success_threshold) {
              RCLCPP_INFO(logger_, "CLIK control successful. Goal reached.");
              trajectory_msgs::msg::JointTrajectory stop_msg;
              stop_msg.joint_names = move_group_->getJointNames();
              trajectory_msgs::msg::JointTrajectoryPoint stop_point;
              stop_point.velocities.assign(stop_msg.joint_names.size(), 0.0);
              stop_point.time_from_start = rclcpp::Duration::from_seconds(0.1);
              stop_msg.points.push_back(stop_point);
              velocity_publisher_->publish(std::move(stop_msg));
              return true;
          }

          const auto robot_model = move_group_->getRobotModel();
          auto current_state = move_group_->getCurrentState();
          const auto* joint_model_group = robot_model->getJointModelGroup(move_group_->getName());
          
          Eigen::MatrixXd jacobian;
          current_state->getJacobian(joint_model_group, 
                                     current_state->getLinkModel(ee_link_),
                                     Eigen::Vector3d(0,0,0), // reference_point_position
                                     jacobian);

          Eigen::MatrixXd jacobian_pinv = jacobian.completeOrthogonalDecomposition().pseudoInverse();
          // Eigen::VectorXd desired_twist = -Kp * error;
          Eigen::VectorXd joint_velocities = jacobian_pinv * desired_twist;

          // 관절 속도 명령 퍼블리시
          
          trajectory_msgs::msg::JointTrajectory vel_msg;
          vel_msg.joint_names = move_group_->getJointNames();
          trajectory_msgs::msg::JointTrajectoryPoint point;
          
          // *** THE FINAL, CRITICAL FIX IS HERE ***
          // Use the correct `copyJointGroupPositions` method.
          std::vector<double> joint_positions;
          current_state->copyJointGroupPositions(joint_model_group, joint_positions);
          point.positions = joint_positions;
          
          point.velocities.assign(joint_velocities.data(), joint_velocities.data() + joint_velocities.size());
          point.accelerations.assign(vel_msg.joint_names.size(), 0.0);
          point.time_from_start = rclcpp::Duration::from_seconds(0.01);
          vel_msg.points.push_back(point);
          
          velocity_publisher_->publish(std::move(vel_msg));
          loop_rate.sleep();
      }
      RCLCPP_WARN(logger_, "CLIK 제어 시간 초과.");
      return false;
  }
  // --- ⬆️ CLIK FALLBACK을 위해 추가된 함수 ---

  moveit_msgs::msg::Constraints make_rpy_constraint(double roll_tol, double pitch_tol, double yaw_tol) {
    geometry_msgs::msg::PoseStamped cur = move_group_->getCurrentPose(ee_link_);
    double r=0,p=0,y=0;
    tf2::Quaternion q_cur;
    tf2::fromMsg(cur.pose.orientation, q_cur);
    tf2::Matrix3x3(q_cur).getRPY(r,p,y);

    tf2::Quaternion q_lock; q_lock.setRPY(r,p,y);
    moveit_msgs::msg::OrientationConstraint ocm;
    ocm.header.frame_id = base_frame_;
    ocm.link_name = ee_link_;
    ocm.orientation = tf2::toMsg(q_lock);
    ocm.absolute_x_axis_tolerance = (roll_tol  > 0.0 ? roll_tol  : 1.0);
    ocm.absolute_y_axis_tolerance = (pitch_tol > 0.0 ? pitch_tol : 1.0);
    ocm.absolute_z_axis_tolerance = (yaw_tol   > 0.0 ? yaw_tol   : 1.0);
    ocm.weight = 1.0;

    moveit_msgs::msg::Constraints cs;
    cs.orientation_constraints.push_back(ocm);
    return cs;
  }
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions opt;
  opt.automatically_declare_parameters_from_overrides(true);
  auto node = std::make_shared<PoseOrCartesianServer>(opt);
  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(node);
  exec.spin();
  rclcpp::shutdown();
  return 0;
}