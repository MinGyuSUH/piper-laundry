#include <memory>
#include <thread>
#include <string>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <control_msgs/action/follow_joint_trajectory.hpp>

// 타입 별칭
using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
using GoalHandleFJT = rclcpp_action::ServerGoalHandle<FollowJointTrajectory>;
using namespace std::placeholders;

// MoveGroup과 액션 서버를 묶는 작은 헬퍼 클래스
class MoveItGroupActionServer
{
public:
    MoveItGroupActionServer(rclcpp::Node::SharedPtr node, const std::string& group_name, const std::string& action_name)
    : node_(node), group_name_(group_name), logger_(node->get_logger())
    {
        move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node, group_name);
        
        if (move_group_->getJointNames().empty()) {
            RCLCPP_ERROR(logger_, "MoveGroup '%s' 초기화 실패.", group_name.c_str());
            return;
        }

        // move_group_->setGoalPositionTolerance(0.1);    // 위치 허용오차 10cm
        // move_group_->setGoalOrientationTolerance(0.5); // 자세 허용오차 약 30도
        // move_group_->setPlanningTime(5.0);              // 플래닝 시간 늘리기
        // move_group_->setMaxVelocityScalingFactor(0.5);  // 속도 줄이면 성공률↑
        // move_group_->setMaxAccelerationScalingFactor(0.5);

        action_server_ = rclcpp_action::create_server<FollowJointTrajectory>(
            node_,
            action_name,
            std::bind(&MoveItGroupActionServer::handle_goal, this, _1, _2),
            std::bind(&MoveItGroupActionServer::handle_cancel, this, _1),
            std::bind(&MoveItGroupActionServer::handle_accepted, this, _1)
        );
        RCLCPP_INFO(logger_, "'%s' 액션 서버가 '%s' 그룹을 위해 준비되었습니다.", action_name.c_str(), group_name.c_str());
    }

private:
    rclcpp::Node::SharedPtr node_;
    std::string group_name_;
    rclcpp::Logger logger_;
    moveit::planning_interface::MoveGroupInterfacePtr move_group_;
    rclcpp_action::Server<FollowJointTrajectory>::SharedPtr action_server_;
    
    // ... (핸들러 함수들은 이전과 거의 동일)
    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID &, std::shared_ptr<const FollowJointTrajectory::Goal> goal)
    {
        RCLCPP_INFO(logger_, "'%s' 액션: 목표 수신 (%ld points)", group_name_.c_str(), goal->trajectory.points.size());
        if (goal->trajectory.joint_names.empty() || goal->trajectory.points.empty()) {
            RCLCPP_ERROR(logger_, "'%s' 액션: 목표 거절 - 조인트/포인트 비어있음.", group_name_.c_str());
            return rclcpp_action::GoalResponse::REJECT;
        }
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleFJT> &)
    {
        RCLCPP_INFO(logger_, "'%s' 액션: 취소 요청 수신. 동작 중지.", group_name_.c_str());
        move_group_->stop();
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<GoalHandleFJT> &goal_handle)
    {
        std::thread{std::bind(&MoveItGroupActionServer::execute, this, _1), goal_handle}.detach();
    }

    void execute(const std::shared_ptr<GoalHandleFJT> &goal_handle)
    {
        const auto goal = goal_handle->get_goal();
        auto result = std::make_shared<FollowJointTrajectory::Result>();
        const auto &target_point = goal->trajectory.points.back();
        
        move_group_->setJointValueTarget(target_point.positions);
        
        moveit::planning_interface::MoveGroupInterface::Plan my_plan;
        if (move_group_->plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS) {
            if (move_group_->execute(my_plan) == moveit::core::MoveItErrorCode::SUCCESS) {
                result->error_code = FollowJointTrajectory::Result::SUCCESSFUL;
                goal_handle->succeed(result);
            } else {
                result->error_code = FollowJointTrajectory::Result::PATH_TOLERANCE_VIOLATED;
                goal_handle->abort(result);
            }
        } else {
            result->error_code = FollowJointTrajectory::Result::PATH_TOLERANCE_VIOLATED;
            goal_handle->abort(result);
        }
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    auto node = std::make_shared<rclcpp::Node>("moveit_action_server_node", node_options);

    // 팔(arm)과 그리퍼(gripper)를 위한 액션 서버를 각각 생성
    auto arm_action_server = MoveItGroupActionServer(node, "arm", "/moveit_action/arm_controller/follow_joint_trajectory");
    auto gripper_action_server = MoveItGroupActionServer(node, "gripper", "/moveit_action/gripper_controller/follow_joint_trajectory");
    
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    
    rclcpp::shutdown();
    return 0;
}