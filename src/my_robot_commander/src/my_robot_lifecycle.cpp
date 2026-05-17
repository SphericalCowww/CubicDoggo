#include <memory>
#include <thread>
#include <atomic>
#include <string>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "moveit/move_group_interface/move_group_interface.hpp"
#include <moveit/robot_state/robot_state.hpp>
#include <moveit/robot_model/robot_model.hpp>
#include <moveit/robot_trajectory/robot_trajectory.hpp>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit_msgs/action/execute_trajectory.hpp>
 
using rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface;
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;
using ExecuteTrajectory  = moveit_msgs::action::ExecuteTrajectory;
using namespace std::placeholders;                  // for using _1, _2
 
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//https://github.com/ros2/example_interfaces/tree/rolling/msg
#include "std_srvs/srv/set_bool.hpp"
#include "std_srvs/srv/trigger.hpp"
#include <example_interfaces/msg/string.hpp>
#include <example_interfaces/msg/float64_multi_array.hpp>    //variable size
#include "my_robot_interface/msg/my_robot_leg1_pose_target.hpp"
#include <example_interfaces/msg/bool.hpp>
using ros_string   = example_interfaces::msg::String;
using ros_array    = example_interfaces::msg::Float64MultiArray;
using custom_array = my_robot_interface::msg::MyRobotLeg1PoseTarget;
using ros_bool     = example_interfaces::msg::Bool;

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
class MyRobotLifecycleManager : public rclcpp_lifecycle::LifecycleNode {
public:
    explicit MyRobotLifecycleManager(const rclcpp::NodeOptions & options)
    : rclcpp_lifecycle::LifecycleNode("my_robot_lifecycle", options) {
        RCLCPP_INFO(get_logger(), "constructor(): %s", current_lifecycle_state_.c_str());

        callback_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
        moveit_node_ = std::make_shared<rclcpp::Node>("moveit_interface_node", this->get_namespace(), options);
        exec_action_client_ = rclcpp_action::create_client<ExecuteTrajectory>(moveit_node_, "/execute_trajectory");
        leg1_interface_ = std::make_shared<MoveGroupInterface>(moveit_node_, planning_group_);
        while (rclcpp::ok() && !leg1_interface_->startStateMonitor(1.0)) {
            RCLCPP_INFO(get_logger(), "on_configure): waiting for valid MoveGroupInterface state...");
        } 
 
        current_lifecycle_state_ = "state_initialized";
        RCLCPP_INFO(get_logger(), "constructor(): %s", current_lifecycle_state_.c_str());
    }
    CallbackReturn on_configure(const rclcpp_lifecycle::State &) override {
        RCLCPP_INFO(get_logger(), "on_configure(): %s", current_lifecycle_state_.c_str());

        leg1_interface_->setEndEffectorLink(endEffector_link_);
        leg1_interface_->setMaxVelocityScalingFactor(1.0);
        leg1_interface_->setMaxAccelerationScalingFactor(1.0);
        leg1_interface_->setGoalPositionTolerance(0.01);
        leg1_interface_->setGoalOrientationTolerance(0.1);
        leg1_interface_->setWorkspace(-1.0, -1.0, -1.0, 1.0, 1.0, 1.0); // world size
        leg1_interface_->setNumPlanningAttempts(10);
        leg1_interface_->setPlanningTime(60.0);
 
        state_service_ = this->create_service<std_srvs::srv::Trigger>(
            "get_robot_state",
            std::bind(&MyRobotLifecycleManager::handleGetState_, this, _1, _2)); 
        walk_service_ = this->create_service<std_srvs::srv::SetBool>(
            "leg1_walk_toggle", 
            std::bind(&MyRobotLifecycleManager::handleWalkRequest_, this, _1, _2), 
            rclcpp::ServicesQoS(), 
            callback_group_);

        loadCurrentRobotState_();
        RCLCPP_INFO(get_logger(), "on_configure(): current end effector (x, y, z) = (%lf, %lf, %lf)",
                    endEffector_x_, endEffector_y_, endEffector_z_);

        current_lifecycle_state_ = "state_configured";
        RCLCPP_INFO(get_logger(), "on_configure(): %s", current_lifecycle_state_.c_str());
        return CallbackReturn::SUCCESS;
    }
    CallbackReturn on_activate(const rclcpp_lifecycle::State &) override {
        RCLCPP_INFO(get_logger(), "on_activation(): %s", current_lifecycle_state_.c_str());

        auto sub_options = rclcpp::SubscriptionOptions();
        sub_options.callback_group = callback_group_;
        leg1_named_subscriber_ = create_subscription<ros_string>  ("/leg1_set_named", 10,
            std::bind(&MyRobotLifecycleManager::leg1NamedCallback_, this, _1), sub_options);
        leg1_joint_subscriber_ = create_subscription<ros_array>   ("/leg1_set_joint", 10,
            std::bind(&MyRobotLifecycleManager::leg1JointCallback_, this, _1), sub_options);
        leg1_pose_subscriber_  = create_subscription<custom_array>("/leg1_set_pose",  10,
            std::bind(&MyRobotLifecycleManager::leg1PoseCallback_,  this, _1), sub_options);

        keep_running_thread_ = true;
        is_walking_          = false; 
        walking_thread_      = std::thread(&MyRobotLifecycleManager::walkingLoop_, this);

        current_lifecycle_state_ = "state_stationary";
        RCLCPP_INFO(get_logger(), "on_activation(): %s", current_lifecycle_state_.c_str());
        return CallbackReturn::SUCCESS;
    }
    CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override {
        RCLCPP_INFO(get_logger(), "on_deactivation(): %s", current_lifecycle_state_.c_str());

        leg1_named_subscriber_.reset();
        leg1_joint_subscriber_.reset();
        leg1_pose_subscriber_.reset();

        keep_running_thread_ = false;
        is_walking_          = false;
        if (walking_thread_.joinable()) {
            walking_thread_.join();
        }

        leg1_interface_->stop();

        current_lifecycle_state_ = "state_stopped";
        RCLCPP_INFO(get_logger(), "on_deactivation(): %s", current_lifecycle_state_.c_str());
        return CallbackReturn::SUCCESS;
    }
    CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override {
        RCLCPP_INFO(get_logger(), "on_cleanup(): %s", current_lifecycle_state_.c_str());

        leg1_interface_.reset();
        leg1_joint_subscriber_.reset();
        leg1_pose_subscriber_.reset();
 
        current_lifecycle_state_ = "state_configured";
        RCLCPP_INFO(get_logger(), "on_clean(): %s", current_lifecycle_state_.c_str());
        return CallbackReturn::SUCCESS;
    }
    CallbackReturn on_shutdown(const rclcpp_lifecycle::State &) override {
        RCLCPP_INFO(get_logger(), "on_shutdown(): %s", current_lifecycle_state_.c_str());

        leg1_interface_->stop();

        current_lifecycle_state_ = "state_stopped";
        RCLCPP_INFO(get_logger(), "on_shutdown(): %s", current_lifecycle_state_.c_str());
        return CallbackReturn::SUCCESS;
    }
    rclcpp::Node::SharedPtr get_moveit_node() const { 
        return moveit_node_; 
    }
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
private:
    void leg1NamedCallback_(const ros_string::SharedPtr msg) {
        RCLCPP_INFO(get_logger(), "leg1NamedCallback(): command received");
        leg1NamedTarget_(msg->data);
    }
    void leg1JointCallback_(const ros_array::SharedPtr msg) {
        RCLCPP_INFO(get_logger(), "leg1JointCallback(): command received");
        auto numberOfJoints = leg1_interface_->getJointNames().size();
        if (msg->data.size() != numberOfJoints) {
            RCLCPP_WARN(get_logger(), "leg1JointCallback(): message empty");
            return;
        }
        leg1JointTarget_(msg->data);
    }
    void leg1PoseCallback_(const custom_array::SharedPtr msg) {
        RCLCPP_INFO(get_logger(), "leg1PoseCallback(): command received");
        leg1PoseTarget_(msg->x, msg->y, msg->z, msg->use_cartesian_path);
    }
    ///////////
    void leg1NamedTarget_(const std::string &name) {
        leg1_interface_->setNamedTarget(name);
        planAndExecute_();
    }
    void leg1JointTarget_(const std::vector<double> &joints) {
        leg1_interface_->setJointValueTarget(joints);
        planAndExecute_();
    }
    void leg1PoseTarget_(double x, double y, double z, bool use_cartesian_path=false) {
        geometry_msgs::msg::Pose target_pose = endEffector_pose_.pose;
        target_pose.position.x = x;
        target_pose.position.y = y;
        target_pose.position.z = z;
        if (use_cartesian_path == false) {
            success_ = leg1_interface_->setApproximateJointValueTarget(target_pose, endEffector_link_);
            if (success_ == false) {
                RCLCPP_ERROR(get_logger(), "leg1PoseTarget_(): failed to find IK solution for target!");
            }
            planAndExecute_();
        } else {
            moveit_msgs::msg::RobotTrajectory trajectory;
            std::vector<geometry_msgs::msg::Pose> waypoints;
            waypoints.push_back(endEffector_pose_.pose);
            waypoints.push_back(target_pose);

            double fraction = leg1_interface_->computeCartesianPath(waypoints, cartesianConstraintStepsize_, 
                                                                    trajectory);
            if (cartesianConstraintFractionThreshold_ <= fraction) {
                leg1_interface_->execute(trajectory);
            } else {
                 RCLCPP_INFO(get_logger(), "leg1PoseTarget_(): Cartesian computation fraction of "
                                                  "%lf, lower than the threshold of %lf", 
                                                  fraction, cartesianConstraintFractionThreshold_);
            }
        }
            
        loadCurrentRobotState_();
        to_target_dist = std::sqrt(std::pow(endEffector_x_ - x, 2) + 
                                   std::pow(endEffector_y_ - y, 2) +
                                   std::pow(endEffector_z_ - z, 2));
        if (to_target_dist > to_target_dist_thres) {
            RCLCPP_WARN(get_logger(), "leg1PoseTarget_(): target unreachable (likely hit a joint limit)."
                                      "Settled %f meters away.", to_target_dist);
        }
        RCLCPP_INFO(get_logger(), "leg1SetPoseTarget_(): current end effector (x, y, z) = (%lf, %lf, %lf)",
                    endEffector_x_, endEffector_y_, endEffector_z_);    
    }
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    void handleGetState_(const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
                         std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        loadCurrentRobotState_();
        response->success = true;
        response->message = "handleGetState_(): current end effector (x, y, z) = ("
                           +std::to_string(endEffector_x_) + ", "
                           +std::to_string(endEffector_y_) + ", "
                           +std::to_string(endEffector_z_) + ")";
    }
    void handleWalkRequest_(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                            std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
        is_walking_ = request->data;
        response->success = true;
        response->message = is_walking_ ? "Walking started" : "Walking stopped";
    }
    void walkingLoop__() {
        double x0 = -0.09;
        double y0 =  0.01;
        double z0 =  0.13;
        double traj_arc_rad = 0.04;

        geometry_msgs::msg::Pose pose_0 = endEffector_pose_.pose;
        pose_0.position.x = x0;
        pose_0.position.y = y0;
        pose_0.position.z = z0;

        geometry_msgs::msg::Pose pose_1 = pose_0;
        pose_1.position.y = y0 + 2*traj_arc_rad;

        geometry_msgs::msg::Pose pose_2 = pose_0;
        pose_2.position.y = y0 + traj_arc_rad;
        pose_2.position.z = z0 - traj_arc_rad;

        loadCurrentRobotState_();
        auto joint_model_group = current_robot_state_->getJointModelGroup(planning_group_);
        moveit::core::RobotState state_0(*current_robot_state_);
        moveit::core::RobotState state_1(*current_robot_state_);
        moveit::core::RobotState state_2(*current_robot_state_);
        success_ = state_0.setFromIK(joint_model_group, pose_0)
                  &state_1.setFromIK(joint_model_group, pose_1)
                  &state_2.setFromIK(joint_model_group, pose_2);
        if (success_ == false) {
            RCLCPP_ERROR(get_logger(), "walkingLoop_(): IK failed");
            return;
        }

        auto traj = std::make_shared<robot_trajectory::RobotTrajectory>(leg1_interface_->getRobotModel(), 
                                                                        planning_group_);
        traj->addSuffixWayPoint(state_0, 0.0);
        traj->addSuffixWayPoint(state_1, 0.0);
        traj->addSuffixWayPoint(state_2, 0.0);
        //traj->addSuffixWayPoint(state_0, 0.0);

        trajectory_processing::TimeOptimalTrajectoryGeneration traj_gen;
        success_ = traj_gen.computeTimeStamps(*traj, 1.0, 0.8); // frac of vel, accel from joint_limits.yaml
        if (success_ == false) {
            RCLCPP_ERROR(get_logger(), "walkingLoop_(): traj timing generation failed");
            return;
        }
        moveit_msgs::msg::RobotTrajectory traj_msg;
        traj->getRobotTrajectoryMsg(traj_msg);
                
        std::vector<double> current_positions;
        //loadCurrentRobotState_();
        while (rclcpp::ok() && is_walking_) {
            auto exec_goal = moveit_msgs::action::ExecuteTrajectory::Goal();
            exec_goal.trajectory = traj_msg;

            loadCurrentRobotState_(); 
            current_robot_state_->copyJointGroupPositions(joint_model_group, current_positions);
            if(!exec_goal.trajectory.joint_trajectory.points.empty()) {
                exec_goal.trajectory.joint_trajectory.points[0].positions = current_positions;
            }

            auto exec_goal_future = exec_action_client_->async_send_goal(exec_goal);
            if (exec_goal_future.wait_for(std::chrono::seconds(2)) != std::future_status::ready) {
                RCLCPP_ERROR(get_logger(), "walkingLoop_(): timeout waiting for goal response");
                break;
            }
            auto exec_goal_handle = exec_goal_future.get(); 
            if (exec_goal_handle) {
                auto exec_result_future = exec_action_client_->async_get_result(exec_goal_handle);
                bool step_finished = false;
                while (rclcpp::ok() && is_walking_ && !step_finished) {
                    auto exec_result_status = exec_result_future.wait_for(std::chrono::milliseconds(10));
                    if (exec_result_status == std::future_status::ready) {
                        step_finished = true;
                        RCLCPP_INFO(get_logger(), "walkingLoop_(): step completed");
                    }
                }
                if (!is_walking_ && !step_finished) {
                    RCLCPP_WARN(get_logger(), "walkingLoop_(): interrupt received; canceling trajectory.");
                    exec_action_client_->async_cancel_goal(exec_goal_handle);
                    break; 
                }
            } else {
                RCLCPP_ERROR(get_logger(), "walkingLoop_(): goal rejected by server");
                break;
            }
        }
    }
    void walkingLoop_() {
        while (keep_running_thread_ && rclcpp::ok()) {
            if (!is_walking_) {
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                continue;
            }
            loadCurrentRobotState_();
            RCLCPP_INFO(get_logger(), "walkingLoop_(): current end effector (x, y, z) = (%lf, %lf, %lf)",
                        endEffector_x_, endEffector_y_, endEffector_z_);

            double x0 = -0.09;
            double y0 =  0.01;
            double z0 =  0.13;
            double traj_arc_rad = 0.04;

            geometry_msgs::msg::Pose pose_0 = endEffector_pose_.pose;
            pose_0.position.x = x0;
            pose_0.position.y = y0;
            pose_0.position.z = z0;

            geometry_msgs::msg::Pose pose_1 = pose_0;
            pose_1.position.y = y0 + 2*traj_arc_rad;

            geometry_msgs::msg::Pose pose_2 = pose_0;
            pose_2.position.y = y0 + traj_arc_rad;
            pose_2.position.z = z0 - traj_arc_rad;

            auto joint_model_group = current_robot_state_->getJointModelGroup(planning_group_);
            moveit::core::RobotState state_0(*current_robot_state_);
            moveit::core::RobotState state_1(*current_robot_state_);
            moveit::core::RobotState state_2(*current_robot_state_);

            success_ = state_0.setFromIK(joint_model_group, pose_0) 
                      &state_1.setFromIK(joint_model_group, pose_1)
                      &state_2.setFromIK(joint_model_group, pose_2);
            if (success_ == false) {
                RCLCPP_ERROR(get_logger(), "walkingLoop_(): IK failed");
                continue; 
            }

            auto traj = std::make_shared<robot_trajectory::RobotTrajectory>(leg1_interface_->getRobotModel(), 
                                                                            planning_group_);
            traj->addSuffixWayPoint(state_0, 0.0);
            traj->addSuffixWayPoint(state_1, 0.0);
            traj->addSuffixWayPoint(state_2, 0.0);
            //traj->addSuffixWayPoint(state_0, 0.0);

            trajectory_processing::TimeOptimalTrajectoryGeneration traj_gen;
            success_ = traj_gen.computeTimeStamps(*traj, 1.0, 0.8); // frac of vel, accel from joint_limits.yaml
            if (success_ == false) {
                RCLCPP_ERROR(get_logger(), "walkingLoop_(): traj timing generation failed");
                continue;
            }
            moveit_msgs::msg::RobotTrajectory traj_msg;
            traj->getRobotTrajectoryMsg(traj_msg);

            auto exec_goal = ExecuteTrajectory::Goal();
            exec_goal.trajectory = traj_msg;

            std::vector<double> current_positions;
            leg1PoseTarget_(x0, y0, z0);
            loadCurrentRobotState_();
            current_robot_state_->copyJointGroupPositions(joint_model_group, current_positions);
            if(!exec_goal.trajectory.joint_trajectory.points.empty()) {
                exec_goal.trajectory.joint_trajectory.points[0].positions = current_positions;
            }

            auto send_goal_options = rclcpp_action::Client<ExecuteTrajectory>::SendGoalOptions();
            auto goal_handle_future = exec_action_client_->async_send_goal(exec_goal, send_goal_options);
            if (goal_handle_future.wait_for(std::chrono::seconds(2)) == std::future_status::ready) {
                auto goal_handle = goal_handle_future.get();
                if (goal_handle) {
                    auto result_future = exec_action_client_->async_get_result(goal_handle);
                    while (rclcpp::ok() && is_walking_) {
                        if (result_future.wait_for(std::chrono::milliseconds(10)) == std::future_status::ready) {
                            break;
                        }
                    }
                    if (!is_walking_) {
                        exec_action_client_->async_cancel_goal(goal_handle);
                    }
                }
            }
        }
    }
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    void planAndExecute_() {
        MoveGroupInterface::Plan plan;
        success_ = (leg1_interface_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
        if (success_) {
            leg1_interface_->execute(plan);
        } else {
            RCLCPP_WARN(get_logger(), "planAndExecute_(): planning failed");
        }
    }
    void loadCurrentRobotState_() {
        current_robot_state_ = leg1_interface_->getCurrentState(0.5);       // wait up to 0.5 seconds 
        if (!current_robot_state_) {
            RCLCPP_ERROR(get_logger(), "loadCurrentRobotState_(): timeout to fetch current robot state");
        }
        endEffector_pose_ = leg1_interface_->getCurrentPose(endEffector_link_);
        endEffector_x_ = endEffector_pose_.pose.position.x;
        endEffector_y_ = endEffector_pose_.pose.position.y;
        endEffector_z_ = endEffector_pose_.pose.position.z;
        leg1_interface_->setStartStateToCurrentState();
        //leg1_interface_->setStartState(*leg1_interface_->getCurrentState()); // faster, but risk stalling
    }
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    rclcpp::Node::SharedPtr moveit_node_;
    std::shared_ptr<MoveGroupInterface> leg1_interface_;
    std::atomic<bool> success_{false};
    std::string current_lifecycle_state_ = "state_uninitialized";
    std::string planning_group_          = "leg1";
    std::string endEffector_link_        = "calfSphere";
    moveit::core::RobotStatePtr     current_robot_state_;
    geometry_msgs::msg::PoseStamped endEffector_pose_;
    double endEffector_x_ = 0;
    double endEffector_y_ = 0;
    double endEffector_z_ = 0;

    rclcpp::CallbackGroup::SharedPtr callback_group_;
    rclcpp::Subscription<ros_string>  ::SharedPtr leg1_named_subscriber_;
    rclcpp::Subscription<ros_array>   ::SharedPtr leg1_joint_subscriber_;
    rclcpp::Subscription<custom_array>::SharedPtr leg1_pose_subscriber_;
    double cartesianConstraintStepsize_          = 0.001;     // meter
    double cartesianConstraintFractionThreshold_ = 1.0;
    double to_target_dist       = 0;
    double to_target_dist_thres = 0.01;

    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr state_service_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr walk_service_; 
    std::atomic<bool> is_walking_{false};
    std::atomic<bool> keep_running_thread_{false};
    std::thread walking_thread_;
    rclcpp_action::Client<ExecuteTrajectory>::SharedPtr exec_action_client_;
};
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);

    auto lifecycle_manager_node = std::make_shared<MyRobotLifecycleManager>(node_options);
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(lifecycle_manager_node->get_node_base_interface());
    executor.add_node(lifecycle_manager_node->get_moveit_node());
    
    std::thread executor_thread([&executor]() { executor.spin(); });
    using lifecycle_msgs::msg::Transition;
    using lifecycle_msgs::msg::State;
    lifecycle_manager_node->trigger_transition(rclcpp_lifecycle::Transition(Transition::TRANSITION_CONFIGURE));
    while (lifecycle_manager_node->get_current_state().id() != State::PRIMARY_STATE_INACTIVE && rclcpp::ok()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    lifecycle_manager_node->trigger_transition(rclcpp_lifecycle::Transition(Transition::TRANSITION_ACTIVATE));
    executor_thread.join();     //join command needs to wait for something

    //executor.spin();

    rclcpp::shutdown();
    return 0;
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

