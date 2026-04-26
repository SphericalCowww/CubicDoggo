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
#include <moveit/trajectory_processing/ruckig_traj_smoothing.hpp>
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
#include "my_robot_interface/msg/cubic_doggo_leg_pose_target.hpp"
#include <example_interfaces/msg/bool.hpp>
using ros_string   = example_interfaces::msg::String;
using ros_array    = example_interfaces::msg::Float64MultiArray;
using custom_array = my_robot_interface::msg::CubicDoggoLegPoseTarget;
using ros_bool     = example_interfaces::msg::Bool;

const double DEFAULT_VEL_SCALE = 0.2;
const double DEFAULT_ACC_SCALE = 0.05;
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
class CubicDoggoLifecycleManager : public rclcpp_lifecycle::LifecycleNode {
public:
    explicit CubicDoggoLifecycleManager(const rclcpp::NodeOptions & options)
    : rclcpp_lifecycle::LifecycleNode("cubic_doggo_lifecycle", options) {
        RCLCPP_INFO(get_logger(), "CubicDoggoLifecycleManager:constructor(): %s", current_lifecycle_state_.c_str());

        callback_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
        moveit_node_ = std::make_shared<rclcpp::Node>("moveit_interface_node", this->get_namespace(), options);
        exec_action_client_ = rclcpp_action::create_client<ExecuteTrajectory>(moveit_node_, "/execute_trajectory");
 
        current_lifecycle_state_ = "state_initialized";
        RCLCPP_INFO(get_logger(), "CubicDoggoLifecycleManager:constructor(): %s", current_lifecycle_state_.c_str());
    }
    CallbackReturn on_configure(const rclcpp_lifecycle::State &) override {
        RCLCPP_INFO(get_logger(), "CubicDoggoLifecycleManager:on_configure(): %s", current_lifecycle_state_.c_str());

        if (!exec_action_client_->wait_for_action_server(std::chrono::seconds(5))) {
            RCLCPP_ERROR(get_logger(), "CubicDoggoLifecycleManager:on_configure(): "
                                       "ExecuteTrajectory action server not available");
            return CallbackReturn::FAILURE;
        }

        for (std::size_t legIdx = 0; legIdx < legN; legIdx++) {
            leg_interface_[legIdx] = std::make_shared<MoveGroupInterface>(moveit_node_, planning_group_[legIdx]);
            while (rclcpp::ok() && !leg_interface_[legIdx]->startStateMonitor(1.0)) {
                RCLCPP_INFO(get_logger(), "CubicDoggoLifecycleManager:on_configure(): "
                                          "waiting for valid MoveGroupInterface state %s",
                            planning_group_[legIdx].c_str());
            }
        }
        all_legs_interface_ = std::make_shared<MoveGroupInterface>(moveit_node_, all_legs_planning_group_);
        while (rclcpp::ok() && !all_legs_interface_->startStateMonitor(1.0)) {
            RCLCPP_INFO(get_logger(), "CubicDoggoLifecycleManager:on_configure(): "
                                      "waiting for valid MoveGroupInterface state %s", 
                        all_legs_planning_group_.c_str());
        }
        for (std::size_t legIdx = 0; legIdx < legN; legIdx++) {
            leg_interface_[legIdx]->setEndEffectorLink(endEffector_link_[legIdx]);
            leg_interface_[legIdx]->setGoalPositionTolerance(0.01);
            leg_interface_[legIdx]->setGoalOrientationTolerance(0.1);
            leg_interface_[legIdx]->setWorkspace(-1.0, -1.0, -1.0, 1.0, 1.0, 1.0); // world size
            leg_interface_[legIdx]->setNumPlanningAttempts(10);
            leg_interface_[legIdx]->setPlanningTime(1.0);
        }
        all_legs_interface_->setGoalPositionTolerance(0.01);
        all_legs_interface_->setGoalOrientationTolerance(0.1);
        all_legs_interface_->setWorkspace(-1.0, -1.0, -1.0, 1.0, 1.0, 1.0); // world size
        all_legs_interface_->setNumPlanningAttempts(10);
        all_legs_interface_->setPlanningTime(1.0);
        setDefaultVelAccScaler_(DEFAULT_VEL_SCALE, DEFAULT_ACC_SCALE);
 
        state_service_ = this->create_service<std_srvs::srv::Trigger>(
            "get_robot_state",
            std::bind(&CubicDoggoLifecycleManager::handleGetState_, this, _1, _2)); 
        walk_service_ = this->create_service<std_srvs::srv::SetBool>(
            "leg_walk_toggle", 
            std::bind(&CubicDoggoLifecycleManager::handleWalkRequest_, this, _1, _2), 
            rclcpp::ServicesQoS(), 
            callback_group_);

        for (std::size_t legIdx = 0; legIdx < legN; ++legIdx) {
            loadCurrentRobotState_(legIdx);
            RCLCPP_INFO(get_logger(), "CubicDoggoLifecycleManager:on_configure(): "
                                      "current end effector (x, y, z) = (%lf, %lf, %lf)",
                        endEffector_x_[legIdx], endEffector_y_[legIdx], endEffector_z_[legIdx]);
        }

        current_lifecycle_state_ = "state_configured";
        RCLCPP_INFO(get_logger(), "CubicDoggoLifecycleManager:on_configure(): %s", current_lifecycle_state_.c_str());
        return CallbackReturn::SUCCESS;
    }
    CallbackReturn on_activate(const rclcpp_lifecycle::State &) override {
        RCLCPP_INFO(get_logger(), "CubicDoggoLifecycleManager:on_activation(): %s", current_lifecycle_state_.c_str());

        auto sub_options = rclcpp::SubscriptionOptions();
        sub_options.callback_group = callback_group_;
        leg_named_subscriber_ = create_subscription<ros_string>  ("/leg_set_named", 10,
            std::bind(&CubicDoggoLifecycleManager::legNamedCallback_, this, _1), sub_options);
        leg_joint_subscriber_ = create_subscription<ros_array>   ("/leg_set_joint", 10,
            std::bind(&CubicDoggoLifecycleManager::legJointCallback_, this, _1), sub_options);
        leg_pose_subscriber_  = create_subscription<custom_array>("/leg_set_pose",  10,
            std::bind(&CubicDoggoLifecycleManager::legPoseCallback_,  this, _1), sub_options);

        keep_running_thread_ = true;
        is_walking_          = false; 
        walking_thread_      = std::thread(&CubicDoggoLifecycleManager::walkingLoop_, this);

        current_lifecycle_state_ = "state_stationary";
        RCLCPP_INFO(get_logger(), "CubicDoggoLifecycleManager:on_activation(): %s", current_lifecycle_state_.c_str());
        return CallbackReturn::SUCCESS;
    }
    CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override {
        RCLCPP_INFO(get_logger(), "CubicDoggoLifecycleManager:on_deactivation(): %s", 
                    current_lifecycle_state_.c_str());
        setDefaultVelAccScaler_(DEFAULT_VEL_SCALE, DEFAULT_ACC_SCALE);

        leg_named_subscriber_.reset();
        leg_joint_subscriber_.reset();
        leg_pose_subscriber_.reset();

        keep_running_thread_ = false;
        is_walking_          = false;
        if (walking_thread_.joinable()) {
            walking_thread_.join();
        }

        for (std::size_t legIdx = 0; legIdx < legN; legIdx++) {
            leg_interface_[legIdx]->stop();
        }
        all_legs_interface_->stop();
        current_lifecycle_state_ = "state_stopped";
        RCLCPP_INFO(get_logger(), "CubicDoggoLifecycleManager:on_deactivation(): %s", 
                                  current_lifecycle_state_.c_str());
        return CallbackReturn::SUCCESS;
    }
    CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override {
        RCLCPP_INFO(get_logger(), "CubicDoggoLifecycleManager:on_cleanup(): %s", current_lifecycle_state_.c_str());
        setDefaultVelAccScaler_(DEFAULT_VEL_SCALE, DEFAULT_ACC_SCALE);

        for (std::size_t legIdx = 0; legIdx < legN; legIdx++) {
            leg_interface_[legIdx].reset();
        }
        all_legs_interface_.reset();
        leg_named_subscriber_.reset();
        leg_joint_subscriber_.reset();
        leg_pose_subscriber_.reset();
 
        current_lifecycle_state_ = "state_configured";
        RCLCPP_INFO(get_logger(), "CubicDoggoLifecycleManager:on_clean(): %s", current_lifecycle_state_.c_str());
        return CallbackReturn::SUCCESS;
    }
    CallbackReturn on_shutdown(const rclcpp_lifecycle::State &) override {
        RCLCPP_INFO(get_logger(), "CubicDoggoLifecycleManager:on_shutdown(): %s", current_lifecycle_state_.c_str());
        setDefaultVelAccScaler_(DEFAULT_VEL_SCALE, DEFAULT_ACC_SCALE);

        keep_running_thread_ = false;
        is_walking_          = false;
        if (walking_thread_.joinable()) {
            walking_thread_.join();
        }
        
        for (std::size_t legIdx = 0; legIdx < legN; legIdx++) {
            leg_interface_[legIdx]->stop();
        }
        all_legs_interface_->stop();
    
        current_lifecycle_state_ = "state_stopped";
        RCLCPP_INFO(get_logger(), "CubicDoggoLifecycleManager:on_shutdown(): %s", current_lifecycle_state_.c_str());
        return CallbackReturn::SUCCESS;
    }
    rclcpp::Node::SharedPtr get_moveit_node() const { 
        return moveit_node_; 
    }
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
private:
    void legNamedCallback_(const ros_string::SharedPtr msg) {
        RCLCPP_INFO(get_logger(), "CubicDoggoLifecycleManager:legNamedCallback(): command received");
        setDefaultVelAccScaler_(DEFAULT_VEL_SCALE, DEFAULT_ACC_SCALE);
        legNamedTarget_(msg->data);
    }
    void legJointCallback_(const ros_array::SharedPtr msg) {
        RCLCPP_INFO(get_logger(), "CubicDoggoLifecycleManager:legJointCallback(): command received");
        setDefaultVelAccScaler_(DEFAULT_VEL_SCALE, DEFAULT_ACC_SCALE);
        if (msg->data.size() != (1+jointNperLeg)) {
            RCLCPP_WARN(get_logger(), "CubicDoggoLifecycleManager:legJointCallback(): message length mismatch");
            return;
        }
        legJointTarget_(msg->data);
    }
    void legPoseCallback_(const custom_array::SharedPtr msg) {
        RCLCPP_INFO(get_logger(), "CubicDoggoLifecycleManager:legPoseCallback(): command received");
        setDefaultVelAccScaler_(DEFAULT_VEL_SCALE, DEFAULT_ACC_SCALE);
        legPoseTarget_(msg->leg_index, msg->x, msg->y, msg->z);
    }
    ///////////
    void legNamedTarget_(const std::string &name) {
        if (name == "stand") {
            all_legs_interface_->setNamedTarget("stand_mid");
            planAndExecute_();
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
        }
        all_legs_interface_->setNamedTarget(name);
        planAndExecute_();
    }
    void legJointTarget_(const std::vector<double> &joints) {
        std::size_t legIdx = static_cast<std::size_t>(joints[0]);
        all_legs_current_robot_state_ = all_legs_interface_->getCurrentState(2.0);
        const std::vector<std::string>& leg_joint_names = leg_interface_[legIdx]->getJointNames();
        for (std::size_t jointIdx = 0; jointIdx < (joints.size()-1); jointIdx++) {
            double joint_val = joints[jointIdx + 1];
            all_legs_current_robot_state_->setJointPositions(leg_joint_names[jointIdx], &joint_val);
        }
        all_legs_interface_->setJointValueTarget(*all_legs_current_robot_state_);
        planAndExecute_();
    }
    void legPoseTarget_(int legIdxInput, double x, double y, double z) {
        std::size_t legIdx = static_cast<std::size_t>(legIdxInput);
        geometry_msgs::msg::Pose target_pose = endEffector_pose_[legIdx].pose;
        target_pose.position.x = x;
        target_pose.position.y = y;
        target_pose.position.z = z;
            
        success_ = leg_interface_[legIdx]->setApproximateJointValueTarget(target_pose, endEffector_link_[legIdx]);
        if (success_ == false) {
            RCLCPP_ERROR(get_logger(), "CubicDoggoLifecycleManager:legPoseTarget_(): "
                                       "failed to find IK solution for target!");
            return;
        }
        std::vector<double> leg_joints;
        leg_interface_[legIdx]->getJointValueTarget(leg_joints);
        leg_joints.insert(leg_joints.begin(), static_cast<double>(legIdx));
        legJointTarget_(leg_joints);
            
        loadCurrentRobotState_(legIdx);
        to_target_dist = std::sqrt(std::pow(endEffector_x_[legIdx] - x, 2) + 
                                   std::pow(endEffector_y_[legIdx] - y, 2) +
                                   std::pow(endEffector_z_[legIdx] - z, 2));
        if (to_target_dist > to_target_dist_thres) {
            RCLCPP_WARN(get_logger(), "CubicDoggoLifecycleManager:legPoseTarget_(): "
                                      "target unreachable (likely hit a joint limit). Settled %f meters away.", 
                                      to_target_dist);
        }
        RCLCPP_INFO(get_logger(), "CubicDoggoLifecycleManager:legSetPoseTarget_(): "
                                  "current end effector (i, x, y, z) = (%zu, %lf, %lf, %lf)",
                    legIdx, endEffector_x_[legIdx], endEffector_y_[legIdx], endEffector_z_[legIdx]);    
    }
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    std::vector<moveit::core::RobotStatePtr> linearWalkGait_(double lift, double stride) {
        std::vector<moveit::core::RobotStatePtr> gait_waypoints;
        for (int gaitPhase = 0; gaitPhase < 2; gaitPhase++) { 
            moveit::core::RobotStatePtr walk_state = 
                std::make_shared<moveit::core::RobotState>(*last_walk_state_);
            for (std::size_t legIdx = 0; legIdx < legN; legIdx++) {
                double target_x = home_x_[legIdx];
                double target_y = home_y_[legIdx];
                double target_z = home_z_[legIdx];

                bool is_group_a = (legIdx == 0 || legIdx == 3);
                bool is_group_b = (legIdx == 1 || legIdx == 2);
                if (gaitPhase == 0) {
                    if (is_group_a == true) { 
                        target_y += stride; 
                        target_z -= lift; 
                    } 
                    if (is_group_b == true) { 
                        target_y -= stride; 
                    }
                } else if (gaitPhase == 1) {
                    if (is_group_a == true) { 
                        target_y -= stride;
                    } 
                    if (is_group_b == true) { 
                        target_y += stride;
                        target_z -= lift; 
                    }
                }
                geometry_msgs::msg::Pose leg_pose = endEffector_pose_[legIdx].pose;
                leg_pose.position.x = target_x;
                leg_pose.position.y = target_y;
                leg_pose.position.z = target_z;

                auto leg_model_group = all_legs_robot_model_->getJointModelGroup(planning_group_[legIdx]);
                success_ = walk_state->setFromIK(leg_model_group, leg_pose, endEffector_link_[legIdx]);
                if (success_ == false) {
                    RCLCPP_ERROR(get_logger(), "CubicDoggoLifecycleManager:linearWalkGait(): "
                                               "IK failed for leg %zu in phase %d", legIdx, gaitPhase);
                    is_walking_ = false;
                    return {};
                }
            }
            if (is_walking_ == false) {
                break;
            }
            gait_waypoints.push_back(walk_state);
            last_walk_state_ = walk_state;
        }
        return gait_waypoints;
    }
    std::vector<moveit::core::RobotStatePtr> triangleWalkGait_(double lift, double stride) {
        std::vector<moveit::core::RobotStatePtr> gait_waypoints;
        for (int trajIdx = 0; trajIdx < 4; trajIdx++) { 
            moveit::core::RobotStatePtr walk_state = 
                std::make_shared<moveit::core::RobotState>(*last_walk_state_);
            for (std::size_t legIdx = 0; legIdx < legN; legIdx++) {
                double target_x = home_x_[legIdx];
                double target_y = home_y_[legIdx];
                double target_z = home_z_[legIdx];

                bool is_group_a = (legIdx == 0 || legIdx == 3);
                bool is_group_b = (legIdx == 1 || legIdx == 2);
                if (trajIdx == 0) {
                    if (is_group_a == true) { 
                        target_z -= lift; 
                    }
                    if (is_group_b  == true) { 
                    }
                } else if (trajIdx == 1) {
                    if (is_group_a == true) { 
                        target_y += stride; 
                    } 
                    if (is_group_b == true) { 
                        target_y -= stride; 
                    }
                } else if (trajIdx == 2) {
                    if (is_group_a == true) {
                    }
                    if (is_group_b == true) { 
                        target_z -= lift;
                    }
                } else if (trajIdx == 3) {
                    if (is_group_a == true) { 
                        target_y -= stride; 
                    } 
                    if (is_group_b == true) { 
                        target_y += stride; 
                    }
                }
                geometry_msgs::msg::Pose leg_pose = endEffector_pose_[legIdx].pose;
                leg_pose.position.x = target_x;
                leg_pose.position.y = target_y;
                leg_pose.position.z = target_z;

                auto leg_model_group = all_legs_robot_model_->getJointModelGroup(planning_group_[legIdx]);
                success_ = walk_state->setFromIK(leg_model_group, leg_pose, endEffector_link_[legIdx]);
                if (success_ == false) {
                    RCLCPP_ERROR(get_logger(), "CubicDoggoLifecycleManager:triangleWalkGait_(): "
                                               "IK failed for leg %zu in trajIdx %d", legIdx, trajIdx);
                    is_walking_ = false;
                    break;
                }
            }
            if (is_walking_ == false) {
                break;
            }
            gait_waypoints.push_back(walk_state);
            last_walk_state_ = walk_state;
        }
        return gait_waypoints;
    }
    std::vector<moveit::core::RobotStatePtr> triangleForwardWalkGait_(double lift, double stride) {
        std::vector<moveit::core::RobotStatePtr> gait_waypoints;
        for (int trajIdx = 0; trajIdx < 4; trajIdx++) { 
            moveit::core::RobotStatePtr walk_state = 
                std::make_shared<moveit::core::RobotState>(*last_walk_state_);
            for (std::size_t legIdx = 0; legIdx < legN; legIdx++) {
                double target_x = home_x_[legIdx];
                double target_y = home_y_[legIdx];
                double target_z = home_z_[legIdx];
                bool is_group_a = (legIdx == 0 || legIdx == 3);
                bool is_group_b = (legIdx == 1 || legIdx == 2);
                bool joint_hold_position = false;
                if (walking_initialized_ == false) {
                    if (trajIdx == 0) {
                        if (is_group_a == true) {
                            target_x = home_x_[legIdx];
                            target_y = home_y_[legIdx] + stride/2.0;
                            target_z = home_z_[legIdx] - lift; 
                        }
                    } else if (trajIdx == 1) {
                        if (is_group_a == true) {
                            target_x = home_x_[legIdx];
                            target_y = home_y_[legIdx] + stride;
                            target_z = home_z_[legIdx];
                        }
                    } else {
                        joint_hold_position = true;
                    }
                    if (is_group_b == true) {
                        joint_hold_position = true;
                    }
                } else {
                    if (trajIdx == 0) {
                        if (is_group_a == true) {
                            target_x = home_x_[legIdx];
                            target_y = home_y_[legIdx] + stride/2.0;
                            target_z = home_z_[legIdx]; 
                        }
                        if (is_group_b  == true) { 
                            target_x = home_x_[legIdx]; 
                            target_y = home_y_[legIdx] + stride/2.0;
                            target_z = home_z_[legIdx] - lift;
                        }
                    } else if (trajIdx == 1) {
                        if (is_group_a == true) {
                            target_x = home_x_[legIdx];
                            target_y = home_y_[legIdx];
                            target_z = home_z_[legIdx];
                        } 
                        if (is_group_b == true) { 
                            target_x = home_x_[legIdx];
                            target_y = home_y_[legIdx] + stride;
                            target_z = home_z_[legIdx];
                        }
                    } else if (trajIdx == 2) {
                        if (is_group_a == true) {
                            target_x = home_x_[legIdx];
                            target_y = home_y_[legIdx] + stride/2.0;
                            target_z = home_z_[legIdx] - lift;
                        }
                        if (is_group_b == true) {
                            target_x = home_x_[legIdx]; 
                            target_y = home_y_[legIdx] + stride/2.0;
                            target_z = home_z_[legIdx];
                        }
                    } else if (trajIdx == 3) {
                        if (is_group_a == true) { 
                            target_x = home_x_[legIdx];
                            target_y = home_y_[legIdx] + stride;
                            target_z = home_z_[legIdx]; 
                        } 
                        if (is_group_b == true) { 
                            target_x = home_x_[legIdx];
                            target_y = home_y_[legIdx];
                            target_z = home_z_[legIdx];
                        }
                    }
                }
                geometry_msgs::msg::Pose leg_pose = endEffector_pose_[legIdx].pose;
                leg_pose.position.x = target_x;
                leg_pose.position.y = target_y;
                leg_pose.position.z = target_z;

                auto leg_model_group = all_legs_robot_model_->getJointModelGroup(planning_group_[legIdx]);
                if ((joint_hold_position == true) && (trajIdx > 0)) {
                    auto leg_model_group = all_legs_robot_model_->getJointModelGroup(planning_group_[legIdx]);
                    std::vector<double> prev_joint_values;
                    gait_waypoints.back()->copyJointGroupPositions(leg_model_group, prev_joint_values);
                    walk_state->setJointGroupPositions(leg_model_group, prev_joint_values);
                } else {
                    success_ = walk_state->setFromIK(leg_model_group, leg_pose, endEffector_link_[legIdx]);
                    if (success_ == false) {
                        RCLCPP_ERROR(get_logger(), "CubicDoggoLifecycleManager:triangleWalkGait_(): "
                                                   "IK failed for leg %zu in trajIdx %d", legIdx, trajIdx);
                        is_walking_ = false;
                        break;
                    }
                }
            }
            if (is_walking_ == false) {
                break;
            }
            gait_waypoints.push_back(walk_state);
            last_walk_state_ = walk_state;
        }
        return gait_waypoints;
    }
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    void handleGetState_(const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
                         std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        for (std::size_t legIdx = 0; legIdx < legN; ++legIdx) {
            loadCurrentRobotState_(legIdx);
            response->message = "CubicDoggoLifecycleManager:handleGetState_(): current end effector (x, y, z) = ("
                               +std::to_string(endEffector_x_[legIdx]) + ", "
                               +std::to_string(endEffector_y_[legIdx]) + ", "
                               +std::to_string(endEffector_z_[legIdx]) + ")";
        }
        response->success = true;
    }
    void handleWalkRequest_(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                            std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
        is_walking_ = request->data;
        response->success = true;
        response->message = is_walking_ ? "walking started" : "walking stopped";
    }
    void walkingLoop_() {
        double maxVelScale = 1.0;
        double maxAccScale = 1.0;
        double maxJrkScale = 1.0;
        
        all_legs_robot_model_ = all_legs_interface_->getRobotModel();
        bool home_captured = false;
        while (keep_running_thread_ && rclcpp::ok()) {
            if (is_walking_ == false) {
                walking_initialized_ = false;
                home_captured = false;
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                continue;
            } else if (walking_initialized_ == false) {
                legNamedTarget_("stand");
                setDefaultVelAccScaler_(maxVelScale, maxAccScale);
            }
            loadCurrentRobotState_();
            last_walk_state_ = std::make_shared<moveit::core::RobotState>(*all_legs_current_robot_state_);
            if (home_captured == false) {
                for (std::size_t legIdx = 0; legIdx < legN; legIdx++) {
                    home_x_[legIdx] = endEffector_x_[legIdx];
                    home_y_[legIdx] = endEffector_y_[legIdx];
                    home_z_[legIdx] = endEffector_z_[legIdx];
                }
                home_captured = true;
                RCLCPP_INFO(get_logger(), "CubicDoggoLifecycleManager:walkingLoop_(): home positions captured.");
            }
            
            ////////////////
            //std::vector<moveit::core::RobotStatePtr> gait_waypoints = linearWalkGait_(0.03, 0.03);
            //std::vector<moveit::core::RobotStatePtr> gait_waypoints = triangleWalkGait_(0.05, 0.03);
            std::vector<moveit::core::RobotStatePtr> gait_waypoints = triangleForwardWalkGait_(0.03, 0.06);
            //std::vector<moveit::core::RobotStatePtr> gait_waypoints = triangleFineWalkGait_(0.04, 0.03);
            ////////////////
            
            auto robo_traj = std::make_shared<robot_trajectory::RobotTrajectory>(all_legs_robot_model_, 
                                                                                 all_legs_planning_group_);
            robo_traj->addSuffixWayPoint(*all_legs_current_robot_state_, 0.05);
            for (const auto& state : gait_waypoints) {
                robo_traj->addSuffixWayPoint(*state, 0.05); 
            }
            
            //trajectory_processing::TimeOptimalTrajectoryGeneration traj_gen;
            //success_ = traj_gen.computeTimeStamps(*robo_traj, maxVelScale, maxAccScale);
            trajectory_processing::RuckigSmoothing traj_gen;
            success_ = traj_gen.applySmoothing(*robo_traj, maxVelScale, maxAccScale, maxJrkScale);
            if (success_ == false) {
                RCLCPP_ERROR(get_logger(), "CubicDoggoLifecycleManager:walkingLoop_(): "
                                           "robot trajectory timing generation failed");
                is_walking_ = false;
                continue;
            }
            
            moveit_msgs::msg::RobotTrajectory traj_msg;
            robo_traj->getRobotTrajectoryMsg(traj_msg);
            
            auto exec_goal = ExecuteTrajectory::Goal();
            exec_goal.trajectory = traj_msg;
            
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
                        RCLCPP_INFO(get_logger(), "CubicDoggoLifecycleManager:walkingLoop_(): "
                                                  "walking stop requested, stopping goal");
                        exec_action_client_->async_cancel_goal(goal_handle);
                    }
                }
            }
            if (walking_initialized_ == false) {
                walking_initialized_ = true;
            }
        }
    }
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    void setDefaultVelAccScaler_(double velScale, double accScale) {
        for (std::size_t legIdx = 0; legIdx < legN; legIdx++) {
            leg_interface_[legIdx]->setMaxVelocityScalingFactor(velScale);
            leg_interface_[legIdx]->setMaxAccelerationScalingFactor(accScale);
        }
        all_legs_interface_->setMaxVelocityScalingFactor(velScale);
        all_legs_interface_->setMaxAccelerationScalingFactor(accScale);
    }
    void planAndExecute_(std::size_t legIdx) {
        success_ = (leg_interface_[legIdx]->plan(move_plan_[legIdx]) == moveit::core::MoveItErrorCode::SUCCESS);
        if (success_) {
            leg_interface_[legIdx]->execute(move_plan_[legIdx]);
        } else {
            RCLCPP_WARN(get_logger(), "CubicDoggoLifecycleManager:planAndExecute_(%zu): planning failed", legIdx);
        }
    }
    void planAndExecute_() {
        success_ = ((all_legs_interface_->plan(all_legs_move_plan_) == moveit::core::MoveItErrorCode::SUCCESS));
        if (success_) {
            all_legs_interface_->execute(all_legs_move_plan_);
        } else {
            RCLCPP_WARN(get_logger(), "CubicDoggoLifecycleManager:planAndExecute_(): planning failed");
        }
    }
    void loadCurrentRobotState_(std::size_t legIdx) {
        current_robot_state_[legIdx] = leg_interface_[legIdx]->getCurrentState(0.5);       // wait up to 0.5 seconds 
        if (!current_robot_state_[legIdx]) {
            RCLCPP_ERROR(get_logger(), "CubicDoggoLifecycleManager:loadCurrentRobotState_(%zu): "
                                       "timeout to fetch current robot state", legIdx);
            return;
        }
        endEffector_pose_[legIdx] = leg_interface_[legIdx]->getCurrentPose(endEffector_link_[legIdx]);
        endEffector_x_[legIdx] = endEffector_pose_[legIdx].pose.position.x;
        endEffector_y_[legIdx] = endEffector_pose_[legIdx].pose.position.y;
        endEffector_z_[legIdx] = endEffector_pose_[legIdx].pose.position.z;
        leg_interface_[legIdx]->setStartStateToCurrentState();
    }
    void loadCurrentRobotState_() {
        all_legs_current_robot_state_ = all_legs_interface_->getCurrentState(2.0);       // wait up to 2.0 seconds 
        if (!all_legs_current_robot_state_) {
            RCLCPP_ERROR(get_logger(), "CubicDoggoLifecycleManager:loadCurrentRobotState_(): "
                                       "timeout to fetch current robot state");
            return;
        }
        const moveit::core::JointModelGroup* joint_model_group = 
            all_legs_current_robot_state_->getJointModelGroup(all_legs_planning_group_);
        all_legs_current_robot_state_->copyJointGroupPositions(joint_model_group, all_legs_jointVals_);
        all_legs_interface_->setStartStateToCurrentState();
        for (std::size_t legIdx = 0; legIdx < legN; legIdx++) {
            endEffector_pose_[legIdx] = leg_interface_[legIdx]->getCurrentPose(endEffector_link_[legIdx]);
            endEffector_x_[legIdx] = endEffector_pose_[legIdx].pose.position.x;
            endEffector_y_[legIdx] = endEffector_pose_[legIdx].pose.position.y;
            endEffector_z_[legIdx] = endEffector_pose_[legIdx].pose.position.z;
            leg_interface_[legIdx]->setStartStateToCurrentState();
        }
    }
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    rclcpp::Node::SharedPtr moveit_node_;
    std::atomic<bool> success_{false};
    std::string current_lifecycle_state_ = "state_uninitialized";

    static constexpr std::size_t legN = 4;
    static constexpr std::size_t jointNperLeg = 3;
    std::array<std::string, legN> planning_group_  {"leg_FL",        "leg_FR",       "leg_BL",       "leg_BR"};
    std::array<std::string, legN> endEffector_link_{"calfSphere_FL", "calfSphere_FR","calfSphere_BL","calfSphere_BR"};
    std::array<std::shared_ptr<MoveGroupInterface>, legN> leg_interface_;
    std::array<moveit::core::RobotStatePtr,         legN> current_robot_state_;
    std::array<geometry_msgs::msg::PoseStamped,     legN> endEffector_pose_;
    std::array<MoveGroupInterface::Plan,            legN> move_plan_;
    std::array<double, legN> endEffector_x_{};
    std::array<double, legN> endEffector_y_{};
    std::array<double, legN> endEffector_z_{};
    std::string                           all_legs_planning_group_ = "all_legs";
    std::vector<std::string>              all_legs_endEffector_links_;
    std::shared_ptr<MoveGroupInterface>   all_legs_interface_;
    moveit::core::RobotStatePtr           all_legs_current_robot_state_;
    std::vector<geometry_msgs::msg::Pose> all_legs_endEffector_pose_;
    MoveGroupInterface::Plan              all_legs_move_plan_;
    std::vector<double>                   all_legs_jointVals_;

    rclcpp::CallbackGroup::SharedPtr callback_group_;
    rclcpp::Subscription<ros_string>  ::SharedPtr leg_named_subscriber_;
    rclcpp::Subscription<ros_array>   ::SharedPtr leg_joint_subscriber_;
    rclcpp::Subscription<custom_array>::SharedPtr leg_pose_subscriber_;
    double to_target_dist       = 0;
    double to_target_dist_thres = 0.01;

    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr state_service_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr walk_service_; 
    moveit::core::RobotStatePtr last_walk_state_;
    std::atomic<bool> is_walking_{false};
    std::atomic<bool> walking_initialized_{false};
    std::atomic<bool> keep_running_thread_{false};
    std::thread walking_thread_;
    std::array<double, legN> home_x_, home_y_, home_z_;
    moveit::core::RobotModelConstPtr all_legs_robot_model_;
    rclcpp_action::Client<ExecuteTrajectory>::SharedPtr exec_action_client_;
};
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);

    auto lifecycle_manager_node = std::make_shared<CubicDoggoLifecycleManager>(node_options);
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(lifecycle_manager_node->get_node_base_interface());
    executor.add_node(lifecycle_manager_node->get_moveit_node());
    
    std::thread executor_thread([&executor]() { executor.spin(); });
    using lifecycle_msgs::msg::Transition;
    using lifecycle_msgs::msg::State;
    lifecycle_manager_node->trigger_transition(rclcpp_lifecycle::Transition(Transition::TRANSITION_CONFIGURE));
    while (rclcpp::ok() && lifecycle_manager_node->get_current_state().id() != State::PRIMARY_STATE_ACTIVE) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        if (lifecycle_manager_node->get_current_state().id() == State::PRIMARY_STATE_INACTIVE) {
            lifecycle_manager_node->trigger_transition(
                rclcpp_lifecycle::Transition(Transition::TRANSITION_ACTIVATE));
        }
    }
    while (rclcpp::ok()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
    executor.cancel();
    if (executor_thread.joinable()) {
        executor_thread.join();
    }
    rclcpp::shutdown();
    return 0;
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

