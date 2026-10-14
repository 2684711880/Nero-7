#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/srv/get_state_validity.hpp>
#include <moveit/robot_state/conversions.h>
#include <thread>
#include <cmath>
#include <stdexcept>
#include <vector>
#include <future>

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("verify_dual",
      rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spinner([&]() { executor.spin(); });
  int result = 0;
  try {
    auto validity = node->create_client<moveit_msgs::srv::GetStateValidity>("check_state_validity");
    if (!validity->wait_for_service(std::chrono::seconds(30)))
      throw std::runtime_error("No state validity service");
    for (const auto& group : {"left_arm", "right_arm", "both_arms", "left_gripper", "right_gripper"}) {
      moveit::planning_interface::MoveGroupInterface arm(node, group);
      arm.setPlanningTime(10.0);
      arm.setMaxVelocityScalingFactor(0.2);
      arm.setMaxAccelerationScalingFactor(0.2);
      auto current = arm.getCurrentState(20.0);
      if (!current) throw std::runtime_error("No fresh joint state");
      auto request = std::make_shared<moveit_msgs::srv::GetStateValidity::Request>();
      moveit::core::robotStateToRobotStateMsg(*current, request->robot_state);
      auto pending = validity->async_send_request(request);
      if (pending.wait_for(std::chrono::seconds(10)) != std::future_status::ready)
        throw std::runtime_error("State validity timeout");
      auto response = pending.get();
      if (!response->valid) {
        for (const auto& contact : response->contacts)
          RCLCPP_ERROR(node->get_logger(), "Collision: %s / %s", contact.contact_body_1.c_str(), contact.contact_body_2.c_str());
        throw std::runtime_error("Initial state is in collision");
      }
      const bool gripper = std::string(group).find("gripper") != std::string::npos;
      for (const auto& target : (gripper ? std::vector<std::string>{"open", "close"} : std::vector<std::string>{"ready", "home"})) {
        arm.setStartStateToCurrentState();
        if (!arm.setNamedTarget(target)) throw std::runtime_error("Missing named state");
        const auto goal = arm.getNamedTargetValues(target);
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        if (arm.plan(plan) != moveit::core::MoveItErrorCode::SUCCESS)
          throw std::runtime_error(std::string(group) + " planning failed");
        if (arm.execute(plan) != moveit::core::MoveItErrorCode::SUCCESS)
          throw std::runtime_error(std::string(group) + " execution failed");
        auto observed = arm.getCurrentState(5.0);
        if (!observed) throw std::runtime_error("No feedback after execution");
        double max_error = 0.0;
        for (const auto& item : goal)
          max_error = std::max(max_error, std::abs(observed->getVariablePosition(item.first) - item.second));
        if (max_error > (gripper ? 0.001 : 0.02))
          throw std::runtime_error("Joint feedback exceeds tolerance (arm: 0.02 rad; gripper: 0.001 m)");
        RCLCPP_INFO(node->get_logger(), "PASS %s -> %s: %zu points, max joint error %.6f %s",
                    group, target.c_str(), plan.trajectory_.joint_trajectory.points.size(), max_error, gripper ? "m" : "rad");
      }
      if (std::string(group) == "left_arm" || std::string(group) == "right_arm") {
        const std::string side = std::string(group) == "left_arm" ? "left" : "right";
        auto target_state = arm.getCurrentState(5.0);
        target_state->setVariablePosition(side + "_joint4", 0.2);
        target_state->update();
        const auto& transform = target_state->getGlobalLinkTransform(side + "_gripper_base");
        const Eigen::Quaterniond rotation(transform.rotation());
        geometry_msgs::msg::Pose pose;
        pose.position.x = transform.translation().x();
        pose.position.y = transform.translation().y();
        pose.position.z = transform.translation().z();
        pose.orientation.x = rotation.x(); pose.orientation.y = rotation.y();
        pose.orientation.z = rotation.z(); pose.orientation.w = rotation.w();
        arm.setStartStateToCurrentState();
        arm.setPoseTarget(pose, side + "_gripper_base");
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        if (arm.plan(plan) != moveit::core::MoveItErrorCode::SUCCESS)
          throw std::runtime_error(side + " IK pose planning failed");
        arm.clearPoseTargets();
        RCLCPP_INFO(node->get_logger(), "PASS %s IK pose planning: %zu points", group,
                    plan.trajectory_.joint_trajectory.points.size());
      }
    }
  } catch (const std::exception& e) {
    RCLCPP_ERROR(node->get_logger(), "FAIL: %s", e.what()); result = 1;
  }
  executor.cancel(); spinner.join(); rclcpp::shutdown(); return result;
}
