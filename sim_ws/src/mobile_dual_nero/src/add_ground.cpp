#include <rclcpp/rclcpp.hpp>
#include <moveit_msgs/srv/get_planning_scene.hpp>
#include <moveit_msgs/srv/apply_planning_scene.hpp>
#include <moveit/collision_detection/collision_matrix.h>
#include <shape_msgs/msg/solid_primitive.hpp>

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto n = rclcpp::Node::make_shared("add_simulation_ground");
  auto get = n->create_client<moveit_msgs::srv::GetPlanningScene>("get_planning_scene");
  auto apply = n->create_client<moveit_msgs::srv::ApplyPlanningScene>("apply_planning_scene");
  if (!get->wait_for_service(std::chrono::seconds(30)) ||
      !apply->wait_for_service(std::chrono::seconds(30))) return 1;
  auto query = std::make_shared<moveit_msgs::srv::GetPlanningScene::Request>();
  query->components.components = moveit_msgs::msg::PlanningSceneComponents::ALLOWED_COLLISION_MATRIX;
  auto fetched = get->async_send_request(query);
  if (rclcpp::spin_until_future_complete(n, fetched, std::chrono::seconds(10)) !=
      rclcpp::FutureReturnCode::SUCCESS) return 1;
  collision_detection::AllowedCollisionMatrix acm(fetched.get()->scene.allowed_collision_matrix);
  // Tires touch the ground. Keep chassis, torso, arms and grippers checked.
  for (const auto& wheel : {"fl_wheel_link", "fr_wheel_link", "rl_wheel_link", "rr_wheel_link"})
    acm.setEntry("simulation_ground", wheel, true);
  auto request = std::make_shared<moveit_msgs::srv::ApplyPlanningScene::Request>();
  request->scene.is_diff = true;
  acm.getMessage(request->scene.allowed_collision_matrix);
  moveit_msgs::msg::CollisionObject floor;
  floor.id = "simulation_ground";
  floor.header.frame_id = "odom";
  floor.operation = moveit_msgs::msg::CollisionObject::ADD;
  shape_msgs::msg::SolidPrimitive box;
  box.type = shape_msgs::msg::SolidPrimitive::BOX;
  box.dimensions = {20.0, 20.0, 0.1};
  geometry_msgs::msg::Pose pose;
  pose.orientation.w = 1.0;
  pose.position.z = -0.05;  // top surface z=0, matching the Gazebo plane
  floor.primitives.push_back(box);
  floor.primitive_poses.push_back(pose);
  request->scene.world.collision_objects.push_back(floor);
  auto result = apply->async_send_request(request);
  if (rclcpp::spin_until_future_complete(n, result, std::chrono::seconds(10)) !=
      rclcpp::FutureReturnCode::SUCCESS || !result.get()->success) return 1;
  RCLCPP_INFO(n->get_logger(), "Gazebo ground registered in MoveIt (top z=0). Arm/ground collisions enabled.");
  rclcpp::shutdown();
  return 0;
}
