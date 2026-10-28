// 从当前姿态画一条直线：规划 -> RViz 预览 -> 人工确认 -> 执行同一条轨迹。
// 只调用已运行的 MoveIt；不创建 SDK/CAN 连接，不自动回 park。
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/empty.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <moveit_msgs/srv/get_planning_scene.hpp>
#include <moveit_msgs/srv/get_state_validity.hpp>
#include <chrono>
#include <algorithm>
#include <cmath>
#include <iostream>
#include <map>
#include <mutex>
#include <thread>

// ===== 后续主要修改这里：米、弧度、比例 =====
const std::string ARM = "left";       // left / right
const std::string FRAME = "world";    // 固定世界坐标：X 前，Y 左，Z 上
constexpr double DX = 0.02;           // +X 方向 2 cm
constexpr double DY = 0.00;
constexpr double DZ = 0.00;
constexpr double SPEED = 0.10;        // 关节速度/加速度上限的比例，不是末端 m/s
constexpr double STEP = 0.001;        // 笛卡尔插值步长 1 mm
const std::string SHAPE = "line";     // line / circle / ptp
constexpr double RADIUS = 0.03;       // 圆半径 3 cm，直径 6 cm
const std::string PLANE = "xy";       // xy / xz / yz，均为世界坐标平面
// 点到点默认填写末端目标 XYZ（world 下绝对坐标，米），保持当前末端朝向。
const std::string TARGET_MODE = "position";  // position / pose / named / joints
const std::vector<double> TARGET_XYZ = {0.20, -0.40, 0.60};  // 示例目标，先预览确认
const std::string TARGET_NAME = "park";
const std::vector<double> TARGET_DEG = {0, 85, 0, 5, 0, 0, 0};
// 位姿示例仅为待修改数值，不代表当前位置可达；米、弧度。
const std::vector<double> TARGET_POSE = {0.20, -0.40, 0.60, 0.0, 0.0, 0.0};
// =========================================

template<class Service>
typename Service::Response::SharedPtr request(
    const rclcpp::Node::SharedPtr& node, const std::string& name,
    const typename Service::Request::SharedPtr& input) {
  auto client = node->create_client<Service>(name);
  if (!client->wait_for_service(std::chrono::seconds(10)))
    throw std::runtime_error(name + " unavailable");
  auto future = client->async_send_request(input);
  if (future.wait_for(std::chrono::seconds(10)) != std::future_status::ready)
    throw std::runtime_error(name + " timeout");
  return future.get();
}

std::vector<uint8_t> sceneSnapshot(const rclcpp::Node::SharedPtr& node) {
  auto query = std::make_shared<moveit_msgs::srv::GetPlanningScene::Request>();
  query->components.components = moveit_msgs::msg::PlanningSceneComponents::WORLD_OBJECT_GEOMETRY |
                                 moveit_msgs::msg::PlanningSceneComponents::ALLOWED_COLLISION_MATRIX;
  const auto scene = request<moveit_msgs::srv::GetPlanningScene>(node, "/get_planning_scene", query)->scene;
  rclcpp::SerializedMessage bytes;
  rclcpp::Serialization<moveit_msgs::msg::PlanningScene> serializer;
  serializer.serialize_message(&scene, &bytes);
  const auto& raw = bytes.get_rcl_serialized_message();
  return {raw.buffer, raw.buffer + raw.buffer_length};
}

void refreshRvizQuery(const rclcpp::Node::SharedPtr& node) {
  // MoveIt RViz 官方接口，只更新查询起点和黄色目标模型，不触发规划/执行。
  auto start = node->create_publisher<std_msgs::msg::Empty>("/rviz/moveit/update_start_state", 10);
  auto goal = node->create_publisher<std_msgs::msg::Empty>("/rviz/moveit/update_goal_state", 10);
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(1);
  while ((start->get_subscription_count() == 0 || goal->get_subscription_count() == 0) &&
         std::chrono::steady_clock::now() < deadline)
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  if (start->get_subscription_count() == 0 || goal->get_subscription_count() == 0) {
    RCLCPP_WARN(node->get_logger(), "RViz query not refreshed: enable Planning -> External Comm. (robot execution already completed).");
    return;
  }
  start->publish(std_msgs::msg::Empty{});
  goal->publish(std_msgs::msg::Empty{});
  std::this_thread::sleep_for(std::chrono::milliseconds(300));
  RCLCPP_INFO(node->get_logger(), "Requested RViz start/goal refresh from current feedback; no additional motion.");
}

int run(const rclcpp::Node::SharedPtr& node) {
  const bool execute = node->declare_parameter("execute", true);
  const std::string side = node->declare_parameter("arm", ARM);
  const double dx = node->declare_parameter("dx", DX);
  const double dy = node->declare_parameter("dy", DY);
  const double dz = node->declare_parameter("dz", DZ);
  const std::string shape = node->declare_parameter("shape", SHAPE);
  const double radius = node->declare_parameter("radius", RADIUS);
  const std::string plane = node->declare_parameter("plane", PLANE);
  const std::string target_mode = node->declare_parameter("target_mode", TARGET_MODE);
  const std::string target_name = node->declare_parameter("target_name", TARGET_NAME);
  const auto target_xyz = node->declare_parameter("target_xyz", TARGET_XYZ);
  const auto target_deg = node->declare_parameter("target_deg", TARGET_DEG);
  const auto target_pose = node->declare_parameter("target_pose", TARGET_POSE);
  if (shape != "line" && shape != "circle" && shape != "ptp")
    throw std::runtime_error("shape must be line, circle or ptp");
  if (shape == "circle" && (!std::isfinite(radius) || radius <= 0 ||
      (plane != "xy" && plane != "xz" && plane != "yz")))
    throw std::runtime_error("Choose a positive radius (metres) and plane xy/xz/yz.");
  if ((side != "left" && side != "right") || !std::isfinite(dx) ||
      !std::isfinite(dy) || !std::isfinite(dz) || (shape == "line" && std::hypot(dx, dy, dz) < 1e-6))
    throw std::runtime_error("Choose left/right and a finite nonzero displacement.");

  moveit::planning_interface::MoveGroupInterface arm(node, side + "_arm");
  arm.setPoseReferenceFrame(FRAME);
  arm.startStateMonitor(3.0);

  // 同时核对两臂反馈；确认期间任一机械臂移动，都应重新预览。
  struct Sample { double position; rclcpp::Time stamp; };
  std::map<std::string, Sample> feedback;
  std::mutex mutex;
  auto subscription = node->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", rclcpp::SensorDataQoS(), [&](sensor_msgs::msg::JointState::ConstSharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex);
        for (size_t i = 0; i < msg->name.size() && i < msg->position.size(); ++i)
          feedback.insert_or_assign(msg->name[i], Sample{msg->position[i], rclcpp::Time(msg->header.stamp)});
      });
  const auto model = arm.getRobotModel();
  auto current = arm.getCurrentState(3.0);
  if (!current) throw std::runtime_error("No current robot state.");
  std::vector<std::string> checked;
  for (const auto* joint : model->getActiveJointModels())
    for (const auto& variable : joint->getVariableNames()) checked.push_back(variable);
  auto freshFeedback = [&]() {
    std::lock_guard<std::mutex> lock(mutex);
    std::map<std::string, double> result;
    for (const auto& name : checked) {
      const auto it = feedback.find(name);
      if (it == feedback.end()) throw std::runtime_error("Missing feedback: " + name);
      const double age = (node->now()-it->second.stamp).seconds();
      if (age < -.05 || age > .5 || !std::isfinite(it->second.position))
        throw std::runtime_error("Stale/invalid feedback: " + name);
      result[name] = it->second.position;
    }
    return result;
  };
  // 等待订阅建立，不使用模型零位作为起点。
  std::this_thread::sleep_for(std::chrono::milliseconds(300));
  const auto start_positions = freshFeedback();
  current->setVariablePositions(start_positions);
  current->update();
  arm.setStartState(*current);

  // A 直接取反馈 FK；B 只改位置，末端朝向完全沿用 A。
  // TCP 默认由当前 MoveIt 规划组确定：实物配置通常为 left/right_tcp_link。
  const auto tcp = arm.getEndEffectorLink();
  if (tcp.empty()) throw std::runtime_error("Planning group has no TCP link.");
  geometry_msgs::msg::Pose a;
  const Eigen::Isometry3d transform = current->getGlobalLinkTransform(FRAME).inverse() *
                                    current->getGlobalLinkTransform(tcp);
  a.position.x = transform.translation().x();
  a.position.y = transform.translation().y();
  a.position.z = transform.translation().z();
  const Eigen::Quaterniond orientation(transform.rotation());
  a.orientation.x = orientation.x(); a.orientation.y = orientation.y();
  a.orientation.z = orientation.z(); a.orientation.w = orientation.w();
  auto b = a;
  b.position.x += dx; b.position.y += dy; b.position.z += dz;
  std::vector<geometry_msgs::msg::Pose> waypoints;
  if (shape == "line") {
    waypoints.push_back(b);
  } else if (shape == "circle") {
    // 当前位置 A 是圆上的起点，不额外运动到另一个起点。
    // XY/XZ 圆心在 A 的 +X；YZ 圆心在 A 的 +Y，距离均为 radius。
    // u=r(1-cosθ), v=r sinθ；最后一点直接复制 A，闭合一整圈。
    const double samples = std::max(72.0, std::ceil(2*M_PI*radius/STEP));
    if (samples > 100000) throw std::runtime_error("Circle has too many waypoints; reduce radius.");
    const int count = static_cast<int>(samples);
    for (int i = 1; i <= count; ++i) {
      const double theta = 2*M_PI*i/count;
      const double u = radius*(1-std::cos(theta));
      const double v = radius*std::sin(theta);
      auto point = a;
      if (plane == "xy") { point.position.x += u; point.position.y += v; }
      if (plane == "xz") { point.position.x += u; point.position.z += v; }
      if (plane == "yz") { point.position.y += u; point.position.z += v; }
      waypoints.push_back(i == count ? a : point);
    }
    RCLCPP_INFO(node->get_logger(), "Circle: plane=%s, radius=%.3f m, %d waypoints, returns to A.",
                plane.c_str(), radius, count);
    b = a;
  }
  RCLCPP_INFO(node->get_logger(), "%s / TCP=%s / frame=%s", side.c_str(), tcp.c_str(), FRAME.c_str());
  if (shape != "ptp")
    RCLCPP_INFO(node->get_logger(), "A=(%.4f, %.4f, %.4f), B=(%.4f, %.4f, %.4f) m",
        a.position.x, a.position.y, a.position.z, b.position.x, b.position.y, b.position.z);

  const auto scene = sceneSnapshot(node);
  moveit_msgs::msg::RobotTrajectory trajectory;
  if (shape == "ptp") {
    // ===== 点到点的任务代码：选一种目标，不规定末端经过的路径形状 =====
    arm.setPlanningTime(5.0);
    arm.setMaxVelocityScalingFactor(SPEED);
    arm.setMaxAccelerationScalingFactor(SPEED);
    if (target_mode == "position") {
      if (target_xyz.size() != 3 || !std::all_of(target_xyz.begin(), target_xyz.end(), [](double v) { return std::isfinite(v); }))
        throw std::runtime_error("target_xyz must contain [x,y,z] in metres, absolute world coordinates.");
      auto target = a;  // 只改位置，朝向复制当前实际姿态，不隐式设为零。
      target.position.x = target_xyz[0];
      target.position.y = target_xyz[1];
      target.position.z = target_xyz[2];
      if (!arm.setPoseTarget(target, tcp)) throw std::runtime_error("Could not set position target.");
      RCLCPP_INFO(node->get_logger(), "PTP A=(%.4f, %.4f, %.4f) -> B=(%.4f, %.4f, %.4f) m in world; orientation unchanged.",
                  a.position.x, a.position.y, a.position.z, target_xyz[0], target_xyz[1], target_xyz[2]);
    } else if (target_mode == "named") {
      if (!arm.setNamedTarget(target_name))
        throw std::runtime_error("Unknown named target: " + target_name);
      RCLCPP_INFO(node->get_logger(), "PTP named target: %s", target_name.c_str());
    } else if (target_mode == "joints") {
      if (target_deg.size() != 7 || !std::all_of(target_deg.begin(), target_deg.end(), [](double v) { return std::isfinite(v); }))
        throw std::runtime_error("target_deg must contain seven finite absolute joint angles in degrees.");
      std::map<std::string, double> target;
      for (size_t i = 0; i < 7; ++i)
        target[side + "_joint" + std::to_string(i+1)] = target_deg[i]*M_PI/180;
      if (!arm.setJointValueTarget(target)) throw std::runtime_error("Joint target outside limits.");
      RCLCPP_INFO(node->get_logger(), "PTP absolute seven-joint target (degrees), no relative addition.");
    } else if (target_mode == "pose") {
      if (target_pose.size() != 6 || !std::all_of(target_pose.begin(), target_pose.end(), [](double v) { return std::isfinite(v); }))
        throw std::runtime_error("target_pose must be [x,y,z,roll,pitch,yaw] in m/rad.");
      geometry_msgs::msg::Pose target;
      target.position.x = target_pose[0]; target.position.y = target_pose[1]; target.position.z = target_pose[2];
      Eigen::Quaterniond q = Eigen::AngleAxisd(target_pose[5], Eigen::Vector3d::UnitZ()) *
                             Eigen::AngleAxisd(target_pose[4], Eigen::Vector3d::UnitY()) *
                             Eigen::AngleAxisd(target_pose[3], Eigen::Vector3d::UnitX());
      target.orientation.x = q.x(); target.orientation.y = q.y();
      target.orientation.z = q.z(); target.orientation.w = q.w();
      if (!arm.setPoseTarget(target, tcp)) throw std::runtime_error("Could not set pose target.");
      RCLCPP_INFO(node->get_logger(), "PTP absolute TCP target in world: (%.4f, %.4f, %.4f) m", target_pose[0], target_pose[1], target_pose[2]);
    } else {
      throw std::runtime_error("target_mode must be position, pose, named or joints");
    }
    moveit::planning_interface::MoveGroupInterface::Plan point_plan;
    if (arm.plan(point_plan) != moveit::core::MoveItErrorCode::SUCCESS)
      throw std::runtime_error("Point-to-point planning failed; no execution.");
    trajectory = point_plan.trajectory_;
  } else {
    // avoid_collisions=true：按当前柜体、地面和机器人模型检查笛卡尔路径。
    const double fraction = arm.computeCartesianPath(waypoints, STEP, 5.0, trajectory, true);
    if (fraction < 1.0-1e-9)
      throw std::runtime_error("Cartesian path incomplete: " + std::to_string(fraction*100) + "%; no execution.");
  }
  if (trajectory.joint_trajectory.points.size() < 2)
    throw std::runtime_error("No nonempty motion trajectory; no execution.");
  robot_trajectory::RobotTrajectory timed(model, side + "_arm");
  timed.setRobotTrajectoryMsg(*current, trajectory);
  trajectory_processing::IterativeParabolicTimeParameterization timing;
  if (!timing.computeTimeStamps(timed, SPEED, SPEED))
    throw std::runtime_error("Trajectory timing failed.");
  timed.getRobotTrajectoryMsg(trajectory);
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  moveit::core::robotStateToRobotStateMsg(*current, plan.start_state_);
  plan.trajectory_ = trajectory;

  auto display = node->create_publisher<moveit_msgs::msg::DisplayTrajectory>(
      "/display_planned_path", rclcpp::QoS(1).reliable().transient_local());
  moveit_msgs::msg::DisplayTrajectory message;
  message.model_id = model->getName();
  message.trajectory_start = plan.start_state_;
  message.trajectory = {plan.trajectory_};
  display->publish(message);
  RCLCPP_INFO(node->get_logger(), "PREVIEW: %zu points; %.3f seconds; no motion sent.",
      trajectory.joint_trajectory.points.size(),
      rclcpp::Duration(trajectory.joint_trajectory.points.back().time_from_start).seconds());
  if (!execute) {
    std::cout << "仅预览。按 Enter 退出（程序保持运行时 RViz 可重复播放）。" << std::endl;
    std::string ignored; std::getline(std::cin, ignored);
    return 0;
  }
  std::cout << "请查看本次 RViz 预览。输入 do 执行这条轨迹，其他输入取消：" << std::endl;
  std::string answer; std::getline(std::cin, answer);
  if (answer != "do") return 0;
  const auto now = freshFeedback();
  for (const auto& [name, start] : start_positions) {
    const auto* joint = model->getJointOfVariable(name);
    const double tolerance = joint->getType() == moveit::core::JointModel::PRISMATIC ? .002 : .5*M_PI/180;
    if (std::abs(now.at(name)-start) > tolerance)
      throw std::runtime_error("Robot moved since preview: " + name + "; plan again.");
  }
  if (scene != sceneSnapshot(node))
    throw std::runtime_error("Planning scene changed since preview; plan again.");
  // 不调用 move() 或再次规划。交给原有 MoveIt -> 控制器 -> 官方驱动链路。
  const auto result = arm.execute(plan);
  if (result != moveit::core::MoveItErrorCode::SUCCESS)
    throw std::runtime_error("MoveIt execution failed: " + std::to_string(result.val));
  // MoveIt 成功之外，再用反馈确认终点；不把消息已发送当作实物已到达。
  bool arrived = false;
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
  while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline) {
    const auto actual = freshFeedback();
    arrived = true;
    for (size_t i = 0; i < trajectory.joint_trajectory.joint_names.size(); ++i) {
      const double goal = trajectory.joint_trajectory.points.back().positions[i];
      if (std::abs(actual.at(trajectory.joint_trajectory.joint_names[i])-goal) > M_PI/180)
        arrived = false;
    }
    if (arrived) break;
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
  if (!arrived) throw std::runtime_error("MoveIt finished, but feedback did not confirm endpoint within 1 degree.");
  RCLCPP_INFO(node->get_logger(), "MoveIt execution succeeded; feedback confirms endpoint within 1 degree.");
  refreshRvizQuery(node);
  return 0;
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("straight_line_example");
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spinner([&]() { executor.spin(); });
  int code = 0;
  try { code = run(node); }
  catch (const std::exception& error) {
    RCLCPP_ERROR(node->get_logger(), "%s", error.what()); code = 1;
  }
  executor.cancel(); spinner.join();
  rclcpp::shutdown();
  return code;
}
