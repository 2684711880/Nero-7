#pragma once
// 任务代码只调用 current_pose() / move_p()；这里复用原有 MoveIt 规划与执行流程。
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/empty.hpp>
#include <rcl_interfaces/srv/set_parameters.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <moveit_msgs/srv/get_planning_scene.hpp>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <map>
#include <mutex>
#include <thread>

namespace nero_task {
struct TaskCancelled {};
struct Pose { double x, y, z, roll, pitch, yaw; }; // m / rad，任务所选参考坐标系
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


class Robot {
  struct Sample { double position; rclcpp::Time stamp; };
  struct Feedback { std::mutex mutex; std::map<std::string, Sample> values; };
  rclcpp::Node::SharedPtr node_;
  moveit::planning_interface::MoveGroupInterface arm_;
  std::shared_ptr<Feedback> feedback_ = std::make_shared<Feedback>();
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_;
  std::vector<std::string> variables_;
  bool preview_, execute_;
  double speed_;
  std::string frame_;
  std::string side_;
  std::vector<double> xyz_override_, pose_override_;

  std::map<std::string, double> feedback() const {
    std::lock_guard<std::mutex> lock(feedback_->mutex);
    std::map<std::string, double> result;
    for (const auto& name : variables_) {
      const auto it = feedback_->values.find(name);
      if (it == feedback_->values.end()) throw std::runtime_error("Missing feedback: " + name);
      const double age = (node_->now()-it->second.stamp).seconds();
      if (age < -.05 || age > .5 || !std::isfinite(it->second.position))
        throw std::runtime_error("Stale/invalid feedback: " + name);
      result[name] = it->second.position;
    }
    return result;
  }

public:
  explicit Robot(const rclcpp::Node::SharedPtr& node, bool preview=true, const std::string& side="right",
                 const std::string& frame="world")
      : node_(node), arm_(node, node->declare_parameter<std::string>("arm", side) + "_arm") {
    side_ = node->get_parameter("arm").as_string();
    preview_ = node->declare_parameter("preview", preview);
    execute_ = node->declare_parameter("execute", true);
    if (!preview_ && !execute_) throw std::runtime_error("preview=false requires execute=true");
    xyz_override_ = node->declare_parameter("target_xyz", std::vector<double>{});
    pose_override_ = node->declare_parameter("target_pose", std::vector<double>{});
    const double speed = speed_ = node->declare_parameter("speed", .10);
    if (!std::isfinite(speed) || speed <= 0 || speed > 1) throw std::runtime_error("speed must be in (0,1]");
    frame_ = node->declare_parameter("frame", frame);
    if (!arm_.getRobotModel()->hasLinkModel(frame_)) throw std::runtime_error("Unknown task frame: " + frame_);
    arm_.setPoseReferenceFrame(frame_);
    arm_.setPlanningTime(5.0);
    arm_.setMaxVelocityScalingFactor(speed);
    arm_.setMaxAccelerationScalingFactor(speed);
    arm_.startStateMonitor(3.0);
    for (const auto* joint : arm_.getRobotModel()->getActiveJointModels())
      for (const auto& name : joint->getVariableNames()) variables_.push_back(name);
    subscription_ = node_->create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", rclcpp::SensorDataQoS(), [state=feedback_](sensor_msgs::msg::JointState::ConstSharedPtr msg) {
          std::lock_guard<std::mutex> lock(state->mutex);
          for (size_t i=0; i<msg->name.size() && i<msg->position.size(); ++i)
            state->values.insert_or_assign(msg->name[i], Sample{msg->position[i], rclcpp::Time(msg->header.stamp)});
        });
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
  }

  Pose current_pose() {
    auto state = arm_.getCurrentState(3.0);
    if (!state) throw std::runtime_error("No current robot state");
    state->setVariablePositions(feedback()); state->update();
    const auto t = state->getGlobalLinkTransform(frame_).inverse()*state->getGlobalLinkTransform(arm_.getEndEffectorLink());
    const auto ypr = t.rotation().eulerAngles(2,1,0);
    return {t.translation().x(),t.translation().y(),t.translation().z(),ypr[2],ypr[1],ypr[0]};
  }

  void move_p(Pose target) {
    // 保留旧命令行目标覆盖；正常写任务时直接修改调用处的 target。
    if (!xyz_override_.empty()) {
      if (xyz_override_.size()!=3) throw std::runtime_error("target_xyz requires 3 values");
      target.x=xyz_override_[0]; target.y=xyz_override_[1]; target.z=xyz_override_[2];
    }
    if (!pose_override_.empty()) {
      if (pose_override_.size()!=6) throw std::runtime_error("target_pose requires 6 values");
      target={pose_override_[0],pose_override_[1],pose_override_[2],pose_override_[3],pose_override_[4],pose_override_[5]};
    }
    for (const double v : {target.x,target.y,target.z,target.roll,target.pitch,target.yaw})
      if (!std::isfinite(v)) throw std::runtime_error("Target must be finite");
    const auto start = feedback();
    auto current = arm_.getCurrentState(3.0);
    if (!current) throw std::runtime_error("No current robot state");
    current->setVariablePositions(start); current->update();
    arm_.setStartState(*current);
    geometry_msgs::msg::Pose goal;
    goal.position.x=target.x; goal.position.y=target.y; goal.position.z=target.z;
    const Eigen::Quaterniond q = Eigen::AngleAxisd(target.yaw,Eigen::Vector3d::UnitZ())*
        Eigen::AngleAxisd(target.pitch,Eigen::Vector3d::UnitY())*Eigen::AngleAxisd(target.roll,Eigen::Vector3d::UnitX());
    goal.orientation.x=q.x(); goal.orientation.y=q.y(); goal.orientation.z=q.z(); goal.orientation.w=q.w();
    if (!arm_.setPoseTarget(goal)) throw std::runtime_error("Could not set target");
    const auto scene = sceneSnapshot(node_);
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    RCLCPP_INFO(node_->get_logger(), "move_p frame=%s target=(%.4f, %.4f, %.4f), RPY=(%.4f, %.4f, %.4f)",
        frame_.c_str(),target.x,target.y,target.z,target.roll,target.pitch,target.yaw);
    const auto result=arm_.plan(plan);
    if (result!=moveit::core::MoveItErrorCode::SUCCESS)
      throw std::runtime_error("Point-to-point planning failed (code="+std::to_string(result.val)+"); no execution");
    finish(plan,start,scene);
  }

  // 末端直线到目标，保持目标中填写的位置和朝向。
  void move_l(Pose target) { cartesian({target}); }

  // 宽度为两指总开口(m)，force 沿用官方驱动的 N 参数；不是实测接触力保证。
  void gripper_open(double width=.07, double force=1.0) { gripper(width,force,false); }
  void gripper_close(double force=1.0) { gripper(0.,force,true); }

  // 当前末端为圆上的起点，整圈返回原点；不额外做准备运动。
  void circle(double radius, const std::string& plane="xy") {
    if (!std::isfinite(radius) || radius<=0 || (plane!="xy" && plane!="xz" && plane!="yz"))
      throw std::runtime_error("Circle requires positive radius and plane xy/xz/yz");
    const Pose a=current_pose();
    const double count=std::max(72.,std::ceil(2*M_PI*radius/.001));
    if (count>100000) throw std::runtime_error("Too many circle samples");
    std::vector<Pose> points;
    for (int i=1;i<=static_cast<int>(count);++i) {
      const double t=2*M_PI*i/count, u=radius*(1-std::cos(t)), v=radius*std::sin(t);
      Pose p=a;
      if (plane=="xy") {p.x+=u; p.y+=v;}
      if (plane=="xz") {p.x+=u; p.z+=v;}
      if (plane=="yz") {p.y+=u; p.z+=v;}
      points.push_back(i==static_cast<int>(count) ? a : p);
    }
    cartesian(points);
  }

private:
  void gripper(double width, double force, bool contact) {
    if (!std::isfinite(width) || width<0 || width>.1 ||
        !std::isfinite(force) || force<.5 || force>3.)
      throw std::runtime_error("Gripper width must be 0..0.1 m; force 0.5..3.0 N");
    moveit::planning_interface::MoveGroupInterface hand(node_,side_+"_gripper");
    const auto start=feedback();
    auto current=arm_.getCurrentState(3.);
    if (!current) throw std::runtime_error("No current state");
    current->setVariablePositions(start); current->update(); hand.setStartState(*current);
    hand.setMaxVelocityScalingFactor(speed_); hand.setMaxAccelerationScalingFactor(speed_);
    if (!hand.setJointValueTarget(std::map<std::string,double>{{side_+"_gripper",width}}))
      throw std::runtime_error("Gripper width exceeds model limits");
    const auto scene=sceneSnapshot(node_);
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    if (hand.plan(plan)!=moveit::core::MoveItErrorCode::SUCCESS)
      throw std::runtime_error("Gripper planning failed; no execution");
    RCLCPP_INFO(node_->get_logger(),"Gripper target width=%.3f m, force parameter=%.2f N",width,force);
    finish(plan,start,scene,&hand,force,contact);
  }

  void cartesian(const std::vector<Pose>& points) {
    const auto start=feedback();
    auto current=arm_.getCurrentState(3.);
    if (!current) throw std::runtime_error("No current state");
    current->setVariablePositions(start); current->update(); arm_.setStartState(*current);
    std::vector<geometry_msgs::msg::Pose> waypoints;
    for (const auto& point : points) {
      for (double v : {point.x,point.y,point.z,point.roll,point.pitch,point.yaw})
        if (!std::isfinite(v)) throw std::runtime_error("Invalid waypoint");
      geometry_msgs::msg::Pose p;
      p.position.x=point.x; p.position.y=point.y; p.position.z=point.z;
      const Eigen::Quaterniond q=Eigen::AngleAxisd(point.yaw,Eigen::Vector3d::UnitZ())*
          Eigen::AngleAxisd(point.pitch,Eigen::Vector3d::UnitY())*Eigen::AngleAxisd(point.roll,Eigen::Vector3d::UnitX());
      p.orientation.x=q.x(); p.orientation.y=q.y(); p.orientation.z=q.z(); p.orientation.w=q.w();
      waypoints.push_back(p);
    }
    const auto scene=sceneSnapshot(node_);
    const auto from=current->getGlobalLinkTransform(frame_).inverse()*current->getGlobalLinkTransform(arm_.getEndEffectorLink());
    const auto& to=points.back();
    RCLCPP_INFO(node_->get_logger(),
        "Cartesian frame=%s: start=(%.4f, %.4f, %.4f), target=(%.4f, %.4f, %.4f), delta=(%.4f, %.4f, %.4f), waypoints=%zu",
        frame_.c_str(),from.translation().x(),from.translation().y(),from.translation().z(),
        to.x,to.y,to.z,to.x-from.translation().x(),to.y-from.translation().y(),to.z-from.translation().z(),points.size());
    moveit_msgs::msg::RobotTrajectory path;
    const double fraction=arm_.computeCartesianPath(waypoints,.001,5.,path,true);
    if (fraction<1.-1e-9 || path.joint_trajectory.points.size()<2)
      throw std::runtime_error("Cartesian path incomplete (fraction="+std::to_string(fraction)+"); no execution");
    robot_trajectory::RobotTrajectory timed(arm_.getRobotModel(),arm_.getName());
    timed.setRobotTrajectoryMsg(*current,path);
    trajectory_processing::IterativeParabolicTimeParameterization timing;
    if (!timing.computeTimeStamps(timed,speed_,speed_)) throw std::runtime_error("Timing failed");
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    moveit::core::robotStateToRobotStateMsg(*current,plan.start_state_);
    timed.getRobotTrajectoryMsg(plan.trajectory_);
    finish(plan,start,scene);
  }

  void finish(const moveit::planning_interface::MoveGroupInterface::Plan& plan,
              const std::map<std::string,double>& start, const std::vector<uint8_t>& scene,
              moveit::planning_interface::MoveGroupInterface* hand=nullptr,
              double force=0., bool contact=false) {
    const auto& trajectory=plan.trajectory_.joint_trajectory;
    if (trajectory.points.empty()) throw std::runtime_error("No trajectory");
    if (preview_) {
      auto display=node_->create_publisher<moveit_msgs::msg::DisplayTrajectory>(
          "/display_planned_path",rclcpp::QoS(1).reliable().transient_local());
      moveit_msgs::msg::DisplayTrajectory message;
      message.model_id=arm_.getRobotModel()->getName();
      message.trajectory_start=plan.start_state_; message.trajectory={plan.trajectory_};
      display->publish(message);
      RCLCPP_INFO(node_->get_logger(), "PREVIEW: %zu points; no motion sent.",trajectory.points.size());
      while (true) {
        std::cout << (execute_ ? "输入 r 回车重新预览，do 回车执行，其他输入取消："
                              : "仅预览：输入 r 回车重新预览，其他输入退出：") << std::endl;
        std::string answer;
        if (!std::getline(std::cin,answer)) throw TaskCancelled{};
        if (answer=="r") {
          display->publish(message); // 重播同一条已规划轨迹，不重新规划、不下发运动。
          RCLCPP_INFO(node_->get_logger(), "REPLAY: current preview republished; no motion sent.");
          continue;
        }
        if (!execute_ || answer!="do") throw TaskCancelled{};
        break;
      }
    }
    const auto now=feedback();
    for (const auto& [name,value] : start) {
      const bool linear=arm_.getRobotModel()->getJointOfVariable(name)->getType()==moveit::core::JointModel::PRISMATIC;
      if (std::abs(now.at(name)-value)>(linear ? .002 : .5*M_PI/180))
        throw std::runtime_error("Robot moved since planning; plan again");
    }
    if (scene!=sceneSnapshot(node_)) throw std::runtime_error("Scene changed; plan again");
    if (hand) {
      auto input=std::make_shared<rcl_interfaces::srv::SetParameters::Request>();
      input->parameters.push_back(rclcpp::Parameter(side_+"_gripper_force",force).to_parameter_msg());
      const auto response=request<rcl_interfaces::srv::SetParameters>(node_,"/dual_joint_bridge/set_parameters",input);
      if (response->results.size()!=1 || !response->results[0].successful)
        throw std::runtime_error("Bridge rejected gripper force; rebuild/restart nero_dual_bringup");
    }
    if ((hand ? hand->execute(plan) : arm_.execute(plan))!=moveit::core::MoveItErrorCode::SUCCESS) throw std::runtime_error("MoveIt execution failed");
    const auto deadline=std::chrono::steady_clock::now()+std::chrono::seconds(5);
    bool arrived=false;
    auto stable_since=std::chrono::steady_clock::now();
    double previous=hand ? feedback().at(side_+"_gripper") : 0.;
    while (rclcpp::ok() && std::chrono::steady_clock::now()<deadline) {
      const auto actual=feedback(); arrived=true;
      for (size_t i=0;i<trajectory.joint_names.size();++i)
        if (std::abs(actual.at(trajectory.joint_names[i])-trajectory.points.back().positions[i])>(hand ? .002 : M_PI/180)) arrived=false;
      if (hand && contact) {
        const double width=actual.at(side_+"_gripper");
        if (std::abs(width-previous)>.0005) {previous=width; stable_since=std::chrono::steady_clock::now();}
        arrived=(std::chrono::steady_clock::now()-stable_since)>=std::chrono::milliseconds(500);
      }
      if (arrived) break;
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    if (!arrived) throw std::runtime_error("Feedback did not confirm endpoint/settled gripper");
    if (hand && contact)
      RCLCPP_INFO(node_->get_logger(),"Closing command completed; opening settled at %.4f m. Object grasp NOT verified.",feedback().at(side_+"_gripper"));
    else if (hand) RCLCPP_INFO(node_->get_logger(),"Gripper opening confirmed within 2 mm.");
    else RCLCPP_INFO(node_->get_logger(), "MoveIt execution succeeded; feedback confirms endpoint within 1 degree.");
    refreshRvizQuery(node_);
  }
};

template<class Task> int run_robot(int argc,char** argv,Task task, bool preview=true, const std::string& side="right",
                                 const std::string& frame="world") {
  rclcpp::init(argc,argv);
  auto node=std::make_shared<rclcpp::Node>("point_to_point_task");
  rclcpp::executors::SingleThreadedExecutor executor; executor.add_node(node);
  std::thread spinner([&]{executor.spin();});
  int code=0;
  try { Robot robot(node,preview,side,frame); task(robot); }
  catch (const TaskCancelled&) { RCLCPP_INFO(node->get_logger(),"Task cancelled; remaining actions skipped."); }
  catch (const std::exception& error) { RCLCPP_ERROR(node->get_logger(),"%s",error.what()); code=1; }
  executor.cancel(); spinner.join(); rclcpp::shutdown(); return code;
}
} // namespace nero_task
