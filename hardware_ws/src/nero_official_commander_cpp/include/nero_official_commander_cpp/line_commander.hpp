#pragma once
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <chrono>
#include <cmath>
#include <stdexcept>
#include <thread>

// 通信细节集中在这里；先读 commander.cpp 的 A、B 和 drawLine 即可。
// 此类对应本项目仿真接收端，结果topic不是官方实机驱动接口。
class LineCommander {
public:
  using Pose = geometry_msgs::msg::Pose;
  LineCommander(const rclcpp::Node::SharedPtr& node, const std::string& arm,
                const std::string& frame)
      : node_(node), frame_(frame), tcp_(arm+"_gripper_base"), arm_(arm),
        buffer_(node->get_clock()), listener_(buffer_, node, true) {
    if (arm!="left" && arm!="right") throw std::invalid_argument("arm must be left or right");
    publisher_=node_->create_publisher<geometry_msgs::msg::PoseStamped>(
        "/"+arm+"/control/move_l",rclcpp::QoS(1).reliable());
    feedback_=node_->create_subscription<std_msgs::msg::String>(
        "/"+arm+"/control/result",10,[this](std_msgs::msg::String::SharedPtr m){result_=m->data;});
  }
  // 准备动作：先通过命名关节姿态收臂，完成后再读取直线起点A。
  void goToStart(const std::string& name, bool send) {
    if (!send) {
      RCLCPP_INFO(node_->get_logger(),"DRY RUN: would first move to %s; printed A below is current pose",name.c_str());
      return;
    }
    auto publisher=node_->create_publisher<std_msgs::msg::String>(
        "/"+arm_+"/control/named_target",rclcpp::QoS(1).reliable());
    waitForReceivers(publisher);
    std_msgs::msg::String message; message.data=name;
    sendAndWait(publisher,message);
  }
  Pose currentPose() {
    const auto deadline=std::chrono::steady_clock::now()+std::chrono::seconds(5);
    while (!buffer_.canTransform(frame_,tcp_,tf2::TimePointZero)) {
      if (!rclcpp::ok() || std::chrono::steady_clock::now()>deadline)
        throw std::runtime_error("No current TCP transform: "+frame_+" -> "+tcp_);
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    const auto transform=buffer_.lookupTransform(frame_,tcp_,tf2::TimePointZero).transform;
    Pose pose;
    pose.position.x=transform.translation.x; pose.position.y=transform.translation.y;
    pose.position.z=transform.translation.z; pose.orientation=transform.rotation;
    return pose;
  }
  void drawLine(const Pose& A, const Pose& B, bool send) {
    validate(A); validate(B);
    if (distance(A,B)<1e-6) throw std::invalid_argument("A and B are identical");
    const auto& a=A.orientation; const auto& b=B.orientation;
    if (std::abs(a.x*b.x+a.y*b.y+a.z*b.z+a.w*b.w)<1-1e-6)
      throw std::invalid_argument("This example keeps orientation fixed");
    RCLCPP_INFO(node_->get_logger(),"frame=%s, TCP=%s",frame_.c_str(),tcp_.c_str());
    RCLCPP_INFO(node_->get_logger(),"A=(%.6f, %.6f, %.6f) m",A.position.x,A.position.y,A.position.z);
    RCLCPP_INFO(node_->get_logger(),"B=(%.6f, %.6f, %.6f) m",B.position.x,B.position.y,B.position.z);
    if (!send) { RCLCPP_INFO(node_->get_logger(),"DRY RUN: no motion command published"); return; }
    waitForReceivers(publisher_);
    // move_l从实际当前位置开始。A是起点检查，不会自动移动到一个远处的A。
    const auto actual=currentPose(); const auto& q=actual.orientation;
    if (distance(actual,A)>0.002 || std::abs(q.x*a.x+q.y*a.y+q.z*a.z+q.w*a.w)<std::cos(0.01/2))
      throw std::runtime_error("TCP is not at A; move to A before drawing this line");
    geometry_msgs::msg::PoseStamped message;
    message.header.frame_id=frame_; message.header.stamp=node_->now(); message.pose=B;
    sendAndWait(publisher_,message);
  }
private:
  template<class Publisher>
  void waitForReceivers(const Publisher& publisher) {
    const auto discovery=std::chrono::steady_clock::now()+std::chrono::seconds(5);
    while (publisher->get_subscription_count()==0 || feedback_->get_publisher_count()==0) {
      if (!rclcpp::ok() || std::chrono::steady_clock::now()>discovery)
        throw std::runtime_error("Simulation command bridge is not connected");
      rclcpp::spin_some(node_);
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
  }
  template<class Publisher,class Message>
  void sendAndWait(const Publisher& publisher,const Message& message) {
    rclcpp::spin_some(node_); result_.clear();
    publisher->publish(message);
    if (!publisher->wait_for_all_acked(std::chrono::seconds(2)))
      throw std::runtime_error("Message acknowledgement unknown; do not retry automatically");
    const auto deadline=std::chrono::steady_clock::now()+std::chrono::seconds(60);
    while (rclcpp::ok()) {
      rclcpp::spin_some(node_);
      if (result_.rfind("SUCCEEDED",0)==0) {
        RCLCPP_INFO(node_->get_logger(),"%s",result_.c_str()); return;
      }
      if (result_.rfind("FAILED",0)==0 || result_.rfind("REJECTED",0)==0)
        throw std::runtime_error(result_);
      if (std::chrono::steady_clock::now()>deadline)
        throw std::runtime_error("Motion result unknown (timeout); this does not cancel motion");
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    throw std::runtime_error("Interrupted while waiting for motion result");
  }
  static double distance(const Pose& a,const Pose& b) {
    return std::hypot(std::hypot(a.position.x-b.position.x,a.position.y-b.position.y),a.position.z-b.position.z);
  }
  static void validate(const Pose& p) {
    const auto& q=p.orientation;
    for (double value:{p.position.x,p.position.y,p.position.z,q.x,q.y,q.z,q.w})
      if (!std::isfinite(value)) throw std::invalid_argument("Pose contains NaN/Inf");
    if (std::abs(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w-1)>1e-6)
      throw std::invalid_argument("Quaternion must be normalized");
  }
  rclcpp::Node::SharedPtr node_;
  std::string frame_,tcp_,arm_,result_;
  tf2_ros::Buffer buffer_; tf2_ros::TransformListener listener_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr feedback_;
};
