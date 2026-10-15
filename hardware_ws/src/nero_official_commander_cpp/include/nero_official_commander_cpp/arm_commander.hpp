#pragma once
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <array>
#include <chrono>
#include <cmath>
#include <stdexcept>
#include <thread>

// ROS is the language boundary: C++ messages -> Python official ROS driver -> SDK.
// Simulation consumes the same messages through official_command_bridge.
class ArmCommander {
public:
  using Pose=geometry_msgs::msg::Pose;
  using Pose6=std::array<double,6>; // x,y,z [m], roll,pitch,yaw [rad]
  enum class Backend { Simulation, OfficialDriver };
  enum class Receipt { DryRun, CompletedInSimulation, PublishedToDriver };

  ArmCommander(const rclcpp::Node::SharedPtr& node,const std::string& arm,
      const std::string& reference_frame,bool send,Backend backend=Backend::Simulation)
    : node_(node),prefix_("/"+arm),reference_(reference_frame),control_(arm+"_arm_base"),
      tcp_(arm+"_gripper_base"),send_(send),backend_(backend),
      buffer_(node->get_clock()),listener_(buffer_,node,true) {
    if(arm!="left" && arm!="right") throw std::invalid_argument("arm must be left or right");
    if(reference_.empty()) throw std::invalid_argument("Reference frame is required");
    if(backend_==Backend::Simulation)
      feedback_=node_->create_subscription<std_msgs::msg::String>(prefix_+"/control/result",10,
          [this](std_msgs::msg::String::SharedPtr m){result_=m->data;});
  }

  // These three frames/names must match the deployed official driver/model.
  // Its callback ignores frame_id, so poses are always transformed to control_.
  void setFrames(const std::string& control,const std::string& tcp) {
    if(control.empty() || tcp.empty()) throw std::invalid_argument("Frames must not be empty");
    control_=control; tcp_=tcp;
  }
  Pose currentPose() {
    const auto t=lookup(reference_,tcp_).transform;
    Pose p; p.position.x=t.translation.x; p.position.y=t.translation.y;
    p.position.z=t.translation.z; p.orientation=t.rotation; return p;
  }
  Receipt move_j(const std::array<double,7>& joints) {
    sensor_msgs::msg::JointState m; m.header.stamp=node_->now();
    for(std::size_t i=0;i<joints.size();++i) {
      if(!std::isfinite(joints[i])) throw std::invalid_argument("Non-finite joint target");
      m.name.push_back("joint"+std::to_string(i+1)); m.position.push_back(joints[i]);
    }
    return publish("move_j",m);
  }
  // Total jaw opening [m], using the official move_j gripper-only schema.
  Receipt gripper(double width,double force=1.0) {
    if(!std::isfinite(width) || width<0 || width>.10 || !std::isfinite(force) || force<=0)
      throw std::invalid_argument("Invalid gripper width/force");
    sensor_msgs::msg::JointState m;m.header.stamp=node_->now();
    m.name={"gripper"};m.position={width};m.effort={force};return publish("move_j",m);
  }
  Receipt move_p(const Pose& target) { return publishPose("move_p",target); }
  Receipt move_l(const Pose& target) { return publishPose("move_l",target); }
  Receipt move_p(const Pose6& target) { return move_p(fromRPY(target)); }
  Receipt move_l(const Pose6& target) { return move_l(fromRPY(target)); }
  Receipt move_c(const Pose& start,const Pose& middle,const Pose& end) {
    validate(start); validate(middle); validate(end);
    const double ux=middle.position.x-start.position.x, uy=middle.position.y-start.position.y,
                 uz=middle.position.z-start.position.z;
    const double vx=end.position.x-start.position.x, vy=end.position.y-start.position.y,
                 vz=end.position.z-start.position.z;
    const double scale=std::hypot(std::hypot(ux,uy),uz)*std::hypot(std::hypot(vx,vy),vz);
    const double cross=std::hypot(std::hypot(uy*vz-uz*vy,uz*vx-ux*vz),ux*vy-uy*vx);
    if(scale<1e-12 || cross/scale<1e-6) throw std::invalid_argument("Arc needs three distinct non-collinear points");
    geometry_msgs::msg::PoseArray m; m.header.frame_id=control_; m.header.stamp=node_->now();
    m.poses={toControl(start),toControl(middle),toControl(end)}; return publish("move_c",m);
  }
  Receipt move_c(const Pose6& a,const Pose6& b,const Pose6& c) {
    return move_c(fromRPY(a),fromRPY(b),fromRPY(c));
  }
private:
  geometry_msgs::msg::TransformStamped lookup(const std::string& target,const std::string& source) {
    const auto deadline=std::chrono::steady_clock::now()+std::chrono::seconds(5);
    while(!buffer_.canTransform(target,source,tf2::TimePointZero)) {
      if(!rclcpp::ok() || std::chrono::steady_clock::now()>deadline)
        throw std::runtime_error("Missing TF: "+target+" <- "+source);
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    return buffer_.lookupTransform(target,source,tf2::TimePointZero);
  }
  static void validate(const Pose& p) {
    const auto& q=p.orientation;
    for(double v:{p.position.x,p.position.y,p.position.z,q.x,q.y,q.z,q.w})
      if(!std::isfinite(v)) throw std::invalid_argument("Non-finite pose");
    if(std::abs(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w-1)>1e-6)
      throw std::invalid_argument("Quaternion must be normalized");
  }
  static Pose fromRPY(const Pose6& a) {
    for(double v:a) if(!std::isfinite(v)) throw std::invalid_argument("Non-finite pose");
    const double cr=std::cos(a[3]/2),sr=std::sin(a[3]/2),cp=std::cos(a[4]/2),
                 sp=std::sin(a[4]/2),cy=std::cos(a[5]/2),sy=std::sin(a[5]/2);
    Pose p; p.position.x=a[0];p.position.y=a[1];p.position.z=a[2];
    p.orientation.x=sr*cp*cy-cr*sp*sy; p.orientation.y=cr*sp*cy+sr*cp*sy;
    p.orientation.z=cr*cp*sy-sr*sp*cy; p.orientation.w=cr*cp*cy+sr*sp*sy;
    return p;
  }
  Pose toControl(const Pose& p) {
    validate(p);
    if(reference_==control_) return p;
    const auto t=lookup(control_,reference_).transform;
    // Quaternion rotation of position, followed by translation.
    const auto& r=t.rotation;
    const double vx=p.position.x,vy=p.position.y,vz=p.position.z;
    const double tx=2*(r.y*vz-r.z*vy),ty=2*(r.z*vx-r.x*vz),tz=2*(r.x*vy-r.y*vx);
    Pose out;
    out.position.x=vx+r.w*tx+r.y*tz-r.z*ty+t.translation.x;
    out.position.y=vy+r.w*ty+r.z*tx-r.x*tz+t.translation.y;
    out.position.z=vz+r.w*tz+r.x*ty-r.y*tx+t.translation.z;
    const auto& q=p.orientation;
    out.orientation.x=r.w*q.x+r.x*q.w+r.y*q.z-r.z*q.y;
    out.orientation.y=r.w*q.y-r.x*q.z+r.y*q.w+r.z*q.x;
    out.orientation.z=r.w*q.z+r.x*q.y-r.y*q.x+r.z*q.w;
    out.orientation.w=r.w*q.w-r.x*q.x-r.y*q.y-r.z*q.z;
    validate(out); return out;
  }
  Receipt publishPose(const std::string& command,const Pose& target) {
    geometry_msgs::msg::PoseStamped m;
    m.header.frame_id=control_;m.header.stamp=node_->now();m.pose=toControl(target);
    return publish(command,m);
  }
  template<class Message> Receipt publish(const std::string& command,const Message& message) {
    if(!send_) {
      RCLCPP_INFO(node_->get_logger(),"DRY RUN %s: no command published",command.c_str());
      return Receipt::DryRun;
    }
    auto publisher=node_->create_publisher<Message>(prefix_+"/control/"+command,rclcpp::QoS(1).reliable());
    const auto discovery=std::chrono::steady_clock::now()+std::chrono::seconds(5);
    while(publisher->get_subscription_count()==0 ||
          (feedback_ && feedback_->get_publisher_count()==0)) {
      if(!rclcpp::ok() || std::chrono::steady_clock::now()>discovery)
        throw std::runtime_error("Command receiver not connected: "+command);
      rclcpp::spin_some(node_);std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    rclcpp::spin_some(node_);result_.clear();publisher->publish(message);
    if(!publisher->wait_for_all_acked(std::chrono::seconds(2)))
      throw std::runtime_error("Delivery unknown; do not retry automatically");
    if(backend_==Backend::OfficialDriver) {
      RCLCPP_INFO(node_->get_logger(),"Published %s to official driver; completion NOT verified",command.c_str());
      return Receipt::PublishedToDriver;
    }
    const auto deadline=std::chrono::steady_clock::now()+std::chrono::seconds(60);
    while(rclcpp::ok()) {
      rclcpp::spin_some(node_);
      if(result_.rfind("SUCCEEDED",0)==0) {
        RCLCPP_INFO(node_->get_logger(),"%s",result_.c_str());return Receipt::CompletedInSimulation;
      }
      if(result_.rfind("FAILED",0)==0 || result_.rfind("REJECTED",0)==0) throw std::runtime_error(result_);
      if(std::chrono::steady_clock::now()>deadline)
        throw std::runtime_error("Result unknown (timeout); motion is not automatically cancelled");
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    throw std::runtime_error("Interrupted");
  }
  rclcpp::Node::SharedPtr node_;
  std::string prefix_,reference_,control_,tcp_,result_;
  bool send_; Backend backend_;
  tf2_ros::Buffer buffer_;tf2_ros::TransformListener listener_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr feedback_;
};
