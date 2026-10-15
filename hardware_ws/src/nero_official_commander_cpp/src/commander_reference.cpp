#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <std_msgs/msg/string.hpp>
#include <chrono>
#include <cmath>
#include <stdexcept>
#include <thread>
#include <vector>

using Pose = geometry_msgs::msg::Pose;

// Official ROS inputs use metres and normalized quaternions, not SDK RPY arrays.
Pose readPose(const std::vector<double>& values, std::size_t offset) {
  Pose pose;
  for (std::size_t i = offset; i < offset + 7; ++i)
    if (!std::isfinite(values.at(i))) throw std::invalid_argument("Pose contains NaN/Inf");
  pose.position.x = values[offset]; pose.position.y = values[offset+1];
  pose.position.z = values[offset+2];
  pose.orientation.x = values[offset+3]; pose.orientation.y = values[offset+4];
  pose.orientation.z = values[offset+5]; pose.orientation.w = values[offset+6];
  double norm = 0.0;
  for (std::size_t i = offset+3; i < offset+7; ++i) norm += values[i]*values[i];
  if (std::abs(norm-1.0) > 1e-6)
    throw std::invalid_argument("Pose quaternion must be normalized");
  return pose;
}

template <class Message>
void publishCommand(const rclcpp::Node::SharedPtr& node, const std::string& topic,
                    const Message& message, bool send) {
  const auto resolved = node->get_node_topics_interface()->resolve_topic_name(topic);
  if (!send) {
    RCLCPP_INFO(node->get_logger(), "DRY RUN: validated command for %s; nothing published", resolved.c_str());
    return;
  }
  if (std::string(node->get_namespace()) == "/")
    throw std::invalid_argument("Select an arm namespace explicitly before sending");
  std::string result;
  const bool wait_result=node->get_parameter("wait_result").as_bool();
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr feedback;
  if(wait_result) feedback=node->create_subscription<std_msgs::msg::String>("control/result",10,
      [&result](std_msgs::msg::String::SharedPtr m){result=m->data;});
  auto publisher = node->create_publisher<Message>(topic, rclcpp::QoS(1).reliable());
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
  while (rclcpp::ok() && (publisher->get_subscription_count() == 0 ||
                        (wait_result && feedback->get_publisher_count() == 0))) {
    if (std::chrono::steady_clock::now() >= deadline)
      throw std::runtime_error("No subscriber for " + resolved);
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  if (!rclcpp::ok()) throw std::runtime_error("Interrupted before send");
  publisher->publish(message);
  if (!publisher->wait_for_all_acked(std::chrono::seconds(2)))
    throw std::runtime_error("DDS acknowledgement timeout; do not automatically retry motion");
  RCLCPP_INFO(node->get_logger(), "Published once to %s; this is NOT motion-completion feedback", resolved.c_str());
  if(wait_result) {
    const auto finish=std::chrono::steady_clock::now()+std::chrono::seconds(60);
    while(rclcpp::ok()) {
      rclcpp::spin_some(node);
      if(result.rfind("SUCCEEDED",0)==0) { RCLCPP_INFO(node->get_logger(),"%s",result.c_str()); return; }
      if(result.rfind("FAILED",0)==0 || result.rfind("REJECTED",0)==0) throw std::runtime_error(result);
      if(std::chrono::steady_clock::now()>finish)
        throw std::runtime_error("Result timeout; command may still be running, do not retry automatically");
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    throw std::runtime_error("Interrupted while waiting for result");
  }
}

// Match the official _move_l_callback input exactly. The upstream callback does
// not transform header.frame_id: callers must supply coordinates in arm base frame.
void moveLine(const rclcpp::Node::SharedPtr& node, const Pose& target, bool send) {
  geometry_msgs::msg::PoseStamped message;
  message.header.stamp = node->now();
  message.header.frame_id = node->get_parameter("frame_id").as_string();
  message.pose = target;
  publishCommand(node, "control/move_l", message, send);
}

void moveArc(const rclcpp::Node::SharedPtr& node, const std::vector<Pose>& points, bool send) {
  const auto& a = points.at(0).position;
  const auto& b = points.at(1).position;
  const auto& c = points.at(2).position;
  const double ux=b.x-a.x, uy=b.y-a.y, uz=b.z-a.z;
  const double vx=c.x-a.x, vy=c.y-a.y, vz=c.z-a.z;
  const double cross=std::hypot(std::hypot(uy*vz-uz*vy, uz*vx-ux*vz), ux*vy-uy*vx);
  const double scale=std::hypot(std::hypot(ux,uy),uz)*std::hypot(std::hypot(vx,vy),vz);
  if (scale < 1e-12 || cross/scale < 1e-6)
    throw std::invalid_argument("Arc needs three distinct, non-collinear points; not a closed full circle");
  geometry_msgs::msg::PoseArray message;
  message.header.stamp = node->now();
  message.header.frame_id = node->get_parameter("frame_id").as_string();
  message.poses = points;
  publishCommand(node, "control/move_c", message, send);
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("official_commander");
  node->declare_parameter("motion", "line");
  node->declare_parameter("target", "ready");
  node->declare_parameter("send", false);
  node->declare_parameter("wait_result", false);  // simulation bridge extension
  node->declare_parameter("frame_id", "base_link");
  node->declare_parameter<std::vector<double>>("poses", std::vector<double>{});
  node->declare_parameter("relative", false);
  node->declare_parameter("tcp_link", "");
  node->declare_parameter<std::vector<double>>("offsets", std::vector<double>{});
  int result = 0;
  try {
    const auto motion = node->get_parameter("motion").as_string();
    auto values = node->get_parameter("poses").as_double_array();
    const bool send = node->get_parameter("send").as_bool();
    if (node->get_parameter("relative").as_bool()) {
      if (!values.empty()) throw std::invalid_argument("Use poses or relative offsets, not both");
      const auto offsets=node->get_parameter("offsets").as_double_array();
      if (offsets.size()!=(motion=="line"?3u:9u))
        throw std::invalid_argument("Relative line needs 3 offsets; arc needs 9");
      const auto tip=node->get_parameter("tcp_link").as_string();
      if(tip.empty()) throw std::invalid_argument("relative requires tcp_link");
      tf2_ros::Buffer buffer(node->get_clock());
      tf2_ros::TransformListener listener(buffer,node,true);
      const auto deadline=std::chrono::steady_clock::now()+std::chrono::seconds(5);
      const auto frame=node->get_parameter("frame_id").as_string();
      while(!buffer.canTransform(frame,tip,tf2::TimePointZero)) {
        if(std::chrono::steady_clock::now()>deadline) throw std::runtime_error("No current TCP transform");
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
      }
      const auto t=buffer.lookupTransform(frame,tip,tf2::TimePointZero).transform;
      for(std::size_t i=0;i<offsets.size();i+=3) {
        values.insert(values.end(),{t.translation.x+offsets[i],t.translation.y+offsets[i+1],
          t.translation.z+offsets[i+2],t.rotation.x,t.rotation.y,t.rotation.z,t.rotation.w});
      }
    }
    if (motion == "named") {
      std_msgs::msg::String message;
      message.data=node->get_parameter("target").as_string();
      publishCommand(node,"control/named_target",message,send);
    } else if (motion == "line") {
      if (values.size() != 7) throw std::invalid_argument("line needs poses=[x,y,z,qx,qy,qz,qw]");
      moveLine(node, readPose(values, 0), send);
    } else if (motion == "arc") {
      if (values.size() != 21) throw std::invalid_argument("arc needs three concatenated 7-value poses");
      moveArc(node, {readPose(values,0), readPose(values,7), readPose(values,14)}, send);
    } else throw std::invalid_argument("motion must be line, arc or named (simulation extension)");
  } catch (const std::exception& error) {
    RCLCPP_ERROR(node->get_logger(), "%s", error.what());
    result = 1;
  }
  rclcpp::shutdown();
  return result;
}
