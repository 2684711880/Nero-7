// Simulation-only base Action server. No blocking waits and no idle cmd_vel stream.
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nero_task_interfaces/action/execute_base_motion.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <yaml-cpp/yaml.h>
#include <nero_task_executor/base_pose_controller.hpp>
#include <chrono>
#include <memory>
#include <set>
#include <string>

using Action = nero_task_interfaces::action::ExecuteBaseMotion;
using Goal = rclcpp_action::ServerGoalHandle<Action>;
using Clock = std::chrono::steady_clock;
using namespace std::chrono_literals;
using nero_base::Pose;
using nero_base::Velocity;
using nero_base::Limits;

struct Request {
  nero_base::Mode mode;
  Pose requested;
  Limits limits;
  double duration=0, timeout=0;
};
static double numeric(const YAML::Node& node) {
  const auto text=node.Scalar();
  if(text=="true" || text=="false")throw std::invalid_argument("Boolean is not numeric");
  const double value=node.as<double>();
  if(!std::isfinite(value))throw std::invalid_argument("Non-finite parameter");
  return value;
}
static Request parse(const std::string& data) {
  if(data.size()>65536)throw std::invalid_argument("Command exceeds 64 KiB");
  const auto a=YAML::Load(data),p=a["parameters"];
  if(!a.IsMap() || !p.IsMap())throw std::invalid_argument("Command and parameters must be objects");
  const auto type=a["type"].as<std::string>();
  Request r;
  if(type=="base_translate")r.mode=nero_base::Mode::Translate;
  else if(type=="base_rotate")r.mode=nero_base::Mode::Rotate;
  else if(type=="base_move_and_rotate")r.mode=nero_base::Mode::Combined;
  else throw std::invalid_argument("Unsupported base action");
  if(p["frame_id"].as<std::string>()!="odom")throw std::invalid_argument("Only odom is supported");
  std::set<std::string> keys={"frame_id","max_linear_speed_m_s","max_angular_speed_rad_s",
    "linear_acceleration_m_s2","angular_acceleration_rad_s2","duration_s"};
  if(r.mode!=nero_base::Mode::Rotate) {
    keys.insert("position_xy_m");const auto xy=p["position_xy_m"];
    if(!xy.IsSequence() || xy.size()!=2)throw std::invalid_argument("Expected two target coordinates");
    r.requested.x=numeric(xy[0]);r.requested.y=numeric(xy[1]);
  }
  if(r.mode!=nero_base::Mode::Translate){keys.insert("yaw_rad");r.requested.yaw=numeric(p["yaw_rad"]);}
  for(auto entry:p)if(!keys.count(entry.first.as<std::string>()))throw std::invalid_argument("Unknown base parameter");
  if(p["max_linear_speed_m_s"])r.limits.linear=numeric(p["max_linear_speed_m_s"]);
  if(p["max_angular_speed_rad_s"])r.limits.angular=numeric(p["max_angular_speed_rad_s"]);
  if(p["linear_acceleration_m_s2"])r.limits.linear_acceleration=numeric(p["linear_acceleration_m_s2"]);
  if(p["angular_acceleration_rad_s2"])r.limits.angular_acceleration=numeric(p["angular_acceleration_rad_s2"]);
  if(p["duration_s"]){r.duration=numeric(p["duration_s"]);if(r.duration<=0)throw std::invalid_argument("Invalid duration");}
  r.timeout=numeric(a["timeout_s"]);
  if(r.timeout<=0 || r.timeout>86400 || r.duration>=r.timeout)throw std::invalid_argument("Invalid timeout/duration");
  r.limits.validate();return r;
}

class BaseServer : public rclcpp::Node {
public:
  BaseServer():Node("sim_base_server") {
    if(!get_parameter("use_sim_time").as_bool())throw std::runtime_error("Simulation time required");
    velocity_=create_publisher<geometry_msgs::msg::Twist>("/cmd_vel",10);
    odom_=create_subscription<nav_msgs::msg::Odometry>("/odom",rclcpp::SensorDataQoS(),
      [this](nav_msgs::msg::Odometry::SharedPtr message){observe(*message);});
    server_=rclcpp_action::create_server<Action>(this,"/task_motion/base",
      [this](const rclcpp_action::GoalUUID&,std::shared_ptr<const Action::Goal> goal) {
        if(goal_ || reserved_ || latched_)return rclcpp_action::GoalResponse::REJECT;
        try {
          if(!fresh())throw std::runtime_error("Missing or stale odometry");
          if(velocity_->get_subscription_count()==0)throw std::runtime_error("No cmd_vel receiver");
          const auto r=parse(goal->command_json);
          const nero_base::Trajectory check(r.mode,pose_,r.requested,r.limits,r.duration);
          if(check.duration()>=r.timeout)throw std::runtime_error("Timeout is shorter than required trajectory duration");
        } catch(const std::exception& e) {
          RCLCPP_WARN(get_logger(),"Rejected base goal: %s",e.what());return rclcpp_action::GoalResponse::REJECT;
        }
        reserved_=true;return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      },
      [](std::shared_ptr<Goal>){return rclcpp_action::CancelResponse::ACCEPT;},
      [this](std::shared_ptr<Goal> goal){begin(goal);});
    timer_=create_wall_timer(20ms,[this]{tick();});
    RCLCPP_INFO(get_logger(),"READY: /task_motion/base, translate / rotate / combined");
  }
  ~BaseServer() {if(rclcpp::ok())publish({});}
private:
  void observe(const nav_msgs::msg::Odometry& m) {
    if(m.header.frame_id!="odom" || m.child_frame_id!="base_link")return;
    const auto& p=m.pose.pose;const auto& q=p.orientation;const auto& v=m.twist.twist;
    for(double x:{p.position.x,p.position.y,q.x,q.y,q.z,q.w,v.linear.x,v.linear.y,v.angular.z})
      if(!std::isfinite(x))return;
    if(std::abs(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w-1)>.0001)return;
    const int64_t stamp=rclcpp::Time(m.header.stamp).nanoseconds();
    const Pose next={p.position.x,p.position.y,std::atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))};
    if(stamp_ && stamp<stamp_){clock_reversed_=true;return;}
    if(stamp>stamp_) {
      const double dt=(stamp-stamp_)*1e-9;
      if(stamp_ && dt>0 && dt<.5) {
        actual_speed_=std::hypot(next.x-pose_.x,next.y-pose_.y)/dt;
        actual_yaw_rate_=std::abs(nero_base::wrap(next.yaw-pose_.yaw))/dt;
      } else {actual_speed_=actual_yaw_rate_=1.;}
      last_advance_=Clock::now();
    }
    stamp_=stamp;pose_=next;wheel_speed_=std::hypot(v.linear.x,v.linear.y);
    wheel_yaw_rate_=std::abs(v.angular.z);last_received_=Clock::now();
  }
  bool fresh() {
    if(stamp_==0 || clock_reversed_)return false;
    const auto wall=Clock::now();const double age=(get_clock()->now().nanoseconds()-stamp_)*1e-9;
    return wall-last_received_<500ms && wall-last_advance_<500ms && age>=-.1 && age<=.5;
  }
  bool stopped() {
    return fresh() && wheel_speed_<.015 && wheel_yaw_rate_<.02 && actual_speed_<.015 && actual_yaw_rate_<.02;
  }
  void begin(std::shared_ptr<Goal> goal) {
    reserved_=false;goal_=goal;
    try {
      if(!fresh())throw std::runtime_error("Odometry became stale before start");
      request_=parse(goal->get_goal()->command_json);
      trajectory_=std::make_unique<nero_base::Trajectory>(request_.mode,pose_,request_.requested,request_.limits,request_.duration);
      start_ros_=last_control_ros_=get_clock()->now().nanoseconds();start_wall_=last_tick_=last_progress_=Clock::now();
      progress_pose_=pose_;last_command_={};phase_=Phase::Tracking;settled_=false;
      RCLCPP_INFO(get_logger(),"Base goal started, reference duration %.3f s",trajectory_->duration());
    } catch(const std::exception& e) {stop(Phase::Failed,e.what());}
  }
  enum class Phase { Tracking, Success, Canceled, Failed };
  void publish(Velocity command) {
    geometry_msgs::msg::Twist message;
    message.linear.x=command.x;message.linear.y=command.y;message.angular.z=command.yaw;
    velocity_->publish(message);
  }
  void stop(Phase terminal,const std::string& message) {
    phase_=terminal;message_=message;stop_started_=Clock::now();settled_=false;last_command_={};publish({});
  }
  void finish(bool confirmed) {
    auto result=std::make_shared<Action::Result>();
    result->success=confirmed && phase_==Phase::Success;result->stop_confirmed=confirmed;
    result->message=confirmed?message_:"STOP_UNCONFIRMED: "+message_;
    result->final_pose={pose_.x,pose_.y,pose_.yaw};
    auto handle=goal_;goal_.reset();trajectory_.reset();reserved_=false;latched_=!confirmed;
    if(handle->is_canceling())handle->canceled(result);
    else if(result->success)handle->succeed(result);else handle->abort(result);
    RCLCPP_INFO(get_logger(),"Base terminal: %s",result->message.c_str());
  }
  void tick() {
    if(!goal_){if(latched_)publish({});return;}
    const auto wall=Clock::now();
    if(goal_->is_canceling() && phase_!=Phase::Canceled)stop(Phase::Canceled,"Base canceled; stop verified");
    if(phase_==Phase::Tracking) {
      if(!fresh()){stop(Phase::Failed,"Missing/stale odometry or clock discontinuity");return;}
      if(wall-start_wall_>std::chrono::duration<double>(request_.timeout)) {
        stop(Phase::Failed,"Base action timeout");return;
      }
      const double elapsed=(get_clock()->now().nanoseconds()-start_ros_)*1e-9;
      if(elapsed<0){stop(Phase::Failed,"Simulation clock reversed");return;}
      const auto reference=trajectory_->sample(elapsed);
      if(reference.finished && nero_base::inside(pose_,trajectory_->goal())) {
        stop(Phase::Success,"Base target and stopped velocity verified");return;
      }
      if(std::hypot(pose_.x-progress_pose_.x,pose_.y-progress_pose_.y)>.003 ||
          std::abs(nero_base::wrap(pose_.yaw-progress_pose_.yaw))>.01) {
        last_progress_=wall;progress_pose_=pose_;
      } else if(wall-last_progress_>12s && !nero_base::inside(pose_,trajectory_->goal(),.03,.04)) {
        stop(Phase::Failed,"BASE_STALLED: no measured progress for 12 seconds");return;
      }
      const auto control_ros=get_clock()->now().nanoseconds();
      const double dt=(control_ros-last_control_ros_)*1e-9;last_control_ros_=control_ros;last_tick_=wall;
      last_command_=nero_base::slew(last_command_,nero_base::feedback(reference,pose_,request_.limits),request_.limits,dt);
      publish(last_command_);
      if(wall-last_feedback_>100ms) {
        auto feedback=std::make_shared<Action::Feedback>();feedback->phase=reference.finished?"DOCKING":"TRACKING";
        feedback->progress=reference.progress;
        feedback->reference_pose={reference.pose.x,reference.pose.y,reference.pose.yaw};
        feedback->measured_pose={pose_.x,pose_.y,pose_.yaw};goal_->publish_feedback(feedback);last_feedback_=wall;
      }
      return;
    }
    publish({});
    if(phase_==Phase::Success && fresh() && !nero_base::inside(pose_,trajectory_->goal(),.025,.03)) {
      // Braking drift left the wider tolerance: correct to the same final goal.
      phase_=Phase::Tracking;settled_=false;last_tick_=wall;last_control_ros_=get_clock()->now().nanoseconds();return;
    }
    if(stopped()) {
      if(!settled_){settled_=true;settled_since_=wall;}
      if(wall-settled_since_>=400ms && (phase_!=Phase::Success || nero_base::inside(pose_,trajectory_->goal(),.025,.03))) {
        finish(true);return;
      }
    } else settled_=false;
    if(wall-stop_started_>5s)finish(false);
  }
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_;
  rclcpp_action::Server<Action>::SharedPtr server_;rclcpp::TimerBase::SharedPtr timer_;
  std::shared_ptr<Goal> goal_;std::unique_ptr<nero_base::Trajectory> trajectory_;
  Request request_{};Phase phase_=Phase::Tracking;std::string message_;
  Pose pose_,progress_pose_;Velocity last_command_;
  int64_t stamp_=0,start_ros_=0,last_control_ros_=0;
  double wheel_speed_=0,wheel_yaw_rate_=0,actual_speed_=1,actual_yaw_rate_=1;
  bool reserved_=false,latched_=false,clock_reversed_=false,settled_=false;
  Clock::time_point last_received_{},last_advance_{},start_wall_{},last_tick_{},last_feedback_{},
      stop_started_{},settled_since_{},last_progress_{};
};

int main(int argc,char** argv) {
  rclcpp::init(argc,argv);
  try {auto node=std::make_shared<BaseServer>();rclcpp::spin(node);}
  catch(const std::exception& e){std::fprintf(stderr,"sim_base_server: %s\n",e.what());rclcpp::shutdown();return 1;}
  rclcpp::shutdown();return 0;
}
