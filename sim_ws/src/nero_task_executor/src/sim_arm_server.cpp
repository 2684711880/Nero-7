// Simulation-only correlated arm adapter. Each arm has its own process and controller.
// Plan with MoveIt; execute through that arm's FollowJointTrajectory action so the
// shared MoveIt ExecuteTrajectory server cannot preempt the other arm's execution.
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nero_task_interfaces/action/execute_motion.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <sensor_msgs/msg/joint_state.hpp>
#include <yaml-cpp/yaml.h>
#include <atomic>
#include <condition_variable>
#include <thread>
#include <mutex>
#include <cmath>

using Motion = nero_task_interfaces::action::ExecuteMotion;
using Goal = rclcpp_action::ServerGoalHandle<Motion>;
using Follow = control_msgs::action::FollowJointTrajectory;
using Group = moveit::planning_interface::MoveGroupInterface;
using Pose = geometry_msgs::msg::Pose;
using namespace std::chrono_literals;

static double finite(const YAML::Node& n) {
  const double v=n.as<double>();
  if(!std::isfinite(v)) throw std::runtime_error("Non-finite target");
  return v;
}
static Eigen::Vector3d xyz(const Pose& p) { return {p.position.x,p.position.y,p.position.z}; }
static Eigen::Quaterniond rotation(const Pose& p) {return {p.orientation.w,p.orientation.x,p.orientation.y,p.orientation.z};}
static Pose makePose(const Eigen::Vector3d& v,const Eigen::Quaterniond& q) {
  Pose p;p.position.x=v.x();p.position.y=v.y();p.position.z=v.z();
  p.orientation.x=q.x();p.orientation.y=q.y();p.orientation.z=q.z();p.orientation.w=q.w();return p;
}

class ArmServer {
public:
  explicit ArmServer(rclcpp::Node::SharedPtr node):node_(node),buffer_(node->get_clock()),listener_(buffer_,node,true) {
    if(!node_->get_parameter("use_sim_time").as_bool()) throw std::runtime_error("Simulation time required");
    if(!node_->has_parameter("side"))node_->declare_parameter<std::string>("side","right");
    side_=node_->get_parameter("side").as_string();
    if(side_!="left" && side_!="right") throw std::runtime_error("Invalid side");
    joints_=node_->create_subscription<sensor_msgs::msg::JointState>("/joint_states",rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::JointState::SharedPtr m) {
        if(m->name.size()!=m->position.size())return;
        for(const auto& suffix:{"_joint1","_joint2","_joint3","_joint4","_joint5","_joint6","_joint7","_gripper_joint1","_gripper_joint2"}) {
          auto it=std::find(m->name.begin(),m->name.end(),side_+suffix);
          if(it==m->name.end() || !std::isfinite(m->position[it-m->name.begin()])) return;
        }
        feedback_stamp_=rclcpp::Time(m->header.stamp).nanoseconds();
      });
    arm_client_=rclcpp_action::create_client<Follow>(node_,"/"+side_+"_arm_controller/follow_joint_trajectory");
    gripper_client_=rclcpp_action::create_client<Follow>(node_,"/"+side_+"_gripper_controller/follow_joint_trajectory");
    server_=rclcpp_action::create_server<Motion>(node_,"/task_motion/"+side_+"_arm",
      [this](const rclcpp_action::GoalUUID&,std::shared_ptr<const Motion::Goal> g) {
        if(g->command_json.size()>65536)return rclcpp_action::GoalResponse::REJECT;
        try {auto a=YAML::Load(g->command_json);if(!a["type"] || !a["parameters"])return rclcpp_action::GoalResponse::REJECT;}
        catch(...) {return rclcpp_action::GoalResponse::REJECT;}
        bool expected=false;
        if(!busy_.compare_exchange_strong(expected,true))return rclcpp_action::GoalResponse::REJECT;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      },
      [](std::shared_ptr<Goal>) {return rclcpp_action::CancelResponse::ACCEPT;},
      [this](std::shared_ptr<Goal> goal) {
        {std::lock_guard<std::mutex> lock(mutex_);pending_=goal;}
        cv_.notify_one();
      });
    worker_=std::thread([this]{work();});
  }
  ~ArmServer() {
    stopping_=true;cv_.notify_one();if(worker_.joinable())worker_.join();
  }
private:
  void check(const std::shared_ptr<Goal>& goal) {
    if(std::chrono::steady_clock::now()>deadline_)throw std::runtime_error("Motion execution timeout");
    if(stopping_ || !rclcpp::ok() || goal->is_canceling())throw std::runtime_error("Canceled before execution");
    const double age=(node_->now().nanoseconds()-feedback_stamp_.load())*1e-9;
    if(feedback_stamp_==0 || age<-.1 || age>.5)throw std::runtime_error("Missing/stale joint feedback");
  }
  moveit::core::RobotStatePtr state(Group& group,const std::shared_ptr<Goal>& goal) {
    check(goal);
    auto s=group.getCurrentState(3);
    if(!s)throw std::runtime_error("Missing robot state");
    const auto b=buffer_.lookupTransform(group.getPlanningFrame(),"base_link",tf2::TimePointZero,tf2::durationFromSec(2));
    const double age=(node_->now()-rclcpp::Time(b.header.stamp)).seconds();
    if(age<-.1 || age>.5)throw std::runtime_error("Stale base TF");
    const auto& t=b.transform;
    Eigen::Isometry3d base=Eigen::Isometry3d::Identity();
    base.translation()=Eigen::Vector3d(t.translation.x,t.translation.y,t.translation.z);
    base.linear()=Eigen::Quaterniond(t.rotation.w,t.rotation.x,t.rotation.y,t.rotation.z).toRotationMatrix();
    s->setJointPositions("mobile_base_joint",base);s->update();
    moveit_msgs::msg::RobotState message;moveit::core::robotStateToRobotStateMsg(*s,message);
    message.is_diff=true;group.setStartState(message);return s;
  }
  Pose target(const YAML::Node& p,const std::string& model) {
    if(p["tcp_id"].as<std::string>()!="default_tcp")throw std::runtime_error("Unknown TCP");
    const auto v=p["position_m"],q=p["orientation_xyzw"];
    if(v.size()!=3 || q.size()!=4)throw std::runtime_error("Invalid pose dimensions");
    Eigen::Vector3d x(finite(v[0]),finite(v[1]),finite(v[2]));
    Eigen::Quaterniond r(finite(q[3]),finite(q[0]),finite(q[1]),finite(q[2]));
    if(std::abs(r.squaredNorm()-1)>1e-6)throw std::runtime_error("Quaternion must be normalized");
    const auto source=p["frame_id"].as<std::string>();
    if(source.empty())throw std::runtime_error("Empty frame");
    if(source!=model) {
      const auto tf=buffer_.lookupTransform(model,source,tf2::TimePointZero,tf2::durationFromSec(2)).transform;
      Eigen::Quaterniond rq(tf.rotation.w,tf.rotation.x,tf.rotation.y,tf.rotation.z);
      x=rq*x+Eigen::Vector3d(tf.translation.x,tf.translation.y,tf.translation.z);r=rq*r;
    }
    return makePose(x,r);
  }
  std::vector<Pose> arc(const std::vector<Pose>& p) {
    const auto a=xyz(p[0]),b=xyz(p[1]),c=xyz(p[2]);
    const Eigen::Vector3d u=b-a,v=c-a,n=u.cross(v);
    if(u.norm()<1e-6 || v.norm()<1e-6 || n.norm()<1e-6*u.norm()*v.norm())throw std::runtime_error("Degenerate arc");
    const Eigen::Vector3d centre=a+(u.squaredNorm()*v.cross(n)+v.squaredNorm()*n.cross(u))/(2*n.squaredNorm());
    const double radius=(a-centre).norm();
    const Eigen::Vector3d x=(a-centre).normalized(),y=n.normalized().cross(x);
    auto angle=[&](const Eigen::Vector3d& point){double value=std::atan2((point-centre).dot(y),(point-centre).dot(x));return value<0?value+2*M_PI:value;};
    const double middle=angle(b),end=angle(c),count=std::ceil(radius*end/.002);
    if(!(middle>0 && end>middle) || !std::isfinite(count) || count>10000)throw std::runtime_error("Invalid arc");
    const int steps=std::max(2,static_cast<int>(count));std::vector<Pose> out{p[0]};
    for(int k=1;k<=steps;++k) {
      const double theta=end*k/steps;
      auto q=theta<=middle?rotation(p[0]).slerp(theta/middle,rotation(p[1])):
        rotation(p[1]).slerp((theta-middle)/(end-middle),rotation(p[2]));
      out.push_back(makePose(centre+radius*(std::cos(theta)*x+std::sin(theta)*y),q));
    }
    out.back()=p[2];return out;
  }
  void controller(const moveit_msgs::msg::RobotTrajectory& trajectory,bool gripper,const std::shared_ptr<Goal>& goal) {
    check(goal);auto client=gripper?gripper_client_:arm_client_;
    if(!client->wait_for_action_server(2s))throw std::runtime_error("Missing controller action server");
    Follow::Goal command;command.trajectory=trajectory.joint_trajectory;
    if(command.trajectory.points.empty())throw std::runtime_error("Empty trajectory");
    auto future=client->async_send_goal(command);
    if(future.wait_for(3s)!=std::future_status::ready) {
      client->async_cancel_all_goals();
      throw std::runtime_error("STOP_UNCONFIRMED: controller goal acceptance unknown");
    }
    auto handle=future.get();if(!handle)throw std::runtime_error("Controller rejected goal");
    auto result=client->async_get_result(handle);
    bool cancel_sent=false;
    auto stop_deadline=std::chrono::steady_clock::time_point::max();
    while(result.wait_for(20ms)!=std::future_status::ready) {
      bool stale=false;
      try {check(goal);}catch(...) {stale=true;}
      if(stale && !cancel_sent) {
        client->async_cancel_goal(handle);cancel_sent=true;stop_deadline=std::chrono::steady_clock::now()+5s;
      }
      if(std::chrono::steady_clock::now()>stop_deadline)
        throw std::runtime_error("STOP_UNCONFIRMED: controller cancellation not confirmed");
    }
    const auto outcome=result.get();
    if(cancel_sent || goal->is_canceling()) {
      if(outcome.code!=rclcpp_action::ResultCode::CANCELED && outcome.code!=rclcpp_action::ResultCode::SUCCEEDED)
        throw std::runtime_error("Controller aborted while stopping");
      throw std::runtime_error("Controller stopped");
    }
    if(outcome.code!=rclcpp_action::ResultCode::SUCCEEDED || outcome.result->error_code!=0)
      throw std::runtime_error("Controller execution failed");
    auto observed=(gripper?gripper_:arm_)->getCurrentState(3);
    if(!observed)throw std::runtime_error("Missing final feedback");
    const auto& names=command.trajectory.joint_names;const auto& positions=command.trajectory.points.back().positions;
    double error=0;
    for(std::size_t k=0;k<names.size();++k)error=std::max(error,std::abs(observed->getVariablePosition(names[k])-positions.at(k)));
    if(error>(gripper?.003:.02))throw std::runtime_error("Final feedback outside tolerance");
  }
  void execute(const std::shared_ptr<Goal>& goal) {
    auto output=std::make_shared<Motion::Result>();
    try {
      auto a=YAML::Load(goal->get_goal()->command_json);const auto type=a["type"].as<std::string>();const auto p=a["parameters"];
      const double timeout=finite(a["timeout_s"]);
      if(timeout<=0 || timeout>86400)throw std::runtime_error("Invalid motion timeout");
      deadline_=std::chrono::steady_clock::now()+std::chrono::duration_cast<std::chrono::steady_clock::duration>(std::chrono::duration<double>(timeout));
      if(type=="gripper_grasp" && (side_!="right" || p["grasp_profile"].as<std::string>()!="water_bottle"))throw std::runtime_error("Unknown grasp profile");
      const bool gripper=type=="gripper_open" || type=="gripper_grasp";
      auto& group=gripper?*gripper_:*arm_;auto start=state(group,goal);group.clearPoseTargets();
      moveit_msgs::msg::RobotTrajectory trajectory;
      if(gripper || type=="move_j" || type=="move_j_partial") {
        std::vector<double> values;start->copyJointGroupPositions(group.getName(),values);
        if(gripper) {
          const double width=type=="gripper_grasp"?.070:finite(p["opening_m"]);
          if(width<0 || width>.1)throw std::runtime_error("Invalid gripper opening");
          values={width/2,-width/2};
        } else if(type=="move_j") {
          if(p["joint_positions_rad"].size()!=7)throw std::runtime_error("Expected seven joints");
          for(int k=0;k<7;++k)values.at(k)=finite(p["joint_positions_rad"][k]);
        } else {
          const auto joints=p["joint_positions_rad"];
          if(!joints.IsMap() || joints.size()==0)throw std::runtime_error("Empty joint patch");
          for(auto it:joints) {
            auto key=it.first.as<std::string>();
            if(key.size()!=2 || key[0]!='J' || key[1]<'1' || key[1]>'7')throw std::runtime_error("Unknown joint");
            values.at(key[1]-'1')=finite(it.second);
          }
        }
        if(!group.setJointValueTarget(values))throw std::runtime_error("Joint target outside limits");
        Group::Plan plan;if(group.plan(plan)!=moveit::core::MoveItErrorCode::SUCCESS)throw std::runtime_error("Joint planning failed");
        trajectory=plan.trajectory_;
      } else if(type=="move_p") {
        group.setPoseTarget(target(p,group.getPlanningFrame()));Group::Plan plan;
        auto result=group.plan(plan);group.clearPoseTargets();
        if(result!=moveit::core::MoveItErrorCode::SUCCESS)throw std::runtime_error("Pose planning failed");
        trajectory=plan.trajectory_;
      } else if(type=="move_l" || type=="move_c") {
        std::vector<Pose> points;
        if(type=="move_l")points={target(p,group.getPlanningFrame())};
        else {
          if(p["poses"].size()!=3)throw std::runtime_error("Arc needs three poses");
          for(auto item:p["poses"])points.push_back(target(item,group.getPlanningFrame()));
          const auto& current=start->getGlobalLinkTransform(group.getEndEffectorLink());
          if((current.translation()-xyz(points[0])).norm()>.002 ||
              Eigen::Quaterniond(current.rotation()).angularDistance(rotation(points[0]))>.01)
            throw std::runtime_error("Move to arc start first");
          points=arc(points);
        }
        const auto fraction=group.computeCartesianPath(points,.002,0.,trajectory,true);
        if(!std::isfinite(fraction) || fraction<1-1e-9 || trajectory.joint_trajectory.points.size()<2)
          throw std::runtime_error("Incomplete Cartesian path");
        std::vector<double> previous;
        for(const auto& name:trajectory.joint_trajectory.joint_names)previous.push_back(start->getVariablePosition(name));
        for(const auto& point:trajectory.joint_trajectory.points) {
          if(point.positions.size()!=previous.size())throw std::runtime_error("Invalid trajectory dimensions");
          for(std::size_t k=0;k<previous.size();++k)
            if(!std::isfinite(point.positions[k]) || std::abs(point.positions[k]-previous[k])>.2)
              throw std::runtime_error("Joint jump rejected");
          previous=point.positions;
        }
        robot_trajectory::RobotTrajectory path(group.getRobotModel(),group.getName());path.setRobotTrajectoryMsg(*start,trajectory);
        trajectory_processing::IterativeParabolicTimeParameterization timing;
        if(!timing.computeTimeStamps(path,.3,.3))throw std::runtime_error("Timing failed");
        path.getRobotTrajectoryMsg(trajectory);
      } else throw std::runtime_error("Unsupported action");
      auto feedback=std::make_shared<Motion::Feedback>();feedback->phase="EXECUTING";goal->publish_feedback(feedback);
      controller(trajectory,gripper,goal);output->success=true;output->message="Controller result and final joints verified";
    } catch(const std::exception& e) {output->message=e.what();}
    // Release before returning the result so a fast next goal is not rejected.
    if(output->message.find("STOP_UNCONFIRMED")!=std::string::npos)latched_=true;
    busy_=latched_.load();
    if(goal->is_canceling())goal->canceled(output);
    else if(output->success)goal->succeed(output);else goal->abort(output);
  }
  void work() {
    try {
      arm_=std::make_unique<Group>(node_,side_+"_arm");gripper_=std::make_unique<Group>(node_,side_+"_gripper");
      arm_->setEndEffectorLink(side_+"_gripper_base");
      for(auto* group:{arm_.get(),gripper_.get()}) {group->setMaxVelocityScalingFactor(.3);group->setMaxAccelerationScalingFactor(.3);group->setPlanningTime(5);}
      while(!stopping_) {
        std::shared_ptr<Goal> goal;
        {std::unique_lock<std::mutex> lock(mutex_);cv_.wait(lock,[this]{return stopping_ || pending_;});if(stopping_)return;goal=std::move(pending_);}
        execute(goal);
      }
    } catch(const std::exception& e) {RCLCPP_ERROR(node_->get_logger(),"Arm server failed: %s",e.what());latched_=true;busy_=true;}
  }
  rclcpp::Node::SharedPtr node_;std::string side_;
  tf2_ros::Buffer buffer_;tf2_ros::TransformListener listener_;
  std::unique_ptr<Group> arm_,gripper_;
  rclcpp_action::Client<Follow>::SharedPtr arm_client_,gripper_client_;
  rclcpp_action::Server<Motion>::SharedPtr server_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joints_;
  std::atomic<int64_t> feedback_stamp_{0};
  std::atomic<bool> busy_{false},latched_{false},stopping_{false};
  std::chrono::steady_clock::time_point deadline_=std::chrono::steady_clock::time_point::max();
  std::mutex mutex_;std::condition_variable cv_;std::shared_ptr<Goal> pending_;std::thread worker_;
};
int main(int argc,char** argv) {
  rclcpp::init(argc,argv);
  auto node=std::make_shared<rclcpp::Node>("sim_arm_server",rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
  try {ArmServer server(node);rclcpp::spin(node);}catch(const std::exception& e){RCLCPP_ERROR(node->get_logger(),"%s",e.what());rclcpp::shutdown();return 1;}
  rclcpp::shutdown();return 0;
}
