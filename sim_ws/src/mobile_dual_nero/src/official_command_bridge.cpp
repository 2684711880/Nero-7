#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <moveit/robot_state/conversions.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <array>
#include <atomic>
#include <condition_variable>
#include <functional>
#include <mutex>
#include <thread>
#include <cmath>

using Pose = geometry_msgs::msg::Pose;
using Group = moveit::planning_interface::MoveGroupInterface;

Eigen::Vector3d xyz(const Pose& p) { return {p.position.x,p.position.y,p.position.z}; }
Eigen::Quaterniond rotation(const Pose& p) {
  Eigen::Quaterniond q(p.orientation.w,p.orientation.x,p.orientation.y,p.orientation.z);
  if (!xyz(p).allFinite() || !q.coeffs().allFinite() || std::abs(q.norm()-1.0)>1e-6)
    throw std::runtime_error("Invalid pose/quaternion");
  return q;
}
Pose pose(const Eigen::Vector3d& v, const Eigen::Quaterniond& q) {
  Pose p; p.position.x=v.x(); p.position.y=v.y(); p.position.z=v.z();
  p.orientation.x=q.x(); p.orientation.y=q.y(); p.orientation.z=q.z(); p.orientation.w=q.w();
  return p;
}

// Simulation adapter for the official ROS topic schema, not an emulation of
// proprietary controller dynamics. One task at a time across both arms.
class Bridge {
public:
  explicit Bridge(const rclcpp::Node::SharedPtr& node)
    : node_(node), buffer_(node->get_clock()), listener_(buffer_, node, true) {
    if (!node_->get_parameter("use_sim_time").as_bool())
      throw std::runtime_error("This bridge is for Gazebo with simulation time");
    markers_=node_->create_publisher<visualization_msgs::msg::MarkerArray>(
        "/arm_trajectory_markers",rclcpp::QoS(1).transient_local());
    display_=node_->create_publisher<moveit_msgs::msg::DisplayTrajectory>(
        "/display_planned_path",rclcpp::QoS(1).transient_local());
    trace_timer_=node_->create_wall_timer(std::chrono::milliseconds(20),[this]{traceActual();});
    for (std::size_t i=0;i<2;++i) {
      const std::string side = i==0 ? "left" : "right";
      results_[i]=node_->create_publisher<std_msgs::msg::String>("/"+side+"/control/result",10);
      joints_[i]=node_->create_subscription<sensor_msgs::msg::JointState>(
          "/"+side+"/control/move_j",1,[this,i](sensor_msgs::msg::JointState::SharedPtr m) {
            // Official driver accepts a gripper-only JointState on move_j.
            if(m->name==std::vector<std::string>{"gripper"} && m->position.size()==1) {
              const double width=m->position[0];
              if(!std::isfinite(width) || width<0 || width>.10) {report(i,"FAILED invalid gripper width");return;}
              bool expected=false;
              if(!busy_.compare_exchange_strong(expected,true)) {report(i,"REJECTED busy");return;}
              {std::lock_guard<std::mutex> lock(mutex_);pending_=[this,i,width] {
                try {
                  auto& g=*grippers_[i];setStart(g,*actualState(g));
                  g.setJointValueTarget(std::vector<double>{width/2,-width/2});
                  Group::Plan p;
                  if(g.plan(p)!=moveit::core::MoveItErrorCode::SUCCESS ||
                     g.execute(p)!=moveit::core::MoveItErrorCode::SUCCESS)
                    throw std::runtime_error("Gripper plan/execution failed");
                  auto state=g.getCurrentState(3);std::vector<double> observed;
                  if(!state) throw std::runtime_error("Missing gripper feedback");
                  state->copyJointGroupPositions(g.getName(),observed);
                  if(observed.size()!=2 || std::abs(observed[0]-width/2)>.003 || std::abs(observed[1]+width/2)>.003)
                    throw std::runtime_error("Gripper width feedback mismatch");
                  report(i,"SUCCEEDED gripper");
                } catch(const std::exception& e) {report(i,"FAILED gripper: "+std::string(e.what()));}
              };}
              report(i,"ACCEPTED gripper");condition_.notify_one();return;
            }
            std::vector<double> target(7);
            if(m->name.size()!=7 || m->position.size()!=7) { report(i,"FAILED joints: require all seven joints");return; }
            for(std::size_t k=0;k<7;++k) {
              const auto expected_name="joint"+std::to_string(k+1);
              const auto found=std::find(m->name.begin(),m->name.end(),expected_name);
              if(found==m->name.end()) { report(i,"FAILED joints: missing "+expected_name);return; }
              target[k]=m->position[std::distance(m->name.begin(),found)];
              if(!std::isfinite(target[k])) { report(i,"FAILED joints: non-finite target");return; }
            }
            bool expected=false;
            if(!busy_.compare_exchange_strong(expected,true)) { report(i,"REJECTED busy");return; }
            { std::lock_guard<std::mutex> lock(mutex_);pending_=[this,i,target] {
              try {
                auto& group=*groups_[i];group.clearPoseTargets();setStart(group,*actualState(group));
                if(!group.setJointValueTarget(target)) throw std::runtime_error("Joint target outside limits");
                Group::Plan plan;
                if(group.plan(plan)!=moveit::core::MoveItErrorCode::SUCCESS) throw std::runtime_error("Joint planning failed");
                executeTrajectory(i,"joints",plan.trajectory_);
              } catch(const std::exception& e) { report(i,"FAILED joints: "+std::string(e.what())); }
            }; }
            report(i,"ACCEPTED joints");condition_.notify_one(); });
      poses_[i]=node_->create_subscription<geometry_msgs::msg::PoseStamped>(
          "/"+side+"/control/move_p",1,[this,i](geometry_msgs::msg::PoseStamped::SharedPtr m) {
            submit(i,"pose",m->header.frame_id,{m->pose}); });
      named_[i]=node_->create_subscription<std_msgs::msg::String>(
          "/"+side+"/control/named_target",1,[this,i](std_msgs::msg::String::SharedPtr m) {
            bool expected=false;
            if(!busy_.compare_exchange_strong(expected,true)) { report(i,"REJECTED busy"); return; }
            { std::lock_guard<std::mutex> lock(mutex_); pending_=[this,i,name=m->data] {
              try {
                auto& group=*groups_[i]; group.clearPoseTargets(); setStart(group,*actualState(group));
                if(!group.setNamedTarget(name)) throw std::runtime_error("Unknown named target");
                Group::Plan plan;
                if(group.plan(plan)!=moveit::core::MoveItErrorCode::SUCCESS)
                  throw std::runtime_error("Named planning failed");
                report(i,"EXECUTING named "+name);
                if(group.execute(plan)!=moveit::core::MoveItErrorCode::SUCCESS)
                  throw std::runtime_error("Named execution failed");
                report(i,"SUCCEEDED named "+name);
              } catch(const std::exception& e) { report(i,"FAILED named: "+std::string(e.what())); }
            }; }
            report(i,"ACCEPTED named"); condition_.notify_one(); });
      lines_[i]=node_->create_subscription<geometry_msgs::msg::PoseStamped>(
          "/"+side+"/control/move_l",1,[this,i](geometry_msgs::msg::PoseStamped::SharedPtr m) {
            submit(i, "line", m->header.frame_id, {m->pose}); });
      arcs_[i]=node_->create_subscription<geometry_msgs::msg::PoseArray>(
          "/"+side+"/control/move_c",1,[this,i](geometry_msgs::msg::PoseArray::SharedPtr m) {
            submit(i, "arc", m->header.frame_id, m->poses); });
    }
    worker_=std::thread([this]{ work(); });
  }
  ~Bridge() {
    { std::lock_guard<std::mutex> lock(mutex_); stopping_=true; }
    condition_.notify_one();
    worker_.join();
  }
private:
  void setStart(Group& group,const moveit::core::RobotState& state) {
    moveit_msgs::msg::RobotState msg;moveit::core::robotStateToRobotStateMsg(state,msg);
    // Preserve scene attachments when the joint monitor has no attached bodies.
    msg.is_diff=true;group.setStartState(msg);
  }
  moveit::core::RobotStatePtr actualState(Group& group) {
    auto state=group.getCurrentState(5);
    if(!state) throw std::runtime_error("Missing joint feedback");
    const auto b=buffer_.lookupTransform(group.getPlanningFrame(),"base_link",tf2::TimePointZero,tf2::durationFromSec(3)).transform;
    Eigen::Isometry3d t=Eigen::Isometry3d::Identity();
    t.translation()=Eigen::Vector3d(b.translation.x,b.translation.y,b.translation.z);
    t.linear()=Eigen::Quaterniond(b.rotation.w,b.rotation.x,b.rotation.y,b.rotation.z).toRotationMatrix();
    state->setJointPositions("mobile_base_joint",t);state->update();return state;
  }
  void report(std::size_t i, const std::string& text) {
    std_msgs::msg::String m; m.data=text; results_[i]->publish(m);
    RCLCPP_INFO(node_->get_logger(), "%s: %s", i==0?"left":"right", text.c_str());
  }
  void executeTrajectory(std::size_t i,const std::string& type,const moveit_msgs::msg::RobotTrajectory& trajectory) {
    if(trajectory.joint_trajectory.points.empty()) throw std::runtime_error("Empty trajectory");
    auto& group=*groups_[i];
    auto reference=group.getCurrentState(5.0);
    if(!reference) throw std::runtime_error("Missing state for trajectory display");
    // JointState does not carry the mobile base virtual joint. Use TF for the
    // reference pose before converting planned FK positions into base_link.
    const auto base=buffer_.lookupTransform("odom","base_link",tf2::TimePointZero,tf2::durationFromSec(3)).transform;
    Eigen::Isometry3d reference_base=Eigen::Isometry3d::Identity();
    reference_base.translation()=Eigen::Vector3d(base.translation.x,base.translation.y,base.translation.z);
    reference_base.linear()=Eigen::Quaterniond(base.rotation.w,base.rotation.x,base.rotation.y,base.rotation.z).toRotationMatrix();
    reference->setJointPositions("mobile_base_joint",reference_base);reference->update();
    moveit_msgs::msg::DisplayTrajectory display;
    display.model_id=group.getRobotModel()->getName();
    moveit::core::robotStateToRobotStateMsg(*reference,display.trajectory_start);
    display.trajectory.push_back(trajectory);display_->publish(display);
    const bool cartesian=type=="line" || type=="arc";
    {
      std::lock_guard<std::mutex> lock(marker_mutex_);
      if(!cartesian) { planned_paths_[i].clear();actual_paths_[i].clear(); }
      else {
        robot_trajectory::RobotTrajectory path(group.getRobotModel(),group.getName());
        path.setRobotTrajectoryMsg(*reference,trajectory);
        for(std::size_t k=0;k<path.getWayPointCount();++k) {
          const Eigen::Vector3d v=reference_base.inverse()*
              path.getWayPoint(k).getGlobalLinkTransform(group.getEndEffectorLink()).translation();
          geometry_msgs::msg::Point p;p.x=v.x();p.y=v.y();p.z=v.z();
          planned_paths_[i].push_back(p);
        }
      }
      publishMarkers();
    }
    tracking_=cartesian ? static_cast<int>(i) : -1;
    report(i,"EXECUTING "+type);
    const auto execution=group.execute(trajectory);
    traceActual();tracking_=-1;
    if(execution!=moveit::core::MoveItErrorCode::SUCCESS)
      throw std::runtime_error("Controller execution failed");
    const auto observed=group.getCurrentState(5.0);
    if(!observed) throw std::runtime_error("Missing final feedback");
    const auto& names=trajectory.joint_trajectory.joint_names;
    const auto& target=trajectory.joint_trajectory.points.back().positions;
    double error=0;
    for(std::size_t j=0;j<names.size();++j)
      error=std::max(error,std::abs(observed->getVariablePosition(names[j])-target.at(j)));
    if(error>0.02) throw std::runtime_error("Final joint error exceeds 0.02 rad");
    report(i,"SUCCEEDED "+type+" joint_error_rad="+std::to_string(error));
  }
  // Called with marker_mutex_ held. Persist the latest complete two-arm snapshot
  // so RViz opened after execution can still display the full recorded circle.
  // Paths use base_link, matching the example's input coordinates; this excludes
  // incidental chassis odometry drift from the arm's requested circular path.
  void publishMarkers() {
    visualization_msgs::msg::MarkerArray array;
    for(std::size_t side=0;side<2;++side) for(int actual=0;actual<2;++actual) {
      visualization_msgs::msg::Marker m;
      m.header.frame_id="base_link";m.frame_locked=true; // zero stamp: use latest TF
      m.ns=(side==0?"left_":"right_")+std::string(actual?"feedback":"planned");m.id=0;
      m.type=visualization_msgs::msg::Marker::LINE_STRIP;
      m.pose.orientation.w=1.0;m.scale.x=actual?0.002:0.004;
      m.color.a=1.0;m.color.r=actual?1.0f:0.1f;m.color.g=actual?0.35f:1.0f;m.color.b=0.1f;
      m.points=actual?actual_paths_[side]:planned_paths_[side];
      m.action=m.points.size()<2 ? visualization_msgs::msg::Marker::DELETE : visualization_msgs::msg::Marker::ADD;
      array.markers.push_back(std::move(m));
    }
    markers_->publish(array);
  }
  void traceActual() {
    const int side=tracking_.load();if(side<0)return;
    try {
      const auto t=buffer_.lookupTransform("base_link",side==0?"left_gripper_base":"right_gripper_base",tf2::TimePointZero);
      geometry_msgs::msg::Point p;
      p.x=t.transform.translation.x;p.y=t.transform.translation.y;p.z=t.transform.translation.z;
      std::lock_guard<std::mutex> lock(marker_mutex_);
      // A timer callback may have read tracking_ before a new task started.
      if(tracking_.load()!=side)return;
      auto& points=actual_paths_[side];
      if(!points.empty()) {
        const auto& last=points.back();
        if(std::hypot(std::hypot(p.x-last.x,p.y-last.y),p.z-last.z)<0.00005)return;
      }
      points.push_back(p);
      if(points.size()>20000) points.erase(points.begin(),points.begin()+10000);
      publishMarkers();
    } catch(const tf2::TransformException&) { /* TF may be unavailable during startup. */ }
  }
  void submit(std::size_t i,const std::string& type,const std::string& frame,std::vector<Pose> points) {
    bool expected=false;
    if (!busy_.compare_exchange_strong(expected,true)) { report(i,"REJECTED busy"); return; }
    { std::lock_guard<std::mutex> lock(mutex_);
      pending_=[this,i,type,frame,points=std::move(points)]{ execute(i,type,frame,points); }; }
    report(i,"ACCEPTED "+type); condition_.notify_one();
  }
  Pose transform(const Pose& p,const std::string& source,const std::string& target) {
    const auto q=rotation(p);
    if(source.empty()) throw std::runtime_error("frame_id is required");
    if(source==target) return p;
    const auto t=buffer_.lookupTransform(target,source,tf2::TimePointZero,tf2::durationFromSec(3));
    const auto& v=t.transform.translation; const auto& r=t.transform.rotation;
    Eigen::Quaterniond rq(r.w,r.x,r.y,r.z);
    return pose(rq*xyz(p)+Eigen::Vector3d(v.x,v.y,v.z),rq*q);
  }
  std::vector<Pose> arc(const std::vector<Pose>& p) {
    if(p.size()!=3) throw std::runtime_error("Arc requires exactly three poses");
    const auto a=xyz(p[0]), b=xyz(p[1]), c=xyz(p[2]);
    const Eigen::Vector3d u=b-a,v=c-a,n=u.cross(v);
    if(u.norm()<1e-6 || v.norm()<1e-6 || n.norm()<1e-6*u.norm()*v.norm())
      throw std::runtime_error("Degenerate arc");
    const Eigen::Vector3d centre=a+(u.squaredNorm()*v.cross(n)+v.squaredNorm()*n.cross(u))/(2*n.squaredNorm());
    const double radius=(a-centre).norm();
    const Eigen::Vector3d x=(a-centre).normalized(), y=n.normalized().cross(x);
    auto angle=[&](const Eigen::Vector3d& point) {
      double value=std::atan2((point-centre).dot(y),(point-centre).dot(x));
      return value<0 ? value+2*M_PI : value;
    };
    const double middle=angle(b), end=angle(c);
    if(!(middle>0 && end>middle)) throw std::runtime_error("Invalid directed arc");
    const double count=std::ceil(radius*end/0.002);
    if(!std::isfinite(count) || count>10000) throw std::runtime_error("Arc exceeds sampling budget");
    const int steps=std::max(2,static_cast<int>(count));
    std::vector<Pose> out{p[0]};
    for(int k=1;k<=steps;++k) {
      const double theta=end*k/steps;
      const auto q=theta<=middle ? rotation(p[0]).slerp(theta/middle,rotation(p[1])) :
          rotation(p[1]).slerp((theta-middle)/(end-middle),rotation(p[2]));
      out.push_back(pose(centre+radius*(std::cos(theta)*x+std::sin(theta)*y),q));
    }
    out.back()=p[2]; return out;
  }
  void execute(std::size_t i,const std::string& type,const std::string& source,const std::vector<Pose>& raw) {
    try {
      auto& group=*groups_[i];
      const std::string model=group.getPlanningFrame();
      std::vector<Pose> points;
      for(const auto& p:raw) points.push_back(transform(p,source,model));
      if(type=="line" && points.size()!=1) throw std::runtime_error("Line requires one pose");
      auto start=group.getCurrentState(5.0);
      if(!start) throw std::runtime_error("No current joint state");
      // JointState lacks the planar odom->base_link joint; copy its actual TF.
      const auto base=buffer_.lookupTransform(model,"base_link",tf2::TimePointZero,tf2::durationFromSec(3));
      Eigen::Isometry3d t=Eigen::Isometry3d::Identity();
      const auto& tr=base.transform.translation; const auto& qr=base.transform.rotation;
      t.translation()=Eigen::Vector3d(tr.x,tr.y,tr.z);
      t.linear()=Eigen::Quaterniond(qr.w,qr.x,qr.y,qr.z).toRotationMatrix();
      start->setJointPositions("mobile_base_joint",t); start->update();
      if(type=="pose") {
        if(points.size()!=1) throw std::runtime_error("Pose command requires one target");
        group.clearPoseTargets();setStart(group,*start);
        group.setPoseTarget(points[0]);Group::Plan plan;
        const auto planned=group.plan(plan);group.clearPoseTargets();
        if(planned!=moveit::core::MoveItErrorCode::SUCCESS) throw std::runtime_error("Pose planning failed");
        executeTrajectory(i,"pose",plan.trajectory_);return;
      }
      if(type=="arc") {
        if(points.size()!=3) throw std::runtime_error("Arc requires three poses");
        const auto& current=start->getGlobalLinkTransform(group.getEndEffectorLink());
        if((current.translation()-xyz(points[0])).norm()>0.002 ||
           Eigen::Quaterniond(current.rotation()).angularDistance(rotation(points[0]))>0.01)
          throw std::runtime_error("Move to arc start first (2 mm / 0.01 rad tolerance)");
        points=arc(points);
      }
      setStart(group,*start);
      moveit_msgs::msg::RobotTrajectory message;
      const double fraction=group.computeCartesianPath(points,0.002,0.0,message,true);
      RCLCPP_INFO(node_->get_logger(), "Cartesian completion: %.4f", fraction);
      if(!std::isfinite(fraction) || fraction<1-1e-9 || message.joint_trajectory.points.size()<2)
        throw std::runtime_error("Incomplete path; not executing");
      const auto& names=message.joint_trajectory.joint_names;
      std::vector<double> previous;
      for(const auto& name:names) previous.push_back(start->getVariablePosition(name));
      for(const auto& point:message.joint_trajectory.points) {
        if(point.positions.size()!=names.size()) throw std::runtime_error("Invalid trajectory dimensions");
        for(std::size_t j=0;j<names.size();++j)
          if(!std::isfinite(point.positions[j]) || std::abs(point.positions[j]-previous[j])>0.2)
            throw std::runtime_error("Joint jump rejected");
        previous=point.positions;
      }
      robot_trajectory::RobotTrajectory trajectory(group.getRobotModel(),group.getName());
      trajectory.setRobotTrajectoryMsg(*start,message);
      trajectory_processing::IterativeParabolicTimeParameterization timing;
      if(!timing.computeTimeStamps(trajectory,0.8,0.8)) throw std::runtime_error("Timing failed");
      trajectory.getRobotTrajectoryMsg(message);
      executeTrajectory(i,type,message);
    } catch(const std::exception& e) { tracking_=-1;report(i,"FAILED "+type+": "+e.what()); }
  }
  void work() {
    try {
      for(std::size_t i=0;i<2;++i) {
        grippers_[i]=std::make_unique<Group>(node_,i==0?"left_gripper":"right_gripper");
        groups_[i]=std::make_unique<Group>(node_,i==0?"left_arm":"right_arm");
        groups_[i]->setEndEffectorLink(i==0?"left_gripper_base":"right_gripper_base");
        groups_[i]->setMaxVelocityScalingFactor(0.8);
        groups_[i]->setMaxAccelerationScalingFactor(0.8);
      }
      RCLCPP_INFO(node_->get_logger(),"READY: official-schema topics /left|right/control/move_j, move_p, move_l, move_c");
    } catch(const std::exception& e) {
      RCLCPP_ERROR(node_->get_logger(),"Bridge initialization failed: %s",e.what());
      rclcpp::shutdown(); return;
    }
    while(true) {
      std::function<void()> task;
      { std::unique_lock<std::mutex> lock(mutex_);
        condition_.wait(lock,[this]{return stopping_ || static_cast<bool>(pending_);});
        if(stopping_) return;
        task=std::move(pending_); pending_=nullptr; }
      task(); busy_=false;
    }
  }
  rclcpp::Node::SharedPtr node_;
  tf2_ros::Buffer buffer_; tf2_ros::TransformListener listener_;
  std::array<std::unique_ptr<Group>,2> groups_,grippers_;
  std::array<rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr,2> lines_;
  std::array<rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr,2> poses_;
  std::array<rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr,2> joints_;
  std::array<rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr,2> arcs_;
  std::array<rclcpp::Publisher<std_msgs::msg::String>::SharedPtr,2> results_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_;
  rclcpp::Publisher<moveit_msgs::msg::DisplayTrajectory>::SharedPtr display_;
  rclcpp::TimerBase::SharedPtr trace_timer_;
  std::mutex marker_mutex_;
  std::array<std::vector<geometry_msgs::msg::Point>,2> planned_paths_,actual_paths_;
  std::atomic<int> tracking_{-1};
  std::array<rclcpp::Subscription<std_msgs::msg::String>::SharedPtr,2> named_;
  std::mutex mutex_; std::condition_variable condition_; std::function<void()> pending_;
  bool stopping_=false; std::atomic<bool> busy_{false}; std::thread worker_;
};

int main(int argc,char** argv) {
  rclcpp::init(argc,argv);
  auto node=std::make_shared<rclcpp::Node>("official_command_bridge",
      rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
  try { Bridge bridge(node); rclcpp::spin(node); }
  catch(const std::exception& e) { RCLCPP_ERROR(node->get_logger(),"%s",e.what()); rclcpp::shutdown(); return 1; }
  rclcpp::shutdown(); return 0;
}
