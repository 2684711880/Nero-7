#include "nero_official_commander_cpp/arm_commander.hpp"
#include <geometry_msgs/msg/twist.hpp>
#include <gazebo_msgs/msg/model_states.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <moveit_msgs/srv/apply_planning_scene.hpp>
#include <moveit_msgs/srv/get_planning_scene.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <fstream>
#include <algorithm>

using Pose=geometry_msgs::msg::Pose;
using namespace std::chrono_literals;

// Simulation-only task. Arm commands retain the official ROS schema; base
// transport is isolated here so its implementation can be replaced later.
class PickPlace {
public:
  explicit PickPlace(const rclcpp::Node::SharedPtr& n)
    : node(n),arm(n,"right","odom",true),other(n,"left","odom",true),
      tf(n->get_clock()),listener(tf,n,true) {
    velocity=node->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel",10);
    grasp=node->create_client<std_srvs::srv::SetBool>("/sim_grasp/right/attach");
    scene=node->create_client<moveit_msgs::srv::ApplyPlanningScene>("/apply_planning_scene");
    get_scene=node->create_client<moveit_msgs::srv::GetPlanningScene>("/get_planning_scene");
    states=node->create_subscription<gazebo_msgs::msg::ModelStates>("/gazebo/model_states",10,
      [this](gazebo_msgs::msg::ModelStates::SharedPtr m) {
        for(std::size_t i=0;i<m->name.size();++i) {
          if(m->name[i]=="water_bottle") {bottle=m->pose[i];have_bottle=true;}
          if(m->name[i]=="mobile_dual_nero") {base=m->pose[i];have_base=true;}
        }
        last=std::chrono::steady_clock::now();
      });
    hold_timer=node->create_wall_timer(40ms,[this] {
      if(!holding || driving) return;
      if(std::chrono::steady_clock::now()-last>500ms) {stop();throw std::runtime_error("Stale base feedback during hold");}
      const double a=yaw(base),dx=hold_x-base.position.x,dy=hold_y-base.position.y,da=wrap(hold_yaw-a);
      if(std::hypot(dx,dy)>.06 || std::abs(da)>.08) {stop();throw std::runtime_error("Base escaped docking hold tolerance");}
      geometry_msgs::msg::Twist cmd;
      cmd.linear.x=1.5*(std::cos(a)*dx+std::sin(a)*dy);
      cmd.linear.y=1.5*(-std::sin(a)*dx+std::cos(a)*dy);
      cmd.angular.z=1.5*da;
      velocity->publish(cmd);
    });
  }
  ~PickPlace() {if(hold_timer)hold_timer->cancel();stop();}
  void run() {
    wait(2);fresh();
    if(std::hypot(base.position.x,base.position.y)>.05 || std::abs(yaw(base))>.05)
      throw std::runtime_error("Start from the reset world with the base at the origin");
    if(!grasp->wait_for_service(5s)) throw std::runtime_error("Launch the dedicated simulation with sim_grasp:=true");
    if(std::hypot(bottle.position.x,bottle.position.y+1.6)>.05)
      throw std::runtime_error("Bottle is not at the source workstation; restart/reset the world");
    table_z=bottle.position.z-.12;
    stage("1 prepare arms and open gripper");
    other.move_j({0,0,0,1,0,0,0});arm.move_j({0,0,0,1,0,0,0});arm.gripper(.09);
    Pose ready;ready.position.x=0;ready.position.y=-.60;ready.position.z=.95;
    ready.orientation.x=.5;ready.orientation.y=.5;ready.orientation.z=-.5;ready.orientation.w=.5;
    arm.move_p(ready);
    stage("2 dock at source table");drive(0,-.80,0);
    fresh();Pose pick=graspPose(bottle,0);
    auto above=pick;above.position.z+=.12;
    allowContact(true);
    stage("3 approach from above");arm.move_p(above);arm.move_l(pick);
    stage("4 close jaws and establish checked simulation grasp");arm.gripper(.070);
    constraint(true);attachScene(true);
    stage("5 lift bottle and move to transport pose");arm.move_l(above);
    Pose carry=above;carry.position.y=base.position.y-.60;carry.position.z=.95;
    arm.move_p(carry);wait(.5);checkHeld();
    stage("6 drive to centre, rotate, dock at destination");
    drive(0,0,0);drive(0,0,M_PI);drive(0,.80,M_PI);checkHeld();
    stage("7 approach destination and lower bottle");
    Pose target=bottle;target.position.x=0;target.position.y=1.60;
    target.position.z=table_z+.12+.006;
    Pose place=graspPose(target,M_PI);above=place;above.position.z+=.12;
    arm.move_p(above);arm.move_l(place);
    stage("8 release, open and withdraw");
    constraint(false);attachScene(false);arm.gripper(.09);
    arm.move_l(above);wait(2);allowContact(false);fresh();
    const auto& q=bottle.orientation;
    const double upright=1-2*(q.x*q.x+q.y*q.y);
    const double xy=std::hypot(bottle.position.x,bottle.position.y-1.6);
    const double height=std::abs(bottle.position.z-(table_z+.12));
    if(xy>.04 || height>.02 || upright<.97)
      throw std::runtime_error("Bottle failed final position/upright acceptance");
    RCLCPP_INFO(node->get_logger(),"PASS bottle=(%.4f,%.4f,%.4f), xy_error=%.4f, z_error=%.4f, upright=%.4f",
      bottle.position.x,bottle.position.y,bottle.position.z,xy,height,upright);
    const auto report=node->get_parameter("report").as_string();
    if(!report.empty()) {
      std::ofstream f(report);
      f<<"{\"passed\":true,\"grasp_model\":\"checked_fixed_joint\",\"final_xyz\":["
       <<bottle.position.x<<","<<bottle.position.y<<","<<bottle.position.z
       <<"],\"xy_error_m\":"<<xy<<",\"z_error_m\":"<<height<<",\"upright_cos\":"<<upright<<"}\n";
    }
  }
private:
  void stage(const std::string& s) {RCLCPP_INFO(node->get_logger(),"TASK %s",s.c_str());}
  void fresh() {
    rclcpp::spin_some(node);
    if(!have_base || !have_bottle || std::chrono::steady_clock::now()-last>500ms)
      throw std::runtime_error("Missing/stale Gazebo feedback");
  }
  void wait(double seconds) {
    const auto until=std::chrono::steady_clock::now()+std::chrono::duration<double>(seconds);
    while(rclcpp::ok() && std::chrono::steady_clock::now()<until) {
      rclcpp::spin_some(node);std::this_thread::sleep_for(20ms);
    }
    if(!rclcpp::ok()) throw std::runtime_error("Interrupted");
  }
  static double yaw(const Pose& p) {const auto& q=p.orientation;return std::atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z));}
  static double wrap(double a) {return std::atan2(std::sin(a),std::cos(a));}
  void stop() {if(rclcpp::ok()) velocity->publish(geometry_msgs::msg::Twist{});}
  // Closed loop using existing /cmd_vel. No blind timed driving, no navigation
  // obstacle planner: the known empty corridor is explicitly prescribed.
  void drive(double x,double y,double heading) {
    driving=true;
    const auto deadline=std::chrono::steady_clock::now()+120s;
    int settled=0;
    while(rclcpp::ok()) {
      fresh();
      auto odom=tf.lookupTransform("odom","base_link",tf2::TimePointZero).transform;
      if(std::hypot(odom.translation.x-base.position.x,odom.translation.y-base.position.y)>.03)
        throw std::runtime_error("Odometry/truth disagree: launch ground_truth_odometry:=true for this simulation task");
      const double a=yaw(base),dx=x-base.position.x,dy=y-base.position.y,da=wrap(heading-a);
      geometry_msgs::msg::Twist cmd;
      if(std::hypot(dx,dy)<.015 && std::abs(da)<.020) {
        stop();if(++settled>=25) break;
      } else {
        settled=0;
        // Translation and turning are separate on this fixed corridor route.
        cmd.angular.z=std::clamp(.8*da,-.18,.18);
        if(std::abs(da)>.020 && std::abs(cmd.angular.z)<.04) cmd.angular.z=std::copysign(.04,da);
        if(std::abs(da)<.03) {
          cmd.linear.x=.6*(std::cos(a)*dx+std::sin(a)*dy);
          cmd.linear.y=.6*(-std::sin(a)*dx+std::cos(a)*dy);
          const double norm=std::hypot(cmd.linear.x,cmd.linear.y);
          if(norm>.10) {cmd.linear.x*=.10/norm;cmd.linear.y*=.10/norm;}
          else if(std::hypot(dx,dy)>.015 && norm>1e-6 && norm<.025) {cmd.linear.x*=.025/norm;cmd.linear.y*=.025/norm;}
        }
        velocity->publish(cmd);
      }
      if(std::chrono::steady_clock::now()>deadline) {stop();throw std::runtime_error("Docking timeout");}
      std::this_thread::sleep_for(40ms);
    }
    hold_x=x;hold_y=y;hold_yaw=heading;holding=true;driving=false;stop();wait(.5);
  }
  Pose graspPose(const Pose& object,double heading) {
    Pose p=object;
    // Tool +Z points horizontally toward the bottle; jaws span world X.
    p.position.x-=.18*std::sin(heading);p.position.y+=.18*std::cos(heading);
    const double c=std::cos(heading/2),s=std::sin(heading/2);
    p.orientation.x=.5*(c-s);p.orientation.y=.5*(c+s);
    p.orientation.z=.5*(s-c);p.orientation.w=.5*(c+s);return p;
  }
  template<class Service> typename Service::Response::SharedPtr call(
      const typename rclcpp::Client<Service>::SharedPtr& client,
      const typename Service::Request::SharedPtr& request) {
    if(!client->wait_for_service(5s)) throw std::runtime_error("Missing service: "+std::string(client->get_service_name()));
    auto result=client->async_send_request(request);
    if(rclcpp::spin_until_future_complete(node,result,8s)!=rclcpp::FutureReturnCode::SUCCESS)
      throw std::runtime_error("Service result unknown; task stopped");
    return result.get();
  }
  void apply(moveit_msgs::msg::PlanningScene s) {
    s.is_diff=true;s.robot_state.is_diff=true;
    auto req=std::make_shared<moveit_msgs::srv::ApplyPlanningScene::Request>();req->scene=s;
    if(!call<moveit_msgs::srv::ApplyPlanningScene>(scene,req)->success)
      throw std::runtime_error("Planning scene update failed");
    wait(.3);
  }
  void allowContact(bool allow) {
    auto req=std::make_shared<moveit_msgs::srv::GetPlanningScene::Request>();
    req->components.components=moveit_msgs::msg::PlanningSceneComponents::ALLOWED_COLLISION_MATRIX;
    auto acm=call<moveit_msgs::srv::GetPlanningScene>(get_scene,req)->scene.allowed_collision_matrix;
    const std::vector<std::string> names={"water_bottle","right_gripper_base","right_gripper_link1","right_gripper_link2"};
    for(const auto& name:names) if(std::find(acm.entry_names.begin(),acm.entry_names.end(),name)==acm.entry_names.end()) {
      acm.entry_names.push_back(name);
      for(auto& row:acm.entry_values) row.enabled.push_back(false);
      moveit_msgs::msg::AllowedCollisionEntry row;row.enabled.resize(acm.entry_names.size(),false);acm.entry_values.push_back(row);
    }
    const auto b=std::find(acm.entry_names.begin(),acm.entry_names.end(),"water_bottle")-acm.entry_names.begin();
    for(std::size_t i=1;i<names.size();++i) {
      const auto h=std::find(acm.entry_names.begin(),acm.entry_names.end(),names[i])-acm.entry_names.begin();
      acm.entry_values[b].enabled[h]=allow;acm.entry_values[h].enabled[b]=allow;
    }
    moveit_msgs::msg::PlanningScene s;s.allowed_collision_matrix=acm;apply(s);
  }
  void constraint(bool attach) {
    auto req=std::make_shared<std_srvs::srv::SetBool::Request>();req->data=attach;
    const auto res=call<std_srvs::srv::SetBool>(grasp,req);
    if(!res->success) throw std::runtime_error(res->message);
    RCLCPP_INFO(node->get_logger(),"%s",res->message.c_str());
  }
  void attachScene(bool attach) {
    fresh();moveit_msgs::msg::PlanningScene s;
    moveit_msgs::msg::AttachedCollisionObject a;a.link_name="right_gripper_base";
    a.touch_links={"right_gripper_base","right_gripper_link1","right_gripper_link2"};
    a.object.id="water_bottle";a.object.header.frame_id="odom";
    a.object.operation=attach?moveit_msgs::msg::CollisionObject::ADD:moveit_msgs::msg::CollisionObject::REMOVE;
    shape_msgs::msg::SolidPrimitive shape;shape.type=shape_msgs::msg::SolidPrimitive::CYLINDER;shape.dimensions={.24,.034};
    if(attach) {
      a.object.primitives={shape};a.object.primitive_poses={bottle};
    } else {
      moveit_msgs::msg::CollisionObject object;object.id="water_bottle";object.header.frame_id="odom";
      object.operation=moveit_msgs::msg::CollisionObject::ADD;object.primitives={shape};object.primitive_poses={bottle};
      s.world.collision_objects={object};
    }
    s.robot_state.attached_collision_objects={a};apply(s);
  }
  void checkHeld() {
    fresh();auto tcp=arm.currentPose();
    const auto& q=tcp.orientation;
    // Quaternion-rotated local +Z offset (pinch centre).
    const double x=tcp.position.x+.36*(q.x*q.z+q.w*q.y);
    const double y=tcp.position.y+.36*(q.y*q.z-q.w*q.x);
    const double z=tcp.position.z+.18*(1-2*(q.x*q.x+q.y*q.y));
    const double e=std::hypot(std::hypot(x-bottle.position.x,y-bottle.position.y),z-bottle.position.z);
    if(e>.03 || bottle.position.z<table_z+.20) throw std::runtime_error("Bottle is not carried at pinch centre");
    RCLCPP_INFO(node->get_logger(),"HOLD verified: pinch error %.4f m, bottle z %.4f",e,bottle.position.z);
  }
  rclcpp::Node::SharedPtr node;ArmCommander arm,other;tf2_ros::Buffer tf;tf2_ros::TransformListener listener;
  rclcpp::TimerBase::SharedPtr hold_timer;
  bool holding=false,driving=false;double hold_x=0,hold_y=0,hold_yaw=0;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity;
  rclcpp::Subscription<gazebo_msgs::msg::ModelStates>::SharedPtr states;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr grasp;
  rclcpp::Client<moveit_msgs::srv::ApplyPlanningScene>::SharedPtr scene;
  rclcpp::Client<moveit_msgs::srv::GetPlanningScene>::SharedPtr get_scene;
  Pose bottle,base;bool have_bottle=false,have_base=false;double table_z=0;
  std::chrono::steady_clock::time_point last;
};

int main(int argc,char** argv) {
  rclcpp::init(argc,argv);auto node=std::make_shared<rclcpp::Node>("bottle_pick_place");
  node->declare_parameter("send",false);node->declare_parameter("report",std::string(""));
  int result=0;
  try {
    if(!node->get_parameter("send").as_bool())
      RCLCPP_INFO(node->get_logger(),"DRY RUN: source dock -> side grasp -> lift -> centre/turn -> destination dock -> release. Set send:=true to run in the dedicated simulation.");
    else {PickPlace task(node);task.run();}
  } catch(const std::exception& e) {
    RCLCPP_ERROR(node->get_logger(),"TASK STOPPED: %s; no automatic retry or release",e.what());result=1;
  }
  rclcpp::shutdown();return result;
}
