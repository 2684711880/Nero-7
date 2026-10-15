// Deterministic simulation grasp: no teleportation and no friction/force claim.
// Physics mutations run exclusively on the Gazebo update thread.
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo_ros/node.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <mutex>
#include <condition_variable>

namespace gazebo {
class BottleGrasp : public WorldPlugin {
public:
  void Load(physics::WorldPtr world,sdf::ElementPtr sdf) override {
    world_=world;node_=gazebo_ros::Node::Get(sdf);
    service_=node_->create_service<std_srvs::srv::SetBool>("/sim_grasp/right/attach",
      [this](const std_srvs::srv::SetBool::Request::SharedPtr req,
             std_srvs::srv::SetBool::Response::SharedPtr res) {
        std::unique_lock<std::mutex> lock(mutex_);
        if(pending_) { res->message="Another grasp request is pending";return; }
        attach_=req->data;pending_=true;done_=false;
        if(!condition_.wait_for(lock,std::chrono::seconds(3),[this]{return done_;})) {
          pending_=false;res->message="Physics update unavailable; no queued retry";return;
        }
        res->success=success_;res->message=message_;
      });
    update_=event::Events::ConnectWorldUpdateBegin(std::bind(&BottleGrasp::Update,this));
  }
private:
  void Update() {
    std::lock_guard<std::mutex> lock(mutex_);if(!pending_) return;
    success_=false;
    try {
      if(attach_) {
        if(joint_) throw std::runtime_error("Bottle already attached");
        auto robot=world_->ModelByName("mobile_dual_nero");
        auto bottle=world_->ModelByName("water_bottle");
        if(!robot || !bottle) throw std::runtime_error("Missing robot/bottle");
        auto hand=robot->GetLink("right_gripper_base");auto body=bottle->GetLink("body");
        auto a=robot->GetJoint("right_gripper_joint1"),b=robot->GetJoint("right_gripper_joint2");
        if(!hand || !body || !a || !b) throw std::runtime_error("Missing grasp links/joints");
        const auto tcp=hand->WorldPose().CoordPositionAdd(ignition::math::Vector3d(0,0,.18));
        const double distance=tcp.Distance(body->WorldPose().Pos());
        const double gap=a->Position(0)-b->Position(0);
        if(distance>.025) throw std::runtime_error("Pinch centre is more than 25 mm from bottle centre: "+std::to_string(distance));
        if(gap<.060 || gap>.075) throw std::runtime_error("Gripper gap must be 60..75 mm: "+std::to_string(gap));
        // Keep the measured relative transform; never move the object to the hand.
        joint_=world_->Physics()->CreateJoint("fixed",robot);
        joint_->Load(hand,body,ignition::math::Pose3d::Zero);joint_->Init();
        message_="Bottle physically attached at current relative pose";
      } else {
        if(!joint_) throw std::runtime_error("Bottle is not attached");
        joint_->Detach();joint_->Fini();joint_.reset();message_="Bottle released to physics";
      }
      success_=true;
    } catch(const std::exception& e) { message_=e.what(); }
    pending_=false;done_=true;condition_.notify_one();
  }
  physics::WorldPtr world_;physics::JointPtr joint_;gazebo_ros::Node::SharedPtr node_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr service_;event::ConnectionPtr update_;
  std::mutex mutex_;std::condition_variable condition_;
  bool pending_=false,done_=false,attach_=false,success_=false;std::string message_;
};
GZ_REGISTER_WORLD_PLUGIN(BottleGrasp)
}
