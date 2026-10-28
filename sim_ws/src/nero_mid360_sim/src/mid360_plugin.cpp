#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/physics/RayShape.hh>
#include <gazebo_ros/node.hpp>
#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <std_msgs/msg/string.hpp>
#include <cmath>
#include <sstream>
#include "nero_mid360_sim/interval_acceleration.hpp"

namespace gazebo {
// Geometric FOV approximation, not Livox's proprietary firing pattern.
// Raycasts run on the physics thread at actual firing times, including moving arms.
class NeroMid360 : public ModelPlugin {
 public:
  void Load(physics::ModelPtr model, sdf::ElementPtr sdf) override {
    model_ = model; world_ = model->GetWorld(); node_ = gazebo_ros::Node::Get(sdf);
    link_ = model->GetLink(sdf->Get<std::string>("link_name", "mid360_link").first);
    if (!link_) { RCLCPP_FATAL(node_->get_logger(), "Missing preserved mid360_link"); return; }
    frame_ = sdf->Get<std::string>("frame_id", "mid360_link").first;
    rate_ = sdf->Get<double>("points_per_second", 32000).first;
    scan_dt_ = 1. / sdf->Get<double>("scan_hz", 10).first;
    imu_dt_ = 1. / sdf->Get<double>("imu_hz", 200).first;
    min_ = sdf->Get<double>("min_range", .1).first;
    max_ = sdf->Get<double>("max_range", 40.).first;
    ray_ = boost::dynamic_pointer_cast<physics::RayShape>(world_->Physics()->CreateShape("ray", physics::CollisionPtr()));
    if (!ray_) { RCLCPP_FATAL(node_->get_logger(), "Physics engine has no ray shape"); return; }
    cloud_pub_ = node_->create_publisher<livox_ros_driver2::msg::CustomMsg>("/mid360/lidar", rclcpp::QoS(10).reliable());
    imu_pub_ = node_->create_publisher<sensor_msgs::msg::Imu>("/mid360/imu", rclcpp::QoS(50).reliable());
    points_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>("/mid360/points", rclcpp::SensorDataQoS());
    stats_pub_ = node_->create_publisher<std_msgs::msg::String>("/mid360/diagnostics", 10);
    start_ = last_ = last_imu_ = world_->SimTime().Double();
    ready_after_ = start_ + 8.0;
    ResetImuVelocity(start_);
    update_ = event::Events::ConnectWorldUpdateEnd(std::bind(&NeroMid360::Update, this));
    RCLCPP_INFO(node_->get_logger(), "MID360 geometric raycasts %.0f/s; self returns retained; ideal co-located IMU", rate_);
  }
 private:
  builtin_interfaces::msg::Time Stamp(double t) {
    return rclcpp::Time(static_cast<int64_t>(std::llround(t*1e9)));
  }
  void ResetImuVelocity(double now) {
    const auto v = link_->WorldLinearVel();
    imu_acceleration_.Reset(now, {v.X(), v.Y(), v.Z()});
  }
  void Update() {
    const double now = world_->SimTime().Double();
    if (now < last_) { start_=last_=last_imu_=now; ready_after_=now+8.; ResetImuVelocity(now); cloud_.points.clear(); budget_=0; return; }
    if (now < ready_after_) { start_=last_=last_imu_=now; ResetImuVelocity(now); return; }
    const auto pose = link_->WorldPose();
    if (now - last_imu_ + 1e-9 >= imu_dt_) {
      sensor_msgs::msg::Imu msg; msg.header.stamp=Stamp(now); msg.header.frame_id=frame_;
      // Integrate over the IMU period instead of aliasing one physics-step
      // contact impulse. Genuine base/lift/rotational accelerations remain.
      const auto velocity = link_->WorldLinearVel();
      const auto integrated = imu_acceleration_.Sample(now, {velocity.X(), velocity.Y(), velocity.Z()});
      if (integrated) {
        const ignition::math::Vector3d world_acceleration((*integrated)[0], (*integrated)[1], (*integrated)[2]);
        auto a=pose.Rot().RotateVectorReverse(world_acceleration-world_->Gravity());
        auto w=pose.Rot().RotateVectorReverse(link_->WorldAngularVel());
        msg.linear_acceleration.x=a.X();msg.linear_acceleration.y=a.Y();msg.linear_acceleration.z=a.Z();
        msg.angular_velocity.x=w.X();msg.angular_velocity.y=w.Y();msg.angular_velocity.z=w.Z();
        msg.orientation_covariance[0]=-1; // no attitude truth sent to the estimator
        imu_pub_->publish(msg);
      }
      last_imu_=now;
    }
    budget_ += (now-last_)*rate_; last_=now;
    const int count=static_cast<int>(budget_);budget_-=count;
    for (int i=0;i<count;++i) {
      // Deterministic low-discrepancy azimuth/elevation samples inside MID360 FOV.
      const double az=2*M_PI*std::fmod(index_*.6180339887498949,1.);
      const double el=(-7.+59.*std::fmod(index_*.4142135623730950,1.))*M_PI/180.;
      ++index_; ++attempts_;
      ignition::math::Vector3d direction(std::cos(el)*std::cos(az),std::cos(el)*std::sin(az),std::sin(el));
      auto wd=pose.Rot().RotateVector(direction);
      ray_->SetPoints(pose.Pos()+wd*min_,pose.Pos()+wd*max_);
      double range;std::string entity;ray_->GetIntersection(range,entity);
      // Rays start at the configured blind radius, outside the sensor housing.
      // Retain the first collision, including chassis, torso and arm links.
      range += min_;
      if (entity.empty() || range>max_) continue;
      const bool self=entity.rfind(model_->GetScopedName()+"::",0)==0;
      if(self)++self_;else ++environment_;
      livox_ros_driver2::msg::CustomPoint p;
      p.x=direction.X()*range;p.y=direction.Y()*range;p.z=direction.Z()*range;
      p.offset_time=static_cast<uint32_t>(std::llround((now-start_)*1e9));
      p.reflectivity=100;p.tag=0x10;p.line=static_cast<uint8_t>(index_%4);
      cloud_.points.push_back(p);
    }
    if(now-start_+1e-9>=scan_dt_) {
      cloud_.header.stamp=Stamp(start_);cloud_.header.frame_id=frame_;
      cloud_.timebase=static_cast<uint64_t>(std::llround(start_*1e9));
      cloud_.point_num=cloud_.points.size(); cloud_pub_->publish(cloud_);
      sensor_msgs::msg::PointCloud2 view;view.header=cloud_.header;
      sensor_msgs::PointCloud2Modifier modifier(view);
      modifier.setPointCloud2Fields(4,"x",1,sensor_msgs::msg::PointField::FLOAT32,
          "y",1,sensor_msgs::msg::PointField::FLOAT32,"z",1,sensor_msgs::msg::PointField::FLOAT32,
          "intensity",1,sensor_msgs::msg::PointField::FLOAT32);
      modifier.resize(cloud_.points.size());
      sensor_msgs::PointCloud2Iterator<float> x(view,"x"),y(view,"y"),z(view,"z"),v(view,"intensity");
      for(const auto &p:cloud_.points){*x=p.x;*y=p.y;*z=p.z;*v=p.reflectivity;++x;++y;++z;++v;}
      points_pub_->publish(view);
      std_msgs::msg::String stats;
      std::ostringstream text;text<<"{\"stamp\":"<<now<<",\"rays\":"<<attempts_
        <<",\"self_hits\":"<<self_<<",\"environment_hits\":"<<environment_
        <<",\"returns\":"<<cloud_.point_num<<"}";stats.data=text.str();stats_pub_->publish(stats);
      cloud_.points.clear();start_=now;attempts_=self_=environment_=0;
    }
  }
  nero_mid360_sim::IntervalAcceleration imu_acceleration_;
  physics::ModelPtr model_;physics::WorldPtr world_;physics::LinkPtr link_;physics::RayShapePtr ray_;
  gazebo_ros::Node::SharedPtr node_;event::ConnectionPtr update_;
  rclcpp::Publisher<livox_ros_driver2::msg::CustomMsg>::SharedPtr cloud_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr points_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr stats_pub_;
  livox_ros_driver2::msg::CustomMsg cloud_;std::string frame_;
  double rate_,scan_dt_,imu_dt_,min_,max_,start_=0,last_=0,last_imu_=0,budget_=0,ready_after_=0;
  uint64_t index_=0,attempts_=0,self_=0,environment_=0;
};
GZ_REGISTER_MODEL_PLUGIN(NeroMid360)
}
