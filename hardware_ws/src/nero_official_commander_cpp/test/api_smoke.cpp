#include "nero_official_commander_cpp/arm_commander.hpp"

int main(int argc,char** argv) {
  rclcpp::init(argc,argv);
  auto node=std::make_shared<rclcpp::Node>("arm_api_smoke");
  const bool send=node->declare_parameter("send",false);
  const auto mode=node->declare_parameter<std::string>("mode","sim");
  int result=0;
  try {
    if(mode=="mock") {
      // Test domain uses subscribers with a mock SDK; never start an official driver.
      ArmCommander robot(node,"left","left_arm_base",send,ArmCommander::Backend::OfficialDriver);
      robot.move_j({0.1,0.2,0.3,0.4,0.5,0.6,0.7});
      const ArmCommander::Pose6 a={-.4,-.2,.3,0,0,0},b={-.4,0,.4,0,0,0},c={-.4,.2,.3,0,0,0};
      robot.move_p(a);robot.move_l(c);robot.move_c(a,b,c);
    } else if(mode=="sim") {
      ArmCommander robot(node,"left","base_link",send);
      robot.move_j({0,0,0,1,0,0,0});
      auto p=robot.currentPose();p.position.x+=.02;robot.move_p(p);
      p=robot.currentPose();p.position.x+=.10;robot.move_l(p);
      robot.move_j({0,0,0,1,0,0,0});
      auto a=robot.currentPose(),b=a,c=a;
      b.position.y-=.005;b.position.z+=.005;c.position.y-=.01;
      robot.move_c(a,b,c);
    } else throw std::invalid_argument("mode must be sim or mock");
  } catch(const std::exception& e) { RCLCPP_ERROR(node->get_logger(),"%s",e.what());result=1; }
  rclcpp::shutdown();return result;
}
