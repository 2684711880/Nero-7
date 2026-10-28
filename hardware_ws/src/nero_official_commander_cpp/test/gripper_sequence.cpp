// Offline regression executable: connect only to isolated no-CAN mock stack.
#include "nero_official_commander_cpp/moveit_robot.hpp"
int main(int argc, char** argv) {
  return nero_task::run_robot(argc,argv,[](nero_task::Robot& robot) {
    robot.gripper_open(.06,1.7);
    robot.gripper_close(.8);
  });
}
