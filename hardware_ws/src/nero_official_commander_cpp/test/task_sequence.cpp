// 无 CAN 测试专用：验证一个任务中依次调用三种运动，以及取消后停止后续动作。
#include "nero_official_commander_cpp/moveit_robot.hpp"
int main(int argc,char** argv) {
  return nero_task::run_robot(argc,argv,[](nero_task::Robot& robot) {
    auto b=robot.current_pose(); b.x+=.01; robot.move_p(b);
    auto c=robot.current_pose(); c.z+=.01; robot.move_l(c);
    robot.circle(.01,"xy");
  });
}
