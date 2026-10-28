#include "nero_official_commander_cpp/moveit_robot.hpp"

// 整个任务在本文件修改；单位 m、rad，夹爪力参数 N。
constexpr bool PREVIEW = true;        // 每段先预览，输入 do 执行；false：直接顺序执行。
const std::string ARM = "right";
const std::string FRAME = "base_link"; // 中间支撑底部：X 前、Y 左、Z 上。
constexpr double radians(double degrees) { return degrees * 3.14159265358979323846 / 180.0; }

void task(nero_task::Robot& robot) {
  // 以下是示例值，先按实测修改。柜顶离地 0.60，支撑底部离地约 0.28。
  constexpr double cabinet_height = 0.60;
  constexpr double support_height = 0.28;
  constexpr double tcp_above_cabinet = 0.10; // TCP 高于柜顶；不等于最低手指的净空。
  constexpr double ready_x = 0.20;          // 夹瓶准备点，不能只沿用贴近身体的起点 XY。
  constexpr double ready_y = -0.40;         // 右臂在机器人右侧；按实测修改。
  constexpr double approach_dx = 0.18;      // 朝机器人前方接近多少，由你测量。
  constexpr double lift_dz = 0.05;          // 夹取后直线上抬 5 cm。
  constexpr double transfer_dx = 0.10;      // 搬运：本体 +X 10 cm。
  constexpr double transfer_dy = 0.20;      // 搬运：本体 +Y 20 cm，与 X 合成一条直线。
  constexpr double lower_dz = -0.04;        // 到达后直线下降 4 cm。
  constexpr double open_width = 0.07;       // 两指总开口 7 cm，按瓶径/实物行程修改。
  constexpr double grip_force = 1.0;        // 官方驱动力参数；0.5..3.0 N，非已标定瓶身接触力。

  std::cout << "任务参数：approach_dx=" << approach_dx << " m（直线接近距离）" << std::endl;

  // ① 点到点，同时到达准备位置和夹瓶朝向；中途由规划器调整各关节。
  // 当前安装模型下：夹爪朝 +X 水平向前，两指沿 Y 左右夹合。
  nero_task::Pose above{
      ready_x, ready_y, cabinet_height - support_height + tcp_above_cabinet,
      radians(-90.0), radians(0.0), radians(0.0)};
  robot.move_p(above);

  // ② 张开。夹爪动作也先在 RViz 预览，输入 do 才下发。
  robot.gripper_open(open_width, grip_force);

  // ③ 保持当前朝向，沿机器人 +X 直线接近瓶子。
  auto bottle = robot.current_pose();
  bottle.x += approach_dx;
  robot.move_l(bottle);

  // ④ 向零开口闭合；碰到瓶子可能无法闭到零，这是正常接触情况。
  // 程序只确认开口稳定，不据此宣称夹住；确认夹持情况后再执行下一段。
  robot.gripper_close(grip_force);

  // ⑤ 保持朝向，直线上抬 5 cm。
  auto lifted = robot.current_pose();
  lifted.z += lift_dz;
  robot.move_l(lifted);

  // ⑥ 同时改变 X/Y，只调用一次 move_l，沿斜向直线到达目标。
  auto placed = robot.current_pose();
  placed.x += transfer_dx;
  placed.y += transfer_dy;
  robot.move_l(placed);

  // ⑦ 保持朝向，直线下降 4 cm。
  auto lowered = robot.current_pose();
  lowered.z += lower_dz;
  robot.move_l(lowered);

  // ⑧ 打开夹爪。
  robot.gripper_open(open_width, grip_force);
}

int main(int argc, char** argv) {
  return nero_task::run_robot(argc, argv, task, PREVIEW, ARM, FRAME);
}
