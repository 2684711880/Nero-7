#include "nero_official_commander_cpp/arm_commander.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("circle_commander");
  int result = 0;
  try {
    // ① 选择机械臂和参考坐标系。
    const std::string arm = "left";          // "left" 或 "right"
    const std::string frame = "base_link";  // 车体坐标系，不是末端自身坐标系
    const bool send = node->declare_parameter("send", false);
    ArmCommander robot(node, arm, frame, send, ArmCommander::Backend::Simulation);

    // ② 先收臂。这个调用完成后，才会执行后面的画圆代码。
    // 与官方 SDK 同名：move_j。七个关节绝对角度，单位 rad。
    robot.move_j({0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0});

    // ③ A：圆的起点，直接取收臂后的实际末端位姿。
    // 后面所有点都复制 A，所以 Z 坐标和末端姿态保持不变。
    auto A = robot.currentPose();
    const double radius = 0.05; // 半径 5 cm，直径 10 cm，单位 m

    // ④ 圆心 O 放在 A 的 +X 方向，距离为 radius。
    // 因此 A 正好是圆的 -X 端，不需要额外移动到圆上。
    auto O = A;
    O.position.x += radius;

    // ⑤ 圆上另外三个点（坐标相对于 base_link）：
    auto M1 = O; M1.position.y += radius; // +Y 侧：第一个半圆的中间点
    auto B  = O; B.position.x  += radius; // +X 端：第一个半圆的终点
    auto M2 = O; M2.position.y -= radius; // -Y 侧：第二个半圆的中间点

    RCLCPP_INFO(node->get_logger(),"XY circle: centre=(%.4f, %.4f, %.4f) m, radius=%.3f m",
        O.position.x,O.position.y,O.position.z,radius);

    // ⑥ 官方风格 move_c(起点, 中间点, 终点)，两段半圆组成完整一圈：
    robot.move_c(A, M1, B); // A → +Y 侧 M1 → +X 端 B
    robot.move_c(B, M2, A); // B → -Y 侧 M2 → 起点 A

    // 注意：两段分别规划、执行，在 B 处会减速/停顿。
    // 这是几何完整圆，不是连续恒速圆。先把接口和几何读明白。
  } catch (const std::exception& error) {
    RCLCPP_ERROR(node->get_logger(), "%s", error.what());
    result = 1;
  }
  rclcpp::shutdown();
  return result;
}
