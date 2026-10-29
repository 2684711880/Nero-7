#!/usr/bin/env python3
"""Replace a cached RViz trajectory with current feedback; never commands motion."""
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import DurabilityPolicy, QoSProfile
from moveit_msgs.msg import DisplayTrajectory, PlanningSceneComponents, RobotTrajectory
from moveit_msgs.srv import GetPlanningScene
from trajectory_msgs.msg import JointTrajectoryPoint

# MoveIt also publishes /display_planned_path. Captured shared-topic messages
# included old starts that overwrote current plans; isolate reviewed arm plans.
PREVIEW_TOPIC = '/nero/real_hole/display_planned_path'


def preview_publisher(node):
    return node.create_publisher(DisplayTrajectory, PREVIEW_TOPIC,
                                QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))


def current_display(state):
    values = dict(zip(state.joint_state.name, state.joint_state.position))
    names = [f'{side}_joint{i}' for side in ('left','right') for i in range(1,8)]
    names += [f'{side}_gripper' for side in ('left','right')]
    if any(name not in values for name in names):
        raise RuntimeError('Current scene is missing joints; cannot reset preview')
    trajectory = RobotTrajectory()
    trajectory.joint_trajectory.joint_names = names
    trajectory.joint_trajectory.points = [JointTrajectoryPoint(positions=[values[name] for name in names])]
    message = DisplayTrajectory()
    message.model_id = 'dual_nero'
    message.trajectory_start = state
    message.trajectory = [trajectory]
    return message


def reset_preview(node, publisher=None):
    client = node.create_client(GetPlanningScene, '/get_planning_scene')
    try:
        if not client.wait_for_service(timeout_sec=3):
            raise RuntimeError('MoveIt scene unavailable; preview not reset')
        req = GetPlanningScene.Request()
        req.components.components = PlanningSceneComponents.ROBOT_STATE | PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS
        future = client.call_async(req)
        event = threading.Event()
        future.add_done_callback(lambda _: event.set())
        if not event.wait(4) or future.result() is None:
            raise RuntimeError('Current-state query failed; preview not reset')
        message = current_display(future.result().scene.robot_state)
        if publisher is None:
            publisher = preview_publisher(node)
        deadline = time.monotonic()+2
        while publisher.get_subscription_count()==0 and time.monotonic()<deadline:
            time.sleep(.05)
        publisher.publish(message)
        time.sleep(.2)
        print('RViz 旧轨迹已替换为当前反馈姿态，没有发送机械臂动作。',flush=True)
    finally:
        node.destroy_client(client)


def main():
    rclpy.init();node=Node('real_hole_preview_reset')
    executor=SingleThreadedExecutor();executor.add_node(node)
    thread=threading.Thread(target=executor.spin,daemon=True);thread.start()
    try:reset_preview(node)
    finally:
        executor.shutdown();thread.join();node.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
