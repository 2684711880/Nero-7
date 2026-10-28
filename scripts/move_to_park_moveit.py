#!/usr/bin/env python3
"""Use the running MoveIt named park pose; no direct SDK/CAN commands."""
import argparse
import sys
import xml.etree.ElementTree as ET
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rcl_interfaces.srv import GetParameters
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint


def main():
    parser=argparse.ArgumentParser(description='通过已运行的双臂 MoveIt 回 park；默认双臂。')
    parser.add_argument('arm', nargs='?', default='both', choices=['both','left','right'])
    args=parser.parse_args()
    group='both_arms' if args.arm=='both' else args.arm+'_arm'
    rclpy.init()
    node=Node('move_to_park')
    handle=None
    try:
        parameters=node.create_client(GetParameters,'/move_group/get_parameters')
        if not parameters.wait_for_service(timeout_sec=5):
            raise RuntimeError('未找到 MoveIt；请先启动 start_dual_nero_moveit.sh。')
        future=parameters.call_async(GetParameters.Request(names=['robot_description_semantic']))
        rclpy.spin_until_future_complete(node,future,timeout_sec=5)
        if not future.done():
            raise RuntimeError('读取 MoveIt 命名姿态超时。')
        semantic=ET.fromstring(future.result().values[0].string_value)
        park=semantic.find(f"group_state[@name='park'][@group='{group}']")
        if park is None:
            raise RuntimeError(f'{group} 中没有 park，请重启更新后的双臂 MoveIt。')
        goal=MoveGroup.Goal()
        goal.request.group_name=group
        goal.request.start_state.is_diff=True  # MoveIt 使用当前反馈状态作为起点。
        goal.request.allowed_planning_time=5.0
        goal.request.num_planning_attempts=1
        goal.request.max_velocity_scaling_factor=0.1
        goal.request.max_acceleration_scaling_factor=0.1
        constraints=Constraints(name='park')
        for joint in park.findall('joint'):
            constraints.joint_constraints.append(JointConstraint(joint_name=joint.get('name'),
                position=float(joint.get('value')),tolerance_above=0.001,
                tolerance_below=0.001,weight=1.0))
        goal.request.goal_constraints=[constraints]
        goal.planning_options.plan_only=False
        client=ActionClient(node,MoveGroup,'/move_action')
        if not client.wait_for_server(timeout_sec=5):
            raise RuntimeError('未找到 MoveIt move_action。')
        print(f'{group} → park：J2=85°，J4=5°，其余=0°；开始规划并执行。',flush=True)
        future=client.send_goal_async(goal)
        rclpy.spin_until_future_complete(node,future)
        handle=future.result()
        if not handle.accepted:
            raise RuntimeError('MoveIt 拒绝了 park 请求。')
        result=handle.get_result_async()
        rclpy.spin_until_future_complete(node,result)
        code=result.result().result.error_code.val
        if code!=1:
            raise RuntimeError(f'MoveIt park 失败，错误码 {code}；未绕过规划或碰撞检查。')
        print('MoveIt 报告 park 执行成功。',flush=True)
        return 0
    except KeyboardInterrupt:
        if handle is not None:
            cancel=handle.cancel_goal_async()
            rclpy.spin_until_future_complete(node,cancel,timeout_sec=3)
        print('\n已请求取消 park。',flush=True)
        return 130
    except Exception as error:
        print(str(error),file=sys.stderr)
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__=='__main__':
    sys.exit(main())
