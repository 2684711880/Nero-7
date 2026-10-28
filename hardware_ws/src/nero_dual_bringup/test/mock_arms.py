"""Synthetic feedback and SetBool services for offline park testing. No SDK/CAN."""
import json
from pathlib import Path
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import SetBool

rclpy.init()
node=Node('mock_arms_no_can')
names=[f'joint{i}' for i in range(1,8)]+['gripper']
states={side:dict(zip(names,[0.0]*7+[0.04])) for side in ('left','right')}
gates={side:False for side in states}
publishers={side:node.create_publisher(JointState,f'/{side}/feedback/joint_states',10) for side in states}
for side in states:
    def gate(request,response,s=side):
        gates[s]=request.data
        response.success=True
        return response
    def command(message,s=side):
        if gates[s]:
            states[s].update(zip(message.name,message.position))
    node.create_service(SetBool,f'/{side}/control_enable',gate)
    node.create_subscription(JointState,f'/{side}/control/joint_states',command,10)

def publish():
    for side,publisher in publishers.items():
        msg=JointState(name=names,position=[states[side][name] for name in names])
        msg.header.stamp=node.get_clock().now().to_msg()
        publisher.publish(msg)

node.create_timer(0.01,publish)
try:
    rclpy.spin(node)
except KeyboardInterrupt:
    pass
finally:
    out=Path('/home/a/Nero-7/hardware_ws/validation/dual_moveit/park_mock_final.json')
    out.write_text(json.dumps({'hardware_connected':False,'joint_positions_rad':states},indent=2)+'\n')
    node.destroy_node()
    rclpy.shutdown()
