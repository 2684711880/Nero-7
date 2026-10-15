"""Asynchronous simulation adapters. No CAN/official hardware driver is started."""
import copy
import json
import math
import time
from types import SimpleNamespace
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import ModelStates
from std_srvs.srv import SetBool
from moveit_msgs.srv import ApplyPlanningScene, GetPlanningScene
from moveit_msgs.msg import (PlanningScene, PlanningSceneComponents, CollisionObject,
                             AttachedCollisionObject, AllowedCollisionEntry)
from shape_msgs.msg import SolidPrimitive
from nero_task_interfaces.action import ExecuteMotion


def wrap(x):
    return math.atan2(math.sin(x), math.cos(x))


class SimBackend:
    def __init__(self, node):
        if not node.get_parameter('use_sim_time').value:
            raise ValueError('Simulation backend requires use_sim_time:=true')
        self.node = node
        self.clients = {lane: ActionClient(node, ExecuteMotion, '/task_motion/'+lane)
                        for lane in ('left_arm', 'right_arm')}
        self.velocity = node.create_publisher(Twist, '/cmd_vel', 10)
        self.odom = None
        self.odom_received = 0.
        self.bottle = None
        self.bottle_received = 0.
        self.attached = False
        self.operations = []
        self.hold = None
        self.hold_error = None
        self.arrival_target = None
        self.arrived = False
        self.constraint = node.create_client(SetBool, '/sim_grasp/right/attach')
        self.scene_get = node.create_client(GetPlanningScene, '/get_planning_scene')
        self.scene_apply = node.create_client(ApplyPlanningScene, '/apply_planning_scene')
        node.create_subscription(Odometry, '/odom', self.on_odom, 10)
        node.create_subscription(ModelStates, '/gazebo/model_states', self.on_models, 10)

    def on_odom(self, msg):
        p, v = msg.pose.pose, msg.twist.twist
        q = p.orientation
        values = [p.position.x, p.position.y, q.x, q.y, q.z, q.w,
                  v.linear.x, v.linear.y, v.angular.z]
        if not all(math.isfinite(x) for x in values) or abs(sum(x*x for x in values[2:6])-1) > 1e-4:
            return
        if msg.header.frame_id != 'odom' or msg.child_frame_id != 'base_link':
            return
        self.odom = msg
        self.odom_received = time.monotonic()

    def on_models(self, msg):
        if 'water_bottle' in msg.name:
            self.bottle = msg.pose[msg.name.index('water_bottle')]
            self.bottle_received = time.monotonic()

    def start(self, lane, action, done):
        if any(x.lane == lane and not x.finished for x in self.operations):
            raise ValueError('Resource already occupied')
        op = SimpleNamespace(lane=lane, action=action, done=done, finished=False,
            canceled=False, stop_done=None, stop_deadline=None, goal=None, awaiting=False,
            settled=None, route=None, route_index=0)
        self.operations.append(op)
        if lane == 'base':
            if self.velocity.get_subscription_count() == 0:
                self.finish(op, False, 'No /cmd_vel receiver')
                return op
            p = action['parameters']
            op.route = list(p.get('via', []))+[[*p['position_xy_m'], p['yaw_rad']]]
            self.hold = None
        else:
            p = action['parameters']
            if p.get('touch_object'):
                self.allow_contact(op, lambda: self.send_arm(op))
            elif action['type'] == 'gripper_open' and self.attached and lane == 'right_arm':
                # Detach before opening; retain the object in the world planning scene.
                self.grasp_constraint(op, False, lambda: self.attach_scene(op, False, lambda: self.send_arm(op)))
            else:
                self.send_arm(op)
        return op

    def finish(self, op, success, message, stop_confirmed=True):
        if op.finished:
            return
        op.finished = True
        op.stop_confirmed = stop_confirmed
        if op.canceled:
            if op.stop_done:
                op.stop_done(stop_confirmed and 'STOP_UNCONFIRMED' not in message, message)
        else:
            op.done(success, message)

    def service(self, op, client, request, continuation):
        if op.canceled:
            self.finish(op, False, 'Stopped before service dispatch')
            return
        if not client.service_is_ready():
            self.finish(op, False, 'Missing service: '+client.srv_name)
            return
        op.awaiting = True
        future = client.call_async(request)
        def response(f):
            op.awaiting = False
            try:
                result = f.result()
                if op.canceled:
                    # Service effects may already have happened; never run the next stage.
                    self.finish(op, False, 'STOP_UNCONFIRMED: service effects require scene/grasp reconciliation', stop_confirmed=False)
                else:
                    continuation(result)
            except Exception as error:
                self.finish(op, False, str(error), stop_confirmed=False)
        future.add_done_callback(response)

    def apply(self, op, scene, continuation):
        scene.is_diff = True
        scene.robot_state.is_diff = True
        def applied(result):
            if not result.success:
                self.finish(op, False, 'Planning scene update failed')
            else:
                continuation()
        self.service(op, self.scene_apply, ApplyPlanningScene.Request(scene=scene), applied)

    def allow_contact(self, op, continuation):
        request = GetPlanningScene.Request()
        request.components.components = PlanningSceneComponents.ALLOWED_COLLISION_MATRIX
        def received(result):
            acm = result.scene.allowed_collision_matrix
            names = ['water_bottle', 'right_gripper_base', 'right_gripper_link1', 'right_gripper_link2']
            for name in names:
                if name not in acm.entry_names:
                    acm.entry_names.append(name)
                    for row in acm.entry_values:
                        row.enabled.append(False)
                    acm.entry_values.append(AllowedCollisionEntry(enabled=[False]*len(acm.entry_names)))
            bottle = acm.entry_names.index('water_bottle')
            for name in names[1:]:
                index = acm.entry_names.index(name)
                acm.entry_values[bottle].enabled[index] = True
                acm.entry_values[index].enabled[bottle] = True
            scene = PlanningScene(allowed_collision_matrix=acm)
            self.apply(op, scene, continuation)
        self.service(op, self.scene_get, request, received)

    def grasp_constraint(self, op, attach, continuation):
        def response(result):
            if not result.success:
                self.finish(op, False, 'Checked simulation grasp: '+result.message)
            else:
                self.attached = attach
                continuation()
        self.service(op, self.constraint, SetBool.Request(data=attach), response)

    def attach_scene(self, op, attach, continuation):
        if self.bottle is None or time.monotonic()-self.bottle_received > .5:
            self.finish(op, False, 'Missing/stale Gazebo bottle feedback')
            return
        obj = CollisionObject()
        obj.header.frame_id = 'odom'
        obj.id = 'water_bottle'
        obj.operation = CollisionObject.ADD if attach else CollisionObject.REMOVE
        shape = SolidPrimitive(type=SolidPrimitive.CYLINDER, dimensions=[.24, .034])
        if attach:
            obj.primitives = [shape]
            obj.primitive_poses = [copy.deepcopy(self.bottle)]
        body = AttachedCollisionObject(link_name='right_gripper_base', object=obj,
            touch_links=['right_gripper_base', 'right_gripper_link1', 'right_gripper_link2'])
        scene = PlanningScene()
        scene.robot_state.attached_collision_objects = [body]
        if not attach:
            world = CollisionObject()
            world.header.frame_id = 'odom'
            world.id = 'water_bottle'
            world.operation = CollisionObject.ADD
            world.primitives = [shape]
            world.primitive_poses = [copy.deepcopy(self.bottle)]
            scene.world.collision_objects = [world]
        self.apply(op, scene, continuation)

    def send_arm(self, op):
        if op.canceled:
            self.finish(op, False, 'Stopped before arm dispatch')
            return
        client = self.clients[op.lane]
        if not client.server_is_ready():
            self.finish(op, False, 'Missing correlated arm server: '+op.lane)
            return
        goal = ExecuteMotion.Goal(command_json=json.dumps(op.action, allow_nan=False))
        op.awaiting = True
        future = client.send_goal_async(goal)
        def accepted(f):
            op.awaiting = False
            try:
                op.goal = f.result()
                if not op.goal.accepted:
                    self.finish(op, False, 'Arm server rejected command')
                    return
                outcome = op.goal.get_result_async()
                outcome.add_done_callback(lambda result: self.arm_finished(op, result))
                if op.canceled:
                    op.goal.cancel_goal_async()
            except Exception as error:
                self.finish(op, False, str(error), stop_confirmed=False)
        future.add_done_callback(accepted)

    def arm_finished(self, op, future):
        try:
            result = future.result()
            message = result.result.message
            if op.canceled:
                self.finish(op, False, message, stop_confirmed='STOP_UNCONFIRMED' not in message)
            elif result.status != GoalStatus.STATUS_SUCCEEDED or not result.result.success:
                # Any unknown controller stop must latch ownership, even without a user cancel.
                if 'STOP_UNCONFIRMED' in message:
                    op.awaiting = True  # cancel() must not report a confirmed idle resource.
                    op.goal = None
                    op.done(False, message)
                else:
                    self.finish(op, False, message)
            elif op.action['type'] == 'gripper_grasp':
                self.grasp_constraint(op, True, lambda: self.attach_scene(op, True,
                    lambda: self.finish(op, True, 'Checked fixed-joint simulation grasp and scene attachment confirmed')))
            else:
                self.finish(op, True, message)
        except Exception as error:
            self.finish(op, False, str(error), stop_confirmed=False)

    def halt_idle(self, done):
        had_hold = self.hold is not None
        self.hold = None
        self.velocity.publish(Twist())
        if not had_hold or any(op.lane == 'base' and not op.finished for op in self.operations):
            done(True, 'Idle base hold disabled')
            return
        op = SimpleNamespace(lane='base', finished=False, canceled=True,
            stop_done=done, stop_deadline=time.monotonic()+8., settled=None, goal=None)
        self.operations.append(op)

    def cancel(self, op, done):
        if op is None or op.finished:
            done(getattr(op, 'stop_confirmed', True), 'No active motion' if getattr(op, 'stop_confirmed', True) else 'STOP_UNCONFIRMED: previous transport error')
            return
        op.canceled = True
        op.stop_done = done
        op.stop_deadline = time.monotonic()+8.
        if op.lane == 'base':
            self.hold = None
            self.velocity.publish(Twist())
            op.settled = None
        elif op.goal:
            op.goal.cancel_goal_async()
        elif not op.awaiting:
            self.finish(op, False, 'No arm goal in flight')

    def fresh_odom(self):
        if self.odom is None or time.monotonic()-self.odom_received > .5:
            return False
        stamp = self.odom.header.stamp
        age = (self.node.get_clock().now().nanoseconds-stamp.sec*10**9-stamp.nanosec)*1e-9
        return -.1 <= age <= .5

    def base_pose(self):
        p = self.odom.pose.pose
        q = p.orientation
        return p.position.x, p.position.y, math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))

    def stopped_base(self):
        v = self.odom.twist.twist
        return math.hypot(v.linear.x, v.linear.y) < .015 and abs(v.angular.z) < .02

    def drive(self, target, hold=False):
        x, y, yaw = self.base_pose()
        dx, dy, da = target[0]-x, target[1]-y, wrap(target[2]-yaw)
        key = tuple(target)
        if key != self.arrival_target:
            self.arrival_target, self.arrived = key, False
        # Hysteresis prevents feedback noise at the docking boundary from
        # alternating drive/stop forever and resetting the settle timer.
        if math.hypot(dx, dy) < .012 and abs(da) < .015:
            self.arrived = True
        elif math.hypot(dx, dy) > .025 or abs(da) > .03:
            self.arrived = False
        reached = self.arrived
        command = Twist()
        if not reached:
            command.angular.z = max(-.18, min(.18, .8*da))
            # Gazebo wheel contact can stall at the old 0.04..0.08 rad/s
            # tail command despite fresh, aligned steering feedback.
            if abs(da) >= .015 and abs(command.angular.z) < .12:
                command.angular.z = math.copysign(.12, da)
            if abs(da) < .03:
                vx = .6*(math.cos(yaw)*dx+math.sin(yaw)*dy)
                vy = .6*(-math.sin(yaw)*dx+math.cos(yaw)*dy)
                length = math.hypot(vx, vy)
                # Keep a bounded drive floor until entering docking tolerance;
                # the J2=60 posture stalled at a 0.0163 m/s tail command.
                scale = min(.1, max(.04, length))/max(length, 1e-9)
                command.linear.x, command.linear.y = vx*scale, vy*scale
        self.velocity.publish(command)
        return reached and self.stopped_base()

    def poll(self):
        now = time.monotonic()
        for op in list(self.operations):
            if op.finished:
                continue
            if op.canceled and now > op.stop_deadline:
                self.finish(op, False, 'STOP_UNCONFIRMED: no terminal stop feedback', stop_confirmed=False)
                continue
            if op.lane != 'base':
                continue
            if not self.fresh_odom():
                self.velocity.publish(Twist())
                if not op.canceled:
                    # Keep this operation active so cancellation must verify physical stop.
                    op.done(False, 'Missing/stale odometry')
                continue
            settled = False
            if op.canceled:
                self.velocity.publish(Twist())
                settled = self.stopped_base()
            else:
                settled = self.drive(op.route[op.route_index])
            if settled:
                if op.settled is None:
                    op.settled = now
                if now-op.settled >= .4:
                    if op.canceled:
                        self.finish(op, False, 'Base stop verified by odometry')
                    elif op.route_index+1 < len(op.route):
                        op.route_index += 1
                        op.settled = None
                    else:
                        self.hold = op.route[-1]
                        self.finish(op, True, 'Base pose and stopped velocity verified')
            else:
                op.settled = None
        self.operations = [op for op in self.operations if not op.finished]
        if self.hold is not None and not any(op.lane == 'base' for op in self.operations):
            if not self.fresh_odom():
                self.hold_error = 'Docking hold lost odometry'
                self.velocity.publish(Twist())
            else:
                x, y, a = self.base_pose()
                if math.hypot(x-self.hold[0], y-self.hold[1]) > .06 or abs(wrap(a-self.hold[2])) > .08:
                    self.hold_error = 'Base escaped docking tolerance'
                    self.velocity.publish(Twist())
                else:
                    self.drive(self.hold, hold=True)

    def shutdown(self):
        self.hold = None
        if not self.node.context.ok():
            return
        self.velocity.publish(Twist())
        for op in self.operations:
            if op.goal and not op.finished:
                op.goal.cancel_goal_async()
