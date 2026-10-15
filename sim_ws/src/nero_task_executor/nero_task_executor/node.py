"""ROS topic ingress and status; state machine and backend callbacks share one thread."""
import json
import os
import signal
import tempfile
import time
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from std_msgs.msg import String
from .engine import Engine, MockBackend


class TaskExecutor(Node):
    def __init__(self):
        super().__init__('robot_task_executor')
        self.declare_parameter('backend', 'mock')
        self.declare_parameter('mock_fail_actions', [''])
        self.declare_parameter('state_file', '')
        mode = self.get_parameter('backend').value
        if mode not in ('mock', 'simulation'):
            raise ValueError('backend must be mock or simulation; hardware is not supported')
        self.mode = mode
        filename = self.get_parameter('state_file').value
        self.state_file = Path(filename).expanduser() if filename else (
            Path.home()/'.local/state/nero_task_executor'/(mode+'.json'))
        self.state_file.parent.mkdir(parents=True, exist_ok=True)
        # Prevent two executors sharing a journal from independently owning the robot.
        import fcntl
        self.lock_file = self.state_file.with_suffix('.lock').open('a')
        fcntl.flock(self.lock_file, fcntl.LOCK_EX | fcntl.LOCK_NB)
        qos = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.status = self.create_publisher(String, '/robot_task/status', qos)
        if mode == 'mock':
            self.backend = MockBackend(fail=self.get_parameter('mock_fail_actions').value)
        else:
            from .sim_backend import SimBackend
            self.backend = SimBackend(self)
        self.engine = Engine(self.backend, self.on_state)
        if self.state_file.exists():
            saved = json.loads(self.state_file.read_text())
            if saved['backend'] != mode:
                raise ValueError('Journal backend does not match')
            self.engine.records = saved['records']
            for key, record in self.engine.records.items():
                if record['state']['status'] in ('RUNNING', 'STOPPING', 'STOP_UNCONFIRMED'):
                    record['state'].update(status='STOP_UNCONFIRMED',
                        message='Interrupted execution recovered; verify robot stopped before journal recovery')
                    self.engine.active = key
                    self.on_state(record['state'])
        self.create_subscription(String, '/robot_task/data', self.receive, 10)
        self.create_subscription(String, '/robot_task/cancel', self.cancel, 10)
        self.create_subscription(String, '/robot_task/query', self.query, 10)
        self.timer = self.create_timer(.02, self.tick, clock=rclpy.clock.Clock(clock_type=rclpy.clock.ClockType.STEADY_TIME))
        self.get_logger().info(f'READY backend={mode}, journal={self.state_file}')

    def persist(self):
        blob = json.dumps({'backend': self.mode, 'records': self.engine.records}, allow_nan=False)
        fd, filename = tempfile.mkstemp(dir=self.state_file.parent, prefix='.task-state-')
        try:
            with os.fdopen(fd, 'w') as out:
                out.write(blob)
                out.flush()
                os.fsync(out.fileno())
            os.replace(filename, self.state_file)
        finally:
            if os.path.exists(filename):
                os.unlink(filename)

    def output(self, state):
        data = dict(state, execution_mode=self.mode)
        self.status.publish(String(data=json.dumps(data, allow_nan=False)))

    def on_state(self, state):
        self.persist()  # Commit acceptance/state BEFORE any subsequent dispatch.
        self.output(state)
        self.get_logger().info(json.dumps(dict(state, execution_mode=self.mode), ensure_ascii=False))

    def rejected(self, msg, error):
        key = ''
        try:
            key = json.loads(msg.data).get('task_id', '')
        except (ValueError, AttributeError):
            pass
        self.output({'task_id': key, 'status': 'REJECTED', 'message': str(error)})

    def receive(self, msg):
        try:
            self.output(self.engine.submit(msg.data))
        except (ValueError, TypeError, RecursionError) as error:
            self.rejected(msg, error)

    def cancel(self, msg):
        try:
            self.output(self.engine.cancel(msg.data))
        except ValueError as error:
            self.output({'task_id': msg.data, 'status': 'REJECTED', 'message': str(error)})

    def query(self, msg):
        if msg.data in self.engine.records:
            self.output(self.engine.snapshot(msg.data))
        else:
            self.output({'task_id': msg.data, 'status': 'UNKNOWN', 'message': 'Unknown task'})

    def tick(self):
        self.backend.poll()
        if getattr(self.backend, 'hold_error', None) and self.engine.active:
            if self.engine.records[self.engine.active]['state']['status'] == 'RUNNING':
                self.engine.stop('FAILED', self.backend.hold_error)
        self.engine.tick()


def main():
    from rclpy.signals import SignalHandlerOptions
    # Keep ROS alive while SIGINT/SIGTERM initiates cancellation and cleanup.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    def interrupt(signum, frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, interrupt)
    node = TaskExecutor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        if node.engine.active and node.engine.records[node.engine.active]['state']['status'] == 'RUNNING':
            node.engine.cancel(node.engine.active)
            deadline = time.monotonic()+9
            while rclpy.ok() and node.engine.active and time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=.05)
    finally:
        if hasattr(node.backend, 'shutdown'):
            node.backend.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
