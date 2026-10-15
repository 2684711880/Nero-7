"""Single-threaded scheduler. Backends must deliver callbacks on its owning thread."""
import copy
import time
from .model import LANES, validate_task


class Engine:
    def __init__(self, backend, emit=lambda state: None, clock=time.monotonic):
        self.backend, self.emit, self.clock = backend, emit, clock
        self.records = {}
        self.active = None
        self.running = {}
        self.stop_pending = set()
        self.stop_failed = False

    def snapshot(self, task_id):
        return copy.deepcopy(self.records[task_id]['state'])

    def publish(self):
        if self.active:
            self.emit(self.snapshot(self.active))

    def submit(self, raw):
        task = validate_task(raw)
        key = task['task_id']
        if key in self.records:
            if task != self.records[key]['task']:
                raise ValueError('TASK_ID_CONFLICT: same id with different content')
            return self.snapshot(key)
        if self.active:
            raise ValueError('BUSY: task '+self.active+' owns the robot')
        if len(self.records) >= 10000:
            raise ValueError('Task history full; no task IDs are silently forgotten')
        self.records[key] = {'task': task, 'state': {
            'task_id': key, 'status': 'RUNNING', 'message': '',
            'actions': {a['action_id']: {'lane': lane, 'status': 'PENDING', 'message': ''}
                        for lane in LANES for a in task[lane+'_actions']}}}
        self.active = key
        self.publish()
        return self.snapshot(key)

    def tick(self):
        if not self.active:
            return
        record = self.records[self.active]
        state = record['state']
        if state['status'] != 'RUNNING':
            return
        now = self.clock()
        for lane, (action, handle, deadline) in list(self.running.items()):
            if now >= deadline:
                self.stop('FAILED', 'EXECUTION_TIMEOUT: '+action['action_id'])
                return
        for lane in LANES:
            if not self.active or state['status'] != 'RUNNING':
                return
            if lane in self.running:
                continue
            for action in record['task'][lane+'_actions']:
                item = state['actions'][action['action_id']]
                if item['status'] == 'SUCCEEDED':
                    continue
                if item['status'] != 'PENDING':
                    break
                if not all(state['actions'][d]['status'] == 'SUCCEEDED' for d in action['wait_for']):
                    break
                token = (self.active, action['action_id'])
                item['status'] = 'RUNNING'
                self.running[lane] = (action, None, now+action['timeout_s'])
                self.publish()
                try:
                    handle = self.backend.start(lane, action,
                        lambda ok, message, l=lane, t=token: self.complete(l, t, ok, message))
                    if lane in self.running:
                        self.running[lane] = (action, handle, now+action['timeout_s'])
                except Exception as error:
                    self.complete(lane, token, False, str(error))
                break
        if self.active and state['status'] == 'RUNNING' and all(
                x['status'] == 'SUCCEEDED' for x in state['actions'].values()):
            state['status'] = 'SUCCEEDED'
            self.publish()
            self.active = None

    def complete(self, lane, token, success, message):
        if self.active != token[0] or lane not in self.running:
            return  # Late feedback cannot complete a different action/task.
        action = self.running[lane][0]
        if action['action_id'] != token[1]:
            return
        state = self.records[self.active]['state']
        if state['status'] != 'RUNNING':
            return  # Cancellation has its own stop-confirmation callback.
        if success:
            self.running.pop(lane)
        state['actions'][token[1]].update(status='SUCCEEDED' if success else 'FAILED', message=message)
        self.publish()
        if not success:
            self.stop('FAILED', 'ACTION_FAILED: '+token[1]+': '+message)

    def cancel(self, task_id):
        if self.active != task_id:
            raise ValueError('Task is not active')
        if self.records[task_id]['state']['status'] == 'RUNNING':
            self.stop('CANCELED', 'Cancellation requested')
        return self.snapshot(task_id)

    def stop(self, final_status, message):
        record = self.records[self.active]
        record['final_status'] = final_status
        state = record['state']
        state.update(status='STOPPING', message=message)
        for item in state['actions'].values():
            if item['status'] == 'PENDING':
                item['status'] = 'BLOCKED'
        self.stop_pending = set(self.running)
        if hasattr(self.backend, 'halt_idle'):
            self.stop_pending.add('__idle__')
        self.stop_failed = False
        self.publish()
        if hasattr(self.backend, 'halt_idle'):
            self.backend.halt_idle(self.stopped_idle)
        # Populate all pending lanes before invoking possibly synchronous callbacks.
        for lane, (action, handle, deadline) in list(self.running.items()):
            try:
                self.backend.cancel(handle, lambda ok, msg, l=lane: self.stopped(l, ok, msg))
            except Exception as error:
                self.stopped(lane, False, str(error))
        self.finish_stop()

    def stopped_idle(self, confirmed, message):
        self.stop_pending.discard('__idle__')
        self.stop_failed |= not confirmed
        self.finish_stop()

    def stopped(self, lane, confirmed, message):
        if lane not in self.stop_pending:
            return
        self.stop_pending.remove(lane)
        action = self.running.pop(lane)[0]
        self.records[self.active]['state']['actions'][action['action_id']].update(
            status=('FAILED' if self.records[self.active]['state']['actions'][action['action_id']]['status'] == 'FAILED' else 'CANCELED') if confirmed else 'STOP_UNCONFIRMED', message=message)
        self.stop_failed |= not confirmed
        self.finish_stop()

    def finish_stop(self):
        if not self.active or self.stop_pending:
            return
        record = self.records[self.active]
        if record['state']['status'] != 'STOPPING':
            return
        record['state']['status'] = 'STOP_UNCONFIRMED' if self.stop_failed else record['final_status']
        self.publish()
        # An unknown stop retains resource ownership and rejects new tasks.
        if not self.stop_failed:
            self.active = None


class MockBackend:
    """Deterministic dry execution; never publishes a robot command."""
    def __init__(self, clock=time.monotonic, durations=None, fail=None):
        self.clock = clock
        self.durations = durations or {}
        self.fail = set(fail or [])
        self.pending = {}
        self.events = []
        self.counter = 0

    def start(self, lane, action, done):
        self.counter += 1
        key = self.counter
        aid = action['action_id']
        self.pending[key] = (self.clock()+self.durations.get(aid, .3), aid, done)
        self.events.append(('start', lane, aid, self.clock()))
        return key

    def poll(self):
        for key, (deadline, aid, done) in list(self.pending.items()):
            if self.clock() >= deadline:
                del self.pending[key]
                self.events.append(('finish', aid, self.clock()))
                done(aid not in self.fail, 'MOCK_RESULT: no physical motion')

    def cancel(self, handle, done):
        self.pending.pop(handle, None)
        done(True, 'MOCK_STOP: no physical motion')
