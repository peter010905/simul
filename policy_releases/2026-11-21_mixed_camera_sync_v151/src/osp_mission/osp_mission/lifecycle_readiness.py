"""Nonblocking Nav2 lifecycle readiness, including launch-time activation."""
import time
from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import GetState


class Nav2Readiness:
    def __init__(self, node, clock=time.monotonic):
        self.clock = clock
        self.clients = {name: node.create_client(GetState, name + '/get_state')
                        for name in ('controller_server', 'planner_server', 'behavior_server', 'bt_navigator')}
        self.states = {name: State.PRIMARY_STATE_UNKNOWN for name in self.clients}
        self.updated = {name: -float('inf') for name in self.clients}
        self.pending = {}
        self.requested = {}

    def poll(self):
        now = self.clock()
        for name, client in self.clients.items():
            future = self.pending.get(name)
            if future is not None:
                if future.done():
                    try:
                        self.states[name] = future.result().current_state.id
                        self.updated[name] = now
                    except Exception:
                        self.states[name] = State.PRIMARY_STATE_UNKNOWN
                    del self.pending[name]
                elif now-self.requested[name] > 3.:
                    future.cancel()
                    client.remove_pending_request(future)
                    del self.pending[name]
                    self.states[name] = State.PRIMARY_STATE_UNKNOWN
            if now-self.updated[name] > 3.:
                self.states[name] = State.PRIMARY_STATE_UNKNOWN
            if name not in self.pending and now-self.requested.get(name, -float('inf')) >= 1.:
                if client.service_is_ready():
                    self.pending[name] = client.call_async(GetState.Request())
                    self.requested[name] = now

    def ready(self):
        self.poll()
        return all(value == State.PRIMARY_STATE_ACTIVE for value in self.states.values())
