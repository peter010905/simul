"""Nonblocking confirmation that Gazebo's physical drive controller is active."""
import time

from controller_manager_msgs.srv import ListControllers


class DriveReadiness:
    def __init__(self, node, clock=time.monotonic):
        self.clock = clock
        self.client = node.create_client(ListControllers, 'controller_manager/list_controllers')
        self.states = {}
        self.pending = None
        self.requested = self.updated = -float('inf')

    def poll(self):
        now = self.clock()
        if self.pending is not None:
            if self.pending.done():
                try:
                    self.states = {item.name: item.state for item in self.pending.result().controller}
                    self.updated = now
                except Exception:
                    self.states = {}
                self.pending = None
            elif now - self.requested > 3.:
                self.pending.cancel()
                self.client.remove_pending_request(self.pending)
                self.pending = None
                self.states = {}
        if now - self.updated > 5.:
            self.states = {}
        if self.pending is None and now - self.requested >= 1. and self.client.service_is_ready():
            self.pending = self.client.call_async(ListControllers.Request())
            self.requested = now

    def ready(self):
        self.poll()
        return self.states.get('diffdrive_controller') == 'active'
