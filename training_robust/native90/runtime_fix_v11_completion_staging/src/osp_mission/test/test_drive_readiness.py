from concurrent.futures import Future
from types import SimpleNamespace as Obj

from osp_mission.drive_readiness import DriveReadiness
from osp_mission.mission import Mission


class Client:
    def __init__(self):
        self.requests=[]
    def service_is_ready(self):return True
    def call_async(self, request):
        future=Future();self.requests.append(future);return future
    def remove_pending_request(self, future):pass
    def respond(self, **states):
        self.requests[-1].set_result(Obj(controller=[Obj(name=k,state=v) for k,v in states.items()]))


def test_joint_state_broadcaster_and_nav2_do_not_prove_drive_ready():
    client=Client();now=[0.]
    ready=DriveReadiness(Obj(create_client=lambda *args:client),clock=lambda:now[0])
    assert not ready.ready()
    client.respond(joint_state_broadcaster='active')
    assert not ready.ready()
    now[0]=1.;ready.poll();client.respond(diffdrive_controller='inactive')
    assert not ready.ready()
    now[0]=2.;ready.poll();client.respond(diffdrive_controller='active')
    assert ready.ready()
    owner=Obj(map=object(),scan_stamp=1.,sensor_fresh=lambda value:value is not None,
              nav=Obj(server_is_ready=lambda:True),spin=Obj(server_is_ready=lambda:True),
              nav2_state=Obj(ready=lambda:True),drive_state=ready,wheel_stamp=None)
    assert Mission.readiness_reason(owner)=='drive_unavailable'


def test_lost_drive_response_is_bounded_and_recoverable():
    client=Client();now=[0.]
    ready=DriveReadiness(Obj(create_client=lambda *args:client),clock=lambda:now[0])
    ready.poll();client.respond(diffdrive_controller='active');assert ready.ready()
    now[0]=1.;ready.poll()
    now[0]=4.1;assert not ready.ready()
    client.respond(diffdrive_controller='active');assert ready.ready()
