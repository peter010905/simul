from concurrent.futures import Future
from types import SimpleNamespace as Obj
from osp_mission.lifecycle_readiness import Nav2Readiness
from osp_mission.mission import Mission


class Client:
    def __init__(self):
        self.requests=[]
        self.available=True
    def service_is_ready(self):return self.available
    def call_async(self, request):
        future=Future();self.requests.append(future);return future
    def remove_pending_request(self, future):pass
    def respond(self, state):self.requests[-1].set_result(Obj(current_state=Obj(id=state)))


def setup():
    clients={};now=[0.]
    def create(_,name):
        client=Client();clients[name]=client;return client
    readiness=Nav2Readiness(Obj(create_client=create),clock=lambda:now[0])
    return readiness,clients,now


def test_discovered_action_servers_do_not_mean_lifecycle_is_active():
    readiness,clients,now=setup()
    assert not readiness.ready()
    for client in clients.values():client.respond(2)  # inactive but discovered
    assert not readiness.ready()
    now[0]=1.;readiness.poll()
    for client in clients.values():client.respond(3)
    assert readiness.ready()
    # Mission readiness itself must enforce the lifecycle result, even when
    # map, scan and both action server endpoints already exist.
    readiness.states['behavior_server']=2
    owner=Obj(map=object(),scan_stamp=1.,sensor_fresh=lambda _:True,
              nav=Obj(server_is_ready=lambda:True),spin=Obj(server_is_ready=lambda:True),
              nav2_state=readiness)
    assert Mission.readiness_reason(owner)=='nav2_unavailable'


def test_lost_lifecycle_service_expires_and_can_recover_without_blocking():
    readiness,clients,now=setup();readiness.poll()
    for client in clients.values():client.respond(3)
    assert readiness.ready()
    now[0]=1.;readiness.poll()  # requests no longer receive responses
    now[0]=4.1
    assert not readiness.ready()
    for client in clients.values():client.respond(3)
    assert readiness.ready()


def test_explicit_deactivation_prevents_further_actions():
    readiness,clients,now=setup();readiness.poll()
    for client in clients.values():client.respond(3)
    assert readiness.ready()
    now[0]=1.;readiness.poll()
    for name,client in clients.items():client.respond(2 if name=='controller_server/get_state' else 3)
    assert not readiness.ready()
