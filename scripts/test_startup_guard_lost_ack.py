"""Exercise a lost lifecycle acknowledgement against actual ROS services."""
import threading
import time

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from lifecycle_msgs.srv import ChangeState,GetState

from osp_mission.startup_guard import StartupGuard


def test_active_state_is_reobserved_without_duplicate_transition_after_lost_ack(tmp_path):
    context=rclpy.Context();rclpy.init(context=context,domain_id=122)
    peer=Node('delayed_slam_ack_peer',context=context)
    # Separate groups permit state queries while ChangeState's reply is held.
    from rclpy.callback_groups import ReentrantCallbackGroup
    group=ReentrantCallbackGroup();state=[2];requests=[];held=threading.Event();release=threading.Event()
    def observe(req,res):res.current_state.id=state[0];return res
    def change(req,res):
        requests.append(req.transition.id);state[0]=3;held.set()
        release.wait(10.);res.success=True;return res
    peer.create_service(GetState,'slam_toolbox/get_state',observe,callback_group=group)
    peer.create_service(ChangeState,'slam_toolbox/change_state',change,callback_group=group)
    guard=StartupGuard(context=context,parameter_overrides=[Parameter('use_sim_time',value=True),
        Parameter('output_dir',value=str(tmp_path)),Parameter('startup_timeout_wall_s',value=25.)])
    executor=MultiThreadedExecutor(num_threads=3,context=context)
    executor.add_node(peer);executor.add_node(guard)
    thread=threading.Thread(target=executor.spin,daemon=True);thread.start()
    try:
        assert held.wait(8.)
        guard.requested_at=time.monotonic()-16.  # Expire the local acknowledgement deadline.
        deadline=time.monotonic()+6.
        while time.monotonic()<deadline and guard.last_state!=3:time.sleep(.02)
        assert guard.last_state==3
        assert requests==[3]
        assert any(event['event']=='transition_response_missing_requery_state' for event in guard.events)
        assert not guard.finished  # Active alone is insufficient without map/TF/scan.
    finally:
        release.set();executor.shutdown(timeout_sec=5.);thread.join(timeout=5.)
        guard.destroy_node();peer.destroy_node();rclpy.shutdown(context=context)
