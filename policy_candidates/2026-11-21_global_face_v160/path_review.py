"""Asynchronous Nav2 path-length verification for a short list of safe observations."""
import math
from action_msgs.msg import GoalStatus
from nav2_msgs.action import ComputePathToPose
from rclpy.action import ActionClient


class PathReview:
    def __init__(self,node):
        self.node=node
        self.client=ActionClient(node,ComputePathToPose,'compute_path_to_pose')
        self.round=None
        self.sequence=0
        self.next_retry=0.
        self.failures={}
        self.rejected={}
        self.queries=0
        self.strikes={}

    @staticmethod
    def key(g):
        return (g.get('object_face'),round(g['x']/.20),round(g['y']/.20),round(g['yaw']/.35))

    def reject(self,g,reason):
        self.failures[reason]=self.failures.get(reason,0)+1
        key=self.key(g)
        count=self.strikes.get(key,0)+1
        self.strikes[key]=count
        self.rejected[key]=self.node.now()+min(120.,12.*2**min(count-1,4))

    def discard(self):
        current=self.round
        self.round=None
        if current and current.get('handle'):
            current['handle'].cancel_goal_async()

    def start(self,ranked,now,pose):
        if now<self.next_retry or not self.client.server_is_ready():return
        self.rejected={k:v for k,v in self.rejected.items() if v>now}
        goals=[g for g in ranked if self.key(g) not in self.rejected][:3]
        if not goals:
            self.next_retry=now+1.
            return
        current=dict(goals=goals,results=[],index=0,start=now,pose=tuple(pose[:2]),handle=None,done=False)
        self.round=current
        self.query(current)

    def query(self,current):
        if current is not self.round:return
        if current['index']>=len(current['goals']):
            current['done']=True
            return
        g=current['goals'][current['index']]
        req=ComputePathToPose.Goal()
        req.goal.header.frame_id=self.node.p('map_frame')
        req.goal.header.stamp=self.node.get_clock().now().to_msg()
        req.goal.pose.position.x=float(g['x']);req.goal.pose.position.y=float(g['y'])
        req.goal.pose.orientation.z=math.sin(g['yaw']/2);req.goal.pose.orientation.w=math.cos(g['yaw']/2)
        req.use_start=False
        self.queries+=1
        def complete(length,reason):
            if current is not self.round:return
            if reason:
                self.reject(g,reason)
            else:
                self.strikes.pop(self.key(g),None)
            current['results'].append((g,length,reason))
            current['handle']=None
            current['index']+=1
            self.query(current)
        def result(future):
            if current is not self.round:return
            try:
                resp=future.result(); path=resp.result.path
                pts=[(p.pose.position.x,p.pose.position.y) for p in path.poses]
                if resp.status!=GoalStatus.STATUS_SUCCEEDED or resp.result.error_code or not pts:
                    return complete(None,'planner_error_'+str(resp.result.error_code))
                if not math.isfinite(sum(x+y for x,y in pts)) or math.dist(pts[-1],(g['x'],g['y']))>.23:
                    return complete(None,'invalid_path_endpoint')
                length=math.dist(current['pose'],pts[0])+sum(math.dist(a,b) for a,b in zip(pts,pts[1:]))
            except Exception:
                return complete(None,'planner_result_error')
            complete(length,None)
        def accepted(future):
            try: handle=future.result()
            except Exception:return complete(None,'planner_request_error')
            if current is not self.round:
                if handle.accepted: handle.cancel_goal_async()
                return
            if not handle.accepted:return complete(None,'planner_rejected')
            current['handle']=handle
            handle.get_result_async().add_done_callback(result)
        self.client.send_goal_async(req).add_done_callback(accepted)

    def poll(self,now):
        if self.round and not self.round['done'] and now-self.round['start']>5.:
            current=self.round
            handle=current['handle']
            if handle:handle.cancel_goal_async()
            # Invalidate late callbacks; keep only the completed, successful paths.
            goal=current['goals'][current['index']]
            self.reject(goal,'review_timeout')
            self.round=dict(current,done=True,handle=None,results=current['results']+[(goal,None,'review_timeout')])

    def take(self,now):
        self.poll(now)
        if not self.round or not self.round['done']:return None
        current=self.round
        self.round=None
        return current
