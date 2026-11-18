"""Install contact-aware recovery in the measured MPPI candidate."""
def apply(package):
    def edit(filename, edits):
        path = package/'osp_mission'/filename
        text = path.read_text()
        for old, new in edits:
            if text.count(old) != 1:
                raise ValueError(f'{filename}: expected one {old[:80]}')
            text = text.replace(old, new)
        path.write_text(text)

    edit('mission.py', [
        ('from osp_mission.transit_control import TransitFlow',
         'from osp_mission.contact_inspection import ContactInspection\nfrom osp_mission.transit_control import TransitFlow'),
        ("        self.navigation_flow=(TransitFlow(", "        self.contact_inspection=ContactInspection()\n        self.create_subscription(String,'mission/marker_bearing',self.on_marker_bearing,10)\n        self.navigation_flow=(TransitFlow("),
        ('    def on_hazard(self,m):', '''    def on_marker_bearing(self, m):
        cue=json.loads(m.data)
        age=self.get_clock().now().nanoseconds/1e9-cue['stamp']
        guard=self.contact_inspection
        if (self.state=='running' and not guard.phase and -.1<=age<=.5 and cue['id'] not in self.targets
                and self.now()>=self.spin_blocked_until):
            guard.bearing=(self.now(),cue['angle'])

    def on_hazard(self,m):'''),
        ("        if bump and self.state=='running' and self.now()-self.last_bump>2.:", """        guard=getattr(self,'contact_inspection',None)
        rising=(guard.hazard([d.header.frame_id for d in m.detections if d.type==1],self.now())
                if guard is not None and self.state=='running' else False)
        if bump and self.state=='running' and (rising or (guard is None and self.now()-self.last_bump>2.)):"""),
        ('            visit=pose.translation','''            visit=pose.translation
            if hasattr(self,'contact_inspection'):
                self.contact_inspection.position=(visit.x,visit.y)'''),
        ('        if self.goal_handle or self.pending:\n            if self.now()-self.action_start>65:', '''        guard=getattr(self,'contact_inspection',None)
        now=self.now()
        bearing_fresh=guard is not None and guard.bearing is not None and now-guard.bearing[0]<=2.
        protected=any(getattr(getattr(self,name,None),'pending',None) is not None for name in ('marker_look','face_look'))
        if self.goal_handle or self.pending:
            if (guard is not None and not guard.phase and self.kind=='nav' and not protected
                    and not getattr(getattr(self,'near_pass',None),'protected_navigation',False)
                    and not self.cancel_reason and now>=self.spin_blocked_until
                    and (guard.due(now) or bearing_fresh)):
                self.cancel_action('inspection_due');return
            if self.now()-self.action_start>65:'''),
        ("        if getattr(self,'near_pass',None) is not None and self.near_pass.tick():return", """        if guard is not None and guard.phase:
            self.recovery_pending=False
            self.resume_action=None
            step=guard.recovery(now,self.rear_clear,self.backup.server_is_ready(),self.spin.server_is_ready())
            if step=='hold':
                self.zero();return
            guard.action=step
            if step=='backup':
                goal=BackUp.Goal();goal.target.x=.12;goal.speed=.06;goal.time_allowance.sec=5
                self.send(self.backup,goal,step)
            else:
                goal=Spin.Goal();goal.target_yaw=guard.side*math.radians(65);goal.time_allowance.sec=10
                self.send(self.spin,goal,step)
            self.get_logger().info(f'Contact recovery: {step}')
            return
        if guard is not None and not protected and now>=self.spin_blocked_until and (guard.due(now) or bearing_fresh):
            self.scan_after=True
        if getattr(self,'near_pass',None) is not None and self.near_pass.tick():return"""),
        ('            self.initial_scan=False; goal.time_allowance.sec=15', '''            if guard is not None:
                if not protected_look:
                    if bearing_fresh:
                        goal.target_yaw=max(-.8,min(.8,guard.bearing[1]))
                        guard.bearing=None
                        if abs(goal.target_yaw)<.08:
                            goal.target_yaw=0.;protected_look=True
                    elif abs(goal.target_yaw)<1e-6 and (self.initial_scan or guard.due(now)):
                        goal.target_yaw=guard.survey_angle()
                if goal.target_yaw or bearing_fresh or self.initial_scan:
                    guard.looked(now)
            self.initial_scan=False; goal.time_allowance.sec=15'''),
        ("        if getattr(self,'contact_pending',False):", """        guard=getattr(self,'contact_inspection',None)
        if guard is not None and guard.action:
            self.contact_pending=False
            guard.result(result.status==4,self.now())
            self.scan_after=False
            self.get_logger().info(f'Contact action {self.kind}: status={result.status} next={guard.phase or "replan"}')
            return
        if getattr(self,'cancel_reason','')=='inspection_due' and result.status==5:
            self.scan_after=True;return
        if guard is not None and self.kind=='nav' and result.status==6 and self.now()-self.action_start<.6:
            self.plan_after=self.now()+2.
        if getattr(self,'contact_pending',False):"""),
        ("        self.pending=False\n        try: handle=future.result()", """        self.pending=False
        guard=getattr(self,'contact_inspection',None)
        try: handle=future.result()"""),
        ("            self.contact_pending=False\n            self.get_logger().error(str(exc))","            if guard is not None and guard.action:guard.result(False,self.now())\n            self.contact_pending=False\n            self.get_logger().error(str(exc))"),
        ("        if not handle.accepted:\n            self.contact_pending=False", "        if not handle.accepted:\n            if guard is not None and guard.action:\n                guard.result(False,self.now());self.contact_pending=False;return\n            self.contact_pending=False"),
        ("        except Exception as exc:\n            if getattr(self,'marker_look',None) is not None:self.marker_look.clear('action_result_error')", """        except Exception as exc:
            guard=getattr(self,'contact_inspection',None)
            if guard is not None and guard.action:guard.result(False,self.now())
            if getattr(self,'marker_look',None) is not None:self.marker_look.clear('action_result_error')"""),
    ])
    edit('detector.py', [
        ('        self.candidate_pub=', """        from osp_mission.contact_inspection import BearingCue
        self.bearing_cue=BearingCue()
        self.bearing_pub=self.create_publisher(String,'mission/marker_bearing',10)
        self.candidate_pub="""),
        ("                if xyz is None: self.reject(ident,'no_range');continue", """                if xyz is None:
                    self.reject(ident,'no_range')
                    timestamp=stamp.nanoseconds/1e9
                    angle=-float(np.arctan(normalized[0]))
                    if self.bearing_cue.observe(int(ident),timestamp,angle):
                        self.bearing_pub.publish(String(data=json.dumps(dict(id=int(ident),stamp=timestamp,angle=angle))))
                    continue""")
    ])
    # Retain the detector's three-frame, range-verified recognition. Gate navigation hints separately.
    edit('mission.py', [
        ('            accepted=self.marker_focus.observe(evidence,observer[:2],', '''            if not evidence.get('provisional',False):
                ident=int(evidence['id']); stamp=float(evidence['stamp'])
                point=np.asarray(evidence['map_point'][:2],dtype=float)
                seeds=getattr(self,'marker_hint_seeds',{})
                previous=seeds.get(ident)
                if previous is not None and stamp<=previous[0]:return
                seeds[ident]=(stamp,point);self.marker_hint_seeds=seeds
                if previous is None or stamp-previous[0]>1. or np.linalg.norm(point-previous[1])>.3:return
            accepted=self.marker_focus.observe(evidence,observer[:2],''')
    ])
    edit('../test/test_marker_focus.py', [
        ("    assert stamps==[10_000_000_000]", """    assert not canceled and not owner.marker_focus.hints
    msg.data=json.dumps(dict(json.loads(msg.data),stamp=10.125))
    Mission.on_marker_hint(owner,msg)
    assert stamps==[10_000_000_000,10_125_000_000]"""),
        ("    assert owner.marker_focus.hints[42]['observer'].tolist()==[5.,2.]", """    assert not owner.marker_focus.hints
    message.data=json.dumps(dict(json.loads(message.data),stamp=10.125))
    Mission.on_marker_hint(owner,message)
    assert owner.marker_focus.hints[42]['observer'].tolist()==[5.,2.]""")
    ])
