"""Add side-wall targeting and route history to the contact recovery stage."""
def apply(package):
    p=package/'osp_mission/mission.py'
    text=p.read_text()
    def edit(old,new):
        nonlocal text
        if text.count(old)!=1:raise ValueError(f'Unexpected parent source: {old[:75]}')
        text=text.replace(old,new)
    edit('from osp_mission.contact_inspection import ContactInspection','from osp_mission.wall_route import WallLook, penalize_routes\nfrom osp_mission.contact_inspection import ContactInspection')
    edit('        self.contact_inspection=ContactInspection()', '        self.wall_look=WallLook()\n        self.contact_inspection=ContactInspection()')
    edit('        if self.goal_handle or self.pending:\n            if (guard is not None', '''        wall=getattr(self,'wall_look',None)
        wall_due=(wall is not None and not protected and not bearing_fresh and not guard.phase
                  and not getattr(getattr(self,'near_pass',None),'protected_navigation',False)
                  and now>=self.spin_blocked_until
                  and (not (self.goal_handle or self.pending) or self.kind=='nav')
                  and wall.check(self,now))
        if self.goal_handle or self.pending:
            if (guard is not None''')
    if text.count('guard.due(now) or bearing_fresh)')!=2:raise ValueError('inspection trigger changed')
    text=text.replace('guard.due(now) or bearing_fresh)', 'guard.due(now) or bearing_fresh or wall_due)')
    edit('                    elif abs(goal.target_yaw)<1e-6 and (self.initial_scan or guard.due(now)):', '''                    elif wall_due:
                        wall_angle=wall.consume(self,now)
                        if wall_angle is not None:
                            goal.target_yaw=wall_angle
                    elif abs(goal.target_yaw)<1e-6 and (self.initial_scan or guard.due(now)):''')
    edit('            if goal.target_yaw or bearing_fresh or self.initial_scan:', '            if goal.target_yaw or bearing_fresh or wall_due or self.initial_scan:')
    edit('        choices,_=self.policy_options',"""        if self.edge_mode and getattr(self,'wall_look',None) is not None:
            option_settings['return_paths']=True
        choices,previous=self.policy_options""")
    edit('                choices=self.exploration_memory.filter(choices,memory_transform,self.now())','''                choices=self.exploration_memory.filter(choices,memory_transform,self.now())
                if getattr(self,'wall_look',None) is not None:
                    penalize_routes(choices,previous,data.shape,m.info.resolution,
                        (m.info.origin.position.x,m.info.origin.position.y),memory_transform,self.exploration_memory.cells)''')
    p.write_text(text)
