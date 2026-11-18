"""Integrate local recovery into the checked v3 mission callbacks."""
def apply(package):
    path = package / 'osp_mission/mission.py'
    source = path.read_text()

    def edit(old, new):
        nonlocal source
        if source.count(old) != 1:
            raise ValueError(f'Unexpected parent source at {old[:100]}')
        source = source.replace(old, new)

    edit('from osp_mission.navigation_flow import NavigationFlow',
         'from osp_mission.recovery_control import RecoveryFlow, navigation_finished, navigation_tree, try_retreat\nfrom osp_mission.local_recovery import mark_departures')
    edit('self.navigation_flow=(NavigationFlow(', 'self.navigation_flow=(RecoveryFlow(')
    edit("        if flow is not None and kind=='nav':flow.begin_navigation()",
         """        if flow is not None and kind=='nav':
            flow.begin_navigation()
            goal.behavior_tree = navigation_tree()""")
    edit("            if flow is not None:flow.record_navigation_result(result.status,getattr(self,'cancel_reason',''),self.now())",
         "            navigation_finished(self, result)")
    edit("            if flow is not None and getattr(self,'kind','')=='nav'",
         """            if getattr(flow, 'local', None) is not None:
                flow.local.observe(visit.x, visit.y, self.now())
            if flow is not None and getattr(self,'kind','')=='nav'""")
    old = source[source.index('        if self.recovery_pending:\n',source.index('    def tick(self)')):
                 source.index('        if self.scan_after and self.now()<self.spin_blocked_until:')]
    edit(old, """        if self.recovery_pending and try_retreat(self, flow):
            return
        local = getattr(flow, 'local', None)
        if local is not None and self.now() < local.retry_after:
            self.zero()
            return
""")
    edit('        if selected is None:\n            # Retry old frontiers',
         """        if selected is None:
            if local is not None and local.anchor is not None:
                # One blocked exit may account for every destination in a corridor.
                if local.request_retreat(self.now()):
                    self.recovery_pending = 'local_blockage'
                    self.retreat_deadline = self.now() + 1.5
                    self.zero()
                    return
                self.plan_after = self.now() + 2.
                self.zero()
                return
            # Retry old frontiers""")
    edit("        choices,_=self.policy_options", """        if local is not None and self.edge_mode:
            option_settings['return_paths'] = True
        choices,previous=self.policy_options""")
    edit('        memory_transform=None', """        if local is not None and self.edge_mode:
            mark_departures(choices, previous, data.shape, m.info.resolution,
                            (m.info.origin.position.x, m.info.origin.position.y),
                            (tr.transform.translation.x, tr.transform.translation.y))
        memory_transform=None""")
    edit("            visit=self.tf.lookup_transform('odom',self.p('base_frame'),Time()).transform.translation",
         """            odom_pose = self.tf.lookup_transform('odom', self.p('base_frame'), Time()).transform
            visit = odom_pose.translation""")
    edit('                flow.observe_position(visit.x,visit.y,self.now())',
         """                flow.observe_position(visit.x,visit.y,self.now())
                if isinstance(flow, RecoveryFlow):
                    q = odom_pose.rotation
                    yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
                    flow.observe_heading(yaw, self.now())""")
    edit('        if self.scan_after and self.now()<self.spin_blocked_until:',
         """        if (isinstance(flow, RecoveryFlow) and self.scan_after and self.initial_scan
                and self.edge_mode and not any(getattr(getattr(self, name, None), 'pending', None)
                                              is not None for name in ('marker_look', 'face_look'))):
            self.initial_scan = False
            self.scan_after = False
        if self.scan_after and self.now()<self.spin_blocked_until:""")
    path.write_text(source)

    path = package / 'launch/robot.launch.py'
    source = path.read_text()
    edit('    if filtering:\n        nav_config=route_costmap_scans(yaml.safe_load(Path(nav2_params).read_text()),raw_scan,planning_scan)',
         """    flow_enabled = value('navigation_flow_enabled') == 'true'
    if filtering or flow_enabled:
        nav_config = yaml.safe_load(Path(nav2_params).read_text())
        if filtering:
            nav_config = route_costmap_scans(nav_config, raw_scan, planning_scan)
        if flow_enabled:
            params = nav_config['controller_server']['ros__parameters']
            for name in params.get('progress_checker_plugins', ['progress_checker']):
                checker = params.get(name, {})
                if checker.get('plugin') == 'nav2_controller::SimpleProgressChecker':
                    checker.update(plugin='nav2_controller::PoseProgressChecker', required_movement_angle=.35)""")
    path.write_text(source)
