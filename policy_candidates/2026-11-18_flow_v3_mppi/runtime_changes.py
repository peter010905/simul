"""Apply the controller and progress-check changes to the exact Flow v3 stage."""


def change(text, old, new):
    if text.count(old) != 1:
        raise ValueError(f'Unexpected parent source at {old[:80]}')
    return text.replace(old, new)


def apply(package):
    path = package/'osp_mission/mission.py'
    text = change(path.read_text(), 'from osp_mission.navigation_flow import NavigationFlow',
                  'from osp_mission.transit_control import TransitFlow')
    text = change(text, 'self.navigation_flow=(NavigationFlow(', 'self.navigation_flow=(TransitFlow(')
    text = change(text, "visit=self.tf.lookup_transform('odom',self.p('base_frame'),Time()).transform.translation",
                  "pose=self.tf.lookup_transform('odom',self.p('base_frame'),Time()).transform\n            visit=pose.translation")
    text = change(text, '                flow.observe_position(visit.x,visit.y,self.now())',
                  '''                flow.observe_position(visit.x,visit.y,self.now())
                if isinstance(flow, TransitFlow):
                    flow.observe_heading(pose.rotation, self.now())''')
    path.write_text(text)

    path = package/'launch/robot.launch.py'
    text = change(path.read_text(), '    if filtering:\n        nav_config=route_costmap_scans(yaml.safe_load(Path(nav2_params).read_text()),raw_scan,planning_scan)',
                  '''    flow_enabled = value('navigation_flow_enabled') == 'true'
    if flow_enabled or filtering:
        nav_config = yaml.safe_load(Path(nav2_params).read_text())
        if filtering:
            nav_config = route_costmap_scans(nav_config, raw_scan, planning_scan)
        if flow_enabled:
            from osp_mission.transit_control import configure_controller
            vendor = Path(share('turtlebot4_navigation'))/'config/nav2.yaml'
            configure_controller(nav_config, yaml.safe_load(vendor.read_text()))''')
    path.write_text(text)
