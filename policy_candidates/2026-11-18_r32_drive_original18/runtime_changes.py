"""Install the pinned navigation dependency in the private build workspace only."""
import hashlib
import os
from pathlib import Path
import shutil

CORE_SHA = '77529b0a07df17bd17eb06769cac0299ab778ce7b4e2f3fd70f36b7433d47944'


def apply(package):
    core = Path(os.environ['OSP_R32_CORE'])
    if hashlib.sha256(core.read_bytes()).hexdigest() != CORE_SHA:
        raise ValueError('Use src/core.py from reference commit a40cbdf')
    shutil.copyfile(core, package/'osp_mission/_r32_navigation_core.py')

    def edit(source, before, after):
        if source.count(before) != 1:
            raise ValueError('Unexpected parent source: ' + before[:75])
        return source.replace(before, after)

    path = package/'osp_mission/mission.py'
    source = path.read_text()
    source = edit(source, 'from osp_mission.local_recovery import mark_departures',
                  'from osp_mission.local_recovery import mark_departures\nfrom osp_mission.r32_drive import R32Drive')
    source = edit(source, '        self.create_timer(.5,self.tick)', """        self.r32_drive = R32Drive()
        self.create_subscription(OccupancyGrid, 'global_costmap/costmap', self.r32_drive.update_costmap, qos_profile_sensor_data)
        self.create_timer(.5,self.tick)""")
    source = edit(source, '        try:\n            save_policy_audit(', """        r32_drive = getattr(self, 'r32_drive', None)
        if r32_drive is not None and not (selected and ('focus_id' in selected or 'approach_id' in selected)):
            selected = r32_drive.choose(self, data, viewed,
                (tr.transform.translation.x, tr.transform.translation.y, heading), memory_transform)
            if selected is not None:
                choices.append(selected)
        try:
            save_policy_audit(""")
    source = edit(source, '        if selected is None:\n', """        if selected is None:
            if r32_drive is not None:
                if local is not None and local.request_retreat(self.now()):
                    self.recovery_pending = 'local_blockage'
                    self.retreat_deadline = self.now() + 1.5
                else:
                    self.plan_after = self.now() + 2.
                self.zero()
                return
""")
    source = edit(source, "        self.send(self.nav,g,'nav')", """        if r32_drive is not None:
            r32_drive.dispatched(selected)
        self.send(self.nav,g,'nav')""")
    source = edit(source, "'navigation_flow':self.navigation_flow.report()", "'drive_policy':getattr(getattr(self,'r32_drive',None),'diagnostic',{}),\n            'navigation_flow':self.navigation_flow.report()")
    path.write_text(source)

    path = package/'launch/robot.launch.py'
    source = path.read_text()
    source = edit(source, "        nav_dest=Path(", """        from osp_mission.r32_nav_config import configure
        configure(nav_config, yaml.safe_load((Path(share('turtlebot4_navigation'))/'config/nav2.yaml').read_text()))
        nav_dest=Path(""")
    path.write_text(source)
