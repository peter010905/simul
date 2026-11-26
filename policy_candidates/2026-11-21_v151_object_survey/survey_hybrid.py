"""Keep the original mission as motion owner while recording object task progress."""
import json
from pathlib import Path
from osp_mission.phased_hybrid import PhasedHybrid
from osp_mission.center_wall import surface_cells
from osp_mission.r32_drive import grid_from_message
from osp_mission.wall_route import yaw
from rclpy.time import Time
from osp_mission.object_survey import ObjectSurvey


class SurveyHybrid(PhasedHybrid):
    def __init__(self):
        super().__init__()
        self.schedule.survey=ObjectSurvey()

    def should_interrupt(self,owner):
        survey=self.schedule.survey
        if survey.region is not None or survey.active_box is not None:
            return False
        return super().should_interrupt(owner)

    def dispatched(self,owner,goal):
        super().dispatched(owner,goal)
        self.schedule.survey.dispatched(goal,owner.now()-owner.started)
        self.next_box_probe=float('inf')
        self.audit(owner,'dispatch')

    def audit(self,owner,event):
        if event=='finish' and owner.map is not None:
            try:
                grid=grid_from_message(owner.map)
                tr=owner.tf.lookup_transform('odom',owner.p('map_frame'),Time()).transform
                base=owner.tf.lookup_transform(owner.p('map_frame'),owner.p('base_frame'),Time()).transform
                transform=(tr.translation.x,tr.translation.y,yaw(tr.rotation))
                samples=surface_cells(grid,(base.translation.x,base.translation.y))
                viewed=owner.coverage.grid(grid.cells,grid.resolution,grid.origin[:2])
                self.schedule.survey.update(samples,grid,viewed,transform,(),owner.now()-owner.started)
            except Exception as exc:owner.get_logger().warning(f'Final object evidence refresh failed: {exc}')
        row=dict(event=event,elapsed=owner.now()-owner.started,**self.schedule.survey.report())
        try:
            with (Path(owner.p('output_dir'))/'object_survey.jsonl').open('a') as out:
                out.write(json.dumps(row,allow_nan=False)+'\n')
        except OSError as exc:owner.get_logger().warning(str(exc))
