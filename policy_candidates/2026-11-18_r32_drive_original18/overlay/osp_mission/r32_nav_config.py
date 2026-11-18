from copy import deepcopy


def configure(nav, vendor):
    controller = nav['controller_server']['ros__parameters']
    follow = deepcopy(vendor['controller_server']['ros__parameters']['FollowPath'])
    if follow['plugin'] != 'nav2_mppi_controller::MPPIController':
        raise ValueError('Expected the Jazzy TurtleBot4 MPPI controller')
    # Retain the speed envelope of the current platform and its independent
    # collision monitor, including the guarded mission-level backup.
    follow.update(vx_max=.25, vx_min=0., wz_max=.5, visualize=False)
    follow['GoalAngleCritic']['threshold_to_consider'] = .25
    follow['PathAlignCritic']['cost_weight'] = 3.
    controller['FollowPath'] = follow
    controller['general_goal_checker']['xy_goal_tolerance'] = .15
    planner = nav['planner_server']['ros__parameters']
    planner['GridBased'] = deepcopy(vendor['planner_server']['ros__parameters']['GridBased'])
    planner['GridBased'].update(allow_unknown=False, tolerance=.06)
