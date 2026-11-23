import numpy as np

from osp_mission._r32_navigation_core import Grid
from osp_mission.exploration_memory import ExplorationMemory
from osp_mission.open_regions import choose_unvisited
from osp_mission.visual_survey import VisualSurvey


def test_long_ago_visited_destination_is_not_new_and_corridor_remains_passable():
    g = Grid(np.zeros((50, 90), dtype=np.int8), .1, (-4., -2., 0.))
    safe = np.ones_like(g.cells, dtype=bool)
    dist = np.full_like(g.cells, 4., dtype=float)
    survey = VisualSurvey()
    memory = ExplorationMemory()
    survey.travel_memory = memory
    for x, t in [(3.2, 0.), (2.8, 5.), (2.4, 10.), (2., 15.)]:
        memory.observe(x, 0., t)
    for i in range(5):
        survey.position(.24*i, 0., 395.+i)
    choices = [dict(x=x, y=0., features=[1., 1., 0., 0., 0.]) for x in [3.2, -2.]]
    grid_args = (choices, g, g, safe, safe, dist, (0.,0.,0.), survey.trail, 409., 191.)
    # Imitate the earlier time-limited choice from the recent trajectory.
    assert choose_unvisited(*grid_args, lambda _:False)['x'] == 3.2
    known = survey.visited_positions(409.)
    goal = choose_unvisited(*grid_args, lambda _:False, visited_xy=known)
    assert goal['x'] == -2. and goal['new_region_path_m'] == 4.
    assert choose_unvisited(*grid_args, lambda _:True, visited_xy=known) is None
    assert safe.all()
