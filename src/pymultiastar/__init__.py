from ._core import __doc__, __version__, PyMultiAStar
from typing import List
from .types import ArrayFloatMxNxK, Coord, CoordPath, Cell, CellPath, CoordRisk


def calculate_pareto_points(
    planner: PyMultiAStar,
    start_cell: Cell,
    goal_cells: List[CoordRisk[int]],
    normalizing_path_cost=1.0,
    goal_weight=0.5,
    path_weight=0.5,
    **kwargs,
):
    pareto_points = []
    for i, goal in enumerate(goal_cells):
        if isinstance(goal, tuple):
            # Tuple of cell index and risk value
            path, path_cost = planner.search_single(start_cell, goal[0])
            goal_risk = goal[1]
        else:
            # LandingSite Class
            result = planner.search_single(start_cell, goal)
            if result is None:
                continue
            path, path_cost = result
            goal_risk = goal.landing_site_risk
        path_risk = path_cost / normalizing_path_cost
        total_risk = goal_weight * goal_risk + path_weight * path_risk
        result = dict(
            goal_idx=i,
            goal=goal,
            path=path,
            path_cost=path_cost,
            path_risk=path_risk,
            goal_risk=goal_risk,
            total_risk=total_risk,
        )
        pareto_points.append(result)
    return pareto_points

__all__ = ["__doc__", "__version__", "PyMultiAStar", "Cell", "CellPath", "calculate_pareto_points"]