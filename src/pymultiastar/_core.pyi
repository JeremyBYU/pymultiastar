from typing import List, Tuple
from .types import ArrayFloatMxNxK, CellPath, MultiPlannerResult, Cell

__version__: str

class PyMultiAStar(object):
    """
    A planner that will search for **multiple** goals with **heterogenous** values 
    using a 3D A-star **discrete** planner.
    """
    def __init__(
        self,
        map: ArrayFloatMxNxK,
        allow_diag: bool = False,
        map_res: float = 2.0,
        obstacle_value: float = 1.0,
        normalizing_path_cost: float = 1.0,
        goal_weight: float = 0.5,
        path_weight: float = 0.5,
        keep_nodes: bool = False,
        path_w0: float = 1.0,
    ) -> None:
        """Creates the multi-goal planner.

        Args:
            map (ArrayFloatMxNxK): 3D NumPy array, often called the voxel grid. 
                                   index (i,j,k) corresponds to (y, x, z)
            allow_diag (bool, optional): Allows diagonal travel in map. Defaults to False.
            map_res (float, optional): The length (m) of an edge in the voxel grid. Defaults to 2.0.
            obstacle_value (float, optional): The value of an obstacle in the map. Defaults to 1.0.
            normalizing_path_cost (float, optional): The length and penalities of a path must
                                                    be normalized. This should generally be the
                                                    the longest path acceptable. Defaults to 1.0.
            goal_weight (float, optional): The weighting of the goal risk. Defaults to 0.5.
            path_weight (float, optional): The weighting of the path risk. Defaults to 0.5.
            keep_nodes (bool, optional): An optimization parameter which, if set to True,
                                         may result in less dynamic memory allocations if many
                                         goals are being searched for. Defaults to False.
            path_w0 (float, optional): The path cost penalty multiplier when encountering
                                        a potential field. Defaults to 1.0.
        """
    def search_multiple(
        self, start_cell: Cell, goal_cells: List[Tuple[Cell, float]]
    ) -> Tuple[CellPath, MultiPlannerResult]: ...
