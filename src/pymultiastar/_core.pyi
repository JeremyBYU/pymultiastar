from typing import List, Tuple
from .types import ArrayFloatMxNxK, CellPath, MultiPlannerResult, Cell

__version__: str

class PyMultiAStar(object):
    def __init__(
        self,
        map: ArrayFloatMxNxK,
        allow_diag: bool,
        map_res: float,
        obstacle_value: float,
        normalizing_path_cost: float,
        goal_weight: float,
        path_weight: float,
        keep_nodes: bool,
        path_w0: float,
    ): ...
    def search_multiple(
        self, start_cell: Cell, goal_cells: List[Tuple[Cell, float]]
    ) -> Tuple[CellPath, MultiPlannerResult]: ...
