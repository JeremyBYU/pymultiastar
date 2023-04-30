from typing import List, Tuple
from .types import ArrayFloatMxNxK, CellPath, MultiPlannerResult, Cell

__version__: str

class PyMultiAStar(object):
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
    ): ...
    def search_multiple(
        self, start_cell: Cell, goal_cells: List[Tuple[Cell, float]]
    ) -> Tuple[CellPath, MultiPlannerResult]: ...
