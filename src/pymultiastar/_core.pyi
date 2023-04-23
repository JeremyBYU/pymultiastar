from typing import List, Tuple
from .types import ArrayFloatMxNxK, CellPath, MultiPlannerResult

__version__:str


class PyMultiAStar(object):
    def __init__(self, map:ArrayFloatMxNxK, allow_diag:bool, map_res:float, obstacle_value:float, 
                goal_weight:float, path_weight:float, keep_nodes:bool, path_w0:float):
        ...                
    def search_multiple(self, start_cell:Tuple[int, int, int], 
                        goal_cells: List[Tuple[Tuple[int, int, int], float]]) \
                        -> Tuple[CellPath, MultiPlannerResult]: 
        ...