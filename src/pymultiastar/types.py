from typing import List, Annotated, Literal, Tuple, TypedDict
import numpy.typing as npt
import numpy as np
import logging
from enum import Enum

ArrayFloatMxNxK = Annotated[npt.NDArray[np.float32], Literal["M", "N", "K"]]
Cell = Tuple[int, int, int]
CellPath = List[Cell]

class MultiPlannerResult(TypedDict):
    goal_index: int
    total_goal_searches: int
    goal_total_cost: float
    goal_path_cost: float
    goal_value: float
    num_expansions: int

class LogLevel(Enum):
    DEBUG="DEBUG"
    INFO="INFO"
    WARNING="WARNING"
