from dataclasses import dataclass, asdict, fields
from typing import Dict, List, Annotated, Literal, TypeVar
import numpy.typing as npt
import numpy as np


DType = TypeVar("DType", bound=np.generic)
Array3x3 = Annotated[npt.NDArray[DType], Literal[3, 3]]
ArrayFloatMxNxK = Annotated[npt.NDArray[np.float], Literal["M", "N", "K"]]

class SuperDataClass():
    def to_dict(self) -> Dict:
        return asdict(self)
    @classmethod
    def from_dict(cls, dict_):
        class_fields = {f.name for f in fields(cls)}
        return cls(**{k: v for k, v in dict_.items() if k in class_fields}) 



@dataclass
class PlannerKwargs(SuperDataClass):
    allow_diag: bool = True
    map_res:float = 2.0
    obstacle_value:float = 1.0
    normalizing_path_cost:float = 1.0
    goal_weight:float = 0.5
    path_weight: float = 0.5
    keep_nodes: float = False
    path_w0: float = 1.0


@dataclass
class VoxelMeta(SuperDataClass):
    srid:str
    nrows:int
    ncols:int
    nslices:int
    xres:float
    yres:float
    zres:float
    xmin:float
    ymin:float
    zmin:float




