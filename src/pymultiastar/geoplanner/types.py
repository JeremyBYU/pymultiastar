from dataclasses import dataclass, asdict, fields
from typing import TypedDict
from typing import Dict, List, Annotated, Literal, TypeVar, Union
import numpy.typing as npt
import numpy as np


DType = TypeVar("DType", bound=np.generic)
Array3x3 = Annotated[npt.NDArray[DType], Literal[3, 3]]
ArrayFloatMxNxK = Annotated[npt.NDArray[np.float32], Literal["M", "N", "K"]]

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


class VoxelMeta(TypedDict):
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

@dataclass
class GPS():
    lat: float
    lon: float
    alt: float = np.nan

    def to_array(self, always_xy=False) -> List:
        if always_xy:
            return [self.lon, self.lat, self.alt]
        else:
            return [self.lat, self.lon, self.alt]

    @staticmethod
    def from_gps_string(centroid:str, alt=np.nan, reverse=False):
        """Converts a lat-long string to a GPS object. Optionally handles height and projection"""
        centroid_ = centroid.split(',')
        # reverse lat,lon if necessary
        if reverse:
            centroid_ = centroid_[::-1] 
        gps = GPS(float(centroid_[0]), float(centroid_[1]), alt=alt)
        return gps

@dataclass
class LandingSite(SuperDataClass):
    centroid: GPS
    "The centroid of the landing site in GPS coordinates"
    landing_site_risk: float
    "The normalized risk of this landing site [0-1]"


class PlannerData(TypedDict):
    path_cells: List[List[int]]
    path_meters: List[List[float]]
    path_length: float
    path_cost: float
    time_ms: float
    goal_index: int
    num_expansions: int
    total_goal_searches: int
    total_cost: float




