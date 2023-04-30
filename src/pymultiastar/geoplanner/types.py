from dataclasses import dataclass, asdict, fields
from typing import Dict, List, Tuple, TypedDict, Optional
import numpy as np
from ..types import ArrayFloatMxNxK, Cell, CellPath


Coord = Tuple[float, float, float]
CoordPath = List[Coord]

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
    keep_nodes: bool = False
    path_w0: float = 1.0

class Scenario(TypedDict):
    name:str
    details:Optional[str]
    position: Coord # assumed to be in GPS!
    active: Optional[bool]
    landing_sites: Optional[List[Dict]]
    planner_kwargs: Optional[Dict]

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

class Plan(TypedDict):
    name:str
    cost_map_fp: str
    voxel_meta: VoxelMeta
    map_bbox: Optional[Dict]
    planner_kwargs: PlannerKwargs
    scenarios: List[Scenario]
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


class GeoMultiPlannerResult(TypedDict):
    start_position: GPS
    path_cells: CellPath
    path_projected: CoordPath
    path_projected_zero_origin: CoordPath
    path_length: float
    time_ms: float
    valid_landing_site_indices: List[int]
    goal_index: int
    total_goal_searches: int
    goal_total_cost: float
    goal_path_cost: float
    goal_value: float
    num_expansions: int

