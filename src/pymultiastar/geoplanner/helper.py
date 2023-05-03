from typing import List, Tuple
import dataclasses
import json
import numpy as np
from pyproj import Transformer

from ..types import Cell
from .types import GPS, ArrayFloatMxNxK, Coord, CoordPath, LandingSite, VoxelMeta
from .log import logger


def convert_cost_map_to_float(
    cost_map, reverse_yaxis=True, normalize=True, set_max_value_to_inf=True
) -> np.ndarray:
    """Will convert a uint8 cost map to a float32


    Note, the normal expectation is that the dimension are like so [y, x, z]
    Also the y-axis is growing DOWN, like an image, if this was generated from https://bitbucket.org/umich_a2sys/create-voxel/src/master/


    Args:
        cost_map (np.ndarray): Three dimensional cost map
        reverse_yaxis (bool, optional): Will reverse y axes of the map. Defaults to True.
        normalize (bool, optional): Will normalize cost from 0 to 1.0. Defaults to True.
        set_max_value_to_inf (bool, optional): All max values will be mapped to np.inf. Defaults True.

    Returns:
        np.ndarray: Your 3D cost map in float32
    """
    cost_map = cost_map.astype(np.float32)
    if reverse_yaxis:
        cost_map = np.flip(cost_map, 0)  # reverses the y-axis
    if normalize:
        max_value = np.max(cost_map)
        cost_map = cost_map / max_value  # convert to float32
    if set_max_value_to_inf:
        cost_map[cost_map == 1.0] = np.inf
    return cost_map


def prepare_planning_args_optimized(
    start_position: GPS, ls_list: List[LandingSite], transformer: Transformer
):
    """Prepares a list of landing sites formatted as a dictionary to be sent to a Path Planner
    Data is in x, y, height all in meters"""
    projected_position: Tuple[float, float, float] = transformer.transform(
        *start_position.to_array()
    )
    projected_goal_positions: List[Tuple[Tuple[float, float, float], float]] = [
        (transformer.transform(*ls.centroid.to_array()), ls.landing_site_risk)
        for ls in ls_list
    ]

    return projected_position, projected_goal_positions


def voxel_projected_to_cell(coord: Coord, voxel_meta: VoxelMeta) -> Cell:
    x_meters = coord[0] - voxel_meta["xmin"]
    y_meters = coord[1] - voxel_meta["ymin"]
    z_meters = coord[2] - voxel_meta["zmin"]

    j = max(
        min(
            int(round((x_meters - voxel_meta["xres"] / 2) / voxel_meta["xres"])),
            voxel_meta["ncols"] - 1,
        ),
        0,
    )  # cols
    i = max(
        min(
            int(round((y_meters - voxel_meta["yres"] / 2) / voxel_meta["yres"])),
            voxel_meta["nrows"] - 1,
        ),
        0,
    )  # rows

    k = max(
        min(int(round((z_meters) / voxel_meta["zres"])), voxel_meta["nslices"] - 1), 0
    )  # depth

    return (i, j, k)


def voxel_cell_to_projected(coord: Cell, voxel_meta: VoxelMeta) -> Coord:
    x_meters: float = float(coord[1] * voxel_meta["xres"] + voxel_meta["xmin"])
    y_meters: float = float(coord[0] * voxel_meta["yres"] + voxel_meta["ymin"])
    z_meters: float = float(coord[2] * voxel_meta["zres"] + voxel_meta["zmin"])

    return (x_meters, y_meters, z_meters)


def voxel_cell_to_projected_zero_origin(coord: Cell, voxel_meta: VoxelMeta) -> Coord:
    x_meters: float = float(coord[1] * voxel_meta["xres"])
    y_meters: float = float(coord[0] * voxel_meta["yres"])
    z_meters: float = float(coord[2] * voxel_meta["zres"])

    return (x_meters, y_meters, z_meters)


def get_free_neighbor_cell(cell: Cell, cost_map: ArrayFloatMxNxK):
    neighbors = [
        (cell[0], cell[1], cell[2] + 1),  # z-up
        (cell[0] + 1, cell[1], cell[2]),  # y-up
        (cell[0] - 1, cell[1], cell[2]),  # y-down
        (cell[0], cell[1] + 1, cell[2]),  # x-right
        (cell[0], cell[1] - 1, cell[2]),  # x-left
        (cell[0] + 1, cell[1], cell[2] + 1),  # y-up, z-up
        (cell[0] - 1, cell[1], cell[2] + 1),  # y-down, z-up
        (cell[0], cell[1] + 1, cell[2] + 1),  # x-right, z-up
        (cell[0], cell[1] - 1, cell[2] + 1),  # x-left, z-up
    ]
    for n in neighbors:
        try:
            if cost_map[n[0], n[1], n[2]] != np.inf:
                return n
        except Exception as e:
            continue
    return None


def get_first_free_cell_up(
    cell: Cell, voxel_meta: VoxelMeta, cost_map: ArrayFloatMxNxK
):
    for i in range(cell[2], voxel_meta["nslices"]):
        val = cost_map[cell[0], cell[1], i]
        if val != np.inf:
            return (cell[0], cell[1], i)
    return None


def get_path_dist(path: CoordPath) -> float:
    dist: float = 0.0
    for i in range(len(path) - 1):
        a = np.array(path[i], dtype=np.float32)
        b = np.array(path[i + 1], dtype=np.float32)
        dist += float(np.linalg.norm(a - b))
    return dist


class EnhancedJSONEncoder(json.JSONEncoder):
    def default(self, o):
        if dataclasses.is_dataclass(o):
            return dataclasses.asdict(o)
        if isinstance(o, np.ndarray):
            return o.tolist()
        return super().default(o)
