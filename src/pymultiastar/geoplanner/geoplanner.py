import time
import pymultiastar as pmstar
from typing import List, Tuple, Optional
from pathlib import Path
import json

import numpy as np
from pyproj import Transformer

from ..types import ArrayFloatMxNxK
from .types import (PlannerKwargs, VoxelMeta, GPS, LandingSite, GeoMultiPlannerResult, Coord)
from .helper import (
    convert_cost_map_to_float,
    prepare_planning_args_optimized,
    voxel_cell_to_projected,
    voxel_projected_to_cell,
    get_free_neighbor_cell,
    get_first_free_cell_up,
    get_path_dist
)
from .log import logger


class GeoPlanner(object):
    """Geographic Planner"""

    cost_map: ArrayFloatMxNxK
    "A 3D numpy array of shape (M,N,K) of type float. Our voxel map"
    voxel_meta: VoxelMeta
    "All metadata concerning the voxel cost_map, e.g. srid, nrows, ncols, xres, etc."
    planner_kwargs: PlannerKwargs
    "The planner keyword arguments sent to pymultiastar"
    planner: pmstar.PyMultiAStar
    "The multi-goal a-star planner"

    def __init__(
        self,
        cost_map_fp: Path,
        voxel_meta: VoxelMeta,
        planner_kwargs: PlannerKwargs = PlannerKwargs(),
    ):
        """GeoPlanner Constructor

        Args:
            cost_map_fp (Path): File path to your 3D numpy array of your cost map
            voxel_meta (VoxelMeta): All meta data concerning the voxel cost_map
            planner_kwargs (PlannerKwargs, optional): Key word arguments sent to the
                multi-goal a-star planner. Defaults to PlannerKwargs().
        """
        self.cost_map: ArrayFloatMxNxK = np.load(Path(cost_map_fp))
        self.voxel_meta = voxel_meta
        self.planner_kwargs = planner_kwargs

        self.cost_map = convert_cost_map_to_float(np.load(cost_map_fp))
        self.planner = pmstar.PyMultiAStar(self.cost_map, **planner_kwargs.to_dict())

        self.transformer = Transformer.from_crs(
            "EPSG:4326", voxel_meta["srid"]
        )

    def plan_multi_goal(
        self, start_position: GPS, ls_list: List[LandingSite]
    ) -> Optional[GeoMultiPlannerResult]:
        """Will find the optimal landing site and path pair from a start position

        Args:
            start_position (GPS): The start position of the aircraft
            ls_list (List[LandingSite]): A list of landing sites with the associated risk

        Returns:
            GeoMultiPlannerResult: The result of the planner
        """

        project_start, projected_goals = prepare_planning_args_optimized(
            start_position, ls_list, self.transformer
        )
        # to cell position
        start_cell = voxel_projected_to_cell(project_start, self.voxel_meta)      
        # logger.debug(f"Start Cell: {start_cell}")
        goal_cells:List[Tuple[Tuple[int, int, int], float]] = []

        # Checking on start and goal cell positions
        bad_start = self.cost_map[start_cell[0], start_cell[1], start_cell[2]] == np.inf
        if bad_start:
            sc_ = start_cell[:]  # makes copy
            start_cell = get_free_neighbor_cell(start_cell, self.cost_map)
            if start_cell is None:
                logger.error(
                    "ERROR - Bad Start Cell! Start Cell: {} - {}".format(
                        sc_, self.cost_map[sc_[0], sc_[1], sc_[2]]
                    )
                )
                logger.error("{}".format(start_position))
                return None
        # This set is used to ensure that every goal as a UNIQUE cell location in the voxel grid.
        unique_goal_cell_set = set()
        # In case a bad goal has been give, keep this list to mark all the valid goals (landing sites)
        valid_landing_site_indices:List[int] = []
        for i, (goal_pos, goal_value) in enumerate(projected_goals):
            gc = voxel_projected_to_cell(goal_pos, self.voxel_meta)
            bad_goal = self.cost_map[gc[0], gc[1], gc[2]] == np.inf
            if bad_goal:
                gc_ = gc[:]  # makes copy
                # looks at neighbors around the cell
                gc = get_free_neighbor_cell(gc, self.cost_map)
                if gc is None:
                    # last chance, going vertically up only!
                    gc = get_first_free_cell_up(gc_, self.voxel_meta, self.cost_map)
                    if gc is None:
                        # Wow this was a really bad goal.  Log the issue an review later
                        logger.error(
                            "ERROR - Bad Goal Cell! Start Cell: {} - {}. Goal Cell: {} - {}".format(
                                start_cell,
                                self.cost_map[
                                    start_cell[0], start_cell[1], start_cell[2]
                                ],
                                gc_,
                                self.cost_map[gc_[0], gc_[1], gc_[2]],
                            )
                        )
                        logger.error("{}, {}".format(start_position, goal_pos))
                        continue
            if str(gc) not in unique_goal_cell_set:
                unique_goal_cell_set.add(str(gc))
                valid_landing_site_indices.append(i)
                goal_cells.append((gc, goal_value))
            else:
                logger.error(
                    "Landing site is mapped to a Map Cell that is already taken! Skipping. Index: %r, Pos: %r, Value: %r",
                    i,
                    goal_pos,
                    goal_value,
                )

        logger.debug(f"Start Cell: {start_cell}")
        logger.debug(f"Goal Cells: {goal_cells}")

        start_time = time.perf_counter()
        path_cells, meta = self.planner.search_multiple(
            start_cell, goal_cells
        )
        elapsed_time = (time.perf_counter() - start_time) * 1000

        if meta["goal_total_cost"] == -1.0:
            logger.error(
                "Could not find path! Starting UTM: %r; Starting Cell: %r",
                start_position,
                start_cell,
            )

        logger.debug("Path Cost: %s ", meta["goal_total_cost"])
        # These are index coordinates! Convert to meters
        path_projected = [
            voxel_cell_to_projected(cell, self.voxel_meta)
            for cell in path_cells
        ]
        path_projected_zero_origin = [
            self.transform_projected_to_zero_origin(coord)
            for coord in path_projected
        ]


        path_length = get_path_dist(path_projected)
        # {'path_cost': dummy_path_risk(start_pos, goal_pos), 'path': path, 'index': index}
        result:GeoMultiPlannerResult = {
            "start_position": start_position,
            "path_cells": path_cells,
            "path_projected": path_projected,
            "path_projected_zero_origin": path_projected_zero_origin,
            "path_length": path_length,
            "time_ms": elapsed_time,
            "valid_landing_site_indices": valid_landing_site_indices,
            **meta,
        }
        return result

    def transform_gps_to_projected(self, gps: GPS) -> Coord:
        coord:Coord = self.transformer.transform(*gps.to_array())
        return coord

    def transform_gps_to_projected_zero_origin(self, gps: GPS) -> Coord:
        # import ipdb; ipdb.set_trace()
        coord:Coord = self.transformer.transform(*gps.to_array())
        x_meters = coord[0] - self.voxel_meta['xmin']
        y_meters = coord[1] - self.voxel_meta['ymin']
        z_meters = coord[2] - self.voxel_meta['zmin']
        
        return (x_meters, y_meters, z_meters)

    def transform_projected_to_zero_origin(self, coord:Coord) -> Coord:
        # import ipdb; ipdb.set_trace()
        x_meters = coord[0] - self.voxel_meta['xmin']
        y_meters = coord[1] - self.voxel_meta['ymin']
        z_meters = coord[2] - self.voxel_meta['zmin']
        
        return (x_meters, y_meters, z_meters)



def create_planner_from_configuration(plan: Path):
    with open(plan, "r") as fh:
        planner_data = json.load(fh)
    planner_data["cost_map_fp"] = plan.parent / planner_data["cost_map_fp"]
    voxel_meta: VoxelMeta = planner_data["voxel_meta"]

    planner_kwargs = PlannerKwargs(**planner_data["planner_kwargs"])
    geo_planner = GeoPlanner(planner_data["cost_map_fp"], voxel_meta, planner_kwargs)
    return geo_planner, planner_data