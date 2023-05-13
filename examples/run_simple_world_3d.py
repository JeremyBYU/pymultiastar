from typing import List
import numpy as np
import pymultiastar as pmstar
import open3d as o3d
from pymultiastar.visualization.vis3d_helpers import (
    create_map,
    create_pcd_map,
    create_planning_objects,
    visualize_world,
    plot_pareto
)
from pymultiastar.types import Cell, CoordRisk

def run_vis():
        #    y,  x,  z
    shape = (20, 30, 15)
    buildings = [
        # Specify bounds of a rectangle (integers only!)
        # ymin,ymax xmin,xmax zmin,zmax value
        [(7, 12), (7, 12), (0, 12), 1.0]
    ]
    map_3d = create_map(shape=shape, buildings=buildings)

    # specify the starting cell, all coordinates in y,x,z
    start_cell: Cell = (0, 0, 1)
    # specify goal cells and the associated risk values (lower is better, 0-1.0)
    goal_cells: List[CoordRisk[int]] = [
        ((9, 5, 2), 1.0), # it's close, but has high risk
        ((19, 29, 14), 0.1), # it's far, but has low risk
        ((9, 15, 3), 0.4), # its kinds close and is kinda low risk! This will be the best!
        ((19, 19, 14), 0.5), # all the rest are not that good!
        ((10, 19, 10), 0.9),
        ((19, 1, 14), 0.8),
        ((5, 29, 14), 0.55),
    ]

    # this is the diagonal from the origin of the map to the top right (opposite corners of a cube)
    # you can choose whatever you want however
    normalizing_path_cost = np.sqrt(shape[0] ** 2 + shape[1] ** 2 + shape[2] ** 2)

    params = dict(
        map_res=1.0,  # a cell is one meter
        obstacle_value=1.0,  # map ranges from 0-1 values. An obstacle will be the value 1.0
        normalizing_path_cost=normalizing_path_cost,  # normalize path distance by dividing by this number
        goal_weight=0.5,  # trade off between path and goal risk
        path_weight=0.5,
    )
    planner = pmstar.PyMultiAStar(map_3d, **params)
    path, meta = planner.search_multiple(start_cell, goal_cells)
    print(f"Found: {meta}")

    world_geoms = create_pcd_map(map_3d, ds_voxel_size=1.0)
    landing_objects, all_labels = create_planning_objects(start_cell, goal_cells, path)

    all_geoms = [*world_geoms, *landing_objects]

    # Calculator the pareto points for all goals. This is only for visualization
    # You can call this by using the Actions dropdown
    actions = [("Show Pareto Plot", lambda x: plot_pareto(planner, start_cell, goal_cells, params))]
    # gives 3d visualization
    visualize_world(all_geoms, all_labels=all_labels, point_size=20, actions=actions)


def main():
    run_vis()


if __name__ == "__main__":
    main()
