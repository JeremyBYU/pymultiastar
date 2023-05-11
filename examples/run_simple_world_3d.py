import numpy as np
import pymultiastar as pmstar
from pymultiastar.visualization.vis3d_helpers import (
    create_map,
    create_pcd_map,
    create_planning_objects,
    visualize_world,
)


def main():
    shape = (20, 30, 15)
    buildings = [
        # Specify bounds of a rectangle (integers only!)
        # xmin,xmax  ymin,ymax zmin,zmax value
        [(7, 12), (7, 12), (0, 12), 1.0]
    ]
    map_3d = create_map(shape=shape, buildings=buildings)

    # specify the starting cell
    start_cell = (0, 0, 1)
    # specify goal cells and the associated risk values (lower is better, 0-1.0)
    goal_cells = [((15, 9, 3), 0.7), ((5, 9, 2), 1.0), ((19, 19, 15), 0.5)]

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
    print(f"path: {path}, meta: {meta}")

    world_geoms = create_pcd_map(map_3d, ds_voxel_size=1.0)
    landing_objects = create_planning_objects(start_cell, goal_cells, path)
    all_geoms = [*world_geoms, *landing_objects]
    visualize_world(all_geoms, point_size=20)


if __name__ == "__main__":
    main()
