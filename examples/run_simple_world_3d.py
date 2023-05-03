import numpy as np

import pymultiastar as pmstar
from pymultiastar.visualization.vis3d_helpers import create_map, create_pcd_map, visualize_world


def main():
    map_3d = create_map()

    start_cell = (0,0,1)
    goal_cells = [([9, 9, 3], 2), ([5,9,2], 4)]

    # this is the diagonal from the origin of the map to the top right (opposite corners of a cube)
    normalizing_path_cost = np.sqrt(10**2 + 10**2 + 5**2)

    params = dict(
        map_res=1.0,
        obstacle_value=1.0, # map ranges from 0-1 values. An obstacle will be the value 1.0
        normalizing_path_cost = normalizing_path_cost, # normalize path distance by dividing by this number
        goal_weight = 0.5, 
        path_weight = 0.5,
    )
    planner = pmstar.PyMultiAStar(map_3d, **params)
    path, meta = planner.search_multiple(start_cell, goal_cells)
    print(f"path: {path}, meta: {meta}")
    
    geoms = create_pcd_map(map_3d, ds_voxel_size=1.0)
    visualize_world(geoms, point_size=25)


if __name__ == "__main__":
    main()
