# Just testing if pmultiastar is working
import numpy as np
import pymultiastar as pmstar


def main():
    # create our map
    # pymultiastar expects a three dimensional map in f32
    map_3d = np.zeros(shape=(3,3,3), dtype='f4') 
    start_cell = (0,0,0)
    goal_cells = [((0,0,1), 6), ((2,2,2), 1), ((2,2,1), 4)]

    # parameters to initialize multi goal a-star search
    params = dict(
        allow_diag=True, # allow diagonal movement
        map_res=1.0, # the length of a voxel cell in meters
        obstacle_value=1.0, # map ranges from 0.0-1.0 values. An obstacle will be the value 1.0
        normalizing_path_cost = 3.0, # normalize path distance by dividing by this number
        goal_weight = 0.5, # weighting between path goal risk and path risk
        path_weight = 0.5, # weighting between path goal risk and path risk
        keep_nodes=False, # optimization strategy to prevent more dynamic memory allocation
        path_w0=1.0 # weighting for potential field in the maze
    )
    planner = pmstar.PyMultiAStar(map_3d, **params)

    path, meta = planner.search_multiple(start_cell, goal_cells)
    print(f"path: {path}, meta: {meta}")

if __name__ == "__main__":
    main()