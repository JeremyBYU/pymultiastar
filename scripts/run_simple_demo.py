# Just testing if pmultiastar is working
import numpy as np
import pymultiastar as pmstar


def main():
    # create our map
    # pymultiastar expects a three dimensional map in f32
    map_3d = np.zeros(shape=(3,3,3), dtype='f4') 
    start_cell = [0,0,0]
    goal_cells = [([0,0,1], 6), ([2,2,2], 1), ([2,2,1], 4)]

    # parameters to initialize mulit goal a star search
    params = dict(
        map_res=1.0,
        obstacle_value=1.0, # map ranges from 0-1 values. An obstacle will be the value 1.0
        normalizing_path_cost = 3.0, # normalize path distance by dividing by this number
        goal_weight = 0.5, 
        path_weight = 0.5,
    )
    planner = pmstar.PyMultiAStar(map_3d, **params)

    path, path_cost, meta = planner.search_multiple(start_cell, goal_cells)
    print(f"path: {path}, path_cost: {path_cost}, meta: {meta}")

    # print(stuff.search_single(start_cell, goal_cells[1][0]))

if __name__ == "__main__":
    main()