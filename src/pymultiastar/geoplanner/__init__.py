import numpy as np
import pymultiastar as pmstar

from .types import PlannerKwargs, ArrayFloatMxNxK, VoxelMeta
from .helper import convert_cost_map_to_float

class GeoPlanner(object):
    """Geographic Planner

    """
    cost_map:ArrayFloatMxNxK
    "A 3D numpy array of shape (M,N,K) of type float. Our voxel map"
    voxel_meta: VoxelMeta
    "All metadata concerning the voxel cost_map, e.g. srid, nrows, ncols, xres, etc."
    planner_kwargs:PlannerKwargs
    "The planner keyword arguments sent to pymultiastar"
    planner:pmstar.PyMultiAStar
    "The multi-goal a-star planner"

    def __init__(self, cost_map_fp:ArrayFloatMxNxK, voxel_meta:VoxelMeta, 
                 planner_kwargs:PlannerKwargs=PlannerKwargs()):
        """GeoPlanner Constructor

        Args:
            cost_map_fp (ArrayFloatMxNxK): File path to 
            voxel_meta (VoxelMeta): All meta data concerning the voxel cost_map
            planner_kwargs (PlannerKwargs, optional): Key word arguments sent to the 
                multi-goal a-star planner. Defaults to PlannerKwargs().
        """
        self.cost_map:ArrayFloatMxNxK = np.load(cost_map_fp)
        self.voxel_meta = voxel_meta
        self.planner_kwargs = planner_kwargs

        self.cost_map = convert_cost_map_to_float(np.load(cost_map_fp))


        self.planner = pmstar.PyMultiAStar(self.cost_map, **planner_kwargs)



    def search_multiple():
        pass

    



# class Planning(object):
#     def __init__(self,  cost_map_fn=COST_MAP_FULL, voxel_meta=None, proj_local='EPSG:5555', **kwargs):

#         self.proj_dist = pyproj.Proj(init=proj_local)
#         self.proj_gps = pyproj.Proj(init='EPSG:4326')
#         self.cost_map_fn = cost_map_fn
#         self.voxel_meta = voxel_meta

#         self.cost_map = np.load(self.cost_map_fn)
#         self.cost_map = np.flip(self.cost_map , 0)
#         self.cost_map = self.cost_map / 255.0
#         # All buildings have infinite cost
#         self.cost_map[self.cost_map == 1.0] = np.inf
#         # print(kwargs)
#         self.pma = PyMultiAStar(self.cost_map, allow_diag=True, **kwargs)
        


#     def get_first_free_cell(self, coord, to_cell=True):
#         if to_cell:
#             coord = self.convert(coord, to_cell=True)
#         for i in range(coord[2], self.voxel_meta['nslices']):
#             val = self.cost_map[coord[0], coord[1], i]
#             if val != np.inf:
#                 return [coord[0], coord[1], i]
#         return None

#     def convert(self, coord, to_cell=True):
#         """
#         i=y,j=x,k=z
#         """
#         if to_cell:
#             # coord=(x,y,z) meter coordinates, offset
#             x_meters = coord[0] - self.voxel_meta['xmin']
#             y_meters = coord[1] - self.voxel_meta['ymin']
#             z_meters = coord[2] - self.voxel_meta['zmin']

#             j = max(min(int(round(
#                 (x_meters - self.voxel_meta['xres'] / 2) / self.voxel_meta['xres'])), self.voxel_meta['ncols'] - 1), 0)  # cols
#             i = max(min(int(round(
#                 (y_meters - self.voxel_meta['yres'] / 2) / self.voxel_meta['yres'])), self.voxel_meta['nrows'] - 1), 0)  # rows

#             k = max(min(
#                 int(round((z_meters) / self.voxel_meta['zres'])), self.voxel_meta['nslices'] - 1), 0)  # depth

#             return [i, j, k]
#         else:
#             # coord=(i,j,k) cell coordinates!
#             x_meters = coord[1] * self.voxel_meta['xres'] + \
#                 self.voxel_meta['xmin']
#             y_meters = coord[0] * self.voxel_meta['yres'] + \
#                 self.voxel_meta['ymin']
#             z_meters = coord[2] * self.voxel_meta['zres'] + \
#                 self.voxel_meta['zmin']

#             return [x_meters, y_meters, z_meters]

#     def to_gps(self, plan):
#         if len(plan) != 0:
#             path_gps = [pyproj.transform(
#                 self.proj_dist, self.proj_gps, *coord) for coord in plan]
#         else:
#             path_gps = pyproj.transform(self.proj_dist, self.proj_gps, *plan)
#         return path_gps

#     def get_free_neighbor_cell(self, cell):
#         neighbors = [
#             [cell[0], cell[1], cell[2] + 1],
#             [cell[0] + 1, cell[1], cell[2]],
#             [cell[0] - 1, cell[1], cell[2]],
#             [cell[0], cell[1] + 1, cell[2]],
#             [cell[0], cell[1] - 1, cell[2]],
#         ]
#         for n in neighbors:
#             try:
#                 if self.cost_map[n[0], n[1], n[2]] != np.inf:
#                     return n
#             except Exception as e:
#                 continue
#         return None

#     def plan(self, start_pos, goal_pos, index):
#         # if debug:
#         #     print(start_pos, goal_pos, index, debug)
#         sc = self.convert(start_pos)
#         gc = self.convert(goal_pos)

#         # Determine if cells are bad (occupied)
#         bad_start = self.cost_map[sc[0], sc[1], sc[2]] == np.inf
#         bad_goal = self.cost_map[gc[0], gc[1], gc[2]] == np.inf
#         if bad_start:
#             # logging.debug("BAD START")
#             # logging.debug("%r - %r", start_pos, goal_pos)
#             sc_ = sc[:]  # makes copy
#             sc = self.get_free_neighbor_cell(sc)
#             if sc is None:
#                 logging.error("ERROR - Bad Start Cell! Start Cell: {} - {}".format(
#                     sc_, self.cost_map[sc_[0], sc_[1], sc_[2]]))
#                 logging.error("{}, {}, {}".format(start_pos, goal_pos, index))
#                 return {'path_cost': np.inf, 'path': [], 'index': index}
#         if bad_goal:
#             gc_ = gc[:]  # makes copy
#             gc = self.get_free_neighbor_cell(gc) # looks at neighbors around the cell
#             if gc is None:
#                 # last chance, going vertically up only!
#                 gc = self.get_first_free_cell(gc_, to_cell=False)
#                 if gc is None:
#                     # Wow this was a really bad goal.  Log the issue an review later
#                     logging.error("ERROR - Bad Goal Cell! Start Cell: {} - {}. Goal Cell: {} - {}".format(
#                         sc, self.cost_map[sc[0], sc[1], sc[2]], gc_, self.cost_map[gc_[0], gc_[1], gc_[2]]))
#                     logging.error("{}, {}, {}".format(start_pos, goal_pos, index))
#                     return {'path_cost': np.inf, 'path': [], 'index': index, 'time': np.nan}

#         logging.debug("Start Cell: {} - {}. Goal Cell: {} - {}".format(sc,
#                                                                      self.cost_map[sc[0], sc[1], sc[2]], gc, self.cost_map[gc[0], gc[1], gc[2]]))
#         start_time = time.time()
#         path_cells, path_cost = self.pma.search_single(sc, gc)
#         elapsed_time = time.time() - start_time

#         logging.debug("Path Risk: %s ", path_cost)
#         # These are index coordinates! Convert to meters
#         path_meters = [self.convert(cell, to_cell=False)
#                        for cell in path_cells]

#         path_length = get_dist(path_meters)
#         # {'path_cost': dummy_path_risk(start_pos, goal_pos), 'path': path, 'index': index}
#         return {'path_cost': path_cost, 'path': path_meters, 'goal_index': index, 'path_length': path_length, 'time': elapsed_time}

  
#     def plan_optimized(self, start_position, goal_positions, **kwargs):
#         """This will find the optimal goal from a set of goal_positions and a given starting position
        
#         Arguments:
#             start_position {([float x,float y, float z], int value)} -- List for starting position in world coordinates
#             goal_positions {List[([float x,float y, float z], int value)]} -- List of Tuples. Each tuple has an world coordinate and a goal value/risk.
#         """
#         # print(start_position)
#         # print(goal_positions)
#         # print(kwargs)
#         start_cell = self.convert(start_position) # to cell position
#         goal_cells = []

#         # print("Start Cell:", start_cell)

#         # Checking on start and goal cell positions
#         bad_start = self.cost_map[start_cell[0], start_cell[1], start_cell[2]] == np.inf
#         if bad_start:
#             sc_ = start_cell[:]  # makes copy
#             start_cell = self.get_free_neighbor_cell(start_cell)
#             if start_cell is None:
#                 logging.error("ERROR - Bad Start Cell! Start Cell: {} - {}".format(
#                     sc_, self.cost_map[sc_[0], sc_[1], sc_[2]]))
#                 logging.error("{}".format(start_position))
#                 return {'path_cost': np.inf, 'path': [], 'index': -1}
#         # This set is used to ensure that every goal as a UNIQUE cell location in the voxel grid.
#         unique_goal_cell_set = set()
#         # In case a bad goal has been give, keep this list to mark all the valid goals (landing sites)
#         valid_landing_site_indices = []
#         for i, (goal_pos, goal_value) in enumerate(goal_positions):
#             gc = self.convert(goal_pos)
#             bad_goal = self.cost_map[gc[0], gc[1], gc[2]] == np.inf
#             if bad_goal:
#                 gc_ = gc[:]  # makes copy
#                 gc = self.get_free_neighbor_cell(gc) # looks at neighbors around the cell
#                 if gc is None:
#                     # last chance, going vertically up only!
#                     gc = self.get_first_free_cell(gc_, to_cell=False)
#                     if gc is None:
#                         # Wow this was a really bad goal.  Log the issue an review later
#                         logging.error("ERROR - Bad Goal Cell! Start Cell: {} - {}. Goal Cell: {} - {}".format(
#                             start_cell, self.cost_map[start_cell[0], start_cell[1], start_cell[2]], gc_, self.cost_map[gc_[0], gc_[1], gc_[2]]))
#                         logging.error("{}, {}".format(start_position, goal_pos))
#                         continue
#             if str(gc) not in unique_goal_cell_set:
#                 unique_goal_cell_set.add(str(gc))
#                 valid_landing_site_indices.append(i)
#                 goal_cells.append((gc, goal_value))
#             else:
#                 logging.error("Landing site is mapped to a Map Cell that is already taken! Skipping. Index: %r, Pos: %r, Value: %r", i, goal_pos, goal_value)

#         # print(start_cell)
#         # print(goal_cells)
#         start_time = time.time()
#         # pma = PyMultiAStar(self.cost_map, **kwargs)
#         path_cells, path_cost, meta = self.pma.search_multiple(start_cell, goal_cells)
#         elapsed_time = time.time() - start_time

#         if path_cost == -1.0:
#             logging.error("Could not find path! Starting UTM: %r; Starting Cell: %r", start_position, start_cell)

#         logging.debug("Path Risk: %s ", path_cost)
#         # These are index coordinates! Convert to meters
#         path_meters = [self.convert(cell, to_cell=False)
#                        for cell in path_cells]

#         path_length = get_dist(path_meters)
#         # {'path_cost': dummy_path_risk(start_pos, goal_pos), 'path': path, 'index': index}
#         return {'path_cost': path_cost, 'path': path_meters, 'path_length': path_length, 'time': elapsed_time, **meta}, valid_landing_site_indices