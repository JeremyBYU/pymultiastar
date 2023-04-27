import pymultiastar as pmstar
import pytest
import numpy as np

@pytest.fixture
def small_map():
    map_3d = np.zeros(shape=(3,3,3), dtype='f4') 
    
    map_data = dict(map=map_3d, start_cell=[0,0,0], 
                    goal_cells=[([0,0,1], 6), ([2,2,2], 1), ([1,1,1], 1.5)])
    return map_data

@pytest.fixture
def small_maze_2d():
    pass
    # np.asarray(PIL.Image.open('test.jpg'))

@pytest.mark.parametrize("goal_weight,path_weight,expected_path", [
(0.5, 0.5, [[0,0,0], [1,1,1]]),
(0.95, 0.05, [[0,0,0], [1,1,1], [2,2,2]]),
(0.05, 0.95, [[0,0,0], [0,0,1]]),
], ids=["balanced", "high_goal_weight", "low_goal_weight"])
def test_small_map(small_map, goal_weight, path_weight, expected_path):
    params = dict(
        allow_diag=True,
        map_res=1.0,
        obstacle_value=1.0, # map ranges from 0-1 values. An obstacle will be the value 1.0
        normalizing_path_cost = 3.0, # normalize path distance by dividing by this number
        goal_weight=goal_weight,
        path_weight=path_weight,
        keep_nodes=False,
        path_w0=1.0
    )
    planner = pmstar.PyMultiAStar(small_map['map'], **params) # type: ignore
    path, meta = planner.search_multiple(small_map['start_cell'], small_map['goal_cells'])

    np.testing.assert_allclose(path, expected_path)

def test_version():
    assert len(pmstar.__version__.split('.')) == 3


