import numpy as np


def convert_cost_map_to_float(cost_map, flip_xy=True, normalize=True, set_max_value_to_inf=True):
    """Will convert a uint8 cost map to a float32

    Args:
        cost_map (np.ndarray): Three dimenstional cost map
        flip_xy (bool, optional): Will flip the x and y axes of the map. Defaults to True.
        normalize (bool, optional): Will normalize cost from 0 to 1.0. Defaults to True.
        set_max_value_to_inf (bool, optional): All max values will be mapped to np.inf. Defaults True.

    Returns:
        np.ndarray: Your 3D cost map in float32
    """
    cost_map = cost_map.astype(np.float32)
    if flip_xy:
        cost_map = np.flip(cost_map , 0) # flips the x and y axis
    if normalize:
        max_value = np.max(cost_map)
        cost_map = cost_map / max_value # convert to float32
    if set_max_value_to_inf:
        cost_map[cost_map == 1.0] = np.inf
    return cost_map