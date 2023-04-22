import open3d as o3d
import matplotlib as mpl
import numpy as np

default_buildings = [
    [(1, 3), (3, 10), (0, 3), 255],
    [(4, 8), (3, 5), (0, 5), 255],
    [(6, 8), (6, 10), (0, 3), 255],
]


def create_map(
    shape=(10, 10, 5), buildings=default_buildings, dtype=np.float32, normalize=True
):
    data = np.zeros(shape, dtype=dtype)
    for x, y, z, value in buildings:
        data[x[0] : x[1], y[0] : y[1], z[0] : z[1]] = value
    if normalize:
        data = data / np.max(data)
    return data


def convert_to_point_cloud(
    data, x_min=0.0, y_min=0.0, z_min=0.0, x_scale=1.0, y_scale=1.0, z_scale=1.0,
    mask=None, cmap='viridis'
):
    mask = mask if mask is not None else data > 0
    x, y, z = np.where(mask)
    x = x_min + x * x_scale
    y = y_min + y * y_scale
    z = z_min + z * z_scale

    values = data[mask].flatten()

    points = np.c_[x, y, z]
    colors = mpl.colormaps.get_cmap(cmap)(values)[:, :3]

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    pcd.colors = o3d.utility.Vector3dVector(colors)

    return pcd

def create_voxel_map(map, obstacle_value=1.0):
    obstacle_mask = map == obstacle_value
    pf_mask = ((~obstacle_mask) & (map > 0))
    params = dict(x_min=0.0, y_min=0.0, z_min=0.0, x_scale=1.0, y_scale=1.0, z_scale=1.0)
    pcd_obstacle = convert_to_point_cloud(map, mask=obstacle_mask, **params)
    pcd_pf = convert_to_point_cloud(map, mask=pf_mask, **params)

    voxel_obstacle = o3d.geometry.VoxelGrid.create_from_point_cloud(pcd_obstacle, voxel_size=1)
    voxel_pf = o3d.geometry.VoxelGrid.create_from_point_cloud(pcd_pf, voxel_size=1)

    obstacle = dict(name="Obstacles", geometry=voxel_obstacle)
    pf = dict(name="Potential Field", geometry=voxel_pf)
    geoms = [obstacle, pf] if np.any(pf_mask) else [obstacle]

    return geoms