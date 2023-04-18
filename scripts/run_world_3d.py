import open3d as o3d
import matplotlib as mpl
import numpy as np

def create_map(buildings=[[(1,3), (3,10), (0,3), 255]]):
    data = np.zeros((10, 10, 5), dtype=np.uint8)
    for x, y, z, value in buildings:
        data[x[0]:x[1], y[0]:y[1], z[0]:z[1]] = value
    return data

def convert_to_point_cloud(data, x_min=0.0, y_min=0.0, z_min=0.0,
                           x_scale=1.0, y_scale=1.0, z_scale=1.0):
    x, y, z = np.where(data > 0)
    x = x_min + x * x_scale
    y = y_min + y * y_scale
    z = z_min + z * z_scale

    values = data[data > 0].flatten() / np.max(data)

    points = np.c_[x, y, z]
    colors = mpl.colormaps.get_cmap('viridis')(values)[:, :3]

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    pcd.colors = o3d.utility.Vector3dVector(colors)

    return pcd


def main():
    data = create_map()
    pcd = convert_to_point_cloud(data)
    voxel_grid = o3d.geometry.VoxelGrid.create_from_point_cloud(pcd,
                                                                voxel_size=1)

    o3d.visualization.draw([voxel_grid])

if __name__ == "__main__":
    main()
