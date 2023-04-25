import open3d as o3d
import matplotlib as mpl
import numpy as np
from ..geoplanner.types import GPS, LandingSite, Coord, GeoMultiPlannerResult
from ..geoplanner import GeoPlanner
from typing import List
from .log import logger

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
    data,
    xmin=0.0,
    ymin=0.0,
    zmin=0.0,
    xres=1.0,
    mask=None,
    cmap="viridis",
    color_by_height=False,
    **kwargs,
):
    mask = mask if mask is not None else data > 0
    y, x, z = np.where(mask)  # notice that the first dimension is y!
    x = xmin + x * xres
    y = ymin + y * xres
    z = zmin + z * xres

    points = np.c_[x, y, z]
    if color_by_height:
        values = points[:, 2] / np.max(points[:, 2])
    else:
        values = data[mask].flatten()
    colors = mpl.colormaps.get_cmap(cmap)(values)[:, :3]

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    pcd.colors = o3d.utility.Vector3dVector(colors)

    return pcd


def create_pcd_map(map, obstacle_value=1.0, **kwargs):
    obstacle_mask = map == obstacle_value
    # pf_mask = (~obstacle_mask) & (map > 0)
    pcd_obstacle = convert_to_point_cloud(
        map, mask=obstacle_mask, color_by_height=True, **kwargs
    )
    # only way to make work better....
    pcd_obstacle = pcd_obstacle.voxel_down_sample(voxel_size=4.0)
    obstacle = dict(name="Obstacles", geometry=pcd_obstacle)

    # when the point cloud is bigger than 7_000_000 points it will reappear if deselected
    # this was just a test to prove it
    # print(pcd_obstacle)
    # pcd = o3d.geometry.PointCloud()
    # colors = mpl.colormaps.get_cmap('viridis')(np.random.rand(10_000_000))[:, :3]
    # pcd.points = o3d.utility.Vector3dVector(np.random.randn(10_000_000, 3) * 10)
    # pcd.colors = o3d.utility.Vector3dVector(colors)
    # pcd_pf = convert_to_point_cloud(map, mask=pf_mask, **kwargs)
    # obstacle = dict(name="Obstacles", geometry=pcd)

    # pf = dict(name="Potential Field", geometry=pcd_pf)
    # geoms = [obstacle, pf] if np.any(pf_mask) else [obstacle]
    geoms = [obstacle]

    return geoms


def create_landing_objects(
    start_gps: GPS,
    ls_list: List[LandingSite],
    geo_planner: GeoPlanner,
    plan_results: GeoMultiPlannerResult,
):
    start_coords = geo_planner.transform_gps_to_projected_zero_origin(start_gps)
    start_object = dict(
        name="Start Position",
        geometry=create_object(start_coords, color=[0.0, 0.0, 1.0]),
    )
    # logger.debug(f"Projected Start Coords {start_coords}")

    ls_coords = list(
        map(
            lambda x: geo_planner.transform_gps_to_projected_zero_origin(x.centroid),
            ls_list,
        )
    )
    ls_objects = list(map(lambda x: create_object(x), ls_coords))
    # logger.debug(f"Projected LS Coords {ls_coords}")
    ls_group = o3d.geometry.TriangleMesh()
    for ob in ls_objects:
        ls_group += ob
    ls_group = dict(name="Landing Sites", geometry=ls_group)

    path_line_set = create_line(plan_results["path_projected_zero_origin"])
    path_line_set = dict(name="Optimal Path", geometry=path_line_set)

    return [start_object, ls_group, path_line_set]


def create_object(object: Coord, object_type="ico", color=[1.0, 0.0, 0.0], radius=3.0):
    object_3d = None
    if object_type == "ico":
        object_3d = o3d.geometry.TriangleMesh.create_icosahedron(radius=radius)

    object_3d.translate(list(object))
    object_3d.paint_uniform_color(color)
    object_3d.compute_vertex_normals()
    object_3d.compute_triangle_normals()
    return object_3d


def create_line(points, color=[0, 1, 0]):
    points = np.array(points)
    lines = np.array([[i, i + 1] for i in range(0, len(points) - 1, 1)])

    line_set = o3d.geometry.LineSet(
        points=o3d.utility.Vector3dVector(points),
        lines=o3d.utility.Vector2iVector(lines),
    )
    line_set.paint_uniform_color(color)
    return line_set
