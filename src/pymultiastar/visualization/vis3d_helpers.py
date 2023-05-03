import open3d as o3d
import matplotlib as mpl
import numpy as np
from ..geoplanner.types import GPS, LandingSite, Coord, GeoMultiPlannerResult
from ..geoplanner import GeoPlanner
from ..geoplanner.helper import convert_cost_map_to_float
from typing import List
from .log import logger

default_buildings = [
    [(7, 12), (7, 12), (0, 12 ), 255],
]

def create_map(
    shape=(20, 30, 15), buildings=default_buildings, dtype=np.float32, normalize=True
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


def create_pcd_map(map, obstacle_value=1.0, ds_voxel_size=4.0, **kwargs):
    obstacle_mask = map == obstacle_value
    # pf_mask = (~obstacle_mask) & (map > 0)
    pcd_obstacle = convert_to_point_cloud(
        map, mask=obstacle_mask, color_by_height=True, **kwargs
    )
    # only way to make work better....
    pcd_obstacle = pcd_obstacle.voxel_down_sample(voxel_size=ds_voxel_size)
    obstacle = dict(name="Obstacles", geometry=pcd_obstacle)

    xres = kwargs.get('xres', 1.0)
    i, j, k = map.shape
    map_bounds = np.array([
        [0, 0, 0],
        [0, i * xres, 0],
        [j*xres, i*xres, 0],
        [j*xres, 0, 0],
        [0, 0, 0],
    ])
    line_set = dict(name="Map Bounds", geometry=create_line(map_bounds))
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
    geoms = [line_set, obstacle]

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

def visualize_world(all_geoms, look_at=None, eye=None, point_size=7):

    def init(vis):
        vis.show_ground = True
        vis.ground_plane = o3d.visualization.rendering.Scene.GroundPlane.XY  # type: ignore
        vis.point_size = point_size
        vis.show_axes = True

    if eye is None or look_at is None:
        boundary = all_geoms[0]['geometry'].get_axis_aligned_bounding_box()
        look_at = boundary.get_center()
        extent = boundary.get_extent()
        eye = [extent[0]/2, -extent[1]/1.5, extent[1]]

    logger.info("Please exit by closing the 3D Visualization Window!")
    o3d.visualization.draw(  # type: ignore
        all_geoms,
        lookat=look_at,
        eye=eye,
        up=[0, 0, 1],
        title="World Viewer",
        on_init=init,
        show_ui=True,
    )

def visualize_plan(planner_data, plan_result, xres=2.0):
    logger.info("Loading Map for Visualization ...")

    landing_objects = create_landing_objects(**plan_result)  # type: ignore
    map_3d = np.load(planner_data["cost_map_fp"])
    map_3d = convert_cost_map_to_float(
        map_3d, reverse_yaxis=True, set_max_value_to_inf=False
    )
    world_geoms = create_pcd_map(map_3d, obstacle_value=1.0, xres=xres)
    logger.info("Finished Loaded Map!")

    all_geoms = [*world_geoms, *landing_objects,]
    visualize_world(all_geoms)

    # def init(vis):
    #     vis.show_ground = True
    #     vis.ground_plane = o3d.visualization.rendering.Scene.GroundPlane.XY  # type: ignore
    #     vis.point_size = 7
    #     vis.show_axes = True

    # logger.info("Please exit by closing the 3D Visualization Window!")
    # o3d.visualization.draw(  # type: ignore
    #     [*world_geoms, *landing_objects],
    #     lookat=[375, 375, 0],
    #     eye=[375, -100, 100],
    #     up=[0, 0, 1],
    #     title="World Viewer",
    #     on_init=init,
    #     show_ui=True,
    # )