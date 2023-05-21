from typing import Tuple
import open3d as o3d
import matplotlib as mpl
import numpy as np
from ..geoplanner.types import (
    GPS,
    LandingSite,
    Coord,
    GeoMultiPlannerResult,
    CoordPath,
    CoordRisk,
    Plan
)
from ..geoplanner import GeoPlanner
from ..geoplanner.helper import convert_cost_map_to_float
from ..types import Cell
from .. import calculate_pareto_points
from typing import List
from .log import logger

default_buildings = [
    [(7, 12), (7, 12), (0, 12), 255],
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

    xres = kwargs.get("xres", 1.0)
    i, j, k = map.shape
    map_bounds = np.array(
        [
            [0, 0, 0],
            [0, i * xres, 0],
            [j * xres, i * xres, 0],
            [j * xres, 0, 0],
            [0, 0, 0],
        ]
    )
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


def swap_xy_list(coord):
    result: List[Coord] = []
    for i in coord:
        result.append((i[1], i[0], i[2]))
    return result


def swap_xy(coord: Coord | CoordPath):
    return (coord[1], coord[0], coord[2])


def create_planning_objects(
    start: Coord,
    goals: List[CoordRisk],
    path: CoordPath,
    radius: float = 0.5,
    flip_xy=True,
):
    start = swap_xy(start) if flip_xy else start
    goals = [(swap_xy(goal[0]), goal[1]) if flip_xy else goal for goal in goals]
    path = swap_xy_list(path) if flip_xy else path
    start_object = dict(
        name="Start Position",
        geometry=create_object(start, color=[0.0, 0.0, 1.0], radius=radius),
    )

    goal_objects = list(
        map(lambda x: create_object(x[0], radius=radius, color=x[1]), goals)
    )
    # logger.debug(f"Projected LS Coords {ls_coords}")
    goal_group = o3d.geometry.TriangleMesh()
    for ob in goal_objects:
        goal_group += ob
    goal_group = dict(name="Landing Sites", geometry=goal_group)

    path_line_set = create_line(path)
    path_line_set = dict(name="Optimal Path", geometry=path_line_set)

    all_labels = [(goal[0], f"{goal[1]:.2f}") for goal in goals]

    return [start_object, goal_group, path_line_set], all_labels


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

    if plan_results and plan_results["path_projected_zero_origin"]:
        path_line_set = create_line(plan_results["path_projected_zero_origin"])
        path_line_set = dict(name="Optimal Path", geometry=path_line_set)
    else:
        path_line_set = dict(name="Optimal Path", geometry=o3d.geometry.TriangleMesh())

    all_labels = [(goal[0], str(goal[1])) for goal in zip(ls_coords, ls_list)]

    return [start_object, ls_group, path_line_set], all_labels


def create_object(
    object: Coord, object_type="ico", color=[1.0, 0.0, 0.0], radius=3.0, cmap="viridis"
):
    object_3d = None
    if object_type == "ico":
        object_3d = o3d.geometry.TriangleMesh.create_icosahedron(radius=radius)

    if not isinstance(color, list):
        color = mpl.colormaps.get_cmap(cmap)(color)[:3]

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


def visualize_world(
    all_geoms,
    all_labels: List[Tuple[Coord, str]] = [],
    look_at=None,
    eye=None,
    point_size=7,
    actions=[],
):
    # using dictionary for easier mutable access
    # used in toggle_labels function
    label_dict = dict(labels_on=True)

    def init(vis):
        vis.show_ground = True
        vis.ground_plane = o3d.visualization.rendering.Scene.GroundPlane.XY  # type: ignore
        vis.point_size = point_size
        vis.show_axes = True
        for label in all_labels:
            vis.add_3d_label(label[0], label[1])

    def toggle_labels(vis, label_dict):
        if vis is not None:
            if label_dict["labels_on"]:
                vis.clear_3d_labels()
            else:
                for label in all_labels:
                    vis.add_3d_label(label[0], label[1])

        label_dict["labels_on"] = not label_dict["labels_on"]

    if eye is None or look_at is None:
        boundary = all_geoms[0]["geometry"].get_axis_aligned_bounding_box()
        look_at = boundary.get_center()
        extent = boundary.get_extent()
        eye = [extent[0] / 2, -extent[1] / 1.5, extent[1]]

    actions.append(("Toggle Labels", lambda vis: toggle_labels(vis, label_dict)))
    logger.info("Please exit by closing the 3D Visualization Window!")
    o3d.visualization.draw(  # type: ignore
        all_geoms,
        lookat=look_at,
        eye=eye,
        up=[0, 0, 1],
        title="World Viewer",
        on_init=init,
        show_ui=True,
        actions=actions,
    )


def visualize_plan(planner_data:Plan, plan_result, xres=2.0, actions=[]):
    logger.info("Loading Map for Visualization ...")

    landing_objects, all_labels = create_landing_objects(**plan_result)  # type: ignore
    map_3d = np.load(planner_data["cost_map_fp"])
    map_3d = convert_cost_map_to_float(
        map_3d, reverse_yaxis=True, set_max_value_to_inf=False
    )
    world_geoms = create_pcd_map(map_3d, obstacle_value=1.0, xres=xres)
    logger.info("Finished Loaded Map!")

    all_geoms = [
        *world_geoms,
        *landing_objects,
    ]

    if planner_data.get("map_glb") is not None:
        model = o3d.io.read_triangle_model(planner_data["map_glb"])
        image_mesh = model.meshes[0].mesh
        image_mesh.rotate(image_mesh.get_rotation_matrix_from_xyz([np.radians(90), 0 , 0]))
        offset = image_mesh.get_axis_aligned_bounding_box().get_extent() / 2.0
        image_mesh.translate(offset)
        all_geoms.append(model)

    visualize_world(all_geoms, all_labels=all_labels, actions=actions)


def is_pareto_efficient(costs):
    """
    :param costs: An (n_points, n_costs) array
    :return: A (n_points, ) boolean array, indicating whether each point is Pareto efficient
    """
    is_efficient = np.ones(costs.shape[0], dtype=bool)
    for i, c in enumerate(costs):
        if is_efficient[i]:
            is_efficient[is_efficient] = np.any(
                costs[is_efficient] <= c, axis=1
            )  # Remove dominated points
    return is_efficient


def plot_pareto(planner, start_cell, goal_cells, planning_parameters, **kwargs):
    import pandas as pd
    import matplotlib.pyplot as plt

    pareto_points = calculate_pareto_points(
        planner, start_cell, goal_cells, **planning_parameters
    )
    df = pd.DataFrame.from_records(pareto_points)
    plot_pareto_(df, **kwargs)
    plt.show()


def plot_pareto_(
    df,
    x="goal_risk",
    y="path_risk",
    goal_name="Goal",
    total_risk="total_risk",
    fname=None,
):
    import seaborn as sns
    import matplotlib.pyplot as plt
    import pandas as pd

    #     df_p = df_p[df_p['path_risk'] < 1.0]
    costs = df[[x, y]].values
    pareto_df = pd.DataFrame(
        costs[is_pareto_efficient(costs)], columns=[x, y]
    ).sort_values(by=[x])
    # ax.scatter(df_[0:4]['total_cost'], df_[0:4]['path_cost'], color='r', label='Chosen Landing Sites')
    ax = sns.scatterplot(data=df, x=x, y=y, color="m", label="All Goals", zorder=5)
    ax.plot(pareto_df[x], pareto_df[y], color="g", label="Pareto Frontier", zorder=1)
    top_item = df.iloc[df[total_risk].idxmin()]
    ax.scatter(
        top_item[x], top_item[y], color="r", label=f"Best {goal_name}", zorder=10
    )
    sns.set(font_scale=1.3)  # crazy big
    ax.set_xlabel(f"{goal_name} Risk ($r_{goal_name[0].lower()}$)")
    ax.set_ylabel("Path Risk ($r_p$)")
    ax.axis("equal")
    plt.legend(loc="upper right")
    if fname:
        plt.savefig(fname, bbox_inches="tight")
    return ax
