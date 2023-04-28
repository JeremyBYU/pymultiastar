import open3d as o3d
import sys
import matplotlib as mpl
import pymultiastar as pmstar
import json
import numpy as np
from pymultiastar.visualization.img2d_helpers import get_maze, write_path_to_maze
from pathlib import Path
from pymultiastar.visualization.vis3d_helpers import (
    create_map,
    create_pcd_map,
    create_landing_objects,
)
from pymultiastar.geoplanner.helper import convert_cost_map_to_float
from pymultiastar.geoplanner import (
    GeoPlanner,
    PlannerKwargs,
    GeoMultiPlannerResult,
    VoxelMeta,
    Scenario,
    GPS,
    LandingSite,
)

from log import logger


WORLD_DIR = Path(__file__).parent.parent / "tests" / "fixtures" / "world"
worlds = [
    (
        "Ann Arbor",
        WORLD_DIR / "annarbor/voxel_map.npy",
        WORLD_DIR / "annarbor/meta.json",
    )
]


def plan_scenario(scenario: Scenario, cost_map_fp: Path, voxel_meta: VoxelMeta):
    planner_kwargs = PlannerKwargs()
    geo_planner = GeoPlanner(cost_map_fp, voxel_meta, planner_kwargs)

    start_pos = GPS(*scenario["position"])
    assert scenario["landing_sites"] is not None
    ls_list = [
        LandingSite(
            GPS.from_gps_string(ls["centroid"], ls["height"]),
            landing_site_risk=ls["landing_site_risk"],
        )
        for ls in scenario["landing_sites"]
    ]

    logger.info(start_pos)
    logger.info(ls_list)

    result = geo_planner.plan_multi_goal(start_pos, ls_list)
    logger.info(result)

    return dict(start_gps=start_pos, ls_list=ls_list, geo_planner=geo_planner, plan_results=result)


def run_world(world_name: str, cost_map_fp: Path, meta_fp: Path):
    # read meta data
    with open(meta_fp, "r") as fh:
        meta_data = json.load(fh)
    voxel_meta: VoxelMeta = meta_data["voxel_meta"]
    scenarios = meta_data["scenarios"]


    

    map_3d = np.load(cost_map_fp)
    map_3d = convert_cost_map_to_float(
        map_3d, reverse_yaxis=True, set_max_value_to_inf=False
    )
    world_geoms = create_pcd_map(map_3d, obstacle_value=1.0, xres=voxel_meta["xres"])

    for i, scenario in enumerate(scenarios):
        print(f"{i}. {scenario['name']}")

    index = int(input("Select a Scenario to run: "))
    scenario = scenarios[index]

    result = plan_scenario(scenario, cost_map_fp, voxel_meta)
    landing_objects = create_landing_objects(**result) # type: ignore

    def init(vis):
        vis.show_ground = True
        vis.ground_plane = o3d.visualization.rendering.Scene.GroundPlane.XY # type: ignore
        vis.point_size = 7
        vis.show_axes = True

    o3d.visualization.draw( # type: ignore
        [*world_geoms, *landing_objects],
        lookat=[375, 375, 0],
        eye=[375, -100, 100],
        up=[0, 0, 1],
        title="World Viewer",
        on_init=init,
        show_ui=True,
    )


def main():
    for i, world_name in enumerate(worlds):
        print(f"{i}. {world_name[0]}")

    index = int(input("Select a 3D world to run: "))
    world = worlds[index]
    run_world(*world)


if __name__ == "__main__":
    main()
