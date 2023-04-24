import open3d as o3d
import sys
import matplotlib as mpl
import pymultiastar as pmstar
import json
import numpy as np
from pymultiastar.visualization.img2d_helpers import get_maze, write_path_to_maze
from pathlib import Path
from pymultiastar.visualization.vis3d_helpers import create_map, create_pcd_map
from pymultiastar.geoplanner.helper import convert_cost_map_to_float
from pymultiastar.geoplanner import (
    GeoPlanner,
    PlannerKwargs,
    GeoMultiPlannerResult,
    VoxelMeta,
    Scenario,
    GPS
)


WORLD_DIR = Path(__file__).parent.parent / "tests" / "fixtures" / "world"
worlds = [
    (
        "Ann Arbor",
        WORLD_DIR / "annarbor/voxel_map.npy",
        WORLD_DIR / "annarbor/meta.json",
    )
]

def find_landing_sites(start_pos):
    pass

def plan_scenario(scenario:Scenario, cost_map_fp: Path, voxel_meta: VoxelMeta):
    planner_kwargs = PlannerKwargs()
    geo_planner = GeoPlanner(cost_map_fp, voxel_meta, planner_kwargs)

    start_pos = GPS(*scenario["position"][:2][::-1], alt=scenario['position'][2])
    print(start_pos)
    # geo_planner.plan_multi_goal()


def run_world(world_name: str, cost_map_fp: Path, meta_fp: Path):
    map_3d = np.load(cost_map_fp)
    map_3d = convert_cost_map_to_float(map_3d, set_max_value_to_inf=False)
    world_geoms = create_pcd_map(map_3d, obstacle_value=1.0)

    with open(meta_fp, "r") as fh:
        meta_data = json.load(fh)

    voxel_meta: VoxelMeta = meta_data["voxel_meta"]
    scenarios = meta_data["plan_multiple"]

    for i, scenario in enumerate(scenarios):
        print(f"{i}. {scenario['name']}")

    index = int(input("Select a Scenario to run: "))
    scenario = scenarios[index]

    plan_scenario(scenario, cost_map_fp, voxel_meta)
    sys.exit(0)

    def init(vis):
        vis.show_ground = True
        vis.ground_plane = o3d.visualization.rendering.Scene.GroundPlane.XY
        vis.point_size = 7
        vis.show_geometry("Potential Field", False)
        vis.show_axes = True

    o3d.visualization.draw(
        [*world_geoms],
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
