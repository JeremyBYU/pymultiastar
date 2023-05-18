import json
from pathlib import Path
from .types import PlannerKwargs, VoxelMeta, LSSKwargs, Plan
from .geoplanner import GeoPlanner
from .landing_selection import LSSPlanner


def create_planner_from_configuration(plan: Path):
    with open(plan, "r") as fh:
        planner_data:Plan = json.load(fh)
    planner_data["cost_map_fp"] = plan.parent / planner_data["cost_map_fp"] # type: ignore
    planner_data["lss_kwargs"]["csv_fp"] =  plan.parent / planner_data["lss_kwargs"]["csv_fp"] # type: ignore
    if planner_data.get('map_glb') is not None:
        planner_data["map_glb"] =  str(plan.parent / planner_data["map_glb"]) # type: ignore
    voxel_meta: VoxelMeta = planner_data["voxel_meta"]
    geo_planner = GeoPlanner(
        planner_data["cost_map_fp"],
        voxel_meta,
        PlannerKwargs(**planner_data["planner_kwargs"]),
    )
    lss_planner = LSSPlanner(**planner_data["lss_kwargs"]) # type: ignore
    return geo_planner, lss_planner, planner_data
