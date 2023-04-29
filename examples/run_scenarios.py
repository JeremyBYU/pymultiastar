from pathlib import Path
import typer
import logging
from pymultiastar.visualization.vis3d_helpers import visualize_plan
from rich.prompt import Prompt

from pymultiastar.geoplanner import (
    GeoPlanner,
    Scenario,
    GPS,
    LandingSite,
    create_planner_from_configuration,
)
from pymultiastar.types import LogLevel
from log import logger


app = typer.Typer()
WORLD_DIR = Path(__file__).parent.parent / "tests" / "fixtures" / "world"
ANNARBOR_PLAN = WORLD_DIR / "annarbor/plan.json"

def plan_scenario(scenario: Scenario, geo_planner: GeoPlanner):
    start_pos = GPS(*scenario["position"])
    if scenario.get("landing_sites") is None:
        raise NotImplementedError(
            "This module relies upon the user providing landing sites"
        )
    assert scenario["landing_sites"] is not None

    ls_list = [
        LandingSite(
            GPS(*ls["position"]),
            landing_site_risk=ls["landing_site_risk"],
        )
        for ls in scenario["landing_sites"]
    ]
    if scenario.get("planner_kwargs") is not None:
        logger.warning(
            "Not implemented! Updating planner arguments in each scenario is not yet supported!"
        )

    logger.debug("Start Pos: %s", start_pos)
    logger.debug("Landing Sites: %s", ls_list)

    result = geo_planner.plan_multi_goal(start_pos, ls_list)
    logger.debug("Plan Result: %s", result)

    return dict(
        start_gps=start_pos,
        ls_list=ls_list,
        geo_planner=geo_planner,
        plan_results=result,
    )

@app.command()
def run_city_plan(
    plan: Path = ANNARBOR_PLAN,
    log_level: LogLevel = typer.Option(
        LogLevel.INFO.value,
        help="Specify log level",
    ),
):
    # set log level
    logger.setLevel(getattr(logging, log_level.value))
    logging.getLogger().setLevel(getattr(logging, log_level.value))

    # read planner data
    geo_planner, planner_data = create_planner_from_configuration(plan)
    voxel_meta = planner_data["voxel_meta"]
    scenarios = planner_data["scenarios"]

    # choose a scenario in the planner data
    scenario_str = Prompt.ask(
        "Choose a scenario",
        choices=[scenario["name"] for scenario in scenarios],
        default=scenarios[0]["name"],
    )
    scenario = next(item for item in scenarios if item["name"] == scenario_str)
    # plan the scenario and visualize
    plan_result = plan_scenario(scenario, geo_planner)
    visualize_plan(planner_data, plan_result, xres=voxel_meta["xres"])


if __name__ == "__main__":
    app()
