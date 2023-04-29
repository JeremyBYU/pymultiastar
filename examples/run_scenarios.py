from pathlib import Path
import typer
import json
import uuid
import logging
from pymultiastar.visualization.vis3d_helpers import visualize_plan
from pymultiastar.geoplanner.helper import EnhancedJSONEncoder
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
THIS_DIR = Path(__file__).parent
WORLD_DIR = Path(__file__).parent.parent / "tests" / "fixtures" / "world"
ANNARBOR_PLAN = WORLD_DIR / "annarbor/plan.json"
OUTPUT_DIR = THIS_DIR.parent / "output"

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
    run_all_scenarios:bool = False,
    visualize:bool = False,
    output_dir:Path = OUTPUT_DIR
):
    # set log level
    logger.setLevel(getattr(logging, log_level.value))
    logging.getLogger().setLevel(getattr(logging, log_level.value))

    # read planner data
    geo_planner, planner_data = create_planner_from_configuration(plan)
    voxel_meta = planner_data["voxel_meta"]
    scenarios = planner_data["scenarios"]

    scenario_results = []
    chosen_scenarios = []
    if not run_all_scenarios:
        # choose a scenario in the planner data
        scenario_str = Prompt.ask(
            "Choose a scenario",
            choices=[scenario["name"] for scenario in scenarios],
            default=scenarios[0]["name"],
        )
        scenario = next(item for item in scenarios if item["name"] == scenario_str)
        chosen_scenarios.append(scenario)

    for scenario in chosen_scenarios:
        scenario_result = plan_scenario(scenario, geo_planner)
        scenario_results.append(scenario_result['plan_results'])
        if visualize:
            visualize_plan(planner_data, scenario_result, xres=voxel_meta["xres"])

    file_name = planner_data.get('name', str(uuid.uuid4())) + ".json"
    output_fp = output_dir / file_name
    logger.info("Writing file to %s", output_fp)
    with open(output_fp, 'w') as fh:
        json.dump(scenario_results, fh, cls=EnhancedJSONEncoder, indent=2)




if __name__ == "__main__":
    app()
