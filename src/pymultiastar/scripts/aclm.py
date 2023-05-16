from pathlib import Path
import logging
import csv

import typer
from .log import logger
from pymultiastar.geoplanner.landing_selection import LSSPlanner, GPS
from pymultiastar.util import TicToc
from pymultiastar.types import LogLevel

app = typer.Typer()


@app.command()
def index_csv(
    path: Path,
    log_level: LogLevel = typer.Option(
        LogLevel.INFO.value,
        help="Specify log level",
    ),
):
    # set log level
    logger.setLevel(getattr(logging, log_level.value))
    logging.getLogger().setLevel(getattr(logging, log_level.value))

    lss = LSSPlanner(path)
    loc = GPS(40.746077, -73.99050, 19.0)

    t1 = TicToc(logger)
    t1.tic()
    landing_sites = lss.query(loc)
    t1.toc("Landing Site Selection")




@app.command()
def run_plan(path: Path):
    pass
    # parse_landing_sites(path)


def main():
    app()


if __name__ == "__main__":
    main()
