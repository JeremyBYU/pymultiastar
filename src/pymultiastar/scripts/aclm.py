from pathlib import Path
import csv

import typer
from .log import logger
from pymultiastar.geoplanner.landing_selection import LandingSiteSelection, GPS

app = typer.Typer()


@app.command()
def index_csv(path: Path):
    lss = LandingSiteSelection(path)
    loc = GPS(40.746077, -73.99050, 19.0)
    lss.query(loc)
    

@app.command()
def run_plan(path: Path):
    pass
    # parse_landing_sites(path)


def main():
    app()

if __name__ == "__main__":
    main()
