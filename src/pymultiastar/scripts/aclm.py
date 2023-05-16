from pathlib import Path
import csv

import typer
from .log import logger
from pymultiastar.geoplanner.landing_selection import parse_landing_sites

app = typer.Typer()


@app.command()
def index_csv(path: Path):
    parse_landing_sites(path)

@app.command()
def run_plan(path: Path):
    parse_landing_sites(path)


def main():
    app()

if __name__ == "__main__":
    main()
