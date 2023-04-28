# Just testing if pmultiastar is working
import pymultiastar as pmstar
from pymultiastar.visualization.img2d_helpers import get_maze, write_path_to_maze
from pathlib import Path
from PIL import Image
from time import perf_counter

MAZE_DIR = Path(__file__).parent.parent / "tests" / "fixtures"
mazes = [("Small Maze", MAZE_DIR / "maze_small.png", [3,3], [1799, 1799]),
         ("Large Maze", MAZE_DIR / "maze_large.png", [0,0], [3999, 3999])]



def run_maze(img_path: Path, maze_start=None, maze_end=None):

    # coordinates should be y,x
    maze_data = get_maze(img_path)

    if maze_start is None:
        maze_start = [0,0]
    if maze_end is None:
        maze_end = [maze_data['img'].shape[0] - 1, maze_data['img'].shape[1] - 1]


    start_cell = [*maze_start, 0]
    end_cell = [*maze_end, 0]

    goal_cells = [(end_cell, 1)]

    # parameters to initialize multi goal a star search
    params = dict(
        map_res=1.0,
        obstacle_value=1.0,  # An obstacle will be the value 1.0
        normalizing_path_cost=maze_data["normalizing_path_cost"],
        goal_weight=0.5,
        path_weight=0.5,
    )

    planner = pmstar.PyMultiAStar(maze_data["map"], **params)
    t0 = perf_counter()
    path, meta = planner.search_multiple(
        start_cell, goal_cells
    )
    t1 = perf_counter()
    dt = (t1-t0) * 1000

    print(f"dt: {dt:.1f}ms, path: {path},  meta: {meta}")
    new_image_path = write_path_to_maze(img_path, path)
    Image.open(new_image_path).show()


def main():
    for i, maze in enumerate(mazes):
        print(f"{i}. {maze[0]}")

    index = int(input("Select a maze to run: "))
    maze = mazes[index]
    maze_path = maze[1]
    maze_start = maze[2]
    maze_end = maze[3]
    run_maze(maze_path, maze_start, maze_end)


if __name__ == "__main__":
    main()
