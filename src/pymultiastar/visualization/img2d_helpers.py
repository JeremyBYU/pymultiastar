import numpy as np
from pathlib import Path
from PIL import Image, ImageDraw
import shutil
from .log import logger

def get_maze(img_path: Path, offset=3):
    """This will load an image of maze into a 3D numpy array

    Args:
        img_path (Path): Path to img holding maze
        offset (int, optional): Where to offset to start and finish. Defaults to 3.

    Returns:
        dict: Returns a dictionary of maze data
    """
    img = np.asarray(Image.open(img_path))
    logger.debug(f"Maze name: {img_path.name}, Shape: {img.shape}")
    logger.debug(f"Unique values: {np.unique(img)}")
    # convert to float and an empty third dimension
    img_f = np.expand_dims((img / np.max(img)).astype(np.float32), axis=2)
    # Best case scenario is a diagonal path, lets guess it will be 50% bigger
    normalizing_path_cost = 1.5 * np.sqrt(img.shape[0] ** 2 + img.shape[1] ** 2)

    return dict(
        img=img,
        map=img_f,
        normalizing_path_cost=normalizing_path_cost,
    )

def write_path_to_maze(img_path: Path, path:np.array, width=1):
    """This will write the solution of the maze as a red line

    Args:
        img_path (Path): Original image of the maze
        path (np.array): Solution path through the maze
    """
    new_img_path = img_path.with_name(img_path.stem + "_solution.png")
    shutil.copy(img_path, new_img_path)
    path = path[:, :2]
    path = np.flip(path, axis=1) # the "map" has the row dimension as the first dimension (because images..)
    path = list(map(tuple, path))
    with Image.open(new_img_path).convert('RGB') as im:
        draw = ImageDraw.Draw(im)
        draw.line(path, fill=(255,0,0), width=width)
        im.save(new_img_path)
    
    return new_img_path