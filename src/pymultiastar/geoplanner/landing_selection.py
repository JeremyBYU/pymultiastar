from pathlib import Path
import csv
import pickle
from typing import List
from .types import LandingSite, GPS
from .log import logger
import pickle

import numpy as np
from scipy import spatial
from pyproj import Transformer
from .types import GPS, Coord
from ..util import TicToc


class LSSPlanner:
    csv_fp: Path
    "The file csv file path of all data"
    index_fp: Path
    "Saved spatial index file"
    srid: str
    transformer: Transformer

    def __init__(self, csv_fp: Path, srid="epsg:26918", shift_alt=0.0):
        self.csv_fp = csv_fp
        self.index_fp = csv_fp.with_suffix('.idx')
        self.srid = srid

        self.transformer = Transformer.from_crs("EPSG:4326", srid)
        self.landing_sites: List[LandingSite] = self.parse_landing_sites(csv_fp, shift_alt)
        self.idx = self.index_landing_sites() # 1 ms to load cache, 200 ms cold

    def index_landing_sites(self) -> spatial.KDTree:

        if self.index_fp.exists():
            idx = pickle.load(open(self.index_fp, 'rb'))
            return idx
        # create data array
        points = []
        for site in self.landing_sites:
            p: Coord[float] = self.transformer.transform(*site.centroid.to_array())
            points.append([p[0], p[1]])
        points = np.array(points)
        # Create the index
        idx = spatial.KDTree(points)
        pickle.dump(idx, open(self.index_fp, 'wb'))
        return idx

    def query(
        self,
        location: GPS,
        radius: float = 200,
        max_altitude: float = 60.0,
        max_ls_risk:float =0.6,
    ):
        p: Coord[float] = self.transformer.transform(*location.to_array())

        nodes:List[int] = self.idx.query_ball_point((p[0], p[1]), radius)
        logger.debug("Found %r sites before filtering", len(nodes))
        # filter data
        final_results:List[LandingSite] = []
        for node in nodes:
            ls = self.landing_sites[node]
            if ls.landing_site_risk <=  max_ls_risk and ls.centroid.alt <= max_altitude:
                final_results.append(ls)
        logger.debug("Found %r sites after filtering", len(final_results))
        return final_results

    @staticmethod
    def parse_landing_sites(csv_fp: Path, shift_alt: float = 0.0) -> List[LandingSite]:
        """Parse landing sites in CSV form

        Args:
            csv_fp (Path): Path to the landing sites csv file

        Returns:
            List[LandingSite]: List of a landing sites
        """
        landing_sites: List[LandingSite] = []
        with open(csv_fp, newline="") as csvfile:
            reader = csv.DictReader(csvfile)
            for row in reader:
                centroid = GPS(
                    float(row["gps.lat"]), float(row["gps.lon"]), float(row["gps.alt"]) + shift_alt
                )
                landing_sites.append(
                    LandingSite(
                        centroid=centroid,
                        landing_site_risk=float(row["landing_site_risk"]),
                        uid=int(row["osm_id"]),
                        radius=float(row["radius"]),
                    )
                )

        return landing_sites
