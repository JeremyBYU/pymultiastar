from pathlib import Path
import csv
from typing import List
from .types import LandingSite, GPS
from .log import logger


class LandingSiteSelection:
    index_fp: Path
    def __init__(self, index_fp: Path):
        self.index_fp = index_fp




    @staticmethod
    def parse_landing_sites(csv_fp: Path) -> List[LandingSite]:
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
                    float(row["gps.lat"]), float(row["gps.lon"]), float(row["gps.alt"])
                )
                landing_sites.append(
                    LandingSite(
                        centroid=centroid, landing_site_risk=float(row["landing_site_risk"]),
                        uid=int(row['osm_id']),
                        radius=float(row["radius"])
                    )
                )

        return landing_sites   