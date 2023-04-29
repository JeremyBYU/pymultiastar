import numpy as np
import pytest
from pymultiastar.geoplanner.types import GPS


@pytest.mark.parametrize("gps_lat,gps_lon", [(42.27772, -83.74824)])
def test_gps_from_string(gps_lat: float, gps_lon: float):
    gps_str = f"{gps_lat},{gps_lon}"
    gps = GPS.from_gps_string(gps_str)
    assert pytest.approx(gps_lat) == gps.lat
    assert pytest.approx(gps_lon) == gps.lon


@pytest.mark.parametrize("gps_lat,gps_lon", [(42.27772, -83.74824)])
def test_gps_to_array(gps_lat: float, gps_lon: float):
    gps = GPS(lat=gps_lat, lon=gps_lon, alt=1.0)
    gps_array = gps.to_array()
    np.testing.assert_allclose(gps_array, [gps_lat, gps_lon, 1.0])