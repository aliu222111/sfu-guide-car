"""Campus map frame: East-North-Up metres around a fixed origin.

The origin matches ``data/sfu_gis/campus-geometry.json`` so both data sets share
one frame. Over the 2 km campus an equirectangular projection differs from a
true local tangent plane by well under a centimetre, so it is used directly.
"""

from __future__ import annotations

import math

import numpy as np

ORIGIN_LAT = 49.278945
ORIGIN_LON = -122.916558
EARTH_RADIUS = 6378137.0  # WGS84 equatorial radius, metres

_COS_LAT0 = math.cos(math.radians(ORIGIN_LAT))


def lonlat_to_xy(lon, lat):
    """Longitude/latitude in degrees to map-frame x (east), y (north) in metres."""
    lon = np.asarray(lon, dtype=float)
    lat = np.asarray(lat, dtype=float)
    x = np.radians(lon - ORIGIN_LON) * EARTH_RADIUS * _COS_LAT0
    y = np.radians(lat - ORIGIN_LAT) * EARTH_RADIUS
    return x, y


def xy_to_lonlat(x, y):
    """Map-frame metres back to longitude/latitude in degrees."""
    x = np.asarray(x, dtype=float)
    y = np.asarray(y, dtype=float)
    lon = ORIGIN_LON + np.degrees(x / (EARTH_RADIUS * _COS_LAT0))
    lat = ORIGIN_LAT + np.degrees(y / EARTH_RADIUS)
    return lon, lat


def shapely_lonlat_to_xy(x, y, z=None):
    """Adapter for ``shapely.ops.transform``."""
    return lonlat_to_xy(x, y)


def frame_info() -> dict:
    return {
        "name": "map",
        "convention": "ENU metres, equirectangular about the origin",
        "origin_lat": ORIGIN_LAT,
        "origin_lon": ORIGIN_LON,
        "earth_radius_m": EARTH_RADIUS,
    }
