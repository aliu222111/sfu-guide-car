import numpy as np
from shapely.geometry import box

from sfu_map import geo
from sfu_map.raster import GridSpec, load_nav2_map, rasterize, write_nav2_map


def test_lonlat_roundtrip():
    x, y = geo.lonlat_to_xy(-122.9150, 49.2800)
    lon, lat = geo.xy_to_lonlat(x, y)
    assert abs(lon - -122.9150) < 1e-9 and abs(lat - 49.2800) < 1e-9


def test_origin_maps_to_zero():
    x, y = geo.lonlat_to_xy(geo.ORIGIN_LON, geo.ORIGIN_LAT)
    assert abs(x) < 1e-9 and abs(y) < 1e-9


def test_north_and_east_are_positive():
    x, y = geo.lonlat_to_xy(geo.ORIGIN_LON + 0.001, geo.ORIGIN_LAT + 0.001)
    assert 70 < x < 75  # 0.001 deg of longitude at 49.28 N is about 72.7 m
    assert 110 < y < 112


def test_grid_orientation_and_roundtrip(tmp_path):
    spec = GridSpec.around((0, 0, 10, 10), 0.1, margin=0)
    # A free square in the north-east corner must land top-right in the image.
    mask = rasterize(box(8, 8, 10, 10), spec)
    rows, cols = np.nonzero(mask)
    assert rows.max() < 25 and cols.min() > 75

    yaml_path = write_nav2_map(mask, spec, tmp_path / "t.png")
    free, spec2 = load_nav2_map(yaml_path)
    assert (free == mask).all()
    assert spec2.origin_x == spec.origin_x and spec2.origin_y == spec.origin_y
    x, y = spec2.to_xy(cols.mean(), rows.mean())
    assert abs(x - 9) < 0.1 and abs(y - 9) < 0.1


def test_holes_and_islands():
    spec = GridSpec.around((0, 0, 10, 10), 0.1, margin=0)
    ring = box(0, 0, 10, 10).difference(box(2, 2, 8, 8))
    island = box(4, 4, 6, 6)
    mask = rasterize(ring.union(island), spec)
    c, r = spec.to_pixel(3, 3)
    assert not mask[int(r), int(c)]
    c, r = spec.to_pixel(5, 5)
    assert mask[int(r), int(c)]
    c, r = spec.to_pixel(1, 1)
    assert mask[int(r), int(c)]
