# sfu_map

Phase 0 of the guide car (spec section 11): turns the committed SFU GIS data
into what Nav2 and the mission node load. Pure Python, no ROS.

```
pip install -e sfu_map[test]
sfu-map build --previews        # writes ../maps/, about a minute
sfu-map route aq_convocation_mall "AQ 3150"
sfu-map route "AQ 3150" aq6106 --walk     # allow stairs
sfu-map candidates              # draft elevator entries for connectors.yaml
sfu-map walkways                # re-extract outdoor walkways from the accessibility map PDF (needs [pdf])
pytest sfu_map
```

## Inputs

- `data/sfu_gis/sfu_rooms_AQ_WMC_MBC.geojson`: room polygons. Corridors
  (`rm_cat = CIRCULATION`, minus stairs and elevator shafts) are free space.
- `sfu_map/connectors.yaml`: hand entries for what the GIS does not encode:
  elevators, entrances, doorways the automatic bridge misses, outdoor routes,
  door position overrides. Every entry has `verified`, false until checked on site.
- `data/sfu_derived/accessible_walkways.geojson`: outdoor walkways and building
  entrances, extracted by `sfu-map walkways` from SFU's accessibility map PDF
  (see below). Committed, so builds do not need the PDF or pymupdf.

## Outputs, in `maps/`

| file | what |
|---|---|
| `floors/<floor>.png` + `.yaml` | Nav2 `map_server` map per floor, 5 cm, trinary. All floors of a building share one origin. PNG because map_server reads it and it is 50x smaller than PGM; `--format pgm` if wanted. |
| `outdoor.png` + `.yaml` | 10 cm outdoor grid, only once `outdoor_links` exist. |
| `route_graph.json` | Nodes and undirected edges. |
| `destinations.json` | `"AQ 3150"` to door node, floor, room type, searchable flag. |
| `build_report.txt` | Counts, graph pieces per floor, rooms without a door node, check routes. |
| `previews/<floor>.png` | Free space with the graph drawn on it, for review. |

Floor ids are `<building>-<GIS fl_id>`: `AQ-03` is AQ 3000 level, `MBC-00B` is
MBC 100 level, `WMC-P1` is WMC P1 level.

### Frame

One `map` frame for the campus: x east, y north, metres, equirectangular about
49.278945, -122.916558, the same origin as `campus-geometry.json` (whose y is
stored south-positive and is flipped on load). Floors share XY.

### Graph

Node types: `corridor`, `corridor_junction`, `door` (one per room, `room` and
`direct` attributes), `doorway` (a door between two corridor pieces), `stairs`,
`elevator`, `entrance`, `outdoor_junction`.

Edge attributes: `type` (`corridor`, `door`, `elevator`, `stairs`, `outdoor`),
`length` (metres; elevators cost 20 m plus 4 m per level, stairs 8 m per
level), `car_traversable` (false for stairs), `verified`.

`door` edges are where the mission node stops and asks the student to hold the
door (spec section 8).

## How it is built

1. Free space per floor is the corridor polygons plus open public rooms a car
   can drive through (lobbies, lounges, study areas, dining, galleries;
   `gisdata.PASSABLE_TYPES`), unioned with a 5 cm seam close, rasterised.
2. Centrelines: medial axis of the free space at 5 cm after removing slivers
   narrower than 0.6 m, traced into branches, spurs under 2.5 m (or 1.5x the
   corridor half-width) pruned, junctions within 1.5 m merged, then nodes every
   5 m or less.
3. Doors: middle of the wall a room shares with the corridor, 0.8 m into the
   corridor (0.6 or 0.4 m in narrow ones) so Nav2 can reach it, attached to the
   nearest centreline edge in line of sight. Rooms up
   to 12 m from a corridor (inside suites) get a door at the nearest corridor
   point with `direct: false`.
4. Corridor pieces the GIS separates with a wall line under 0.45 m are joined by
   an unverified `door` edge at the middle of the shared wall. Pieces still
   apart that face the same single room across walls under 3 m are joined
   through that room (`through_room` on the edge, a "check on site" warning,
   and a 60 m routing penalty so routes avoid it when they can).
5. Every corridor edge is checked against the Nav2 grid as the planner sees
   it: 5 cm cells, walls grown by the car's 0.19 m half-width. An edge whose
   ends are not connected there (a GIS wall line too thin for a coarser raster,
   or a gap narrower than the car) is removed and listed in the build report,
   whose "graph pieces the Nav2 grid splits" line must read 0.
6. Outdoor: the walkway lines become `outdoor_junction` nodes at most 10 m
   apart joined by `outdoor` edges; loose ends within 8 m are joined, and
   pieces within 15 m (40 m across the Convocation Mall plaza) when the join
   crosses no building. Each walkway entrance becomes an `entrance` node joined
   to the largest free-space piece of its building and level within 25 m.
7. Stairs on consecutive levels overlapping by IoU 0.3 are joined, never
   car-traversable. Elevators, entrances, links and outdoor routes come from
   `connectors.yaml`.

## Outdoor walkways from the accessibility map

`walkways.py` reads SFU's Burnaby accessibility map (`data/sfu_pdf/sfu_Accessibility_campus_map_2024.pdf`):
renders it at 300 dpi, keeps the navy accessible-route colour, removes text and
icons with a 4 px opening, closes the dashed routes, georeferences the page to
the GIS footprints (similarity transform, seeded from building labels, refined
by footprint IoU: 0.57), skeletonises and bridges gaps under 40 m that keep
direction. Entrances come from the map's level labels next to each building.
Only AQ, MBC and WMC entrances are kept (the buildings in the GIS rooms).
Accuracy is a few metres; the car follows the walkways with GPS and its own
cameras, not the line.

## Known gaps

- Outdoor walkways and entrances come from a map drawing, not a survey. Check
  each entrance's level and door on site; a map entrance can be a stair.
- Reachability from the AQ Convocation Mall entrance: 1764 of 1923
  destinations, 96 of 98 classrooms (`build_report.txt`). WMC 2200 and 2202,
  MBC-04 and a few suites stay apart until someone looks on site and adds
  `links` entries.
- Elevator stops are guesses from shaft footprints; see `connectors.yaml`.
- About 95 rooms get no door node (see the report); none are classrooms.
- Nav2 loading is checked by reading the YAML the way map_server does; it has
  not been loaded in a running map_server yet.
