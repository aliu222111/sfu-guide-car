# sfu_map

Phase 0 of the guide car (spec section 11): turns the committed SFU GIS data
into what Nav2 and the mission node load. Pure Python, no ROS.

```
pip install -e sfu_map[test]
sfu-map build --previews        # writes ../maps/, about 20 s
sfu-map route aq_convocation_mall "AQ 3150"
sfu-map route "AQ 3150" aq6106 --walk     # allow stairs
sfu-map candidates              # draft elevator entries for connectors.yaml
pytest sfu_map
```

## Inputs

- `data/sfu_gis/sfu_rooms_AQ_WMC_MBC.geojson`: room polygons. Corridors
  (`rm_cat = CIRCULATION`, minus stairs and elevator shafts) are free space.
- `sfu_map/connectors.yaml`: hand entries for what the GIS does not encode:
  elevators, entrances, doorways the automatic bridge misses, outdoor routes,
  door position overrides. Every entry has `verified`, false until checked on site.

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

1. Corridor polygons per floor are unioned with a 5 cm seam close, rasterised.
2. Centrelines: medial axis of the free space at 10 cm after removing slivers
   narrower than 0.6 m, traced into branches, spurs under 2.5 m (or 1.5x the
   corridor half-width) pruned, junctions within 1.5 m merged, then nodes every
   5 m or less.
3. Doors: middle of the wall a room shares with the corridor, 0.4 m into the
   corridor, attached to the nearest centreline edge in line of sight. Rooms up
   to 12 m from a corridor (inside suites) get a door at the nearest corridor
   point with `direct: false`.
4. Corridor pieces the GIS separates with a wall line under 0.45 m are joined by
   an unverified `door` edge at the middle of the shared wall.
5. Stairs on consecutive levels overlapping by IoU 0.3 are joined, never
   car-traversable. Elevators, entrances, links and outdoor routes come from
   `connectors.yaml`.

## Known gaps

- No outdoor paths yet: the OSM footpath pull still fails (spec section 5) and
  this environment cannot reach SFU GIS or Overpass. Until `outdoor_links` are
  drawn, there is no car route between buildings.
- Only one entrance (`aq_convocation_mall`) is entered, derived from where AQ
  3000 meets the Convocation Mall roof footprint. MBC and WMC entrances need
  the level at mall grade confirmed on site.
- Elevator stops are guesses from shaft footprints; see `connectors.yaml`.
- About 95 rooms get no door node (see the report); none are classrooms.
- Nav2 loading is checked by reading the YAML the way map_server does; it has
  not been loaded in a running map_server yet.
