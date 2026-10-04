# sfu-guide-car

Navigation software for a wheeled robot that guides SFU Burnaby students to classrooms in AQ, WMC and MBC.

Start with the design spec: docs/superpowers/specs/2026-10-03-campus-guide-design.md

Map data in data/ was pulled from SFU public GIS services on 2026-10-03; sources and query details are in section 5 of the spec.

Phase 0, the map pipeline, is in `sfu_map/` (see its README). Its output is committed in `maps/`: Nav2 floor grids, the campus route graph and the destination index. Rebuild with `pip install -e sfu_map && sfu-map build --previews`.

Phases 1 to 6, the car's ROS 2 Jazzy software, are in `ros2_ws/` (see its README for install, the phase runbook and the topic map). Try it without hardware: `ros2 launch guide_bringup sim.launch.py`, then open http://localhost:8080.

Audit of all phases, with what was found and fixed and what still needs the hardware or a site visit: docs/superpowers/audits/2026-10-03-software-audit.md
