# 10. Contribution

Each member wrote and committed their own module from their own GitHub account, through pull requests checked by CI. The commit and pull-request history of the repository is the record.

| Member | SAP ID | Module | Report sections |
|---|---|---|---|
| Devansh Singh Rawat | 500124378 | M1 – World and Sensors | World and Sensors |
| Bhavya Talwar | 500121992 | M2 – Route Planner, integration | 3, 4, 13, Route Planner |
| Harshil Mittal | 500121907 | M3 – Parking Manoeuvres | _to be added by Harshil_ |
| Mohammad Shahvez | 500124132 | M4 – Control and Results | _to be added by Shahvez_ |

## Bhavya Talwar (M2 – Route Planner and integration)

- **Route planner** (`planner/`): the occupancy-grid map of the lot, grid A*, gradient smoothing, Hybrid A* with the bicycle model, the three-circle collision check, and the metrics and script that compare the three methods.
- **Bay choice and replanning**: two-stage slot choice (A* ranks, Hybrid A* plans), route checking against the live scan, and replanning when the bay is taken or the route is blocked.
- **Hand-over to the parking manoeuvre**: the pre-park pose derived from M3's reverse arc, the exact cubic final approach, and the check for set-up room at end bays.
- **Integration** (`agent/`, `controllers/valet_agent/`): the agent that joins M1–M4 in one loop, the offline simulator with a ray-cast lidar, the three scenarios and the 40-bay sweep, and the Webots controller that runs the same agent.
- **Project infrastructure**: the GitHub Actions workflow that runs every module's tests on each pull request, and `docs/integration.md`.
- **Report and slides**: Sections 3 (Literature Review), 4 (Novelty) and 13 (References), the Route Planner section, and the slide deck.

## Devansh Singh Rawat (M1 – World and Sensors)

_To be written by Devansh._

## Harshil Mittal (M3 – Parking Manoeuvres)

- Implemented the M3 state machine, perpendicular reverse-parking manoeuvre, sensor and clearance checks, stopping-distance rules, actuator limits, bounded alignment corrections and parking-completion checks in the behaviours module.
- Integrated the manoeuvre with the M1 pose and lidar data, M2 pre-parking route and M4 control-command interface.
- Tuned the parking speed and lookahead using Webots comparisons and documented the measured A2, B0 and A5 trials in docs/m3_webots_tuning.md.
- Ran the integrated agent on the latest-main snapshot in Webots with valet_agent; it selected A2 and reached DONE at 52.93 simulated seconds.
- Wrote the M3 report section and prepared the latest-main run evidence.

## Mohammad Shahvez (M4 – Control and Results)

_To be written by Shahvez._
