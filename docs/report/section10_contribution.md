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

_To be written by Harshil._

## Mohammad Shahvez (M4 – Control and Results)

_To be written by Shahvez._
