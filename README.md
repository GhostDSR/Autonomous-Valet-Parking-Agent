# Autonomous Valet Parking Agent

**CSAI4014 – Algorithms for Intelligent Systems and Robotics (AISR)**
B.Tech CSE (AI & ML), Semester VII | Project P13 | Group G08 | Batch B3

A car-like (Ackermann-steering) vehicle that finds a free slot in a parking lot and parks itself, simulated in **Webots** with a **Python** controller. A deliberative planner plans the route to the slot, and reactive rules handle the close-range parking manoeuvres.

---

## Team

| Member | SAP ID | Module |
|---|---|---|
| Devansh Singh Rawat | 500124378 | TBD |
| Mohammad Shahvez | 500124132 | TBD |
| Harshil Mittal | 500121907 | TBD |
| Bhavya Talwar | 500121992 | M2 – Route Planner |

## Modules

| Module | Description |
|---|---|
| **M1 – World and Sensors** | Webots parking lot (slots, parked cars), Ackermann car setup, lidar / distance sensors, GPS, compass, free-slot detection |
| **M2 – Route Planner** | Grid map of the parking lot; route planning from the entrance to the free slot using A* and Hybrid A* |
| **M3 – Parking Manoeuvres** | Reactive close-range rules as a state machine: search → drive → align → park → done |
| **M4 – Control and Results** | Path tracking (pure pursuit / Stanley + PID speed control), experiments, plots and comparison tables |

## AISR Syllabus Topics Covered

- Declarative-Procedural-Reflexive Hierarchy
- Intelligent Agents
- Path Planning
- Moving (vehicle kinematics and control)

## Repository Structure

```
Autonomous-Valet-Parking-Agent/
├── worlds/            # Webots world files (parking lot)
├── controllers/       # Webots Python controllers
│   └── valet_controller/
├── planner/           # A* / Hybrid A* route planner
├── behaviours/        # Parking state machine and reactive rules
├── control/           # Path tracking and speed control
├── results/           # Plots, tables and experiment logs
├── docs/              # Report, slides and meeting log
├── requirements.txt
└── README.md
```

## Requirements

- [Webots](https://cyberbotics.com/) (R2023b or later)
- Python 3.8+
- Python packages: `numpy`, `matplotlib`

Install the Python packages:

```bash
pip install -r requirements.txt
```

## How to Run

1. Clone the repository:
   ```bash
   git clone https://github.com/<username>/Autonomous-Valet-Parking-Agent.git
   ```
2. Open Webots and load `worlds/parking_lot.wbt`.
3. Press **Run** to start the simulation. The car will search for a free slot, plan a route and park itself.

To test the route planner on its own (without Webots):

```bash
python planner/astar.py
```

## Results

_To be added: parking time, path length, final position and angle error, success rate, and comparison of methods (e.g. A* vs Hybrid A*)._

## Links

- 🎥 Demo video: _to be added_
- 📄 Project report: _to be added_

## Contribution Guidelines (for team members)

- Each member commits **their own code from their own GitHub account**. Commits are used to track individual contributions.
- Work on your own branch (e.g. `m2-planner`) and open a pull request to merge into `main`.
- Write clear commit messages, e.g. `Add A* planner with obstacle grid`.
- Pull the latest `main` before starting work to avoid conflicts.

---

_Course faculty: [Faculty name]_
