"""
M2 - Route Planner for the Autonomous Valet Parking Agent.

Public interface (everything else is internal):

    build_parking_lot(occupied)                      -> (GridMap, {slot_id: Slot})
    plan_to_slot(grid_map, start_pose, slot, method) -> (path, stats, goal_pose)
    plan_route(grid_map, start_pose, goal_pose, method) -> path

Poses are (x, y, heading) of the car's rear-axle centre, in Webots world
metres / radians. See docs/m2_route_planner.md.
"""

from planner.astar import plan_path
from planner.grid_map import GridMap
from planner.hybrid_astar import HybridAStarConfig
from planner.parking_lot import ENTRANCE_POSE, Slot, build_parking_lot, free_slots, pre_park_poses
from planner.route_planner import METHODS, plan_route, plan_route_with_stats, plan_to_slot
from planner.vehicle import CollisionChecker, VehicleParams

__all__ = [
    "plan_path", "GridMap", "HybridAStarConfig", "ENTRANCE_POSE", "Slot",
    "build_parking_lot", "free_slots", "pre_park_poses", "METHODS", "plan_route",
    "plan_route_with_stats", "plan_to_slot", "CollisionChecker", "VehicleParams",
]
