"""
The valet agent: one object that runs the whole system, once per time step.

This is the top of the declarative - procedural - reflexive hierarchy:

    DELIBERATIVE (M2, this file)   which slot? which route? replan when the
                                   world changes. Slow (about 1 s), runs rarely.
    PROCEDURAL   (M3 state machine) DRIVE -> ALIGN -> PARK -> DONE, the
                                   parking manoeuvre itself.
    REFLEXIVE    (M3 safety)       emergency stop when an obstacle is too close.
    CONTROL      (M4)              steering + throttle/brake to follow the
                                   route or M3's parking commands.

Sensing (M1): the agent is fed a lidar scan, pose and speed. It uses M1's
functions to turn the scan into body clearances and slot occupancy.

Per step:   actuation = agent.step(now_s, pose, speed, lidar_ranges)

The same code runs in Webots (controllers/valet_agent) and offline
(agent/run_offline.py); only where the sensor data comes from differs.

Life cycle of a run:
    SEARCH   stopped; look at the slots, choose one and plan a route to it
    DRIVE    M4 follows the route; every `replan_period_s` the route is
             re-checked against the latest scan (slot taken? path blocked?)
    STOPPING something changed (or M3 made an emergency stop): brake to a
             standstill, then plan again from where the car is
    ALIGN / PARK   M3's manoeuvre into the bay (no replanning here: the car
             is committed and M3's reflexive safety layer stops it if needed)
    DONE / FAILED
"""

import dataclasses
import math
from dataclasses import dataclass

from behaviours.parking_fsm import ParkingFSM
from behaviours.parking_manoeuvre import ParkingManoeuvre
from behaviours.parking_types import (ControlMode, ParkingCommand, ParkingObservation,
                                      ParkingSlot, ParkingState, Pose, VehicleGeometry)
from controllers.valet_controller.m1_sensors import classify_slots_from_scan, lidar_points_world
from m4_control.follower import FollowerConfig
from m4_control.lateral import PurePursuit, Stanley
from m4_control.m4_controller import Actuation, M4Controller
from m4_control.planner_bridge import path_from_planner
from planner.hybrid_astar import HybridAStarConfig
from planner.parking_lot import BAY_DEPTH, BAY_WIDTH, make_slots
from planner.replanning import SlotChoice, build_map, check_route, choose_slot, nearest_index
from planner.route_planner import plan_to_slot
from planner.vehicle import CollisionChecker, VehicleParams


@dataclass
class AgentConfig:
    method: str = "hybrid_astar"     # M2 planner: "hybrid_astar" | "astar_smoothed" | "astar"
    lateral: str = "stanley_ff"      # M4 steering: "stanley_ff" | "stanley" | "pure_pursuit"
    cruise_speed: float = 1.0        # m/s on the route; M3 wants 0.85 m clear ahead at this speed
    reverse_speed: float = 0.5       # m/s when the route reverses (tight turns track badly faster)
    planning_margin_m: float = 0.4   # extra clearance the route keeps, for path-tracking error
    replan_period_s: float = 1.0     # how often the route is re-checked while driving
    retry_period_s: float = 5.0      # wait after a failed plan before trying again
    lidar_fov: float = 2 * math.pi
    lidar_max_range: float = 40.0
    obstacle_radius_m: float = 10.0  # only lidar points this close can become new obstacles
    static_tolerance_m: float = 0.4  # points this close to a known wall / car are not new obstacles
    slots_from_lidar: bool = True    # False: keep the slot states given at the start (experiments)


def body_clearances(ranges, field_of_view, front_extent, rear_extent, half_width,
                    max_range=40.0):
    """
    Distance from the car body to the nearest lidar point, for each side of
    the car: {"front", "rear", "left", "right"} in metres (None = no point).

    Each lidar point is put into the car's frame (x forward from the rear
    axle, y to the left). The body is the rectangle
    -rear_extent <= x <= front_extent, |y| <= half_width, and the exact
    distance from a point to it is

        hypot(max(x - front_extent, -rear_extent - x, 0), max(|y| - half_width, 0))

    The point counts for the side it lies furthest outside of (a point
    beside the front wheel counts as "left", not "front"), so the corners are
    covered as well.

    Why not M1's body_clearances_from_scan: it subtracts the body's extent in
    the ray's direction (front_extent*|cos| + half_width*|sin|). That is a
    safe lower bound but a very loose one for side rays - for a car parked
    0.6 m beside us it reports 0 m - so M3 would stop the car in every bay
    that has a neighbour. This is the exact distance instead.
    """
    gaps = {"front": None, "rear": None, "left": None, "right": None}
    if len(ranges) < 2:
        return gaps
    step = field_of_view / (len(ranges) - 1)
    for i, r in enumerate(ranges):
        if not math.isfinite(r) or r <= 0 or r >= max_range:
            continue
        a = -field_of_view / 2 + i * step
        x, y = r * math.cos(a), r * math.sin(a)
        out = {"front": x - front_extent, "rear": -rear_extent - x,
               "left": y - half_width, "right": -half_width - y}
        side = max(out, key=out.get)
        gap = math.hypot(max(out["front"], out["rear"], 0.0), max(out["left"], out["right"], 0.0))
        if gaps[side] is None or gap < gaps[side]:
            gaps[side] = gap
    # No point on a side means nothing within lidar range there.
    return {k: (max_range if v is None else v) for k, v in gaps.items()}


# Simple placeholder vehicle; must match planner/vehicle.py and the Webots car.
DEFAULT_GEOMETRY = VehicleGeometry(wheelbase_m=2.8, length_m=4.6, width_m=1.9,
                                   rear_overhang_m=1.0, max_steer_rad=0.5)


class ValetAgent:
    def __init__(self, geometry=DEFAULT_GEOMETRY, config=None):
        self.geometry = geometry
        self.config = config or AgentConfig()
        # The planner and M3 describe the same car with different classes.
        # Routes keep `planning_margin_m` from obstacles, because the car does
        # not follow them exactly (it cuts corners a little, more in reverse).
        # If the car has stopped somewhere tight, that margin can make every
        # route look blocked, so planning falls back to no extra margin.
        self.vehicle = VehicleParams(wheelbase=geometry.wheelbase_m, max_steer=geometry.max_steer_rad,
                                     length=geometry.length_m, width=geometry.width_m,
                                     rear_overhang=geometry.rear_overhang_m,
                                     safety_margin=self.config.planning_margin_m)
        self.tight_vehicle = dataclasses.replace(self.vehicle, safety_margin=0.0)
        self.route_vehicle = self.vehicle   # the one the current route was planned with
        self.slots = make_slots()
        self.fsm = ParkingFSM(geometry, ParkingManoeuvre(geometry))
        # Hybrid A* with an exact final approach: M3 needs the route to end
        # precisely on the pre-park pose (see planner/final_approach.py).
        self.planner_config = HybridAStarConfig(final_approach_m=5.0)
        # M3 drives about one turning radius past the bay before reversing in
        # (radius at 85 % steering, see behaviours/parking_manoeuvre.py), plus a margin.
        self.setup_distance_m = geometry.wheelbase_m / math.tan(0.85 * geometry.max_steer_rad) + 0.3
        self.follower_cfg = FollowerConfig(max_steer=geometry.max_steer_rad,
                                           v_fwd=self.config.cruise_speed,
                                           v_rev=self.config.reverse_speed)

        self.slot_status = {}        # slot id -> "free" / "occupied", from the lidar
        self.obstacles = []          # new obstacles seen by the lidar: (cx, cy, l, w, yaw)
        self.choice = None           # planner.replanning.SlotChoice being driven
        self.m4 = None               # M4 controller for the current route
        self.progress = 0            # index of the route pose the car has reached
        self.phase = "SEARCH"
        self.reason = "Looking for a free slot"
        self.next_check_s = 0.0
        self.excluded = set()        # slots we gave up on
        self.events = []             # (time, text) log for the report / video
        self.plans = []              # every route planned: (time, slot id, path)
        self.plan_pose = (0.0, 0.0, 0.0)  # where the car was when the current route was planned
        self.stuck_count = 0         # emergency stops in a row without the car moving

    # ------------------------------------------------------------ sensing (M1)

    def _update_slots(self, ranges, pose):
        status = classify_slots_from_scan(ranges, pose, self.slots, self.config.lidar_fov,
                                          self.config.lidar_max_range)
        for sid, s in status.items():
            if s != "unknown":            # keep what we saw earlier for hidden slots
                self.slot_status[sid] = s

    def occupied(self):
        """Slots to treat as obstacles: everything not seen to be free (unknown = maybe a car)."""
        return {sid for sid in self.slots if self.slot_status.get(sid) != "free"}

    def free_slots(self):
        return [sid for sid, s in self.slot_status.items() if s == "free" and sid not in self.excluded]

    def _update_obstacles(self, ranges, pose):
        """
        Lidar points that are not explained by the walls or a parked car are
        new obstacles (a pedestrian, a car pulling out). Each becomes a small
        box on the planner's map.
        """
        static = build_map(self.occupied())[0]
        tol = self.config.static_tolerance_m
        boxes = []
        points = lidar_points_world(ranges, pose, self.config.lidar_fov, self.config.lidar_max_range)
        for px, py in points:
            if math.hypot(px - pose[0], py - pose[1]) > self.config.obstacle_radius_m:
                continue
            explained = any(static.is_occupied_world(px + dx, py + dy)
                            for dx in (-tol, 0, tol) for dy in (-tol, 0, tol))
            if not explained:
                boxes.append((px, py, 0.3, 0.3, 0.0))
        self.obstacles = boxes

    # ------------------------------------------------------------ planning (M2)

    def _log(self, now_s, text):
        self.events.append((round(now_s, 2), text))

    def _plan(self, now_s, pose, same_slot=None):
        """Plan from `pose`, to `same_slot` if given and still reachable, else to the best free slot."""
        grid_map = build_map(self.occupied(), self.obstacles)[0]
        choice = None
        for vehicle in (self.vehicle, self.tight_vehicle):   # wide margin first
            checker = CollisionChecker(grid_map, vehicle)
            if same_slot and same_slot not in self.occupied():
                path, stats, goal = plan_to_slot(grid_map, pose, self.slots[same_slot],
                                                 self.config.method, vehicle, self.planner_config,
                                                 checker, setup_distance_m=self.setup_distance_m)
                if path:
                    choice = SlotChoice(same_slot, path, stats, goal, [same_slot])
            if choice is None:
                choice = choose_slot(grid_map, pose, self.slots, self.free_slots(),
                                     self.config.method, vehicle, self.planner_config, checker,
                                     setup_distance_m=self.setup_distance_m)
            if choice is not None:
                self.route_vehicle = vehicle
                break
        if choice is None:
            return False

        self.choice = choice
        self.plans.append((now_s, choice.slot_id, choice.path))
        self.plan_pose = pose
        self.progress = 0
        lateral = {"pure_pursuit": PurePursuit, "stanley": Stanley,
                   "stanley_ff": lambda L: Stanley(L=L, use_ff=True)}[self.config.lateral]
        route = path_from_planner(choice.path, choice.stats["directions"])
        self.m4 = M4Controller(route, lateral(L=self.geometry.wheelbase_m),
                               self.geometry.max_steer_rad, self.follower_cfg)

        s = self.slots[choice.slot_id]
        self.fsm.reset()
        self.fsm.select_slot(ParkingSlot(s.slot_id, s.x, s.y, s.yaw, BAY_WIDTH, BAY_DEPTH),
                             Pose(*choice.goal))
        self.phase = "DRIVE"
        self._log(now_s, f"route to {choice.slot_id}: {len(choice.path)} poses, "
                         f"{choice.stats['time_s']:.2f} s planning")
        return True

    # ------------------------------------------------------------ main step

    def _stop_actuation(self, speed):
        """Brake to a standstill when there is no M4 controller yet."""
        a = -self.follower_cfg.a_brk if abs(speed) > 0.05 else 0.0
        return Actuation(0.0, a, 1, 0.0, abs(speed) < 0.1, False, "stop")

    def step(self, now_s, pose, speed, lidar_ranges):
        """
        One control step.

        now_s        : simulation time (s), the time the sensors were read
        pose         : (x, y, yaw) of the rear-axle centre
        speed        : signed speed along the heading (m/s)
        lidar_ranges : the lidar range image (Webots format, inf = no return)
        Returns an m4_control Actuation (steer, accel, direction, ...).
        """
        ranges = lidar_ranges
        stopped = abs(speed) < 0.05

        # --- deliberative layer: (re)plan when needed, at most once a period
        if self.phase in ("SEARCH", "DRIVE", "STOPPING") and now_s >= self.next_check_s:
            self.next_check_s = now_s + self.config.replan_period_s
            if self.config.slots_from_lidar:
                self._update_slots(ranges, pose)
            self._update_obstacles(ranges, pose)

            if self.phase == "DRIVE":
                self.progress = nearest_index(self.choice.path, pose, self.progress)
                checker = CollisionChecker(build_map(self.occupied(), self.obstacles)[0],
                                           self.route_vehicle)
                reason = check_route(self.choice, self.progress, self.occupied(), checker)
                if reason:
                    self.phase, self.reason = "STOPPING", reason
                    self._log(now_s, f"replan: {reason}")

            if self.phase in ("SEARCH", "STOPPING") and stopped:
                previous = self.choice.slot_id if self.choice else None
                if not self._plan(now_s, pose, same_slot=previous):
                    # Planning failed (e.g. a pedestrian blocks the way). A failed
                    # search is slow, so wait a few seconds before trying again.
                    self.reason = "no reachable free slot yet; waiting"
                    self.next_check_s = now_s + self.config.retry_period_s

        if self.phase == "STOPPING" and not stopped:
            self.next_check_s = now_s     # plan as soon as the car stands still

        if self.phase == "SEARCH" or self.m4 is None:
            return self._stop_actuation(speed)

        # --- procedural + reflexive layers (M3), then control (M4)
        clear = body_clearances(
            ranges, self.config.lidar_fov,
            front_extent=self.geometry.length_m - self.geometry.rear_overhang_m,
            rear_extent=self.geometry.rear_overhang_m, half_width=self.geometry.width_m / 2,
            max_range=self.config.lidar_max_range)
        obs = ParkingObservation(now_s, Pose(*pose), speed, clear["front"], clear["rear"],
                                 clear["left"], clear["right"], self.m4.route_complete)
        before = self.fsm.state
        cmd = self.fsm.update(obs, now_s)

        if self.phase == "STOPPING":
            # Our own stop request: brake, whatever M3 says.
            cmd = ParkingCommand(ParkingState.DRIVE, ControlMode.STOP, 0.0, reason=self.reason)
        elif cmd.state == ParkingState.EMERGENCY_STOP and before == ParkingState.DRIVE:
            # M3's reflex stopped the car on the route: once stopped, plan round the obstacle.
            moved = math.hypot(pose[0] - self.plan_pose[0], pose[1] - self.plan_pose[1])
            self.stuck_count = self.stuck_count + 1 if moved < 0.1 else 0
            if self.stuck_count >= 3:
                # Every new route is stopped again before the car moves: it is
                # boxed in (M3 allows no motion with an obstacle this close).
                self.phase = "FAILED"
                self.reason = "boxed in: " + self.fsm.reason
                self._log(now_s, self.reason)
            else:
                self.phase, self.reason = "STOPPING", "emergency stop on the route: " + self.fsm.reason
                self._log(now_s, self.reason)
                self.next_check_s = now_s       # re-plan as soon as the car stands still
        elif cmd.state in (ParkingState.ALIGN, ParkingState.PARK):
            self.phase = cmd.state.name
        elif cmd.state in (ParkingState.DONE, ParkingState.FAILED, ParkingState.EMERGENCY_STOP):
            if self.phase != cmd.state.name:
                self._log(now_s, f"{cmd.state.name}: {self.fsm.reason}")
            self.phase, self.reason = cmd.state.name, self.fsm.reason

        return self.m4.step(cmd, obs.pose, speed, self._dt(now_s))

    def _dt(self, now_s):
        last = getattr(self, "_last_now", None)
        self._last_now = now_s
        return 0.032 if last is None or now_s <= last else now_s - last
