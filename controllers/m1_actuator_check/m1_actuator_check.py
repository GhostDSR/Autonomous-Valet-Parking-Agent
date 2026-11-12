"""M1 actuator bring-up check (NOT part of the valet pipeline).

Purpose: prove that the steering joints and wheel motors added to
worlds/parking_lot.wbt work, and that their signs match the convention M4
should use. It is the only controller besides M4's that sends motor commands,
so use it on its own: set the valet_car ``controller`` field to
"m1_actuator_check", run the world once, read the PASS/FAIL lines in the
console, then set it back to "valet_controller".

Conventions checked
  * wheel motor velocity > 0  -> car moves forward (+x in the body frame)
  * steering angle        > 0 -> car turns left (heading increases, CCW)
  * steering limits are +/-0.65 rad (inner wheel at 0.5 rad bicycle angle needs 0.57)

Sequence (about 20 s of simulated time)
  settle -> forward 3 s -> stop -> steer left, forward 4 s -> stop
  -> straighten, reverse 3 s -> stop -> report
"""
import math
import os
import sys

REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
if REPO_ROOT not in sys.path:
    sys.path.insert(0, REPO_ROOT)

from controllers.valet_controller.m1_sensors import compass_yaw  # noqa: E402

WHEELBASE = 2.8        # m, rear axle to front axle
TRACK = 1.6            # m, distance between left and right wheel centres
WHEEL_RADIUS = 0.3     # m
STEER_LIMIT = 0.65     # rad, matches the joint limits in the world file


def ackermann_angles(delta, wheelbase=WHEELBASE, track=TRACK):
    """(left, right) front steering angles for a bicycle angle delta (+ = left)."""
    if abs(delta) < 1e-9:
        return 0.0, 0.0
    radius = wheelbase / math.tan(delta)        # signed radius of the rear-axle centre
    return (math.atan(wheelbase / (radius - track / 2.0)),
            math.atan(wheelbase / (radius + track / 2.0)))


def wheel_speeds(v, delta, wheelbase=WHEELBASE, track=TRACK, radius=WHEEL_RADIUS):
    """Angular speeds (rad/s) of rear-left, rear-right, front-left, front-right wheels.

    ``v`` is the signed speed of the rear-axle centre. Each wheel's rim speed
    is v scaled by its distance from the turn centre, so the inner wheels turn
    slower and the wheels do not fight each other in a turn.
    """
    if abs(delta) < 1e-9:
        w = v / radius
        return w, w, w, w
    rho = wheelbase / math.tan(delta)           # signed turn radius of the rear-axle centre
    # Distances from the turn centre (left wheels are on the +y side). The
    # steering limit keeps |rho| well above track / 2, so none of these flip sign.
    rear_left, rear_right = abs(rho - track / 2.0), abs(rho + track / 2.0)
    front_left = math.hypot(rho - track / 2.0, wheelbase)
    front_right = math.hypot(rho + track / 2.0, wheelbase)
    scale = v / abs(rho) / radius               # v keeps its sign: negative = reverse
    return (scale * rear_left, scale * rear_right,
            scale * front_left, scale * front_right)


def _wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def main():
    from controller import Robot  # only available inside Webots

    robot = Robot()
    dt_ms = int(robot.getBasicTimeStep())
    dt = dt_ms / 1000.0

    gps = robot.getDevice("gps")
    compass = robot.getDevice("compass")
    gps.enable(dt_ms)
    compass.enable(dt_ms)

    wheels = {name: robot.getDevice(name + "_wheel_motor")
              for name in ("rear_left", "rear_right", "front_left", "front_right")}
    steers = {name: robot.getDevice(name + "_steer") for name in ("front_left", "front_right")}
    steer_sensors = {}
    for name, motor in steers.items():
        sensor = robot.getDevice(name + "_steer_sensor")
        sensor.enable(dt_ms)
        steer_sensors[name] = sensor
    for motor in wheels.values():
        motor.setPosition(float("inf"))     # velocity control
        motor.setVelocity(0.0)
    for motor in steers.values():
        motor.setPosition(0.0)

    def drive(v, delta):
        left, right = ackermann_angles(delta)
        steers["front_left"].setPosition(max(-STEER_LIMIT, min(STEER_LIMIT, left)))
        steers["front_right"].setPosition(max(-STEER_LIMIT, min(STEER_LIMIT, right)))
        rl, rr, fl, fr = wheel_speeds(v, delta)
        wheels["rear_left"].setVelocity(rl)
        wheels["rear_right"].setVelocity(rr)
        wheels["front_left"].setVelocity(fl)
        wheels["front_right"].setVelocity(fr)

    def pose():
        g = gps.getValues()
        return g[0], g[1], compass_yaw(compass.getValues())

    def run_phase(seconds, v, delta):
        drive(v, delta)
        for _ in range(max(1, int(seconds / dt))):
            if robot.step(dt_ms) == -1:
                return False
        return True

    results = []

    def check(name, ok, detail):
        results.append(ok)
        print("M1_ACTUATOR_CHECK %s %s (%s)" % ("PASS" if ok else "FAIL", name, detail), flush=True)

    if not run_phase(1.0, 0.0, 0.0):
        return

    # 1. forward, straight
    x0, y0, yaw0 = pose()
    if not run_phase(3.0, 1.0, 0.0):
        return
    x1, y1, yaw1 = pose()
    along = (x1 - x0) * math.cos(yaw0) + (y1 - y0) * math.sin(yaw0)
    check("forward_moves_forward", along > 1.5, "moved %.2f m along heading, expected about 3" % along)
    check("straight_keeps_heading", abs(_wrap(yaw1 - yaw0)) < math.radians(3),
          "heading changed %.2f deg" % math.degrees(_wrap(yaw1 - yaw0)))
    run_phase(1.5, 0.0, 0.0)

    # 2. steer left while moving forward
    x0, y0, yaw0 = pose()
    delta_cmd = 0.3
    if not run_phase(4.0, 1.0, delta_cmd):
        return
    x1, y1, yaw1 = pose()
    dyaw = _wrap(yaw1 - yaw0)
    expected = 4.0 * 1.0 * math.tan(delta_cmd) / WHEELBASE
    check("positive_steer_turns_left", dyaw > math.radians(5),
          "heading changed %+.1f deg, bicycle model predicts %+.1f deg"
          % (math.degrees(dyaw), math.degrees(expected)))
    sl = steer_sensors["front_left"].getValue()
    sr = steer_sensors["front_right"].getValue()
    el, er = ackermann_angles(delta_cmd)
    check("steer_sensors_follow_command", abs(sl - el) < 0.05 and abs(sr - er) < 0.05,
          "left %.3f (cmd %.3f), right %.3f (cmd %.3f) rad" % (sl, el, sr, er))
    run_phase(1.5, 0.0, 0.0)

    # 3. reverse, straight
    x0, y0, yaw0 = pose()
    if not run_phase(3.0, -1.0, 0.0):
        return
    x1, y1, yaw1 = pose()
    along = (x1 - x0) * math.cos(yaw0) + (y1 - y0) * math.sin(yaw0)
    check("reverse_moves_backward", along < -1.5, "moved %.2f m along heading, expected about -3" % along)
    run_phase(1.0, 0.0, 0.0)

    print("M1_ACTUATOR_CHECK SUMMARY %d/%d passed" % (sum(results), len(results)), flush=True)
    while robot.step(dt_ms) != -1:
        pass


if __name__ == "__main__":
    main()
