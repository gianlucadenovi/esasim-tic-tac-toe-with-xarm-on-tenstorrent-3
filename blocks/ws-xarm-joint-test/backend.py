# ── EsaSim block ──────────────────────────────────────────────────────
# xArm joint test (ws-xarm-joint-test) · version 2 · machine block, Python: def run(ctx)
# Checks that the xArm is online and healthy, then moves it one joint at a time, slowly, from the Controls view.
# Workspace: Gianluca De Novi's workspace · Author: Gianluca De Novi
# Saved: 2026-11-07
#
# Inputs   ctx.inputs["id"] is a list of files: .path .name .size .read_text() .read_json()
#   (none)
# Outputs  ctx.output("id", "name.ext") is the path to write; ctx.write_json / ctx.write_text
#   log         Table (csv)               Test log · joint_test.csv: every move asked, refused and reached, with the joint angles
#   telemetry   Live telemetry            ctx.telemetry("channel", data) while it runs
# Settings ctx.params["key"]
#   robot       select    Robot = "real xArm" of real xArm | simulation
#   ip          text      Arm IP address = "192.168.1.200"
#   step        number    Step (degrees per press) = 5
#   joint_speed number    Joint speed (degrees/s) = 10
#   joint_accel number    Joint acceleration (degrees/s²) = 40
#   table_z     number    Table height (mm) = 0
#   clearance   number    Clearance above the table (mm) = 40
#   tool_length number    Tool length below the flange (mm) = 0
#   minutes     number    Longest test (minutes) = 30
#   channel     text      Controls channel = "control"
#   sim_start   text      Start pose in simulation (degrees) = "0,-20,-30,0,50,0"
# Runs on  Any machine. Return a dict to fill the "result" output.
# Packages numpy, xarm-python-sdk (installed by the worker with pip when missing)
# EsaSim rewrites these lines from the block's definition each time it is saved: change the definition, not them.
# ── end of EsaSim block header ────────────────────────────────────────

"""xArm joint test: connects to the arm, reports whether it is healthy (connection, firmware, state, errors, joint
angles, temperatures), then moves one joint at a time by a small step from the Controls view (J1 − / J1 + … J6 +),
slowly, each step checked before it is sent: inside the joint's limits (with a margin), and the elbow, wrist, flange
and tool kept above the table. Back to start returns every joint to where it was when the test began. Stop stops
the motion at once. With Robot = simulation nothing is sent anywhere: the arm's digital twin moves instead."""
import csv
import math
import signal
import time

XARM6_JOINTS = [((0.0, 0.0, 267.0), (0.0, 0.0, 0.0)), ((0.0, 0.0, 0.0), (-1.5708, 0.0, 0.0)), ((53.5, -284.5, 0.0), (0.0, 0.0, 0.0)),
                ((77.5, 342.5, 0.0), (-1.5708, 0.0, 0.0)), ((0.0, 0.0, 0.0), (1.5708, 0.0, 0.0)), ((76.0, 97.0, 0.0), (-1.5708, 0.0, 0.0))]
XARM6_LIMITS = [(-178.2, 178.2), (-118.0, 120.0), (-178.2, 11.0), (-178.2, 178.2), (-97.0, 178.2), (-178.2, 178.2)]
MAX_JOINT_SPEED = 30.0   # degrees/s: a test goes slowly, whatever the settings say
MAX_JOINT_ACC = 120.0
MAX_STEP = 15.0          # degrees per press, at most
MARGIN = 5.0             # degrees kept from each joint's end stop


class Stop(Exception):
    pass


def _rpy(r, p, y):
    import numpy as np
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def fk(q_deg):
    """Frames (4x4, mm) of the base and links 1-6 for joint angles in degrees."""
    import numpy as np
    T = np.eye(4)
    links = [T.copy()]
    for (xyz, rpy), qi in zip(XARM6_JOINTS, q_deg):
        O = np.eye(4)
        O[:3, :3] = _rpy(*rpy)
        O[:3, 3] = xyz
        T = T @ O
        c, s = math.cos(math.radians(qi)), math.sin(math.radians(qi))
        T = T @ np.array([[c, -s, 0, 0], [s, c, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1.0]])
        links.append(T.copy())
    return links


def trapezoid(dist, vmax, amax):
    """Duration and position s(t) of a move of `dist` from rest to rest within vmax and amax."""
    dist, vmax, amax = max(0.0, float(dist)), max(1e-6, float(vmax)), max(1e-6, float(amax))
    ta = vmax / amax
    if amax * ta * ta >= dist:
        ta = math.sqrt(dist / amax) if dist > 0 else 0.0
        vp, tc = amax * ta, 0.0
    else:
        vp, tc = vmax, (dist - amax * ta * ta) / vmax
    T = 2 * ta + tc

    def s(t):
        t = min(max(t, 0.0), T)
        if t < ta:
            return 0.5 * amax * t * t
        if t < ta + tc:
            return 0.5 * amax * ta * ta + vp * (t - ta)
        return dist - 0.5 * amax * (T - t) ** 2
    return T, s


def reachable(ip):
    """Why this machine cannot talk to the arm at ip (None when its control port answers): which of its addresses
    it would use, and whether the arm's port 502 answers, refuses or stays silent."""
    import socket
    try:
        u = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        u.connect((ip, 502))
        local = u.getsockname()[0]
        u.close()
    except OSError as e:
        return "This machine has no route to %s (%s): give its Ethernet port a fixed address on the arm's network." % (ip, e)
    net = ip.rsplit(".", 1)[0]
    if local.rsplit(".", 1)[0] != net:
        return ("This machine would talk to the arm at %s from its address %s, which is not on the arm's network: the Ethernet port has no address there yet. "
                "In Windows: Settings > Network & internet > Ethernet > IP assignment > Edit > Manual, IPv4 on, IP address %s.100, subnet mask 255.255.255.0, "
                "gateway and DNS empty; then ping %s." % (ip, local, net, ip))
    t = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    t.settimeout(3)
    try:
        t.connect((ip, 502))
        return None
    except socket.timeout:
        return ("This machine (address %s) is on the arm's network, but nothing answers at %s: check that the control box is on and its network port's lights are lit, "
                "that %s is the address on its label, and ping %s. If %s is your Wi-Fi's address, your home network uses the same addresses as the arm: "
                "turn Wi-Fi off for the test, or move the arm to another network (192.168.2.x) in UFACTORY Studio." % (local, ip, ip, ip, local))
    except ConnectionRefusedError:
        return "Something at %s answers but refuses the arm's port: it may be another device with that address, or the controller is still starting. Wait a minute and run again." % ip
    except OSError as e:
        return "This machine (address %s) cannot open a connection to %s (%s)." % (local, ip, e)
    finally:
        t.close()


def run(ctx):
    p = ctx.params
    sim = str(p.get("robot", "real xArm")).lower().startswith("sim")
    speed = max(1.0, min(MAX_JOINT_SPEED, float(p.get("joint_speed", 10))))
    accel = max(5.0, min(MAX_JOINT_ACC, float(p.get("joint_accel", 40))))
    step = max(0.5, min(MAX_STEP, float(p.get("step", 5))))
    table = float(p.get("table_z", 0))
    clearance = float(p.get("clearance", 40))
    tool_len = float(p.get("tool_length", 0))
    channel = str(p.get("channel") or "control")
    deadline = time.time() + float(p.get("minutes", 30)) * 60
    log = csv.writer(open(ctx.output("log", "joint_test.csv"), "w", newline=""))
    log.writerow(["t", "event", "j1", "j2", "j3", "j4", "j5", "j6", "note"])

    def interrupted(*_):
        raise KeyboardInterrupt()
    try:
        signal.signal(signal.SIGTERM, interrupted)
    except (ValueError, OSError):
        pass

    def say(text):
        ctx.log(text)
        ctx.telemetry("status", {"text": text})

    def lowest(q):
        """(how far the lowest of elbow, wrist, flange is above the table less the clearance, the same for the tool's
        tip): negative means too low."""
        L = fk(q)
        parts = min(L[3][2, 3], L[5][2, 3], L[6][2, 3]) - (table + clearance)
        tip = (L[6] @ [0, 0, tool_len, 1.0])[2] - (table + 5.0) if tool_len > 0 else 1e9
        return min(parts, tip)

    def why_not(q_from, q_to):
        """Why the move from q_from to q_to is not allowed (None when it is), checked every half degree."""
        for i, (a, (lo, hi)) in enumerate(zip(q_to, XARM6_LIMITS)):
            if not (lo + MARGIN <= a <= hi - MARGIN) and abs(a - q_from[i]) > 1e-6:
                inward = abs(a - (lo + hi) / 2) < abs(q_from[i] - (lo + hi) / 2)
                if not inward:
                    return "J%d would reach %.1f°: its limits are %.0f° to %.0f° (and %.0f° are kept from each end)" % (i + 1, a, lo, hi, MARGIN)
        turn = max(abs(b - a) for a, b in zip(q_from, q_to))
        n = max(1, int(math.ceil(turn / 0.5)))
        start = lowest(q_from)
        for k in range(1, n + 1):
            q = [a + (b - a) * k / n for a, b in zip(q_from, q_to)]
            h = lowest(q)
            if h < 0 and h < start - 0.01:  # too low, and lower than where it started: refused
                return "the arm would come within %.0f mm of the table (elbow, wrist, flange%s)" % (clearance, " or tool" if tool_len > 0 else "")
        return None

    # -- the arm --------------------------------------------------------------------------------------------------
    api, prev_reduced = None, None
    q = [float(v) for v in str(p.get("sim_start") or "0,-20,-30,0,50,0").split(",")][:6] if sim else None

    def angles():
        if sim:
            return list(q)
        code, a = api.get_servo_angle()
        if code != 0 or not a:
            raise Stop("The arm does not answer with its joint angles (code %s)." % code)
        return [float(v) for v in a[:6]]

    def report(cur):
        ctx.telemetry("joints", {"j%d" % (i + 1): round(v, 2) for i, v in enumerate(cur)})
        ctx.telemetry("jog", {"j%d" % (i + 1): "J%d   %+.1f°      (%.0f° to %.0f°)" % (i + 1, v, lo, hi) for i, (v, (lo, hi)) in enumerate(zip(cur, XARM6_LIMITS))})

    def health(extra=None):
        h = {"robot": "simulation" if sim else str(p.get("ip"))}
        if not sim:
            h.update({"connected": bool(api.connected), "state": api.state, "mode": getattr(api, "mode", None), "error": api.error_code, "warning": api.warn_code})
            try:
                temps = getattr(api, "temperatures", None)
                if temps:
                    h["hottest_joint_C"] = max(float(t) for t in temps[:6])
            except Exception:
                pass
        h.update(extra or {})
        ctx.telemetry("health", h)
        return h

    if sim:
        say("Simulation: no arm is moved. The digital twin starts at %s." % ", ".join("%.0f°" % v for v in q))
    else:
        from xarm.wrapper import XArmAPI
        ip = str(p.get("ip") or "").strip()
        say("Connecting to the xArm at %s…" % ip)
        why = reachable(ip)
        if why:
            raise RuntimeError(why)
        try:
            api = XArmAPI(ip, is_radian=False)
        except Exception as e:
            raise RuntimeError("The arm at %s did not accept the connection (%s). Is UFACTORY Studio or another program connected to it? Close it and run again." % (ip, e))
        if not api.connected:
            raise RuntimeError("Cannot reach the xArm at %s: check that the controller is on, the address, and that this machine is on the arm's network (ping %s)." % (ip, ip))
        info = {"firmware": str(getattr(api, "version", "") or ""), "serial": str(getattr(api, "sn", "") or ""), "axes": getattr(api, "axis", None)}
        ctx.log("Connected: %s" % ", ".join("%s %s" % (k, v) for k, v in info.items() if v not in ("", None)))
        h = health(info)
        if api.error_code:
            say("The arm reports error %s: press Clear error once the cause is fixed (see UFACTORY's manual for the code)." % api.error_code)
        else:
            if api.warn_code:
                ctx.log("Warning %s from the arm, cleared." % api.warn_code)
                api.clean_warn()
            api.motion_enable(enable=True)
            api.set_mode(0)
            api.set_state(state=0)
            time.sleep(0.5)
        try:  # the controller limits the joints' speed by itself too, for the whole test
            code, st = api.get_reduced_states()
            if code == 0 and st:
                prev_reduced = st
            api.set_reduced_max_joint_speed(speed)
            api.set_joint_maxacc(accel)
            api.set_self_collision_detection(True)
            api.set_reduced_mode(True)
            ctx.log("Controller limits: joints at most %.0f°/s and %.0f°/s², self-collision detection on." % (speed, accel))
        except Exception as e:
            ctx.log("The controller did not take its limits (%s): the test still checks every step and moves slowly." % e)

    start = angles()
    report(start)
    ctx.telemetry("scene", {"units": "mm", "up": "z", "floor": table - 740, "objects": [
        {"id": "table", "kind": "box", "center": [300, 0, table - 15], "size": [1200, 900, 30], "color": "#b79b7a", "fit": False}]
        + ([{"id": "tool", "kind": "tool", "length": tool_len, "radius": 6}] if tool_len > 0 else [])})
    log.writerow([round(time.time(), 2), "start"] + [round(v, 2) for v in start] + [""])
    low = lowest(start)
    if low < 0:
        ctx.log("Note: the arm starts %.0f mm lower than the clearance asked: only moves that raise it are allowed." % -low)
    say("Ready. Step %.0f°, at most %.0f°/s. Press J1 − … J6 + to move one joint." % (step, speed) if sim or not api.error_code else "Waiting: clear the arm's error first.")

    def drain():
        """Presses that came while the arm moved are dropped (a press is one step, never a queue of them)."""
        n = 0
        for m in ctx.messages(channel):
            c = str((m.get("data") or {}).get("control", "")).strip().lower() if isinstance(m.get("data"), dict) else ""
            if c in ("stop", "end test"):
                return c
            n += 1
        return n

    def move(target, why):
        cur = angles()
        err = why_not(cur, target)
        if err:
            say("Not moved: " + err + ".")
            log.writerow([round(time.time(), 2), "refused"] + [round(v, 2) for v in target] + [err])
            return
        turn = max(abs(b - a) for a, b in zip(cur, target))
        if turn < 0.05:
            return
        say(why)
        log.writerow([round(time.time(), 2), "move"] + [round(v, 2) for v in target] + [why])
        T, s_of = trapezoid(turn, speed, accel)
        if sim:
            t0 = time.time()
            while True:
                t = time.time() - t0
                f = s_of(t) / turn
                q[:] = [a + (b - a) * f for a, b in zip(cur, target)]
                report(q)
                if drain() == "stop":
                    say("Stopped.")
                    return
                if t >= T:
                    break
                time.sleep(0.05)
        else:
            code = api.set_servo_angle(angle=[float(v) for v in target] + ([0.0] * (api.axis - 6) if api.axis > 6 else []), speed=speed, mvacc=accel,
                                       is_radian=False, wait=False)
            if code != 0:
                say("The arm refused the move (code %s, error %s, warning %s)." % (code, api.error_code, api.warn_code))
                return
            t0 = time.time()
            time.sleep(0.15)
            while time.time() - t0 < T + 10:
                report(angles())
                if drain() == "stop":
                    api.set_state(4)
                    time.sleep(0.3)
                    api.set_state(0)
                    say("Stopped.")
                    return
                try:
                    moving = api.get_is_moving()
                except Exception:
                    moving = time.time() - t0 < T + 0.3
                if not moving:
                    break
                time.sleep(0.1)
            if api.error_code:
                say("The arm stopped with error %s. Press Clear error once the cause is fixed." % api.error_code)
                health()
                return
        now = angles()
        report(now)
        off = max(abs(a - b) for a, b in zip(now, target))
        log.writerow([round(time.time(), 2), "reached"] + [round(v, 2) for v in now] + ["%.2f° from the target" % off])
        if not sim and off > 1.0:
            say("Reached, but %.1f° away from the target: check that joint." % off)
        else:
            say("Done: " + ", ".join("J%d %.1f°" % (i + 1, v) for i, v in enumerate(now)))
        health()

    stopped = False
    try:
        while time.time() < deadline:
            for m in ctx.messages(channel, wait=0.5):
                d = m.get("data") if isinstance(m.get("data"), dict) else {"control": str(m.get("data")), "value": True}
                c, v = str(d.get("control", "")).strip(), d.get("value")
                cl = c.lower()
                if cl == "end test":
                    raise Stop("Test ended.")
                if cl == "step":
                    try:
                        step = max(0.5, min(MAX_STEP, float(v)))
                        say("Step %.0f° per press." % step)
                    except (TypeError, ValueError):
                        pass
                    continue
                if cl == "clear error" and not sim:
                    api.clean_error(); api.clean_warn(); api.motion_enable(enable=True); api.set_mode(0); api.set_state(state=0)
                    time.sleep(0.5)
                    h = health()
                    say("Error cleared: ready." if not api.error_code else "The arm still reports error %s." % api.error_code)
                    continue
                if cl == "stop":
                    if not sim:
                        api.set_state(4); time.sleep(0.3); api.set_state(0)
                    say("Stopped.")
                    continue
                if not sim and api.error_code:
                    say("The arm reports error %s: press Clear error first." % api.error_code)
                    continue
                if cl == "back to start":
                    move(start, "Back to the start pose…")
                    continue
                if len(c) >= 3 and cl[0] == "j" and cl[1] in "123456":
                    i = int(cl[1]) - 1
                    sign = -1 if any(ch in c for ch in ("-", "−", "–")) else 1
                    target = angles()
                    target[i] += sign * step
                    move(target, "J%d %s%.0f°…" % (i + 1, "+" if sign > 0 else "−", step))
            if not sim and int(time.time()) % 5 == 0:
                health()
    except Stop as e:
        say(str(e))
    except KeyboardInterrupt:
        stopped = True
        if api is not None:
            try:
                api.set_state(4)
            except Exception:
                pass
        raise
    finally:
        if api is not None:
            try:  # the controller as it was before the test
                if prev_reduced is not None:
                    api.set_reduced_mode(bool(prev_reduced[0]))
                if not stopped:
                    api.set_state(0)
            except Exception:
                pass
            api.disconnect()
        if not stopped:
            say("The test is over: the arm stays where it is.")
    return {"robot": "simulation" if sim else str(p.get("ip")), "start": [round(v, 2) for v in start]}
