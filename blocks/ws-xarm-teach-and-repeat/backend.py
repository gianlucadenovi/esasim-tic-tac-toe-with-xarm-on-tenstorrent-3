# ── EsaSim block ──────────────────────────────────────────────────────
# xArm teach and repeat (ws-xarm-teach-and-repeat) · version 1 · machine block, Python: def run(ctx)
# Record a movement by moving the xArm by hand, then have it repeat the movement any number of times, slower or faster, and go back to where it started.
# Workspace: Gianluca De Novi's workspace · Author: Gianluca De Novi
# Saved: 2026-11-09
#
# Inputs   ctx.inputs["id"] is a list of files: .path .name .size .read_text() .read_json()
#   robot       Files (gz, txt)           Robot model · optional · From xArm model: connected, the model is made before this block starts, so the 3D view shows the arm
#   recording   Table (csv)               Recording · optional · A recording.csv from an earlier run, to play again
# Outputs  ctx.output("id", "name.ext") is the path to write; ctx.write_json / ctx.write_text
#   recording   Table (csv)               Recording · recording.csv: t (s) and the joint angles (degrees), smoothed and trimmed
#   log         Table (csv)               Session log · session.csv: recordings, repeats, speed changes, stops
#   telemetry   Live telemetry            ctx.telemetry("channel", data) while it runs
# Settings ctx.params["key"]
#   robot       select    Robot = "real xArm" of real xArm | simulation
#   arm         select    Arm = "xArm 6" of xArm 6 | xArm 7
#   ip          text      Arm IP address = "192.168.1.200"
#   repeats     number    Repeats = 3
#   speed       number    Speed (%) = 100
#   max_speed   number    Max joint speed (degrees/s) = 60
#   move_speed  number    Move speed (degrees/s) = 20
#   rate        number    Recording rate (per second) = 50
#   max_seconds number    Longest recording (s) = 120
#   minutes     number    Longest session (minutes) = 60
#   channel     text      Controls channel = "control"
# Runs on  Any machine. Return a dict to fill the "result" output.
# Packages xarm-python-sdk (installed by the worker with pip when missing)
# EsaSim rewrites these lines from the block's definition each time it is saved: change the definition, not them.
# ── end of EsaSim block header ────────────────────────────────────────

"""xArm teach and repeat: Record puts the arm in manual mode (it holds its own weight and you move it by hand) and
records its joints; Stop ends the recording; Repeat plays the movement back as many times as asked, then the arm goes
back to where the movement started. The Speed slider plays it slower or faster, also while it plays.

Playback streams the recorded joint angles to the controller (servo mode, about 100 per second), so the arm follows
the same path with the same rhythm, scaled by the speed. No joint is ever asked to go faster than Max joint speed:
where the recording is faster, the playback slows down there. Going to the start pose and back is a plain joint move
at Move speed. Robot = simulation sends nothing: a pretend arm is moved by a pretend hand."""
import csv
import math
import signal
import time

LIMITS = {
    6: [(-360.0, 360.0), (-118.0, 120.0), (-225.0, 11.0), (-360.0, 360.0), (-97.0, 180.0), (-360.0, 360.0)],
    7: [(-360.0, 360.0), (-118.0, 120.0), (-360.0, 360.0), (-11.0, 225.0), (-360.0, 360.0), (-97.0, 180.0), (-360.0, 360.0)],
}
STREAM_HZ = 100.0


class Stop(Exception):
    pass


class SimArm:
    """A pretend xArm: in manual mode (2) a pretend hand moves it; set_servo_angle (mode 0) moves it at the speed
    asked; set_servo_angle_j (mode 1) puts it where it is told. Nothing is sent anywhere."""

    def __init__(self, n):
        self.axis, self.connected, self.version, self.sn = n, True, "simulation", "-"
        self.mode, self.state, self.error_code, self.warn_code = 0, 0, 0, 0
        self.q = [0.0, -20.0, -30.0, 0.0, 50.0, 0.0, 0.0][:n]
        self.target, self.v, self.t_last, self.t_manual, self.base = None, 20.0, time.time(), None, None

    def _tick(self):
        now = time.time()
        dt, self.t_last = now - self.t_last, now
        if self.mode == 2 and self.t_manual is not None:
            t = now - self.t_manual
            amp = [25.0, 12.0, 15.0, 20.0, 18.0, 30.0, 10.0]
            self.q = [b + a * math.sin(0.9 * t + i) * min(1.0, t) for i, (b, a) in enumerate(zip(self.base, amp))]
        elif self.target is not None:
            d = [b - a for a, b in zip(self.q, self.target)]
            far = max(abs(x) for x in d)
            step = self.v * dt
            if far <= step:
                self.q, self.target = list(self.target), None
            else:
                self.q = [a + x * step / far for a, x in zip(self.q, d)]

    def connect(self):
        self.connected = True

    def disconnect(self):
        self.connected = False

    def get_servo_angle(self, is_radian=False):
        self._tick()
        return 0, list(self.q)

    def set_mode(self, m):
        self._tick()
        self.mode = m
        if m == 2:
            self.t_manual, self.base = time.time(), list(self.q)
        return 0

    def set_state(self, state=0):
        self.state = state
        if state == 4:
            self.target = None
        return 0

    def motion_enable(self, enable=True):
        return 0

    def set_servo_angle(self, angle=None, speed=None, mvacc=None, is_radian=False, wait=False, **kw):
        self._tick()
        self.target, self.v = list(angle)[:self.axis], float(speed or 20.0)
        return 0

    def set_servo_angle_j(self, angles, is_radian=False, **kw):
        self.q = list(angles)[:self.axis]
        return 0

    def get_is_moving(self):
        self._tick()
        return self.target is not None

    def clean_error(self):
        return 0

    def clean_warn(self):
        return 0


def run(ctx):
    p = ctx.params
    sim = str(p.get("robot", "real xArm")).lower().startswith("sim")
    n = 7 if "7" in str(p.get("arm", "xArm 6")) else 6
    lim = LIMITS[n]
    channel = str(p.get("channel") or "control")
    rate = max(10.0, min(100.0, float(p.get("rate", 50))))
    max_speed = max(5.0, min(120.0, float(p.get("max_speed", 60))))
    move_speed = max(5.0, min(45.0, float(p.get("move_speed", 20))))
    max_seconds = max(5.0, min(600.0, float(p.get("max_seconds", 120))))
    deadline = time.time() + float(p.get("minutes", 60)) * 60
    S = {"rec": None, "speed": max(0.25, min(1.5, float(p.get("speed", 100)) / 100.0)),
         "repeats": max(1, min(100, int(float(p.get("repeats", 3))))), "state": "idle", "last_joints": 0.0}
    log = csv.writer(open(ctx.output("log", "session.csv"), "w", newline=""))
    log.writerow(["time", "event", "detail"])

    def interrupted(*_):
        raise KeyboardInterrupt()
    try:
        signal.signal(signal.SIGTERM, interrupted)
    except (ValueError, OSError):
        pass

    def event(name, detail=""):
        log.writerow([time.strftime("%Y-%m-%d %H:%M:%S"), name, detail])

    def status(text, detail=None):
        ctx.telemetry("status", {"text": text, "detail": detail if detail is not None else describe(), "state": S["state"]})
        ctx.log(text)

    def describe():
        r = S["rec"]
        if not r:
            return "Nothing recorded yet. Repeats: %d · speed %d%%." % (S["repeats"], round(S["speed"] * 100))
        return ("Recording: %.1f s, %d points, fastest joint %.0f°/s. Repeats: %d · speed %d%%."
                % (r["t"][-1], len(r["t"]), r["vmax"], S["repeats"], round(S["speed"] * 100)))

    # -- the arm --------------------------------------------------------------------------------------------------
    if sim:
        api = SimArm(n)
        ctx.log("Simulation: nothing is sent to any arm; a pretend hand moves a pretend arm while you record.")
    else:
        from xarm.wrapper import XArmAPI
        ip = str(p.get("ip") or "").strip()
        ctx.telemetry("status", {"text": "Connecting to the xArm at %s…" % ip, "detail": ""})
        try:
            api = XArmAPI(ip, is_radian=False)
        except Exception as e:
            raise RuntimeError("The arm at %s did not accept the connection (%s). Close UFACTORY Studio's control page and run again." % (ip, e))
        if not api.connected:
            raise RuntimeError("Cannot reach the xArm at %s: check the controller is on and this machine is on the arm's network (ping %s)." % (ip, ip))
        ctx.log("Connected: firmware %s." % getattr(api, "version", "?"))
        if api.warn_code:
            api.clean_warn()

    for f in (ctx.inputs.get("robot") or []):
        if f.name.endswith(".gz"):
            ctx.log("Robot model for the 3D view: %s" % f.name)

    def angles():
        try:
            code, a = api.get_servo_angle(is_radian=False)
            if code == 0 and a:
                return [float(v) for v in a[:n]]
        except Exception:
            pass
        return None

    def show_joints(q, force=False):
        now = time.time()
        if q and (force or now - S["last_joints"] >= 0.1):
            S["last_joints"] = now
            ctx.telemetry("joints", {"j%d" % (i + 1): round(v, 2) for i, v in enumerate(q)})

    def ok_to_move():
        if getattr(api, "error_code", 0):
            status("The arm reports error C%s: clear it (UFACTORY Studio or flow 0b), then try again." % api.error_code)
            return False
        return True

    def hold():
        """Normal (position) mode: the arm holds where it is."""
        try:
            api.set_mode(0)
            api.set_state(state=0)
        except Exception as e:
            ctx.log("Back to normal mode: %s" % e)
        time.sleep(0.3)

    def controls(allowed):
        """Reads the controls while the arm works: speed and repeats change at once; Stop (or End) stops."""
        for m in ctx.messages(channel):
            d = m.get("data") if isinstance(m.get("data"), dict) else {"control": str(m.get("data")), "value": True}
            c, v = str(d.get("control", "")).strip().lower(), d.get("value")
            if c == "speed":
                try:
                    S["speed"] = max(0.25, min(1.5, float(v) / 100.0))
                    event("speed", "%d%%" % round(S["speed"] * 100))
                except (TypeError, ValueError):
                    pass
            elif c == "repeats":
                try:
                    S["repeats"] = max(1, min(100, int(float(v))))
                except (TypeError, ValueError):
                    pass
            elif c in ("stop", "end"):
                return c
            elif c:
                ctx.log("“%s” is not taken now (%s): press Stop first." % (d.get("control"), allowed))
        return None

    # -- the recording --------------------------------------------------------------------------------------------
    def clean(pts):
        """Smooths the hand's tremor (a moving average of about 80 ms) and trims the still parts at both ends."""
        if len(pts) < 3:
            return None
        t = [x[0] for x in pts]
        q = [x[1] for x in pts]
        w = max(1, int(round(0.04 * rate)))
        sm = []
        for i in range(len(q)):
            a, b = max(0, i - w), min(len(q), i + w + 1)
            sm.append([sum(q[k][j] for k in range(a, b)) / (b - a) for j in range(n)])
        moved = lambda x, y: max(abs(u - v) for u, v in zip(x, y)) > 0.3
        i0 = 0
        while i0 < len(sm) - 1 and not moved(sm[i0], sm[0]):
            i0 += 1
        i1 = len(sm) - 1
        while i1 > i0 and not moved(sm[i1], sm[-1]):
            i1 -= 1
        i0, i1 = max(0, i0 - 1), min(len(sm) - 1, i1 + 1)
        if i1 - i0 < 2:
            return None
        t = [x - t[i0] for x in t[i0:i1 + 1]]
        sm = sm[i0:i1 + 1]
        vmax = 0.0
        for k in range(1, len(t)):
            dt = max(1e-3, t[k] - t[k - 1])
            vmax = max(vmax, max(abs(a - b) for a, b in zip(sm[k], sm[k - 1])) / dt)
        return {"t": t, "q": sm, "vmax": vmax}

    def save(r):
        with open(ctx.output("recording", "recording.csv"), "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["t"] + ["j%d" % (i + 1) for i in range(n)])
            for t, q in zip(r["t"], r["q"]):
                w.writerow([round(t, 4)] + [round(v, 3) for v in q])

    def load(path):
        t, q = [], []
        with open(path, newline="") as f:
            for row in csv.DictReader(f):
                t.append(float(row["t"]))
                q.append([float(row["j%d" % (i + 1)]) for i in range(n)])
        if len(t) < 2:
            return None
        vmax = max(max(abs(a - b) for a, b in zip(q[k], q[k - 1])) / max(1e-3, t[k] - t[k - 1]) for k in range(1, len(t)))
        return {"t": t, "q": q, "vmax": vmax}

    def record():
        if not ok_to_move():
            return
        api.motion_enable(enable=True)
        api.set_mode(2)
        api.set_state(state=0)
        time.sleep(0.5)
        mode = getattr(api, "mode", 2)
        if mode != 2:
            hold()
            status("The arm did not go into manual mode (it is in mode %s): check it has no error and UFACTORY Studio is not controlling it." % mode)
            return
        S["state"] = "recording"
        event("record", "manual mode")
        status("Recording: move the arm by hand — it holds its own weight. Press Stop when you are done.", "")
        pts, t0, tick, why = [], time.perf_counter(), 1.0 / rate, "Stop"
        nxt = t0
        while True:
            c = controls("recording")
            if c:
                why = c
                break
            q = angles()
            t = time.perf_counter() - t0
            if q:
                pts.append((t, q))
                show_joints(q)
            if len(pts) % int(rate) == 1:
                ctx.telemetry("status", {"text": "Recording: %.0f s — move the arm by hand, then press Stop." % t, "detail": "%d points" % len(pts), "state": "recording"})
            if t > max_seconds:
                why = "the longest recording (%.0f s)" % max_seconds
                break
            nxt += tick
            time.sleep(max(0.0, nxt - time.perf_counter()))
        hold()
        S["state"] = "idle"
        r = clean(pts)
        if not r:
            status("Nothing recorded: the arm did not move. Press Record and move it by hand.")
            return "end" if why == "end" else None
        S["rec"] = r
        save(r)
        event("recorded", "%.1f s, %d points, fastest joint %.0f°/s" % (r["t"][-1], len(r["t"]), r["vmax"]))
        status("Recorded %.1f s (ended by %s). Set Repeats and Speed, then press Repeat." % (r["t"][-1], why))
        return "end" if why == "end" else None

    # -- playback -------------------------------------------------------------------------------------------------
    def inside(q):
        bad = [i for i, (a, (lo, hi)) in enumerate(zip(q, lim)) if not lo <= a <= hi]
        return None if not bad else "J%d at %.1f° is outside its range (%.0f° to %.0f°)" % (bad[0] + 1, q[bad[0]], lim[bad[0]][0], lim[bad[0]][1])

    def go_to(target, why):
        """A joint move to target at Move speed; False if stopped."""
        cur = angles()
        if cur is None:
            status("The arm does not answer with its joint angles.")
            return False
        far = max(abs(a - b) for a, b in zip(cur, target))
        if far < 0.2:
            return True
        hold()
        status(why)
        code = api.set_servo_angle(angle=list(target), speed=move_speed, mvacc=move_speed * 4, is_radian=False, wait=False)
        if code not in (0, 2):
            status("The arm refused to move (code %s, error C%s)." % (code, getattr(api, "error_code", "?")))
            return False
        t0 = time.time()
        time.sleep(0.2)
        while time.time() - t0 < far / move_speed + 15:
            c = controls("moving")
            if c:
                api.set_state(4)
                time.sleep(0.3)
                hold()
                status("Stopped." if c == "stop" else "Ended.")
                return c
            show_joints(angles())
            try:
                moving = api.get_is_moving()
            except Exception:
                moving = time.time() - t0 < far / move_speed + 1
            if not moving:
                break
            time.sleep(0.05)
        if getattr(api, "error_code", 0):
            status("The arm stopped with error C%s." % api.error_code)
            return False
        return True

    def at(r, t):
        ts, qs = r["t"], r["q"]
        if t <= 0:
            return list(qs[0])
        if t >= ts[-1]:
            return list(qs[-1])
        lo, hi = 0, len(ts) - 1
        while hi - lo > 1:
            mid = (lo + hi) // 2
            if ts[mid] <= t:
                lo = mid
            else:
                hi = mid
        f = (t - ts[lo]) / max(1e-9, ts[hi] - ts[lo])
        return [a + (b - a) * f for a, b in zip(qs[lo], qs[hi])]

    def play(r, k, total):
        """Streams the recording once (servo mode); False if stopped."""
        api.set_mode(1)
        api.set_state(state=0)
        time.sleep(0.3)
        T, t, q_prev = r["t"][-1], 0.0, list(r["q"][0])
        tick = 1.0 / STREAM_HZ
        last = time.perf_counter()
        nxt = last
        shown = 0.0
        while t < T:
            c = controls("playing")
            if c:
                api.set_state(4)
                time.sleep(0.3)
                hold()
                status("Stopped during repeat %d of %d." % (k, total) if c == "stop" else "Ended.")
                return c
            now = time.perf_counter()
            dt = min(0.05, now - last)
            last = now
            adv = dt * S["speed"]
            q = at(r, t + adv)
            v = max(abs(a - b) for a, b in zip(q, q_prev)) / max(dt, 1e-3)
            if v > max_speed:
                adv *= max_speed / v
                q = at(r, t + adv)
            t += adv
            code = api.set_servo_angle_j(q, is_radian=False)
            if code not in (0, 2) and getattr(api, "error_code", 0):
                hold()
                status("The arm stopped with error C%s during repeat %d." % (api.error_code, k))
                return False
            q_prev = q
            show_joints(q)
            if now - shown > 0.5:
                shown = now
                ctx.telemetry("status", {"text": "Repeat %d of %d · %d%% speed" % (k, total, round(S["speed"] * 100)),
                                         "detail": "%.0f%% of the movement" % (100.0 * t / T), "state": "playing", "progress": round(t / T, 3)})
            nxt += tick
            time.sleep(max(0.0, nxt - time.perf_counter()))
            if nxt < time.perf_counter() - 0.1:
                nxt = time.perf_counter()
        hold()
        return True

    def repeat():
        r = S["rec"]
        if not r:
            status("Nothing recorded yet: press Record first.")
            return
        if not ok_to_move():
            return
        out = next((inside(q) for q in r["q"] if inside(q)), None)
        if out:
            status("The recording cannot be played: " + out + ".")
            return
        api.motion_enable(enable=True)
        hold()
        total = S["repeats"]
        S["state"] = "playing"
        event("repeat", "%d times at %d%%" % (total, round(S["speed"] * 100)))
        try:
            for k in range(1, total + 1):
                res = go_to(r["q"][0], "Repeat %d of %d: going to the start pose…" % (k, total))
                if res is not True:
                    return res
                res = play(r, k, total)
                if res is not True:
                    return res
                event("played", "%d of %d" % (k, total))
            res = go_to(r["q"][0], "Done %d repeats: back to the initial position…" % total)
            if res is not True:
                return res
            status("Done: %d repeats; the arm is back at the initial position." % total)
        finally:
            S["state"] = "idle"

    # -- the session ----------------------------------------------------------------------------------------------
    for f in (ctx.inputs.get("recording") or []):
        if f.name.endswith(".csv"):
            try:
                S["rec"] = load(f.path)
                ctx.log("Recording loaded from %s." % f.name)
            except Exception as e:
                ctx.log("The recording %s could not be read (%s)." % (f.name, e))
    if S["rec"]:
        save(S["rec"])
    if not sim:
        if api.error_code:
            status("The arm reports error C%s: clear it first (UFACTORY Studio or flow 0b), then press Record." % api.error_code)
        else:
            api.motion_enable(enable=True)
            hold()
    if sim or not api.error_code:
        status("Ready. Press Record, move the arm by hand, then Stop." if not S["rec"] else "Ready: a recording is loaded. Press Repeat, or Record a new one.")
    show_joints(angles(), force=True)

    stopped = False
    try:
        while time.time() < deadline:
            for m in ctx.messages(channel, wait=0.2):
                d = m.get("data") if isinstance(m.get("data"), dict) else {"control": str(m.get("data")), "value": True}
                c, v = str(d.get("control", "")).strip().lower(), d.get("value")
                res = None
                if c == "record":
                    res = record()
                elif c == "repeat":
                    res = repeat()
                elif c == "go to start":
                    if S["rec"] and ok_to_move():
                        res = go_to(S["rec"]["q"][0], "Going to the start pose…")
                        if res is True:
                            status("At the start pose.")
                    else:
                        status("Nothing recorded yet: press Record first.")
                elif c == "speed":
                    S["speed"] = max(0.25, min(1.5, float(v) / 100.0))
                    status("Speed %d%%." % round(S["speed"] * 100))
                elif c == "repeats":
                    S["repeats"] = max(1, min(100, int(float(v))))
                    status("Repeats: %d." % S["repeats"])
                elif c == "stop":
                    status("Nothing to stop: the arm is holding still.")
                elif c == "end":
                    res = "end"
                if res == "end":
                    raise Stop("Ended.")
            show_joints(angles())
    except Stop:
        pass
    except KeyboardInterrupt:
        stopped = True
        try:
            api.set_state(4)
        except Exception:
            pass
        raise
    finally:
        if not stopped:
            hold()
        try:
            api.disconnect()
        except Exception:
            pass
        if not stopped:
            S["state"] = "idle"
            status("The session is over: the arm holds where it is.")
    r = S["rec"]
    return {"robot": "simulation" if sim else str(p.get("ip")), "recorded_s": round(r["t"][-1], 2) if r else 0, "points": len(r["t"]) if r else 0}
