# ── EsaSim block ──────────────────────────────────────────────────────
# xArm teach and repeat (ws-xarm-teach-and-repeat) · version 2 · machine block, Python: def run(ctx)
# Free the xArm and move it by hand, record a movement between two moments, repeat it any number of times as one smooth motion, then send the arm to its zero position.
# Workspace: Gianluca De Novi's workspace · Author: Gianluca De Novi
# Saved: 2026-11-09
#
# Inputs   ctx.inputs["id"] is a list of files: .path .name .size .read_text() .read_json()
#   robot       Files (gz, txt)           Robot model · optional · From xArm model: connected, the model is made before this block starts, so the 3D view shows the arm
#   recording   Table (csv)               Recording · optional · A recording.csv from an earlier run, to play again
# Outputs  ctx.output("id", "name.ext") is the path to write; ctx.write_json / ctx.write_text
#   recording   Table (csv)               Recording · recording.csv: t (s) and the joint angles (degrees), smoothed
#   log         Table (csv)               Session log · session.csv: start, recordings, repeats, speed changes, stops, end
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

"""xArm teach and repeat. Start frees the arm: it holds its own weight and you move it by hand (manual mode).
Record records the joints from where the arm is at that moment; Stop recording ends it where the arm is then, and
the arm holds still. Repeat plays the movement the number of times asked, as one continuous motion: a gentle
transition takes the arm to where the movement starts, and between two repetitions another gentle transition joins
the end of one to the start of the next. End takes the arm gently to its zero position (every joint at 0°) and ends
the session. Stop stops any motion at once. The Speed slider plays slower or faster, also while it plays.

Playback streams joint angles to the controller (servo mode, 100 a second). No joint is ever asked to go faster than
Max joint speed: where the recording is faster, the playback slows down there. Transitions follow a minimum-jerk
profile (they start and end at rest) and reach at most Move speed. Robot = simulation sends nothing."""
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
            self.q = [b + a * math.sin(0.9 * t) * min(1.0, t) for b, a in zip(self.base, amp)]
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


def minjerk(u):
    u = min(1.0, max(0.0, u))
    return u * u * u * (10.0 - 15.0 * u + 6.0 * u * u)


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
         "repeats": max(1, min(100, int(float(p.get("repeats", 3))))), "state": "holding", "last_joints": 0.0}
    log = csv.writer(open(ctx.output("log", "session.csv"), "w", newline="", encoding="utf-8"))
    log.writerow(["time", "event", "detail"])

    def interrupted(*_):
        raise KeyboardInterrupt()
    try:
        signal.signal(signal.SIGTERM, interrupted)
    except (ValueError, OSError):
        pass

    def event(name, detail=""):
        log.writerow([time.strftime("%Y-%m-%d %H:%M:%S"), name, detail])

    def describe():
        r = S["rec"]
        mode = {"holding": "The arm holds still.", "free": "The arm is free: move it by hand.",
                "recording": "Recording.", "playing": "Playing."}.get(S["state"], "")
        if not r:
            return "%s Nothing recorded yet. Repeats %d · speed %d%%." % (mode, S["repeats"], round(S["speed"] * 100))
        return ("%s Recording: %.1f s, fastest joint %.0f°/s. Repeats %d · speed %d%%."
                % (mode, r["t"][-1], r["vmax"], S["repeats"], round(S["speed"] * 100)))

    def status(text, detail=None):
        ctx.telemetry("status", {"text": text, "detail": detail if detail is not None else describe(), "state": S["state"]})
        ctx.log(text)

    # -- the arm --------------------------------------------------------------------------------------------------
    if sim:
        api = SimArm(n)
        ctx.log("Simulation: nothing is sent to any arm; a pretend hand moves a pretend arm while it is free.")
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
        S["state"] = "holding"

    def free():
        """Manual mode: the arm holds its own weight and moves by hand. True if it went into it."""
        if not ok_to_move():
            return False
        api.motion_enable(enable=True)
        api.set_mode(2)
        api.set_state(state=0)
        time.sleep(0.5)
        mode = getattr(api, "mode", 2)
        if mode != 2:
            hold()
            status("The arm did not become free (it is in mode %s): check it has no error and UFACTORY Studio is not controlling it." % mode)
            return False
        S["state"] = "free"
        event("start", "manual mode")
        return True

    def stopper():
        """Pulls the messages while the arm works: speed and repeats change at once; returns stop or end if asked."""
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
            elif c in ("stop", "end", "stop recording"):
                return c
            elif c:
                ctx.log("“%s” waits: press Stop first." % d.get("control"))
        return None

    # -- the recording --------------------------------------------------------------------------------------------
    def clean(pts):
        """Smooths the hand's tremor (a moving average of about 80 ms); the first and last poses are kept exactly as
        they were when Record and Stop recording were pressed."""
        if len(pts) < 2:
            return None
        t0 = pts[0][0]
        t = [x[0] - t0 for x in pts]
        q = [x[1] for x in pts]
        w = max(1, int(round(0.04 * rate)))
        sm = []
        for i in range(len(q)):
            h = min(w, i, len(q) - 1 - i)  # the window narrows to the pose itself at both ends
            sm.append([sum(q[k][j] for k in range(i - h, i + h + 1)) / (2 * h + 1) for j in range(n)])
        vmax = 0.0
        for k in range(1, len(t)):
            dt = max(0.5 / rate, t[k] - t[k - 1])
            vmax = max(vmax, max(abs(a - b) for a, b in zip(sm[k], sm[k - 1])) / dt)
        return {"t": t, "q": sm, "vmax": vmax}

    def save(r):
        with open(ctx.output("recording", "recording.csv"), "w", newline="", encoding="utf-8") as f:
            w = csv.writer(f)
            w.writerow(["t"] + ["j%d" % (i + 1) for i in range(n)])
            for t, q in zip(r["t"], r["q"]):
                w.writerow([round(t, 4)] + [round(v, 3) for v in q])

    def load(path):
        t, q = [], []
        with open(path, newline="", encoding="utf-8") as f:
            for row in csv.DictReader(f):
                t.append(float(row["t"]))
                q.append([float(row["j%d" % (i + 1)]) for i in range(n)])
        if len(t) < 2:
            return None
        vmax = max(max(abs(a - b) for a, b in zip(q[k], q[k - 1])) / max(1e-3, t[k] - t[k - 1]) for k in range(1, len(t)))
        return {"t": t, "q": q, "vmax": vmax}

    def record():
        if S["state"] != "free" and not free():
            return
        S["state"] = "recording"
        event("record", "from %s" % ", ".join("%.1f" % v for v in (angles() or [])))
        status("Recording from here. Move the arm through the movement, then press Stop recording where it ends.", "")
        pts, t0, tick, why = [], time.perf_counter(), 1.0 / rate, "stop recording"
        nxt = t0
        while True:
            q = angles()
            t = time.perf_counter() - t0
            if q:
                pts.append((t, q))
                show_joints(q)
            c = stopper()
            if c:
                why = c
                break
            if len(pts) % int(rate) == 1:
                ctx.telemetry("status", {"text": "Recording: %.0f s. Press Stop recording where the movement ends." % t, "detail": "%d poses" % len(pts), "state": "recording"})
            if t > max_seconds:
                why = "the longest recording (%.0f s)" % max_seconds
                break
            nxt += tick
            time.sleep(max(0.0, nxt - time.perf_counter()))
        q, t = angles(), time.perf_counter() - t0
        if q and (not pts or t - pts[-1][0] >= 0.5 / rate):
            pts.append((t, q))
        hold()
        r = clean(pts)
        if not r or max(max(abs(a - b) for a, b in zip(x, r["q"][0])) for x in r["q"]) < 0.3:
            status("Nothing recorded: the arm did not move. Press Start, then Record, and move it by hand.")
            return "end" if why == "end" else None
        S["rec"] = r
        save(r)
        event("recorded", "%.1f s, %d poses, fastest joint %.0f°/s" % (r["t"][-1], len(r["t"]), r["vmax"]))
        status("Recorded %.1f s. The arm holds where you stopped. Set Repeats and Speed, then press Repeat "
               "(or Start to move it by hand again, Record to record a new movement)." % r["t"][-1])
        return "end" if why == "end" else None

    # -- playback -------------------------------------------------------------------------------------------------
    def inside(q):
        bad = [i for i, (a, (lo, hi)) in enumerate(zip(q, lim)) if not lo <= a <= hi]
        return None if not bad else "J%d at %.1f° is outside its range (%.0f° to %.0f°)" % (bad[0] + 1, q[bad[0]], lim[bad[0]][0], lim[bad[0]][1])

    def recorded(r):
        ts, qs = r["t"], r["q"]

        def at(t):
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
        return {"T": ts[-1], "at": at, "label": "movement"}

    def transition(a, b, label):
        """A gentle joint move from a to b: minimum jerk, at rest at both ends, at most Move speed."""
        far = max(abs(x - y) for x, y in zip(a, b))
        if far < 0.2:
            return None
        T = max(1.0, 1.875 * far / move_speed)
        return {"T": T, "at": lambda t: [x + (y - x) * minjerk(t / T) for x, y in zip(a, b)], "label": label}

    def stream(segments, describe_seg):
        """Plays the segments one after the other as one continuous motion (servo mode). True when done; stop or end
        if pressed (the arm stops at once and holds)."""
        if not ok_to_move():
            return False
        api.motion_enable(enable=True)
        api.set_mode(1)
        api.set_state(state=0)
        time.sleep(0.3)
        S["state"] = "playing"
        tick = 1.0 / STREAM_HZ
        q_prev = angles() or segments[0]["at"](0.0)
        nxt = last = time.perf_counter()
        shown = 0.0
        for k, seg in enumerate(segments):
            t = 0.0
            while t < seg["T"]:
                c = stopper()
                if c:
                    api.set_state(4)
                    time.sleep(0.3)
                    hold()
                    event("stopped", seg["label"])
                    status("Stopped. The arm holds where it is." if c != "end" else "Ending…")
                    return c
                now = time.perf_counter()
                dt = min(0.05, now - last)
                last = now
                adv = dt * S["speed"]
                q = seg["at"](t + adv)
                v = max(abs(a - b) for a, b in zip(q, q_prev)) / max(dt, 1e-3)
                if v > max_speed:
                    adv *= max_speed / v
                    q = seg["at"](t + adv)
                t += adv
                code = api.set_servo_angle_j(q, is_radian=False)
                if code not in (0, 2) and getattr(api, "error_code", 0):
                    hold()
                    status("The arm stopped with error C%s (%s)." % (api.error_code, seg["label"]))
                    return False
                q_prev = q
                show_joints(q)
                if now - shown > 0.5:
                    shown = now
                    ctx.telemetry("status", {"text": describe_seg(k, seg), "detail": "%d%% speed · %.0f%% of this part" % (round(S["speed"] * 100), 100.0 * t / seg["T"]),
                                             "state": "playing"})
                nxt += tick
                time.sleep(max(0.0, nxt - time.perf_counter()))
                if nxt < time.perf_counter() - 0.1:
                    nxt = time.perf_counter()
        hold()
        return True

    def repeat():
        r = S["rec"]
        if not r:
            status("Nothing recorded yet: press Start, Record, move the arm, then Stop recording.")
            return
        out = next((inside(q) for q in r["q"] if inside(q)), None)
        if out:
            status("The recording cannot be played: " + out + ".")
            return
        if S["state"] == "free":
            hold()
        here = angles()
        if here is None:
            status("The arm does not answer with its joint angles.")
            return
        total = S["repeats"]
        mv = recorded(r)
        segs = []
        first = transition(here, r["q"][0], "to the start")
        if first:
            segs.append(first)
        for k in range(1, total + 1):
            if k > 1:
                link = transition(r["q"][-1], r["q"][0], "from the end back to the start")
                if link:
                    segs.append(link)
            segs.append(dict(mv, label="repeat %d" % k, k=k))
        event("repeat", "%d times at %d%%" % (total, round(S["speed"] * 100)))

        def say(i, seg):
            if "k" in seg:
                return "Repeat %d of %d" % (seg["k"], total)
            return "Moving gently %s…" % seg["label"]
        res = stream(segs, say)
        if res is True:
            event("played", "%d repeats" % total)
            status("Done: %d repeats. The arm holds at the end of the movement. Press Repeat again, or End to go to the zero position." % total)
        return res

    def end():
        here = angles()
        if here is not None and not getattr(api, "error_code", 0):
            if S["state"] == "free":
                hold()
            seg = transition(here, [0.0] * n, "to the zero position")
            if seg:
                status("Going gently to the zero position…")
                res = stream([seg], lambda i, s: "Going gently to the zero position…")
                if res is True:
                    status("At the zero position.")
                    event("end", "zero position")
                    return
                if res == "stop":
                    status("Stopped before the zero position: the arm holds where it is.")
                    return
        event("end", "")

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
    if not sim and api.error_code:
        status("The arm reports error C%s: clear it first (UFACTORY Studio or flow 0b)." % api.error_code)
    else:
        api.motion_enable(enable=True)
        hold()
        status("Ready. Press Start to move the arm by hand." if not S["rec"] else "Ready: a recording is loaded. Press Repeat, or Start to record a new one.")
    show_joints(angles(), force=True)

    stopped = False
    try:
        while time.time() < deadline:
            for m in ctx.messages(channel, wait=0.2):
                d = m.get("data") if isinstance(m.get("data"), dict) else {"control": str(m.get("data")), "value": True}
                c, v = str(d.get("control", "")).strip().lower(), d.get("value")
                res = None
                if c == "start":
                    if S["state"] == "free":
                        status("The arm is already free: move it by hand, then press Record.")
                    elif free():
                        status("The arm is free: it holds its own weight. Move it by hand to where the movement starts, then press Record.")
                elif c == "record":
                    res = record()
                elif c == "stop recording":
                    status("Not recording now: press Record first.")
                elif c == "repeat":
                    res = repeat()
                elif c == "speed":
                    S["speed"] = max(0.25, min(1.5, float(v) / 100.0))
                    status("Speed %d%%." % round(S["speed"] * 100))
                elif c == "repeats":
                    S["repeats"] = max(1, min(100, int(float(v))))
                    status("Repeats: %d." % S["repeats"])
                elif c == "stop":
                    if S["state"] == "free":
                        hold()
                        status("The arm holds still. Press Start to move it by hand again.")
                    else:
                        status("Nothing to stop: the arm is holding still.")
                elif c == "end":
                    res = "end"
                if res == "end":
                    end()
                    raise Stop()
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
            status("The session is over: the arm holds where it is.")
    r = S["rec"]
    return {"robot": "simulation" if sim else str(p.get("ip")), "recorded_s": round(r["t"][-1], 2) if r else 0, "points": len(r["t"]) if r else 0}
