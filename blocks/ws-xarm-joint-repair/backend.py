# ── EsaSim block ──────────────────────────────────────────────────────
# xArm joint repair (ws-xarm-joint-repair) · version 1 · machine block, Python: def run(ctx)
# UFACTORY's procedure for a joint's encoder error (S17, S18) and for setting a joint's zero again, step by step from the Controls view, without Studio's Joint debug page.
# Workspace: Gianluca De Novi's workspace · Author: Gianluca De Novi
# Saved: 2026-11-09
#
# Inputs   ctx.inputs["id"] is a list of files: .path .name .size .read_text() .read_json()
#   (none)
# Outputs  ctx.output("id", "name.ext") is the path to write; ctx.write_json / ctx.write_text
#   log         Table (csv)               Repair log · joint_repair.csv: every command sent, the arm's answer, its errors and joint angles at that moment
#   telemetry   Live telemetry            ctx.telemetry("channel", data) while it runs
# Settings ctx.params["key"]
#   robot       select    Robot = "rehearsal" of rehearsal | real xArm
#   ip          text      Arm IP address = "192.168.1.200"
#   rehearsal_joint number    Joint with the error (rehearsal) = 5
#   minutes     number    Longest session (minutes) = 60
#   channel     text      Controls channel = "control"
# Runs on  Any machine. Return a dict to fill the "result" output.
# Packages xarm-python-sdk (installed by the worker with pip when missing)
# EsaSim rewrites these lines from the block's definition each time it is saved: change the definition, not them.
# ── end of EsaSim block header ────────────────────────────────────────

"""xArm joint repair: UFACTORY's procedure for a joint's encoder error (S17 single-turn, S18 multi-turn) and for
setting a joint's zero again, one step at a time from the Controls view, for arms whose UFACTORY Studio has no
Joint debug page (Studio before 2.4.0). It sends the same commands as that page, through the xArm SDK:

    H101 D0813 V2   I<joint>   clear the single-turn encoder error (S17)      set_servo_addr_16(j, 0x0813, 2)
    H101 D0813 V256 I<joint>   clear the multi-turn encoder error (S18)       set_servo_addr_16(j, 0x0813, 256)
    H101 D0104 V1   I<joint>   unlock the joint, to turn it by hand           set_servo_addr_16(j, 0x0104, 1)
    D13 I<joint>               set the joint's present position as 0°        set_servo_zero(j)

Every command that changes the arm is sent only on a second press of its button within 15 seconds, and is written
to the log with the arm's answer. Robot = rehearsal sends nothing: a pretend arm with an S17 error on the chosen
joint answers instead, to learn the steps."""
import csv
import signal
import time

ENCODER_REG = 0x0813
UNLOCK_REG = 0x0104
CONFIRM_S = 15.0
CARRIES_ARM = (2, 3)    # joints that hold the arm up: unlocked, the arm falls


class Stop(Exception):
    pass


class Rehearsal:
    """A pretend xArm 6 with an S17 error on one joint, answering like the SDK; nothing is sent anywhere."""

    def __init__(self, joint):
        self.connected, self.axis, self.version, self.sn = True, 6, "rehearsal (no arm)", "-"
        self.state, self.mode, self.warn_code = 4, 0, 0
        self.q = [0.0, -20.0, -30.0, 0.0, 47.0, 0.0]
        self.bad, self.code, self.unlocked, self.estop_needed = joint, 17, None, False
        self.error_code = 10  # C10..C17: a joint reports an error
        self.zero_offset = 0.0

    def connect(self):
        self.connected = True

    def disconnect(self):
        self.connected = False

    def get_servo_angle(self, is_radian=False):
        if self.unlocked:  # someone turns it by hand towards zero
            j = self.unlocked - 1
            self.q[j] += max(-4.0, min(4.0, -self.q[j] * 0.3)) + 0.0
        return 0, list(self.q)

    def get_servo_debug_msg(self):
        out = []
        for i in range(1, 8):
            c = self.code if i == self.bad else 0
            out.append({"servo_id": i, "status": 0, "code": c, "title": {17: "Single-turn encoder error", 18: "Multi-turn encoder error"}.get(c, "Normal"), "desc": ""})
        return 0, out

    def set_servo_addr_16(self, servo_id=None, addr=None, value=None):
        if addr == ENCODER_REG and servo_id == self.bad and value in (2, 256):
            self.code = 0
            self.q[servo_id - 1] += 3.6  # the zero moved a little: the case where it must be set again
        if addr == UNLOCK_REG and value == 1:
            self.unlocked = servo_id
        return 0

    def set_servo_zero(self, servo_id=None):
        self.q[servo_id - 1] = 0.0
        return 0

    def clean_error(self):
        if not self.code:
            self.error_code = 0
        return 0

    def clean_warn(self):
        return 0

    def motion_enable(self, enable=True):
        self.unlocked = None
        return 0

    def set_mode(self, m):
        return 0

    def set_state(self, state=0):
        self.state = 0 if not self.error_code else 4
        return 0

    def estop(self):
        self.unlocked = None


def run(ctx):
    p = ctx.params
    rehearsal = str(p.get("robot", "real xArm")).lower().startswith("rehears")
    channel = str(p.get("channel") or "control")
    deadline = time.time() + float(p.get("minutes", 60)) * 60
    log = csv.writer(open(ctx.output("log", "joint_repair.csv"), "w", newline=""))
    log.writerow(["time", "step", "joint", "command", "sdk call", "answer", "error", "servo codes", "joint angles"])

    def interrupted(*_):
        raise KeyboardInterrupt()
    try:
        signal.signal(signal.SIGTERM, interrupted)
    except (ValueError, OSError):
        pass

    S = {"joint": None, "chosen": None, "pending": None, "step": "check", "before": None, "after": None, "clear": "S17-single-turn"}

    # -- the arm -----------------------------------------------------------------------------------------------
    if rehearsal:
        api = Rehearsal(int(p.get("rehearsal_joint", 5) or 5))
        ctx.log("Rehearsal: nothing is sent to any arm. A pretend arm with an S17 error on joint %d answers." % api.bad)
    else:
        from xarm.wrapper import XArmAPI
        ip = str(p.get("ip") or "").strip()
        ctx.telemetry("guide", {"step": "Connecting", "text": "Connecting to the xArm at %s…" % ip})
        try:
            api = XArmAPI(ip, is_radian=False)
        except Exception as e:
            raise RuntimeError("The arm at %s did not accept the connection (%s). Close UFACTORY Studio's page and any other program connected to it, and run again." % (ip, e))
        if not api.connected:
            raise RuntimeError("Cannot reach the xArm at %s: check the controller is on, the address, and that this machine is on the arm's network (ping %s). "
                               "Flow 0 (Robot test) says in detail what is missing." % (ip, ip))

    def axes():
        try:
            return max(5, min(7, int(api.axis or 6)))
        except Exception:
            return 6

    def ensure():
        if not api.connected:
            try:
                api.connect()
                time.sleep(1.0)
            except Exception:
                pass
        return api.connected

    def angles():
        try:
            code, a = api.get_servo_angle(is_radian=False)
            if code == 0 and a:
                return [float(v) for v in a[:axes()]]
        except Exception:
            pass
        return None

    def servo():
        """[(joint, code, title)] of the joints that report an error."""
        try:
            code, msgs = api.get_servo_debug_msg()
        except Exception as e:
            ctx.log("The joints' error codes could not be read (%s)." % e)
            return None
        out = []
        for m in msgs or []:
            sid = int(m.get("servo_id", 9))
            if sid <= axes() and m.get("code"):
                t = m.get("title")
                out.append((sid, int(m["code"]), str(t.get("en") if isinstance(t, dict) else t or "").strip()))
        return out

    def snapshot():
        q, bad = angles(), servo()
        h = {"connected": bool(api.connected), "state": getattr(api, "state", None), "error": getattr(api, "error_code", None), "warning": getattr(api, "warn_code", None)}
        if q:
            ctx.telemetry("joints", {"j%d" % (i + 1): round(v, 2) for i, v in enumerate(q)})
            h["angles"] = "   ".join("J%d %+.1f°" % (i + 1, v) for i, v in enumerate(q))
        h["servo"] = "; ".join("J%d S%d %s" % b for b in bad) if bad else ("none" if bad is not None else "unknown")
        ctx.telemetry("health", h)
        return q, bad, h

    def joint():
        return S["chosen"] or S["joint"]

    def record(command, call, answer):
        q, bad, h = snapshot()
        log.writerow([time.strftime("%Y-%m-%d %H:%M:%S"), S["step"], joint(), command, call, answer, h.get("error"), h.get("servo"), h.get("angles", "")])

    def guide(step, text):
        S["step"] = step
        j = joint()
        ctx.telemetry("guide", {"step": step, "text": text, "joint": ("J%d" % j) if j else "not chosen"})
        ctx.log("[%s] %s" % (step, text))

    def confirm(action, what):
        """True on the second press of the same action within CONFIRM_S seconds."""
        now = time.time()
        pend = S["pending"]
        if pend and pend[0] == action and pend[1] == joint() and now - pend[2] <= CONFIRM_S:
            S["pending"] = None
            return True
        S["pending"] = (action, joint(), now)
        ctx.telemetry("guide", {"step": S["step"], "joint": "J%d" % joint(),
                                "text": "To send %s, press the same button again within %d seconds. Nothing has been sent yet." % (what, CONFIRM_S)})
        return False

    def reading(j, q):
        return ("%+.1f°" % q[j - 1]) if q and 0 < j <= len(q) else "unknown"

    # -- step 1: what the arm says -------------------------------------------------------------------------------
    def check(first=False):
        q, bad, h = snapshot()
        if first:
            S["before"] = q
            ctx.log("Arm: firmware %s, serial %s, %s joints." % (getattr(api, "version", "?"), getattr(api, "sn", "?"), axes()))
            ctx.log("Joint angles now: %s" % h.get("angles", "unknown"))
        if bad:
            S["joint"] = bad[0][0]
            if bad[0][1] == 18:
                S["clear"] = "S18-multi-turn"
        if bad is None:
            guide("1 · Check", "The joints' error codes could not be read. Choose the joint UFACTORY told you about in Joint, then go on.")
            return
        if not bad:
            msg = "No joint reports an error%s." % ("" if not h.get("error") else (" (the arm reports C%s: press Clear the arm's error)" % h["error"]))
            guide("1 · Check", msg + " To set a joint's zero anyway, choose it in Joint and go to step 4 (Unlock the joint).")
            return
        j, code, title = bad[0]
        more = (" Other joints too: " + ", ".join("J%d S%d" % (b[0], b[1]) for b in bad[1:]) + ": repair one at a time.") if len(bad) > 1 else ""
        guide("1 · Check",
              "Joint %d reports S%d (%s); it reads %s now.%s Before anything: put a strip of tape across joint %d and draw a line over both halves, "
              "and photograph the arm: the repair can lose the joint's zero, and this is where you can see it. "
              "Then press Clear the encoder error (twice); it sends H101 D0813 V%d (Clear: %s)." % (j, code, title or "joint error", reading(j, q), more, j, 256 if "18" in S["clear"] else 2, S["clear"]))

    def clear_encoder():
        j = joint()
        if not j:
            guide(S["step"], "Choose the joint first (Joint).")
            return
        value = 256 if "18" in S["clear"] else 2
        cmd = "H101 D0813 V%d I%d" % (value, j)
        if not confirm("clear", cmd):
            return
        ensure()
        code = api.set_servo_addr_16(servo_id=j, addr=ENCODER_REG, value=value)
        record(cmd, "set_servo_addr_16(%d, 0x0813, %d)" % (j, value), code)
        if code != 0:
            guide("2 · Clear the encoder error", "The arm answered code %s to %s: it did not take it. Write the code to UFACTORY (support@ufactory.cc) with the log." % (code, cmd))
            return
        S["after"] = "clear"
        guide("3 · E-stop", "Sent %s; the arm answered 0 (accepted). Now press the red E-stop, wait 10 seconds, release it, then press I pressed and released the E-stop." % cmd)

    def unlock():
        j = joint()
        if not j:
            guide(S["step"], "Choose the joint first (Joint).")
            return
        cmd = "H101 D0104 V1 I%d" % j
        what = cmd + (" — joint %d holds the arm up: when it is unlocked the arm FALLS unless someone holds it" % j if j in CARRIES_ARM else "")
        if not confirm("unlock", what):
            return
        ensure()
        code = api.set_servo_addr_16(servo_id=j, addr=UNLOCK_REG, value=1)
        record(cmd, "set_servo_addr_16(%d, 0x0104, 1)" % j, code)
        if code != 0:
            guide("4 · Unlock", "The arm answered code %s to %s: the joint may still be locked; do not force it." % (code, cmd))
            return
        S["after"] = "unlock"
        guide("4 · Turn it to zero",
              "Joint %d is unlocked. Turn it slowly by hand to its zero: the position it has in the arm's zero pose (all joints 0°), the way it was before "
              "the error. The reading (live, under Arm) is %s; if the zero was not lost, it reads 0.0° there. Hold it there and press the E-stop, "
              "wait 10 seconds, release it, then press I pressed and released the E-stop." % (j, reading(j, angles())))

    def set_zero():
        j = joint()
        if not j:
            guide(S["step"], "Choose the joint first (Joint).")
            return
        cmd = "D13 I%d" % j
        if not confirm("zero", cmd + " (the present position of joint %d becomes 0°)" % j):
            return
        ensure()
        code = api.set_servo_zero(servo_id=j)
        record(cmd, "set_servo_zero(%d)" % j, code)
        if code != 0:
            guide("5 · Set the zero", "The arm answered code %s to %s: it did not take it." % (code, cmd))
            return
        S["after"] = "zero"
        guide("6 · E-stop", "Sent %s; the arm answered 0. Press the E-stop, wait 10 seconds, release it, then press I pressed and released the E-stop: "
                            "the new zero takes effect." % cmd)

    def after_estop():
        if rehearsal:
            api.estop()
        for _ in range(10):
            if ensure():
                break
            time.sleep(1.0)
        time.sleep(1.0)
        q, bad, h = snapshot()
        j = joint()
        before = reading(j, S["before"]) if j else "?"
        now = reading(j, q) if j else "?"
        still = [b for b in (bad or []) if b[0] == j]
        record("E-stop pressed and released", "-", "-")
        if S["after"] == "clear":
            if still:
                guide("3 · E-stop", "Joint %d still reports S%d. Press the E-stop once more for 10 seconds; if it stays, choose Clear: S18-multi-turn "
                                   "if the code is S18, else write to UFACTORY with the log." % (j, still[0][1]))
                return
            guide("4 · Is the zero still right?",
                  "Joint %d reports no error now. It read %s before the repair and reads %s now, with the joint where it was. "
                  "If the line on your tape still matches and the two readings agree (within a degree), the zero is fine: press Clear the arm's error, then Finish. "
                  "If not, the zero must be set again: someone holds the arm%s, then press Unlock the joint (twice)." % (j, before, now, " (joint %d carries it)" % j if j in CARRIES_ARM else ""))
        elif S["after"] == "unlock":
            guide("5 · Set the zero", "Joint %d is locked again at %s. If it is at its zero, press Set this position as zero (twice); if not, unlock it again." % (j, now))
        elif S["after"] == "zero":
            guide("7 · Check", "Joint %d now reads %s (0.0° expected). Press Clear the arm's error; then check in UFACTORY Studio that the arm moves to its zero pose "
                              "and the joint's line matches." % (j, now))
        else:
            guide(S["step"], "Read again: joint %s reads %s; errors: %s." % (j, now, h.get("servo")))
        S["after"] = None

    def clear_arm_error():
        ensure()
        try:
            api.clean_error()
            api.clean_warn()
            api.motion_enable(enable=True)
            api.set_mode(0)
            api.set_state(state=0)
        except Exception as e:
            ctx.log("Clearing the arm's error: %s" % e)
        time.sleep(1.0)
        record("clear error", "clean_error, clean_warn, motion_enable, set_mode(0), set_state(0)", getattr(api, "error_code", None))
        q, bad, h = snapshot()
        if h.get("error") or bad:
            guide("Clear the arm's error", "The arm still reports C%s%s." % (h.get("error"), (" (" + h.get("servo") + ")") if bad else ""))
        else:
            guide("Done?", "No errors: the arm is enabled and ready. Check the joint's zero in UFACTORY Studio (move to the zero pose slowly), then press Finish. "
                           "Flow 0 · Robot test moves one joint at a time to check it.")

    check(first=True)

    # -- the controls --------------------------------------------------------------------------------------------
    last_tick = 0.0
    try:
        while time.time() < deadline:
            for m in ctx.messages(channel, wait=0.5):
                d = m.get("data") if isinstance(m.get("data"), dict) else {"control": str(m.get("data")), "value": True}
                c, v = str(d.get("control", "")).strip(), d.get("value")
                cl = c.lower()
                if cl == "joint":
                    S["chosen"] = None if str(v).lower().startswith("auto") else int(str(v).strip()[0])
                    S["pending"] = None
                    guide(S["step"], "Joint %s chosen." % (joint() or "auto (none reports an error)"))
                elif cl == "clear":
                    S["clear"] = str(v)
                    S["pending"] = None
                    guide(S["step"], "Clear the encoder error will send H101 D0813 V%d." % (256 if "18" in S["clear"] else 2))
                elif cl.startswith("check"):
                    check()
                elif cl.startswith("clear the encoder"):
                    clear_encoder()
                elif "e-stop" in cl:
                    after_estop()
                elif cl.startswith("unlock"):
                    unlock()
                elif cl.startswith("set this position"):
                    set_zero()
                elif cl.startswith("clear the arm"):
                    clear_arm_error()
                elif cl in ("finish", "end"):
                    raise Stop("Finished.")
            if time.time() - last_tick > 1.0:
                last_tick = time.time()
                ensure()
                snapshot()
    except Stop as e:
        guide("Finished", str(e) + " Every command sent is in joint_repair.csv (Results).")
    finally:
        try:
            api.disconnect()
        except Exception:
            pass
    return {"robot": "rehearsal" if rehearsal else str(p.get("ip")), "joint": joint()}
