# ── EsaSim block ──────────────────────────────────────────────────────
# xArm joint repair (ws-xarm-joint-repair) · version 3 · machine block, Python: def run(ctx)
# A wizard for UFACTORY's joint-encoder repair (S17, S18) and zero reset: one instruction at a time, a ✓ or ✗ after each, then the next step.
# Workspace: Gianluca De Novi's workspace · Author: Gianluca De Novi
# Saved: 2026-11-09
#
# Inputs   ctx.inputs["id"] is a list of files: .path .name .size .read_text() .read_json()
#   robot       Files (gz, txt)           Robot model · optional · From xArm model: connected, the model is made before the repair starts, so the 3D view shows the arm
# Outputs  ctx.output("id", "name.ext") is the path to write; ctx.write_json / ctx.write_text
#   log         Table (csv)               Repair log · joint_repair.csv: every command sent, the arm's answer, its errors and joint angles at that moment
#   telemetry   Live telemetry            ctx.telemetry("channel", data) while it runs
# Settings ctx.params["key"]
#   robot       select    Robot = "rehearsal" of rehearsal | real xArm
#   arm         select    Arm = "xArm 6" of xArm 6 | xArm 7 | xArm 5
#   ip          text      Arm IP address = "192.168.1.200"
#   rehearsal_joint number    Joint with the error (rehearsal) = 1
#   minutes     number    Longest session (minutes) = 60
#   channel     text      Controls channel = "control"
# Runs on  Any machine. Return a dict to fill the "result" output.
# Packages xarm-python-sdk (installed by the worker with pip when missing)
# EsaSim rewrites these lines from the block's definition each time it is saved: change the definition, not them.
# ── end of EsaSim block header ────────────────────────────────────────

"""xArm joint repair, as a wizard: one instruction at a time in the Step-by-step guide view; you do it, press the
button the step shows, the block checks what it can on the arm, says whether it worked (✓ / ✗), and shows the next step.

It follows UFACTORY's procedure for a joint's encoder error (S17 single-turn, S18 multi-turn) and for setting a
joint's zero again, for arms whose UFACTORY Studio has no Joint debug page (Studio before 2.4.0). The commands are
those of that page, sent through the xArm SDK:

    H101 D0813 V2   I<joint>   clear the encoder error (as UFACTORY support wrote)   set_servo_addr_16(j, 0x0813, 2)
    H101 D0813 V256 I<joint>   clear the multi-turn error (UFACTORY's S18 article)    set_servo_addr_16(j, 0x0813, 256)
    H101 D0104 V1   I<joint>   unlock the joint, to turn it by hand                  set_servo_addr_16(j, 0x0104, 1)
    D13 I<joint>               the joint's present position becomes 0°              set_servo_zero(j)

A command is sent only from the step that shows it, with the button Send this command; every command and the arm's
answer go to joint_repair.csv. Robot = rehearsal sends nothing: a pretend arm with an S18 error answers."""
import csv
import signal
import time

ENCODER_REG = 0x0813
UNLOCK_REG = 0x0104

BUTTONS = {"done, next": "done", "send this command": "send", "yes": "yes", "no": "no",
           "back": "back", "start over": "restart", "finish": "finish"}


class Stop(Exception):
    pass


class Rehearsal:
    """A pretend xArm with an S18 error (status 1) on one joint, answering like the SDK; nothing is sent anywhere."""

    def __init__(self, joint, n=6):
        self.connected, self.axis, self.version, self.sn = True, n, "rehearsal (no arm)", "-"
        self.state, self.mode, self.warn_code, self.error_code = 4, 0, 0, 11
        self.true_q = [12.0, -10.0, -20.0, 30.0, 40.0, 20.0, 0.0][:n]
        self.offset = [0.0] * n
        self.bad, self.code, self.status, self.unlocked, self.cleared = joint, 18, 1, None, False
        self.offset[joint - 1] = 7.5  # the zero moved: the case where it must be set again

    def connect(self):
        self.connected = True

    def disconnect(self):
        self.connected = False

    def get_servo_angle(self, is_radian=False):
        if self.error_code:
            return 0, [0.0] * self.axis  # in error the controller gives no angles
        if self.unlocked:  # someone turns it by hand towards its true zero
            j = self.unlocked - 1
            self.true_q[j] += max(-3.0, min(3.0, -self.true_q[j] * 0.4))
        return 0, [a + o for a, o in zip(self.true_q, self.offset)]

    def get_servo_debug_msg(self):
        out = []
        for i in range(1, 9):
            bad = i == self.bad and self.code
            out.append({"servo_id": i, "status": self.status if bad else 0, "code": self.code if bad else 0,
                        "title": "Multi-turn Encoder Error" if bad else "Normal", "desc": ""})
        return 0, out

    def set_servo_addr_16(self, servo_id=None, addr=None, value=None):
        if addr == ENCODER_REG and servo_id == self.bad:
            self.cleared = True
        if addr == UNLOCK_REG and value == 1:
            self.unlocked = servo_id
        return 0

    def set_servo_zero(self, servo_id=None):
        self.offset[servo_id - 1] = -self.true_q[servo_id - 1]
        return 0

    def estop(self):
        self.unlocked = None
        if self.cleared:
            self.code, self.cleared = 0, False

    def clean_error(self):
        if not self.code:
            self.error_code = 0
        return 0

    def clean_warn(self):
        return 0

    def motion_enable(self, enable=True):
        return 0

    def set_mode(self, m):
        return 0

    def set_state(self, state=0):
        self.state = 4 if self.error_code else 0
        return 0


def run(ctx):
    p = ctx.params
    rehearsal = str(p.get("robot", "real xArm")).lower().startswith("rehears")
    n_axes = 7 if "7" in str(p.get("arm", "xArm 6")) else 5 if "5" in str(p.get("arm", "")) else 6
    channel = str(p.get("channel") or "control")
    deadline = time.time() + float(p.get("minutes", 60)) * 60
    log = csv.writer(open(ctx.output("log", "joint_repair.csv"), "w", newline=""))
    log.writerow(["time", "step", "joint", "action", "sdk call", "answer", "controller error", "joint errors", "joint angles"])

    def interrupted(*_):
        raise KeyboardInterrupt()
    try:
        signal.signal(signal.SIGTERM, interrupted)
    except (ValueError, OSError):
        pass

    W = {"step": None, "history": [], "joint": None, "chosen": None, "value": 2, "last": "", "info": {}}

    # -- the arm -----------------------------------------------------------------------------------------------
    if rehearsal:
        api = Rehearsal(max(1, min(n_axes, int(p.get("rehearsal_joint", 1) or 1))), n_axes)
        ctx.log("Rehearsal: nothing is sent to any arm. A pretend xArm %d with an S18 error on joint %d answers." % (n_axes, api.bad))
    else:
        from xarm.wrapper import XArmAPI
        ip = str(p.get("ip") or "").strip()
        ctx.telemetry("guide", {"step": "Connecting", "text": "Connecting to the xArm at %s…" % ip, "press": "Wait"})
        try:
            api = XArmAPI(ip, is_radian=False)
        except Exception as e:
            raise RuntimeError("The arm at %s did not accept the connection (%s)." % (ip, e))
        if not api.connected:
            raise RuntimeError("Cannot reach the xArm at %s: check the controller is on, the address, and that this machine is on the arm's network (ping %s)." % (ip, ip))

    def axes():
        """The joints of the arm, from the setting Arm: older firmware (1.4) does not tell the SDK which arm it is."""
        return n_axes

    def carries():
        """Joints that hold the arm up: unlocked, the arm falls."""
        return (2, 4) if axes() == 7 else (2, 3)

    def ensure(tries=1):
        for _ in range(tries):
            if api.connected:
                return True
            try:
                api.connect()
            except Exception:
                pass
            time.sleep(1.0)
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
        """[(joint, code, status, title)] of the joints that report an error; None if they cannot be read."""
        try:
            code, msgs = api.get_servo_debug_msg()
        except Exception as e:
            ctx.log("The joints' error codes could not be read (%s)." % e)
            return None
        if code != 0 and not msgs:
            return None
        out = []
        for m in msgs or []:
            sid = int(m.get("servo_id", 9))
            if sid <= axes() and m.get("code"):
                t = m.get("title")
                out.append((sid, int(m["code"]), int(m.get("status") or 0), str(t.get("en") if isinstance(t, dict) else t or "").strip()))
        return out

    def snapshot():
        q, bad = angles(), servo()
        err = getattr(api, "error_code", None)
        h = {"connected": "yes" if api.connected else "NO", "error": ("C%s" % err) if err else "none",
             "servo": "; ".join("J%d S%d (status %d) %s" % b for b in bad) if bad else ("none" if bad is not None else "unknown")}
        if q:
            ctx.telemetry("joints", {"j%d" % (i + 1): round(v, 2) for i, v in enumerate(q)})
            h["angles"] = "   ".join("J%d %+.1f°" % (i + 1, v) for i, v in enumerate(q))
            if err and not any(abs(v) > 1e-6 for v in q):
                h["angles"] += "   (the arm gives no angles while it is in error)"
        ctx.telemetry("health", h)
        return q, bad, err, h

    def joint():
        return W["chosen"] or W["joint"]

    def record(action, call, answer):
        q, bad, err, h = snapshot()
        log.writerow([time.strftime("%Y-%m-%d %H:%M:%S"), W["step"], joint(), action, call, answer, h["error"], h["servo"], h.get("angles", "")])

    def command():
        return "H101 D0813 V%d I%d" % (W["value"], joint() or 0)

    def reading():
        q, bad, err, h = snapshot()
        j = joint()
        if not q or not j or j > len(q) or (err and not any(abs(v) > 1e-6 for v in q)):
            return None
        return q[j - 1]

    def after_estop():
        """After the E-stop: wait for the arm to answer again and read it."""
        if rehearsal:
            api.estop()
        ok = ensure(tries=15)
        time.sleep(2.0)
        record("E-stop pressed and released (as you said)", "-", "connected" if ok else "not connected")
        return ok

    def estop_text(why):
        return ("1. Press the red E-stop button on the control box. It stays down.\n"
                "2. Wait 10 seconds.\n"
                "3. Turn the button in the direction of its arrows until it pops back up.\n"
                "%s\nThen press Done, next." % why)

    # -- the steps: (title, text, buttons it takes) and what each button does ------------------------------------
    def step_read():
        q, bad, err, h = snapshot()
        W["info"] = {"bad": bad, "err": err}
        if bad:
            j, code, status, title = bad[0]
            W["joint"] = j
            others = (" Other joints report errors too (%s): repair one at a time." % ", ".join("J%d S%d" % (b[0], b[1]) for b in bad[1:])) if len(bad) > 1 else ""
            return ("Read the arm",
                    "This is an xArm %d, firmware %s.\nJoint %d reports S%d (%s), status %d; the controller reports %s.%s\n"
                    "This wizard repairs joint %d. If that is the joint UFACTORY told you about, press Done, next. "
                    "(Another joint: choose it in Joint first.)" % (axes(), getattr(api, "version", "?"), j, code, title, status, h["error"], others, joint()),
                    ("done",))
        if bad is None:
            return ("Read the arm", "The joints' error codes could not be read. Choose the joint UFACTORY told you about in Joint, then press Done, next.", ("done",))
        return ("Read the arm", "No joint reports an error (controller error: %s). To set a joint's zero anyway, choose it in Joint, then press Done, next." % h["error"], ("done",))

    def act_read(b):
        if not joint():
            W["last"] = "✗ No joint chosen: choose one in Joint."
            return None
        bad = W["info"].get("bad") or []
        if any(x[0] == joint() for x in bad):
            return "mark"
        return "clear_arm" if W["info"].get("err") else "zero_ok"

    def step_mark():
        j = joint()
        return ("Mark the joint",
                "1. Put a strip of tape across joint %d so it covers both halves of the joint (the part that turns and the part that does not).\n"
                "2. Draw one straight line across the tape with a marker.\n"
                "3. Take a photo of the whole arm.\n"
                "Later this line shows whether the joint's zero moved. When it is done, press Done, next." % j, ("done",))

    def step_send_clear():
        j = joint()
        bad = [x for x in (W["info"].get("bad") or []) if x[0] == j]
        note = ""
        if bad and bad[0][1] == 18:
            note = ("\nNote: joint %d reports S18 (multi-turn), status %d. Minna wrote V2. UFACTORY's article for S18 errors uses V256 instead, "
                    "and only when the status is 0 or 8 (otherwise it says to contact them). If you are not sure which, ask Minna before sending. "
                    "Clear command below chooses V2 or V256." % (j, bad[0][2]))
        return ("Clear the encoder error",
                "This sends the command that clears the error of joint %d:\n      %s\n"
                "Nothing moves. Press Send this command.%s" % (j, command(), note), ("send",))

    def act_send_clear(b):
        j = joint()
        ensure(3)
        code = api.set_servo_addr_16(servo_id=j, addr=ENCODER_REG, value=W["value"])
        record(command(), "set_servo_addr_16(%d, 0x0813, %d)" % (j, W["value"]), code)
        if code != 0:
            W["last"] = "✗ Sent %s: the arm answered code %s, it did not take it. Press Send this command to try again, or Finish and write to UFACTORY." % (command(), code)
            return None
        W["last"] = "✓ Sent %s: the arm accepted it (answer 0)." % command()
        return "estop1"

    def step_estop1():
        return ("E-stop, so the command takes effect", estop_text("(The arm loses power for a moment; it stays where it is.)"), ("done",))

    def act_estop1(b):
        if not after_estop():
            W["last"] = "✗ The arm does not answer yet. Check that the E-stop is released (popped up), wait 10 seconds and press Done, next again."
            return None
        q, bad, err, h = snapshot()
        still = [x for x in (bad or []) if x[0] == joint()]
        if still:
            W["last"] = "✗ Joint %d still reports S%d (status %d)." % (joint(), still[0][1], still[0][2])
            return "retry"
        W["last"] = "✓ Joint %d no longer reports an error." % joint()
        return "clear_arm"

    def step_retry():
        return ("The error is still there",
                "What you can do:\n"
                "• Press Back to send the command again (you can change Clear command to V2 or V256 first), then the E-stop again.\n"
                "• Or press Finish and write to UFACTORY (support@ufactory.cc): the file joint_repair.csv in Results has every command and the arm's answers.",
                ("back", "finish"))

    def step_clear_arm():
        err = snapshot()[3]["error"]
        if err != "none":
            return ("Clear the arm's error",
                    "The controller still remembers the error (%s). This clears it and turns the motors on, like Clear Error in UFACTORY Studio. "
                    "Nothing moves. Press Send this command." % err, ("send",))
        return ("Turn the motors on",
                "This turns the arm's motors back on and checks that no error is left, like Clear Error in UFACTORY Studio. "
                "Nothing moves. Press Send this command.", ("send",))

    def act_clear_arm(b):
        ensure(3)
        try:
            api.clean_error()
            api.clean_warn()
            api.motion_enable(enable=True)
            api.set_mode(0)
            api.set_state(state=0)
        except Exception as e:
            ctx.log("Clearing the arm's error: %s" % e)
        time.sleep(1.5)
        record("clear the arm's error", "clean_error, clean_warn, motion_enable(True), set_mode(0), set_state(0)", getattr(api, "error_code", None))
        q, bad, err, h = snapshot()
        if err or bad:
            W["last"] = "✗ The arm still reports %s (joints: %s). Press the E-stop for 10 seconds, release it, and press Send this command again." % (h["error"], h["servo"])
            return None
        W["last"] = "✓ The arm has no errors and its motors are on."
        return "zero_ok"

    def step_zero_ok():
        a = reading()
        j = joint()
        if a is None:
            return ("Is the zero right?", "The arm gives no angle for joint %d yet. Press Back and clear the arm's error first." % j, ("back",))
        return ("Is the zero right?",
                "Joint %d now reads %+.1f°.\n"
                "Open UFACTORY Studio and look at its 3D picture of the arm: Studio draws it from these readings. "
                "Does joint %d in Studio's picture sit like the real arm? (Look at your tape line and photo too.)\n"
                "Press Yes if they match: the zero is right. Press No if the real joint is turned differently: its zero must be set again." % (j, a, j), ("yes", "no"))

    def act_zero_ok(b):
        return "done" if b == "yes" else "zero_intro"

    def step_zero_intro():
        j = joint()
        return ("Set the zero again",
                "You will put joint %d at its true zero and tell the arm \"this is 0°\".\n"
                "Its true zero is where it sits when the whole arm is in its zero pose (every joint at 0°: see Zero position in the xArm user manual).\n"
                "Is joint %d already exactly at its true zero (for example, you moved it there with UFACTORY Studio's joint controls)?\n"
                "Yes: go on without unlocking it. No: you will unlock it and turn it by hand." % (j, j), ("yes", "no"))

    def act_zero_intro(b):
        return "estop2" if b == "yes" else "unlock"

    def step_unlock():
        j = joint()
        warn = ("\nJoint %d holds the arm up: when it is unlocked, the arm FALLS. Before you press Send, someone must hold the arm firmly." % j
                if j in carries() else "\nJoint %d does not hold the arm up: it turns freely once unlocked, the arm does not fall." % j)
        return ("Unlock the joint",
                "This unlocks joint %d (H101 D0104 V1 I%d): its motor lets go, so you can turn it by hand.%s\nPress Send this command." % (j, j, warn), ("send",))

    def act_unlock(b):
        j = joint()
        ensure(3)
        code = api.set_servo_addr_16(servo_id=j, addr=UNLOCK_REG, value=1)
        record("H101 D0104 V1 I%d" % j, "set_servo_addr_16(%d, 0x0104, 1)" % j, code)
        if code != 0:
            W["last"] = "✗ The arm answered code %s: the joint may still be locked. Do not force it." % code
            return None
        W["last"] = "✓ Joint %d is unlocked." % j
        return "turn"

    def step_turn():
        j = joint()
        return ("Turn it to its zero",
                "Turn joint %d slowly by hand to its true zero. Its reading is under Joints below (J%d), live.\n"
                "Hold it there and press Done, next." % (j, j), ("done",))

    def act_turn(b):
        return "estop2"

    def step_estop2():
        return ("E-stop, to lock the joint", estop_text("(The joint locks where it is. Keep holding it until the button is down.)"), ("done",))

    def act_estop2(b):
        if not after_estop():
            W["last"] = "✗ The arm does not answer yet. Check that the E-stop is released, wait 10 seconds and press Done, next again."
            return None
        W["last"] = "✓ The arm answers again."
        return "send_zero"

    def step_send_zero():
        j = joint()
        return ("Make this position 0°",
                "This tells joint %d that where it is now is 0° (D13 I%d, sent as the SDK's set_servo_zero(%d)). Nothing moves.\n"
                "Press Send this command." % (j, j, j), ("send",))

    def act_send_zero(b):
        j = joint()
        ensure(3)
        code = api.set_servo_zero(servo_id=j)
        record("D13 I%d" % j, "set_servo_zero(%d)" % j, code)
        if code != 0:
            W["last"] = "✗ The arm answered code %s: it did not take it." % code
            return None
        W["last"] = "✓ Sent D13 I%d: the arm accepted it (answer 0)." % j
        return "estop3"

    def step_estop3():
        return ("E-stop, so the new zero takes effect", estop_text(""), ("done",))

    def act_estop3(b):
        if not after_estop():
            W["last"] = "✗ The arm does not answer yet. Check that the E-stop is released, wait 10 seconds and press Done, next again."
            return None
        W["last"] = "✓ The arm answers again."
        return "clear_arm"

    def step_done():
        return ("Finished",
                "✓ The arm has no errors and joint %d's zero is right.\n"
                "Press Finish to end. Then move the arm slowly once with UFACTORY Studio to check it." % joint(), ("finish",))

    STEPS = {"read": (step_read, act_read), "mark": (step_mark, None), "send_clear": (step_send_clear, act_send_clear),
             "estop1": (step_estop1, act_estop1), "retry": (step_retry, None), "clear_arm": (step_clear_arm, act_clear_arm),
             "zero_ok": (step_zero_ok, act_zero_ok), "zero_intro": (step_zero_intro, act_zero_intro), "unlock": (step_unlock, act_unlock),
             "turn": (step_turn, act_turn), "estop2": (step_estop2, act_estop2), "send_zero": (step_send_zero, act_send_zero),
             "estop3": (step_estop3, act_estop3), "done": (step_done, None)}
    NEXT = {"mark": "send_clear"}
    LABEL = {"done": "Done, next", "send": "Send this command", "yes": "Yes", "no": "No", "back": "Back", "finish": "Finish"}

    CHOICES = {"read": ("joint",), "send_clear": ("command",), "retry": ("command",)}

    def choices():
        out = []
        for c in CHOICES.get(W["step"], ()):
            if c == "joint":
                out.append({"name": "joint", "label": "Joint", "options": ["auto"] + [str(i) for i in range(1, axes() + 1)],
                            "value": str(W["chosen"]) if W["chosen"] else "auto"})
            else:
                out.append({"name": "command", "label": "Clear command", "options": ["V2", "V256"], "value": "V%d" % W["value"]})
        return out

    def show(busy=None):
        title, text, takes = STEPS[W["step"]][0]()
        W["takes"] = takes
        W["seq"] = W.get("seq", 0) + 1
        n = len(W["history"]) + 1
        ctx.telemetry("guide", {"seq": W["seq"], "step": "Step %d · %s" % (n, title), "text": text, "joint": ("J%d" % joint()) if joint() else "not chosen",
                                "press": "Press: " + " or ".join(LABEL[t] for t in takes), "last": W["last"] or "—", "command": command(),
                                "buttons": [{"label": LABEL[t], "kind": "send" if t == "send" else ""} for t in takes if t not in ("back", "finish")],
                                "extra": [l for l in ("Back", "Start over", "Finish") if l != "Back" or W["history"]],
                                "choices": choices(), "busy": busy})
        if not busy:
            ctx.log("[Step %d · %s] %s%s" % (n, title, (W["last"] + " ") if W["last"] else "", text.replace("\n", " ")))

    def go(step, remember=True):
        if remember and W["step"]:
            W["history"].append(W["step"])
        W["step"] = step
        show()

    def press(b):
        if b == "finish":
            raise Stop("Finished.")
        if b == "restart":
            W["history"], W["last"] = [], ""
            W["step"] = None
            go("read")
            return
        if b == "back":
            if W["history"]:
                W["step"] = W["history"].pop()
                W["last"] = ""
                show()
            return
        if b not in W.get("takes", ()):
            W["last"] = "✗ Nothing done: this step needs " + " or ".join(LABEL[t] for t in W["takes"]) + "."
            show()
            return
        W["last"] = ""
        act = STEPS[W["step"]][1]
        if act and (b == "send" or W["step"].startswith("estop")):
            show(busy="Working: talking to the arm… (after an E-stop this can take up to 20 seconds)")
        nxt = act(b) if act else NEXT.get(W["step"])
        if nxt:
            go(nxt)
        else:
            show()

    for f in (ctx.inputs.get("robot") or []):
        ctx.log("Robot model for the 3D view: %s" % f.name)
    go("read", remember=False)

    last_tick = 0.0
    try:
        while time.time() < deadline:
            for m in ctx.messages(channel, wait=0.5):
                d = m.get("data") if isinstance(m.get("data"), dict) else {"control": str(m.get("data")), "value": True}
                c, v = str(d.get("control", "")).strip(), d.get("value")
                cl = c.lower()
                if cl == "joint":
                    W["chosen"] = None if str(v).lower().startswith("auto") else int(str(v).strip()[0])
                    W["last"] = "✓ Joint %s chosen." % (joint() or "none")
                    show()
                elif cl == "command":
                    W["value"] = 256 if "256" in str(v) else 2
                    W["last"] = "✓ The clear command is now %s." % command()
                    show()
                elif cl in BUTTONS:
                    press(BUTTONS[cl])
            if time.time() - last_tick > 1.0:
                last_tick = time.time()
                ensure()
                snapshot()
    except Stop as e:
        ctx.telemetry("guide", {"seq": W.get("seq", 0) + 1, "step": "Finished", "text": str(e) + " Every command sent is in joint_repair.csv (Results).", "press": "—",
                                "last": W["last"] or "—", "buttons": [], "extra": [], "choices": []})
        ctx.log("Finished.")
    finally:
        try:
            api.disconnect()
        except Exception:
            pass
    return {"robot": "rehearsal" if rehearsal else str(p.get("ip")), "joint": joint()}
