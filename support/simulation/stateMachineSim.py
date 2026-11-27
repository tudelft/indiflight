"""Regression scenarios for the catapult, throw-to-arm, learner query and
prober state machines, in the MOCKUP library with the PyNDIflight quad.

    make TARGET=MOCKUP PROFILE=CineRat
    python support/simulation/stateMachineSim.py obj/main/indiflight_MOCKUP.so \\
        configs/profiles/CineRat.txt all

Prints one line per state change and checks the expected behaviour at the
end of each scenario. Exit code 1 if a check fails. Runs headless and faster
than real time (a few seconds per scenario).

Scenarios:
  catapult            catapult + query with board misalignment: misalignment
                      after the launch, kept until LEARNER_MODE off; LEARNER on
                      again in flight starts no new query
  catapult_restart    query DONE, disarm, catapult mode cycled: misalignment
                      undone, query stays DONE (re-arming needs LEARNER toggle)
  throw               throw-to-arm + query
  idle_throw          throw_to_arm_idle_before_throw, LEARNER on: motors idle in
                      hand, then a normal query and position hold
  idle_throw_nolearn  same without LEARNER (no controller may run in hand)
  inflight            in-flight learning without probing (learner_mode_* = 1)

The MOCKUP does not run processRxModes(): flight modes are set directly, so the
LEARNER_MODE-off switch back to INDI profile 0 (core.c) is not exercised here.
"""
import sys, os, ctypes as ct
import numpy as np
from argparse import ArgumentParser

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "../.."))
sys.path.insert(0, os.path.join(ROOT, "support/simulation"))
sys.path.insert(0, os.path.join(ROOT, "src/utils"))

from PyNDIflight.crafts import MultiRotor, IMU
from PyNDIflight.interfaces import IndiflightSITLWrapper
from PyNDIflight.sim import Sim
from indiflight_mockup_interface import flightModeFlags, boxId

QUERY = {-1: "IDLE", 0: "WAIT_LAUNCH", 1: "DELAY", 2: "ACTIVE", 3: "DONE"}
CAT = {-1: "IDLE", 0: "WAIT_ARM", 1: "DELAY", 2: "LAUNCHING", 3: "ROTATING", 4: "DONE"}
THROW = {-1: "IDLE", 0: "WAIT_THROW", 1: "ACC_HIGH", 2: "MOMENTUM", 3: "LEFT_HAND", 4: "THROWN", 5: "ARMED_AFTER"}
THROWN = 4
LEARNED_INDI_PROFILE = 2  # INDI_PROFILE_COUNT - 1

SCENARIOS = {
    "catapult": dict(T=14., cli=["set learner_roll_misalignment = 15", "set learner_yaw_misalignment = -20"]),
    "catapult_restart": dict(T=16., cli=["set learner_roll_misalignment = 15"]),
    "throw": dict(T=10., cli=[]),
    "idle_throw": dict(T=10., cli=["set throw_to_arm_idle_before_throw = 1"]),
    "idle_throw_nolearn": dict(T=10., cli=["set throw_to_arm_idle_before_throw = 1"]),
    "inflight": dict(T=10., cli=["set learner_mode_probing = 0", "set learner_mode_fx = 1",
                                 "set learner_mode_act = 1", "set learner_mode_hover = 1"]),
}


def run(libfile, profile, sc, extra_cli):
    cfg = SCENARIOS[sc]
    mc = MultiRotor()
    mc.setInertia(m=0.41, I=np.diag([0.75e-3, 0.8e-3, 0.9e-3]))
    mc.setRotor(0, X=[-0.05, +0.0635, 0.0], k=1.88e-7, cm=-0.01, wmax=4900., tau=0.02, kESC=0.5, I=5e-7)
    mc.setRotor(1, X=[+0.05, +0.0635, 0.0], k=1.88e-7, cm=+0.01, wmax=4900., tau=0.02, kESC=0.5, I=5e-7)
    mc.setRotor(2, X=[-0.05, -0.0635, 0.0], k=1.88e-7, cm=+0.01, wmax=4900., tau=0.02, kESC=0.5, I=5e-7)
    mc.setRotor(3, X=[+0.05, -0.0635, 0.0], k=1.88e-7, cm=-0.01, wmax=4900., tau=0.02, kESC=0.5, I=5e-7)
    imu = IMU(mc, r=[0., 0., 0.], qBody=[1., 0., 0., 0.], accStd=0., gyroStd=0.)

    sil = IndiflightSITLWrapper(mc, imu, libfile, Nr=mc.Nr, Ns=0)
    m = sil.mockup
    m.load_profile(profile)
    for c in cfg["cli"] + extra_cli:
        m.issueCliCommand(c)
    m.saveConfig()
    m.setLogging(False)

    lib = m.lib
    qs = ct.c_int.in_dll(lib, "learningQueryState")
    cs = ct.c_int.in_dll(lib, "catapultState")
    ts = ct.c_int.in_dll(lib, "throwState")
    board = (ct.c_int32 * 3).in_dll(lib, "boardAlignment_System")
    sysconf = (ct.c_uint8 * 4).in_dll(lib, "systemConfig_System")  # pid, rate, indi, position profile index

    sil.sendMocap()
    m.sendPositionSetpoint([0., 0., -5.0], 0.)
    m.enableFlightMode(flightModeFlags.ANGLE_MODE | flightModeFlags.POSITION_MODE | flightModeFlags.LEARNER_MODE)
    if sc.startswith("catapult"):
        m.enableFlightMode(flightModeFlags.CATAPULT_MODE)
    if "throw" in sc:
        m.enableRxBox(boxId.BOXTHROWTOARM)
        m.enableRxBox(boxId.BOXARM)
    if sc == "idle_throw_nolearn":
        m.disableFlightMode(flightModeFlags.LEARNER_MODE)

    mc.setPose(x=[0., 0., -0.1], q=[1., 0., 0., 0.])
    mc.setTwist(v=[0., 0., 0.], w=[0., 0., 0.])
    sim = Sim(mc, imu, None, None, sil)
    if "throw" in sc:
        mc.throw(height=4., wB=[2., -4., 3.], vHorz=[1., -2.], at_time=4.0)

    dt = 0.000125
    hist = []  # (t, query, cat, throw, armed, board, indiProfile)
    heldNonzero = []
    armedOnce = False
    last = None
    print(f"===== {sc}")
    print(f"{'t':>7} | {'query':>11} {'cat':>9} {'throw':>11} armed board indiProf")
    while sim.t < cfg["T"]:
        t = sim.t
        ev = None
        if not armedOnce and (("throw" not in sc and t > 2.5) or (sc.startswith("idle_throw") and t > 2.0)):
            m.arm(); armedOnce = True; ev = "arm"
        if sc == "catapult":
            if 9.0 < t <= 9.0 + dt:
                m.disableFlightMode(flightModeFlags.LEARNER_MODE); ev = "LEARNER off"
            if 10.0 < t <= 10.0 + dt:
                m.enableFlightMode(flightModeFlags.LEARNER_MODE); ev = "LEARNER on"
        if sc == "catapult_restart":
            if 9.0 < t <= 9.0 + dt:
                m.disarm(); ev = "disarm"
            if 11.0 < t <= 11.0 + dt:
                m.disableFlightMode(flightModeFlags.CATAPULT_MODE); ev = "CATAPULT off"
            if 11.5 < t <= 11.5 + dt:
                mc.setPose(x=[0., 0., -0.1], q=[1., 0., 0., 0.]); mc.setTwist(v=[0., 0., 0.], w=[0., 0., 0.])
                m.enableFlightMode(flightModeFlags.CATAPULT_MODE); ev = "CATAPULT on"
        if ev:
            print(f"{t:7.3f} | >>> {ev}")

        sim.tick(dt)

        armed = bool(m.armingFlags.value & 1)
        if sc.startswith("idle_throw") and armed and ts.value < THROWN:
            cmd = m.getMotorCommands()
            if np.max(np.abs(cmd)) > 1e-6:
                heldNonzero.append(t)
        s = (qs.value, cs.value, ts.value, armed, tuple(board), sysconf[2])
        hist.append((t,) + s)
        if s != last:
            print(f"{t:7.3f} | {QUERY.get(s[0], s[0]):>11} {CAT.get(s[1], s[1]):>9} {THROW.get(s[2], s[2]):>11} "
                  f"{int(armed)} {s[4]} {s[5]}   pos=({mc.xI[0]:.1f},{mc.xI[1]:.1f},{mc.xI[2]:.1f})")
            last = s

    # --- checks
    fails = []
    def check(cond, msg):
        print(f"  [{'ok' if cond else 'FAIL'}] {msg}")
        if not cond:
            fails.append(msg)
    at = lambda T: next(h for h in hist if h[0] >= T)
    speed = float(np.linalg.norm(mc.vI))
    print(f"final pos {np.round(mc.xI, 2)}, speed {speed:.2f} m/s")

    if sc not in ("idle_throw_nolearn", "inflight"):
        check(any(h[1] == 3 for h in hist), "query reaches DONE")
    if sc == "inflight":
        check(all(h[1] == -1 for h in hist), "no query without probing")
        check(at(cfg["T"] - 1)[6] == LEARNED_INDI_PROFILE, "learned INDI profile applied in flight")
    if sc == "catapult":
        tq = next(h[0] for h in hist if h[1] == 1)
        check(at(tq)[5] != (0, 0, 0), "misaligned when the query starts (after the launch)")
        check(all(h[5] == (0, 0, 0) for h in hist if h[2] < 4), "not misaligned before the catapult is done")
        check(at(8.9)[5] != (0, 0, 0) and at(8.9)[1] == 3, "misalignment kept after the query")
        check(at(9.5)[5] == (0, 0, 0) and at(9.5)[1] == -1, "LEARNER off: misalignment undone, query IDLE")
        check(all(h[1] == -1 for h in hist if h[0] > 9.5), "LEARNER on in flight: no new query")
    if sc == "catapult_restart":
        check(at(10.9)[5] != (0, 0, 0), "misalignment kept after disarm")
        check(at(11.2)[5] == (0, 0, 0), "catapult restart undoes the misalignment")
        check(all(h[1] == 3 for h in hist if h[0] > 4.6), "query stays DONE (no re-arm without LEARNER toggle)")
    if "throw" in sc:
        check(any(h[3] == 5 for h in hist), "armed after throw")
    if sc.startswith("idle_throw"):
        # the MOCKUP arms by writing armingFlags, the first ticks return the disarmed command
        tArm = next(h[0] for h in hist if h[4])
        late = [t for t in heldNonzero if t > tArm + 0.001]
        check(not late, f"motors idle while held in hand ({len(heldNonzero)} non-zero ticks, {len(late)} after the arm edge)")
    if sc != "catapult_restart":
        check(speed < 0.5 and mc.xI[2] < -1., "holds position in the air at the end")
    return fails


if __name__ == "__main__":
    p = ArgumentParser()
    p.add_argument("lib")
    p.add_argument("profile")
    p.add_argument("scenario", choices=list(SCENARIOS) + ["all"])
    p.add_argument("--cli", action="append", default=[], help="extra CLI command, after the scenario's")
    args = p.parse_args()

    if args.scenario != "all":
        sys.exit(1 if run(args.lib, args.profile, args.scenario, args.cli) else 0)

    # one process per scenario: the library keeps its state between runs
    import subprocess
    failed = []
    for sc in SCENARIOS:
        cmd = [sys.executable, "-u", __file__, args.lib, args.profile, sc]
        for c in args.cli:
            cmd += ["--cli", c]
        if subprocess.run(cmd).returncode != 0:
            failed.append(sc)
    print("\nSUMMARY: " + ("all checks passed" if not failed else f"FAILED: {failed}"))
    sys.exit(1 if failed else 0)
