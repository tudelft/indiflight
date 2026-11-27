#!/usr/bin/env python3
"""Local build daemon for Indiflight.

Serves a small HTTP API on localhost so the web configurator can list
board/profile configs and compile this working tree inside the
indiflight-builder Docker image, or run a SIL simulation of it
(support/simulation, pyndiflight-sil image). Standard library only.

    python3 start.py [--host 127.0.0.1] [--port 8742] [--no-token] [--image indiflight-builder]
"""

import argparse
import datetime
import glob
import hashlib
import json
import os
import platform
import re
import secrets
import shutil
import signal
import stat
import subprocess
import sys
import tempfile
import threading
import time
import urllib.parse
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

SERVICE = "indiflight-builder"
API_VERSION = 1
HASH_LABEL = "indiflight.builder.hash"
CUSTOM_NAME = "__custom__"
MAX_JOBS = 20
MAX_BODY = 1024 * 1024
KEEPALIVE_S = 15
PROGRESS_INTERVAL_S = 0.1
NAME_RE = re.compile(r"^[A-Za-z0-9][A-Za-z0-9._-]*$")

# Browser origins allowed to use the daemon (CORS). Anything else is refused outright,
# because a build runs make, i.e. code, with this working tree mounted read-write.
DEFAULT_ORIGINS = ["https://app.indiflight.org"]
# local dev servers (vite) and the Tauri desktop build
LOCAL_ORIGIN_RE = re.compile(r"^(https?://(localhost|127\.0\.0\.1|\[::1\])(:\d+)?|tauri://localhost|https?://tauri\.localhost)$")

# A define as make passes it to the compiler. The Makefile wraps it in single quotes and
# EXTRA_FLAGS goes into the compile command unquoted, so anything beyond this reaches a shell.
DEFINE_TOKEN_RE = re.compile(r"^[A-Za-z_][A-Za-z0-9_]*(=[A-Za-z0-9_.+:/-]*)?$")

REPO = os.path.dirname(os.path.abspath(__file__))
SIM_DIR = os.path.join(REPO, "support", "simulation")
CONFIG_DIRS = {
    "boards": os.path.join(REPO, "configs", "boards"),
    "profiles": os.path.join(REPO, "configs", "profiles"),
    "sims": SIM_DIR,
}
CONFIG_EXT = {"boards": ".txt", "profiles": ".txt", "sims": ".py"}
# support/simulation/entrypoint.sh passes this option; scripts without it are helpers, not sims
SIM_MARKER = "--sil-profile-txt"
# Flask visualization and gdbserver ports inside the SIL container
VIS_CONTAINER_PORT = 5000
GDB_PORT = 3333
# Sim script arguments go to argparse as an argv list (no shell), kept to plain option syntax
SIM_ARG_RE = re.compile(r"^[A-Za-z0-9_.:/=+,-]+$")
# Set by entrypoint.sh / the daemon itself; a duplicate would make argparse fail or point elsewhere
RESERVED_SIM_ARGS = ("--sil", "--sil-profile-txt", "--hil")
MAX_SIM_ARGS = 32
TOKEN_FILE = os.path.join(os.path.expanduser("~"), ".config", "indiflight-builder", "token")


def native_platform():
    """Docker platform matching the host CPU, so the builder runs without emulation."""
    return "linux/arm64" if platform.machine().lower() in ("arm64", "aarch64") else "linux/amd64"


# Docker --platform for image build and run; set from --platform in main()
DOCKER_PLATFORM = native_platform()


def now_iso():
    return datetime.datetime.now(datetime.timezone.utc).isoformat(timespec="seconds").replace("+00:00", "Z")


def run(cmd, timeout=30):
    """Run a short command, return (returncode, stdout, stderr); rc None if it could not start."""
    try:
        p = subprocess.run(cmd, cwd=REPO, capture_output=True, text=True, timeout=timeout)
        return p.returncode, p.stdout, p.stderr
    except (OSError, subprocess.TimeoutExpired) as e:
        return None, "", str(e)


# --------------------------------------------------------------------------- git

def git_info():
    rc, branch, _ = run(["git", "symbolic-ref", "--quiet", "--short", "HEAD"])
    _, commit, _ = run(["git", "rev-parse", "HEAD"])
    _, describe, _ = run(["git", "describe", "--always", "--tags", "--dirty"])
    _, status, _ = run(["git", "status", "--porcelain", "--untracked-files=no"])
    _, subs, _ = run(["git", "submodule", "status"])
    missing = []
    for line in subs.splitlines():
        # A leading '-' marks a submodule that has not been initialised.
        if line.startswith("-"):
            parts = line[1:].split()
            if len(parts) >= 2:
                missing.append(parts[1])
    return {
        "branch": branch.strip() if rc == 0 and branch.strip() else None,
        "commit": commit.strip(),
        "describe": describe.strip(),
        "dirty": bool(status.strip()),
        "submodulesMissing": missing,
    }


# --------------------------------------------------------------------------- configs

def list_configs(kind):
    d, ext = CONFIG_DIRS[kind], CONFIG_EXT[kind]
    try:
        names = [f[: -len(ext)] for f in os.listdir(d) if f.endswith(ext) and os.path.isfile(os.path.join(d, f))]
    except OSError:
        return []
    names = [n for n in names if n != CUSTOM_NAME and NAME_RE.match(n)]
    if kind == "sims":
        names = [n for n in names if is_sim_script(os.path.join(d, n + ext))]
    return sorted(names)


def is_sim_script(path):
    try:
        with open(path, encoding="utf-8", errors="replace") as f:
            return SIM_MARKER in f.read()
    except OSError:
        return False


def config_path(kind, name):
    """Path of a listed config, or None. Only names from the listing are accepted (no traversal)."""
    if not isinstance(name, str) or name not in list_configs(kind):
        return None
    return os.path.join(CONFIG_DIRS[kind], name + CONFIG_EXT[kind])


# --------------------------------------------------------------------------- docker image

class ImageSpec:
    """A docker image the daemon builds itself, from a Dockerfile and a context dir (relative to REPO)."""

    def __init__(self, name, dockerfile, context=".", base=None):
        self.name = name
        self.dockerfile = dockerfile
        self.context = context
        # Image this one is built FROM: rebuilding the base must invalidate this one too
        self.base = base

    @property
    def context_dir(self):
        return os.path.normpath(os.path.join(REPO, self.context))

    @property
    def dockerfile_rel(self):
        """Dockerfile path relative to the context dir."""
        return os.path.relpath(os.path.join(REPO, self.dockerfile), self.context_dir)


def dockerfile_copy_sources(spec):
    """Source paths of all COPY/ADD instructions in the Dockerfile (relative to the context)."""
    sources = []
    try:
        with open(os.path.join(REPO, spec.dockerfile), encoding="utf-8") as f:
            text = f.read()
    except OSError:
        return sources
    text = re.sub(r"\\\r?\n", " ", text)  # join continuation lines
    for line in text.splitlines():
        m = re.match(r"^\s*(COPY|ADD)\s+(.*)$", line, re.IGNORECASE)
        if not m:
            continue
        rest = m.group(2).strip()
        if rest.startswith("["):
            try:
                args = json.loads(rest)
            except ValueError:
                continue
        else:
            args = rest.split()
        args = [a for a in args if not a.startswith("--")]
        if any(a.startswith("--from") for a in m.group(2).split()):
            continue  # copies from another stage, not from the context
        sources.extend(args[:-1])
    return sources


def context_files(spec):
    """Sorted paths (relative to the context dir) of every context file the image build reads."""
    ctx = spec.context_dir
    files = {spec.dockerfile_rel}
    missing = []
    for src in dockerfile_copy_sources(spec):
        if re.match(r"^[a-z]+://", src):
            missing.append(src)  # remote ADD: hash the URL itself
            continue
        matches = glob.glob(os.path.join(ctx, src))
        if not matches:
            missing.append(src)
        for m in matches:
            if os.path.isdir(m):
                for root, dirs, fnames in os.walk(m):
                    dirs[:] = [d for d in dirs if d != ".git"]
                    for fn in fnames:
                        files.add(os.path.relpath(os.path.join(root, fn), ctx))
            else:
                files.add(os.path.relpath(m, ctx))
    return sorted(files), sorted(missing)


def image_hash(spec):
    files, missing = context_files(spec)
    h = hashlib.sha256()
    # an image built for another platform must not count as up to date
    h.update(b"platform:" + DOCKER_PLATFORM.encode() + b"\0")
    if spec.base is not None:
        h.update(b"base:" + image_hash(spec.base).encode() + b"\0")
    for rel in files:
        h.update(rel.encode() + b"\0")
        try:
            with open(os.path.join(spec.context_dir, rel), "rb") as f:
                h.update(hashlib.sha256(f.read()).digest())
        except OSError:
            h.update(b"<unreadable>")
    for src in missing:
        h.update(b"<missing>" + src.encode() + b"\0")
    return h.hexdigest()[:32]


def docker_status(spec):
    st = {"available": False, "image": spec.name, "exists": False, "upToDate": False, "error": None}
    rc, _, err = run(["docker", "version", "--format", "{{.Server.Version}}"], timeout=15)
    if rc != 0:
        st["error"] = (err.strip() or "docker not found") if rc is not None else (err or "docker not found")
        return st
    st["available"] = True
    rc, out, _ = run(["docker", "image", "inspect", "--format", "{{json .Config.Labels}}", spec.name], timeout=15)
    if rc != 0:
        return st
    st["exists"] = True
    try:
        labels = json.loads(out.strip() or "null") or {}
    except ValueError:
        labels = {}
    st["upToDate"] = labels.get(HASH_LABEL) == image_hash(spec)
    return st


# --------------------------------------------------------------------------- jobs

class Cancelled(Exception):
    pass


FINISHED_STATES = ("success", "failed", "cancelled", "stopped")


class Job:
    kind = "build"
    # Sent to every process in the container except the entrypoint to stop the job (see Builder.cancel)
    stop_signal = "TERM"

    def __init__(self, jid, req, git):
        self.id = jid
        self.board = req.get("board")
        self.profile = req["profile"]
        self.board_text = req.get("boardText")
        self.profile_text = req.get("profileText")
        self.debug = req.get("debug")
        self.extra_flags = req.get("extraFlags") or ""
        self.clean = bool(req.get("clean"))
        self.git = {k: git[k] for k in ("commit", "describe", "dirty")}
        self.state = "queued"
        self.created_at = now_iso()
        self.started_ts = time.time()
        self.finished_at = None
        self.exit_code = None
        self.error = None
        self.artifacts = []  # dicts with name, kind, size, path
        self.done = 0
        self.total = None
        self.last_file = ""
        self.progress_seq = 0
        self.events = []  # (event, data) for log + state, replayed to new subscribers
        self.cond = threading.Condition()
        self.proc = None
        self.cancel_requested = False
        self.container = "indiflight-%s-%s" % (self.kind, jid)
        self.workdir = tempfile.mkdtemp(prefix="indiflight-job-%s-" % jid)

    @property
    def finished(self):
        return self.state in FINISHED_STATES

    def summary(self):
        return {
            "id": self.id,
            "kind": self.kind,
            "state": self.state,
            "board": self.board,
            "profile": self.profile,
            "custom": {"board": self.board_text is not None, "profile": self.profile_text is not None},
            "git": dict(self.git),
            "createdAt": self.created_at,
            "finishedAt": self.finished_at,
            "exitCode": self.exit_code,
            "progress": {"done": self.done, "total": self.total},
            "artifacts": [{k: a[k] for k in ("name", "kind", "size")} for a in self.artifacts],
            "error": self.error,
        }

    def log(self, line, stream="meta"):
        with self.cond:
            self.events.append(("log", {"line": line, "stream": stream}))
            self.cond.notify_all()

    def set_state(self, state):
        with self.cond:
            self.state = state
            if self.finished:
                self.finished_at = now_iso()
            self.events.append(("state", self.summary()))
            self.cond.notify_all()

    def progress(self, done, total, file):
        with self.cond:
            self.done, self.total, self.last_file = done, total, file
            self.progress_seq += 1
            self.cond.notify_all()


class SimJob(Job):
    """A SIL simulation: the MOCKUP build plus a PyNDIflight script, running until stopped."""

    kind = "sim"
    # KeyboardInterrupt in the sim script: Python exits normally, so the SIL library's
    # blackbox log gets flushed. SIGTERM would kill it with the last log blocks still buffered.
    stop_signal = "INT"

    def __init__(self, jid, req, git):
        super().__init__(jid, req, git)
        self.sim = req["sim"]
        self.sim_text = req.get("simText")
        self.args = list(req.get("args") or [])
        self.gdbserver = bool(req.get("gdbserver"))
        self.host_network = bool(req.get("hostNetwork"))
        self.vis_port = req["visPort"]
        self.vis_ready = False
        self.status = ""  # latest tqdm status line

    def summary(self):
        s = super().summary()
        s["custom"]["sim"] = self.sim_text is not None
        s.update({
            "sim": self.sim,
            "args": list(self.args),
            "visPort": self.vis_port,
            "visReady": self.vis_ready,
            "gdbPort": GDB_PORT if self.gdbserver else None,
            "status": self.status,
        })
        return s


class Builder:
    def __init__(self, image, sil_image):
        self.image = image  # ImageSpec of the firmware builder
        self.sil_image = sil_image  # ImageSpec of the PyNDIflight SIL runner (FROM the builder)
        self.lock = threading.Lock()
        self.jobs = {}  # insertion ordered, oldest first
        self.active = None

    def summaries(self):
        with self.lock:
            jobs = list(self.jobs.values())
        return [j.summary() for j in reversed(jobs)]

    def get(self, jid):
        with self.lock:
            return self.jobs.get(jid)

    def start(self, req, job_cls=Job):
        created = now_iso()
        if self.active is not None:
            return None
        git = git_info()  # takes ~1-2 s on large trees; keep it outside the lock
        with self.lock:
            if self.active is not None:
                return None
            jid = time.strftime("%Y%m%d-%H%M%S") + "-" + secrets.token_hex(2)
            job = job_cls(jid, req, git)
            job.created_at = created
            self.jobs[jid] = job
            self.active = job
            while len(self.jobs) > MAX_JOBS:
                old = next(iter(self.jobs.values()))
                if old is job or not old.finished:
                    break
                del self.jobs[old.id]
                shutil.rmtree(old.workdir, ignore_errors=True)
        threading.Thread(target=self._run, args=(job,), daemon=True).start()
        return job

    def cancel(self, job):
        with job.cond:
            if job.finished:
                return
            job.cancel_requested = True
        # Signal make and the compilers first (everything except the entrypoint,
        # PID 1): on SIGTERM make deletes half-written objects. A plain `docker kill`
        # SIGKILLs them, leaving truncated .o files newer than their sources that
        # break the next incremental build with undefined references.
        # Retried because right after `docker run` starts the container may not exist yet.
        # SIGINT is delivered only once: a second KeyboardInterrupt would abort the sim's cleanup.
        start = time.time()
        signalled = False
        while time.time() - start < 25:
            proc = job.proc
            if proc is None or proc.poll() is not None:
                break
            if time.time() - start < 10:
                if not signalled or job.stop_signal != "INT":
                    rc, _, _ = run(["docker", "exec", job.container, "bash", "-c", "kill -%s -1" % job.stop_signal],
                                   timeout=10)
                    signalled = rc == 0
                if job.state == "image":
                    proc.terminate()
            else:
                run(["docker", "kill", job.container], timeout=10)
                try:
                    proc.terminate()
                except OSError:
                    pass
            time.sleep(1.0)

    def shutdown(self):
        with self.lock:
            job = self.active
            jobs = list(self.jobs.values())
        if job is not None:
            print("Stopping running %s %s ..." % (job.kind, job.id), flush=True)
            self.cancel(job)
            deadline = time.time() + 20
            while not job.finished and time.time() < deadline:
                time.sleep(0.2)
        for j in jobs:
            shutil.rmtree(j.workdir, ignore_errors=True)

    # ------------------------------------------------------------------ job run

    def _run(self, job):
        if isinstance(job, SimJob):
            return self._run_sim(job)
        try:
            self._run_phases(job)
            job.exit_code = 0
            self._collect_artifacts(job)
            # entrypoint.sh on older branches exits 0 even when make fails
            if not any(a["kind"] == "hex" for a in job.artifacts):
                raise RuntimeError("make exited 0 but produced no .hex")
            job.log("Build finished successfully")
            job.set_state("success")
        except Cancelled:
            job.error = "Cancelled"
            job.log("Build cancelled")
            job.set_state("cancelled")
        except Exception as e:  # noqa: BLE001 - any failure ends the job
            if job.cancel_requested:
                job.error = "Cancelled"
                job.set_state("cancelled")
            else:
                job.error = str(e)
                job.log("ERROR: %s" % e)
                job.set_state("failed")
        finally:
            with self.lock:
                if self.active is job:
                    self.active = None

    def _check_cancel(self, job):
        if job.cancel_requested:
            raise Cancelled()

    def _run_phases(self, job):
        job.log("Repository: %s (%s%s)" % (REPO, job.git["describe"], ", dirty" if job.git["dirty"] else ""))
        self._ensure_image(job, self.image)

        mounts = self._config_mounts(job)
        board = CUSTOM_NAME if job.board_text is not None else job.board
        profile = CUSTOM_NAME if job.profile_text is not None else job.profile

        if job.clean:
            job.set_state("cleaning")
            rc = self._docker_run(job, board, profile, mounts, ["clean"])
            self._check_cancel(job)
            if rc != 0:
                job.exit_code = rc
                raise RuntimeError("make clean failed (exit %d)" % rc)

        job.set_state("building")
        # YES=y: the default 'hex' goal depends on check_dirty, which otherwise
        # prompts on stdin (dirty tree or uncleaned obj dir) and aborts without a tty.
        rc = self._docker_run(job, board, profile, mounts, ["PROGRESS=y", "YES=y"])
        self._check_cancel(job)
        if rc != 0:
            job.exit_code = rc
            raise RuntimeError("Build failed (exit %d)" % rc)

    def _config_mounts(self, job):
        """Bind mounts for custom config text.

        The whole configs/<kind> directory is replaced by a temp copy that also
        contains __custom__.txt. Mounting a single file at a path that does not
        exist in the bind-mounted repo would make Docker create an empty
        placeholder file in the user's working tree.
        """
        mounts = []
        for kind, text in (("boards", job.board_text), ("profiles", job.profile_text)):
            if text is None:
                continue
            d = os.path.join(job.workdir, "configs", kind)
            os.makedirs(d, exist_ok=True)
            for name in list_configs(kind):
                shutil.copy2(os.path.join(CONFIG_DIRS[kind], name + ".txt"), d)
            with open(os.path.join(d, CUSTOM_NAME + ".txt"), "w", encoding="utf-8", newline="\n") as f:
                f.write(text if text.endswith("\n") else text + "\n")
            mounts += ["--mount", "type=bind,src=%s,dst=/indiflight/configs/%s,readonly" % (d, kind)]
        return mounts

    def _ensure_image(self, job, spec):
        """Build spec (and first its base) unless the image is up to date."""
        if spec.base is not None:
            self._ensure_image(job, spec.base)
        st = docker_status(spec)
        if not st["available"]:
            raise RuntimeError("Docker is not available: %s" % st["error"])
        self._check_cancel(job)
        if not st["upToDate"]:
            job.set_state("image")
            job.log("Building docker image %s (%s) ..." % (spec.name, "outdated" if st["exists"] else "missing"))
            self._build_image(job, spec)
        else:
            job.log("Docker image %s is up to date" % spec.name)
        self._check_cancel(job)

    def _build_image(self, job, spec):
        h = image_hash(spec)
        files, missing = context_files(spec)
        if missing:
            job.log("Warning: Dockerfile sources not found locally: %s" % ", ".join(missing))
        # Minimal build context: only the files the Dockerfile reads, instead of
        # sending the whole repository (.git, obj, logs) to the daemon.
        ctx = os.path.join(job.workdir, "image-context-" + spec.name)
        for rel in files:
            dst = os.path.join(ctx, rel)
            os.makedirs(os.path.dirname(dst), exist_ok=True)
            shutil.copy2(os.path.join(spec.context_dir, rel), dst)
        cmd = ["docker", "build", "--platform", DOCKER_PLATFORM, "--progress=plain", "--label", "%s=%s" % (HASH_LABEL, h),
               "-t", spec.name, "-f", os.path.join(ctx, spec.dockerfile_rel), ctx]
        rc = self._stream(job, cmd, parse_progress=False)
        shutil.rmtree(ctx, ignore_errors=True)
        self._check_cancel(job)
        if rc != 0:
            job.exit_code = rc
            raise RuntimeError("docker build failed (exit %d)" % rc)

    def _docker_run(self, job, board, profile, mounts, make_args):
        cmd = ["docker", "run", "--rm", "--platform", DOCKER_PLATFORM, "--name", job.container,
               "-v", "%s:/indiflight" % REPO] + mounts
        # PROFILE is always set: entrypoint.sh reads an empty one as "no profile" and only
        # falls back to .vscode/build_selection.json when it is unset
        cmd += ["-e", "BOARD=%s" % board, "-e", "PROFILE=%s" % (profile or "")]
        if job.debug:
            cmd += ["-e", "DEBUG=%s" % job.debug]
        if job.extra_flags:
            cmd += ["-e", "EXTRA_FLAGS=%s" % job.extra_flags]
        cmd += [self.image.name] + make_args
        return self._stream(job, cmd, parse_progress=True)

    def _stream(self, job, cmd, parse_progress):
        job.log("$ " + " ".join(cmd))
        with job.cond:
            self._check_cancel(job)
            job.proc = subprocess.Popen(cmd, cwd=REPO, stdin=subprocess.DEVNULL,
                                        stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        compiled = [0]  # %% lines since the last regular line, summarised into one log line
        out_lock = threading.Lock()

        def flush_compiled():
            if compiled[0]:
                job.log("[compiled %d file%s]" % (compiled[0], "" if compiled[0] == 1 else "s"))
                compiled[0] = 0

        def reader(pipe, stream):
            for raw in iter(pipe.readline, b""):
                line = raw.decode("utf-8", "replace").rstrip("\r\n")
                with out_lock:
                    if parse_progress and line.startswith("%%TOTAL "):
                        try:
                            total = int(line.split()[1])
                        except (IndexError, ValueError):
                            total = None
                        job.progress(job.done, total, job.last_file)
                        continue
                    if parse_progress and line.startswith("%% "):
                        f = line.split()[-1]
                        f = f[2:] if f.startswith("./") else f
                        compiled[0] += 1
                        job.progress(job.done + 1, job.total, f)
                        continue
                    flush_compiled()
                    job.log(line, stream)
            pipe.close()

        threads = [threading.Thread(target=reader, args=(job.proc.stdout, "stdout"), daemon=True),
                   threading.Thread(target=reader, args=(job.proc.stderr, "stderr"), daemon=True)]
        for t in threads:
            t.start()
        rc = job.proc.wait()
        for t in threads:
            t.join()
        with out_lock:
            flush_compiled()
        job.proc = None
        return rc

    # ------------------------------------------------------------------ simulation

    def _run_sim(self, job):
        state = "failed"
        try:
            rc = self._run_sim_phases(job)
            job.exit_code = rc
            if job.cancel_requested:
                job.log("Simulation stopped")
                state = "stopped"
            elif rc == 0:
                job.log("Simulation finished")
                state = "success"
            else:
                raise RuntimeError("Simulation exited with code %d" % rc)
        except Cancelled:
            job.error = "Cancelled"
            job.log("Simulation cancelled")
            state = "cancelled"
        except Exception as e:  # noqa: BLE001 - any failure ends the job
            if job.cancel_requested:
                state = "stopped"
            else:
                job.error = str(e)
                job.log("ERROR: %s" % e)
        try:
            job.vis_ready = False
            # Before the final state: the "end" event must already list the logs.
            # Logs written up to a crash or stop are still useful.
            try:
                self._collect_sim_logs(job)
            except OSError as e:
                job.log("Could not collect logs: %s" % e)
            job.set_state(state)
        finally:
            with self.lock:
                if self.active is job:
                    self.active = None

    def _run_sim_phases(self, job):
        job.log("Repository: %s (%s%s)" % (REPO, job.git["describe"], ", dirty" if job.git["dirty"] else ""))
        self._ensure_image(job, self.sil_image)

        mounts = self._config_mounts(job)
        profile = CUSTOM_NAME if job.profile_text is not None else job.profile
        sim = job.sim
        env = ["-e", "YES=y", "-e", "PROGRESS=y", "-e", "PYTHONUNBUFFERED=1",
               "-e", "PROFILE=%s" % profile, "-e", "GDBSERVER=%s" % ("y" if job.gdbserver else "n")]
        if job.sim_text is not None:
            # entrypoint.sh runs ./support/simulation/${SIM}.py. Mounting a single file there would
            # make Docker create an empty placeholder in the user's tree, so the script lives in
            # /sim-custom and SIM walks up to it; PYTHONPATH keeps `import PyNDIflight` working.
            d = os.path.join(job.workdir, "sim")
            os.makedirs(d, exist_ok=True)
            with open(os.path.join(d, CUSTOM_NAME + ".py"), "w", encoding="utf-8", newline="\n") as f:
                f.write(job.sim_text)
            mounts += ["--mount", "type=bind,src=%s,dst=/sim-custom,readonly" % d]
            sim = "../../../sim-custom/" + CUSTOM_NAME
            env += ["-e", "PYTHONPATH=/indiflight/support/simulation"]
        env += ["-e", "SIM=%s" % sim]
        if job.debug:
            env += ["-e", "DEBUG=%s" % job.debug]

        if job.host_network:
            ports = ["--network", "host"]
        else:
            # loopback only: the visualization and gdbserver have no authentication
            ports = ["-p", "127.0.0.1:%d:%d" % (job.vis_port, VIS_CONTAINER_PORT)]
            if job.gdbserver:
                ports += ["-p", "127.0.0.1:%d:%d" % (GDB_PORT, GDB_PORT)]

        cmd = (["docker", "run", "--rm", "--platform", DOCKER_PLATFORM, "--name", job.container,
                "-v", "%s:/indiflight" % REPO] + mounts + env + ports + [self.sil_image.name] + job.args)
        job.set_state("building")
        threading.Thread(target=self._watch_vis, args=(job,), daemon=True).start()
        return self._stream_sim(job, cmd)

    def _watch_vis(self, job):
        """Flag the job once the visualization answers. docker-proxy accepts connections before
        Flask listens and then resets them, so only a real HTTP response counts."""
        import urllib.request
        opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))
        url = "http://127.0.0.1:%d/" % job.vis_port
        while not job.finished and not job.cancel_requested:
            if job.state in ("building", "running"):
                try:
                    with opener.open(url, timeout=1) as r:
                        if r.status == 200:
                            job.vis_ready = True
                            job.log("Visualization ready on %s" % url)
                            job.set_state("running")
                            return
                except OSError:
                    pass
            time.sleep(0.5)

    def _stream_sim(self, job, cmd):
        """Like _stream, but tqdm's carriage-return redraws become status updates, not log lines."""
        job.log("$ " + " ".join(cmd))
        with job.cond:
            self._check_cancel(job)
            job.proc = subprocess.Popen(cmd, cwd=REPO, stdin=subprocess.DEVNULL,
                                        stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        compiled = [0]
        out_lock = threading.Lock()
        ansi_re = re.compile(r"\x1b\[[0-9;?]*[A-Za-z]")
        iter_re = re.compile(r"(\d+)/(\d+)")
        last_status = [0.0]

        def flush_compiled():
            if compiled[0]:
                job.log("[compiled %d file%s]" % (compiled[0], "" if compiled[0] == 1 else "s"))
                compiled[0] = 0

        def on_status(text):
            now = time.time()
            if now - last_status[0] < PROGRESS_INTERVAL_S:
                return
            last_status[0] = now
            m = iter_re.search(text)
            with job.cond:
                job.status = text
            if job.state == "building":
                job.set_state("running")
            if m:
                job.progress(int(m.group(1)), int(m.group(2)), text)
            else:
                job.progress(job.done, job.total, text)

        def on_line(line, stream):
            if line.startswith("%%TOTAL "):
                return
            if line.startswith("%% "):
                compiled[0] += 1
                return
            flush_compiled()
            job.log(line, stream)

        def reader(pipe, stream):
            fd = pipe.fileno()
            buf = b""
            while True:
                chunk = os.read(fd, 65536)
                if chunk:
                    buf += chunk
                while True:
                    i = min((k for k in (buf.find(b"\r"), buf.find(b"\n")) if k >= 0), default=-1)
                    if i < 0 or (buf[i:i + 1] == b"\r" and i == len(buf) - 1 and chunk):
                        break  # no terminator yet, or a \r that may be the first half of \r\n
                    term = buf[i:i + 1]
                    seg = ansi_re.sub("", buf[:i].decode("utf-8", "replace"))
                    if term == b"\r" and buf[i + 1:i + 2] == b"\n":
                        term, i = b"\n", i + 1
                    buf = buf[i + 1:]
                    with out_lock:
                        if term == b"\r":
                            if seg.strip():
                                on_status(seg.strip())
                        else:
                            on_line(seg, stream)
                if not chunk:
                    break
            if buf:
                with out_lock:
                    on_line(ansi_re.sub("", buf.decode("utf-8", "replace")), stream)
            pipe.close()

        threads = [threading.Thread(target=reader, args=(job.proc.stdout, "stdout"), daemon=True),
                   threading.Thread(target=reader, args=(job.proc.stderr, "stderr"), daemon=True)]
        for t in threads:
            t.start()
        rc = job.proc.wait()
        for t in threads:
            t.join()
        with out_lock:
            flush_compiled()
        job.proc = None
        return rc

    def _collect_sim_logs(self, job):
        """Blackbox logs the SIL wrote into ./logs-mockup during this job.

        The sim script can write to the repo, so links are not followed: a planted
        logs-mockup/X -> /home/<user>/... would otherwise be served as an artifact.
        """
        d = os.path.join(REPO, "logs-mockup")
        if os.path.islink(d) or not os.path.isdir(d):
            return
        outdir = os.path.join(job.workdir, "artifacts")
        os.makedirs(outdir, exist_ok=True)
        known = {a["name"] for a in job.artifacts}
        for name in sorted(os.listdir(d)):
            if name in known or not NAME_RE.match(name):
                continue
            try:
                fd = os.open(os.path.join(d, name), os.O_RDONLY | getattr(os, "O_NOFOLLOW", 0))
            except OSError:
                continue  # symlink (ELOOP), vanished, or unreadable
            with os.fdopen(fd, "rb") as src:
                st = os.fstat(src.fileno())
                if not stat.S_ISREG(st.st_mode) or st.st_mtime < job.started_ts - 2:
                    continue
                dst = os.path.join(outdir, name)
                with open(dst, "wb") as out:
                    shutil.copyfileobj(src, out)
            kind = os.path.splitext(name)[1][1:].lower() or "file"
            job.artifacts.append({"name": name, "kind": kind, "size": os.path.getsize(dst), "path": dst})
            job.log("Log: logs-mockup/%s (%d bytes)" % (name, os.path.getsize(dst)))

    def _collect_artifacts(self, job):
        """Copy this build's outputs into the job dir (obj/ file names don't depend on board/profile,
        so a later build would overwrite them)."""
        paths = []
        hex_re = re.compile(r"^Creating (?:HEX|BIN|DFU|EXST) (\S+)")
        uptodate_re = re.compile(r"'(\S+\.(?:hex|bin|dfu))' is up to date")
        with job.cond:
            lines = [d["line"] for e, d in job.events if e == "log" and d["stream"] == "stdout"]
        for line in lines:
            m = hex_re.search(line) or uptodate_re.search(line)
            if m:
                paths.append(m.group(1))
        # Fallback: anything written into obj/ during this job.
        for pat in ("obj/*.hex", "obj/*.bin", "obj/*.dfu", "obj/main/*.elf"):
            for p in glob.glob(os.path.join(REPO, pat)):
                try:
                    if os.path.getmtime(p) >= job.started_ts - 2:
                        paths.append(p)
                except OSError:
                    pass

        resolved = []
        for p in paths:
            if p.startswith("/indiflight/"):
                p = p[len("/indiflight/"):]
            p = os.path.realpath(os.path.join(REPO, p))
            if p.startswith(REPO + os.sep) and os.path.isfile(p) and p not in resolved:
                resolved.append(p)
        # The ELF belonging to a hex: obj/indiflight_<ver>_<TARGET>.hex -> obj/main/indiflight_<TARGET>.elf
        for p in list(resolved):
            m = re.match(r"^indiflight_[0-9.]+_(.+)\.hex$", os.path.basename(p))
            if m:
                elf = os.path.join(REPO, "obj", "main", "indiflight_%s.elf" % m.group(1))
                if os.path.isfile(elf) and elf not in resolved:
                    resolved.append(elf)

        outdir = os.path.join(job.workdir, "artifacts")
        os.makedirs(outdir, exist_ok=True)
        for p in resolved:
            kind = os.path.splitext(p)[1][1:]
            if kind not in ("hex", "bin", "dfu", "elf"):
                continue
            name = os.path.basename(p)
            dst = os.path.join(outdir, name)
            shutil.copyfile(p, dst)
            job.artifacts.append({"name": name, "kind": kind, "size": os.path.getsize(dst), "path": dst})
            job.log("Artifact: %s (%d bytes)" % (os.path.relpath(p, REPO), os.path.getsize(dst)))


# --------------------------------------------------------------------------- HTTP

class ApiError(Exception):
    def __init__(self, status, message):
        super().__init__(message)
        self.status = status
        self.message = message


def check_custom_defines(text, field):
    """Reject a submitted board/profile text whose #define tokens could escape make's quoting."""
    for n, line in enumerate(text.splitlines(), 1):
        if not line.startswith("#define"):  # make greps '^#define'
            continue
        fields = line.split()
        # awk's $2 is what make passes on; a line like "#defineX" has fields[0] != "#define"
        token = fields[1] if len(fields) > 1 and fields[0] == "#define" else None
        if token is None or not DEFINE_TOKEN_RE.match(token):
            raise ApiError(400, "%s line %d: unsupported #define %r (expected NAME or NAME=value "
                                "with letters, digits and _ . + : / -)" % (field, n, line.strip()))


class Handler(BaseHTTPRequestHandler):
    server_version = "indiflight-builder/1"
    protocol_version = "HTTP/1.1"

    # set in main()
    builder = None
    token = None
    allowed_hosts = None
    allowed_origins = set(DEFAULT_ORIGINS)

    def log_message(self, fmt, *args):
        pass

    # ---------------------------------------------------------------- plumbing

    def _origin_allowed(self):
        origin = self.headers.get("Origin")
        if origin is None:
            # Browsers send Origin on every cross-origin fetch, EventSource and POST. Without one,
            # refuse what the browser still marks as cross-site (e.g. <img>/<script> loads).
            return self.headers.get("Sec-Fetch-Site", "none") in ("none", "same-origin")
        return origin in self.allowed_origins or bool(LOCAL_ORIGIN_RE.match(origin))

    def _cors(self):
        origin = self.headers.get("Origin")
        if origin and self._origin_allowed():
            self.send_header("Access-Control-Allow-Origin", origin)
            self.send_header("Vary", "Origin")
            self.send_header("Access-Control-Allow-Headers", "Authorization, Content-Type")
            self.send_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
            self.send_header("Access-Control-Allow-Private-Network", "true")
            self.send_header("Access-Control-Expose-Headers", "Content-Disposition")
            self.send_header("Access-Control-Max-Age", "600")

    def _send(self, status, body, ctype, extra=None):
        self.send_response(status)
        self._cors()
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        for k, v in (extra or {}).items():
            self.send_header(k, v)
        self.end_headers()
        if self.command != "HEAD":
            self.wfile.write(body)

    def _json(self, status, obj):
        self._send(status, json.dumps(obj).encode(), "application/json")

    def _check_host(self):
        # DNS-rebinding guard: a loopback-bound daemon only answers to loopback host names.
        if self.allowed_hosts is None:
            return
        host = (self.headers.get("Host") or "").strip().lower()
        if host.startswith("["):
            host = host[: host.find("]") + 1]
        else:
            host = host.rsplit(":", 1)[0]
        if host not in self.allowed_hosts:
            raise ApiError(403, "Host not allowed")

    def _check_auth(self, query):
        if self.token is None:
            return
        auth = self.headers.get("Authorization", "")
        given = auth[7:].strip() if auth.lower().startswith("bearer ") else None
        if given is None:
            given = (query.get("token") or [None])[0]
        if not given or not secrets.compare_digest(given.encode(), self.token.encode()):
            raise ApiError(401, "Missing or invalid token")

    def _read_json(self, allow_empty=False):
        length = int(self.headers.get("Content-Length") or 0)
        if length > MAX_BODY:
            raise ApiError(400, "Request body too large")
        body = self.rfile.read(length) if length else b""
        if not body and allow_empty:
            return {}
        ctype = (self.headers.get("Content-Type") or "").split(";")[0].strip().lower()
        if ctype != "application/json":
            raise ApiError(400, "Content-Type must be application/json")
        try:
            obj = json.loads(body.decode("utf-8"))
        except (UnicodeDecodeError, ValueError):
            raise ApiError(400, "Invalid JSON body")
        if not isinstance(obj, dict):
            raise ApiError(400, "JSON body must be an object")
        return obj

    def _dispatch(self, method):
        url = urllib.parse.urlsplit(self.path)
        query = urllib.parse.parse_qs(url.query)
        parts = [urllib.parse.unquote(p) for p in url.path.split("/") if p]
        try:
            self._check_host()
            if not self._origin_allowed():
                raise ApiError(403, "Origin not allowed (start.py --allow-origin)")
            if not parts or parts[0] != "api":
                raise ApiError(404, "Not found")
            if method == "GET" and parts == ["api", "ping"]:
                return self._json(200, {"service": SERVICE, "version": API_VERSION,
                                        "tokenRequired": self.token is not None})
            self._check_auth(query)
            self._route(method, parts[1:], query)
        except ApiError as e:
            self._json(e.status, {"error": e.message})
        except (BrokenPipeError, ConnectionResetError):
            pass
        except Exception as e:  # noqa: BLE001
            try:
                self._json(500, {"error": "Internal error: %s" % e})
            except OSError:
                pass

    def do_OPTIONS(self):
        # a refused preflight carries no CORS headers, so the browser blocks the request
        self.send_response(204 if self._origin_allowed() else 403)
        self._cors()
        self.send_header("Content-Length", "0")
        self.end_headers()

    def do_GET(self):
        self._dispatch("GET")

    def do_POST(self):
        self._dispatch("POST")

    # ---------------------------------------------------------------- routes

    def _route(self, method, p, query):
        b = self.builder
        if method == "GET" and p == ["info"]:
            active = b.active
            return self._json(200, {
                "service": SERVICE, "version": API_VERSION, "repo": REPO,
                "git": git_info(), "docker": docker_status(b.image), "simDocker": docker_status(b.sil_image),
                "activeJob": active.id if active else None,
                "activeJobKind": active.kind if active else None,
            })
        if method == "GET" and p == ["configs"]:
            return self._json(200, {kind: list_configs(kind) for kind in CONFIG_DIRS})
        if method == "GET" and len(p) == 3 and p[0] == "configs" and p[1] in CONFIG_DIRS:
            path = config_path(p[1], p[2])
            if path is None:
                raise ApiError(404, "Unknown %s config: %s" % (p[1][:-1], p[2]))
            with open(path, "rb") as f:
                return self._send(200, f.read(), "text/plain; charset=utf-8")
        if p == ["builds"]:
            if method == "GET":
                return self._json(200, b.summaries())
            return self._create_build(self._read_json())
        if p == ["sims"] and method == "POST":
            return self._create_sim(self._read_json())
        # build and sim jobs share one id space and the same per-job routes
        if len(p) >= 2 and p[0] in ("builds", "sims"):
            job = b.get(p[1])
            if job is None:
                raise ApiError(404, "Unknown build: %s" % p[1])
            if method == "GET" and len(p) == 2:
                return self._json(200, job.summary())
            if method == "GET" and p[2:] == ["events"]:
                return self._events(job)
            if method == "GET" and len(p) == 4 and p[2] == "artifacts":
                art = next((a for a in job.artifacts if a["name"] == p[3]), None)
                if art is None or not os.path.isfile(art["path"]):
                    raise ApiError(404, "Unknown artifact: %s" % p[3])
                with open(art["path"], "rb") as f:
                    data = f.read()
                return self._send(200, data, "application/octet-stream",
                                  {"Content-Disposition": 'attachment; filename="%s"' % art["name"]})
            if method == "POST" and p[2:] == ["cancel"]:
                self._read_json(allow_empty=True)
                b.cancel(job)
                deadline = time.time() + 10
                while not job.finished and time.time() < deadline:
                    time.sleep(0.1)
                return self._json(200, job.summary())
        raise ApiError(404, "Not found")

    def _parse_configs(self, body, keys):
        req = {}
        for key in keys:
            kind = key + "s"
            text_key = key + "Text"
            name = body.get(key)
            text = body.get(text_key)
            if text is not None:
                if not isinstance(text, str):
                    raise ApiError(400, "%s must be a string" % text_key)
                if name is not None and not isinstance(name, str):
                    raise ApiError(400, "%s must be a string" % key)
                check_custom_defines(text, text_key)
                req[text_key] = text
                req[key] = name if name else CUSTOM_NAME
            elif name is None and key == "profile":
                req[key] = None
            else:
                if config_path(kind, name) is None:
                    raise ApiError(400, "Unknown %s: %r" % (key, name))
                req[key] = name
        return req

    @staticmethod
    def _parse_debug(body):
        debug = body.get("debug")
        if debug not in (None, "INFO", "GDB"):
            raise ApiError(400, "debug must be \"INFO\", \"GDB\" or null")
        return debug

    def _create_build(self, body):
        req = self._parse_configs(body, ("board", "profile"))
        req["debug"] = self._parse_debug(body)
        extra = body.get("extraFlags")
        if extra is not None and not isinstance(extra, str):
            raise ApiError(400, "extraFlags must be a string")
        for flag in (extra or "").split():
            if not (flag[:2] in ("-D", "-U") and DEFINE_TOKEN_RE.match(flag[2:])
                    and not (flag.startswith("-U") and "=" in flag)):
                raise ApiError(400, "extraFlags only accepts -DNAME, -DNAME=value and -UNAME "
                                    "(value: letters, digits and _ . + : / -), got %r" % flag)
        req["extraFlags"] = " ".join((extra or "").split())
        clean = body.get("clean", False)
        if not isinstance(clean, bool):
            raise ApiError(400, "clean must be a boolean")
        req["clean"] = clean
        return self._start(req, Job)

    def _create_sim(self, body):
        sim_text = body.get("simText")
        if sim_text is not None:
            # Arbitrary Python by design, with the repo mounted read-write: the token is the only gate
            if self.token is None:
                raise ApiError(403, "Edited simulation scripts need token authentication; restart the "
                                    "daemon without --no-token, or save the script in support/simulation")
            if not isinstance(sim_text, str):
                raise ApiError(400, "simText must be a string")
            name = body.get("sim")
            if name is not None and not isinstance(name, str):
                raise ApiError(400, "sim must be a string")
            req = {"simText": sim_text, "sim": name or CUSTOM_NAME}
        else:
            if config_path("sims", body.get("sim")) is None:
                raise ApiError(400, "Unknown sim: %r" % body.get("sim"))
            req = {"sim": body["sim"]}
        if body.get("profile") is None and body.get("profileText") is None:
            raise ApiError(400, "A simulation needs a profile")  # entrypoint.sh requires PROFILE
        req.update(self._parse_configs(body, ("profile",)))
        req["debug"] = self._parse_debug(body)
        args = body.get("args") or []
        if not isinstance(args, list) or len(args) > MAX_SIM_ARGS:
            raise ApiError(400, "args must be a list of at most %d strings" % MAX_SIM_ARGS)
        for a in args:
            if not isinstance(a, str) or not SIM_ARG_RE.match(a):
                raise ApiError(400, "Unsupported sim argument %r (letters, digits and _ . : / = + , -)" % (a,))
            if a.split("=", 1)[0] in RESERVED_SIM_ARGS:
                raise ApiError(400, "%s is set by the daemon" % a)
        req["args"] = args
        for key in ("gdbserver", "hostNetwork"):
            if not isinstance(body.get(key, False), bool):
                raise ApiError(400, "%s must be a boolean" % key)
            req[key] = body.get(key, False)
        req["visPort"] = VIS_CONTAINER_PORT if req["hostNetwork"] else free_port(VIS_CONTAINER_PORT)
        return self._start(req, SimJob)

    def _start(self, req, job_cls):
        job = self.builder.start(req, job_cls)
        if job is None:
            active = self.builder.active
            raise ApiError(409, "A %s is already running: %s" % (
                "simulation" if active.kind == "sim" else "build", active.id))
        return self._json(201, {"id": job.id})

    def _events(self, job):
        self.send_response(200)
        self._cors()
        self.send_header("Content-Type", "text/event-stream")
        self.send_header("Cache-Control", "no-store")
        self.send_header("Connection", "close")
        self.send_header("X-Accel-Buffering", "no")
        self.end_headers()
        self.close_connection = True

        def send(event, data):
            self.wfile.write(("event: %s\ndata: %s\n\n" % (event, json.dumps(data))).encode())

        idx = 0
        seen_progress = -1
        last_progress_t = 0.0
        last_write = time.time()
        try:
            while True:
                with job.cond:
                    if (idx >= len(job.events) and job.progress_seq == seen_progress and not job.finished):
                        job.cond.wait(timeout=1.0)
                    batch = job.events[idx:]
                    idx += len(batch)
                    pseq = job.progress_seq
                    prog = {"done": job.done, "total": job.total, "file": job.last_file}
                    finished = job.finished and idx >= len(job.events)
                    summary = job.summary()
                for ev, data in batch:
                    send(ev, data)
                now = time.time()
                if batch:
                    last_write = now
                if pseq != seen_progress and (now - last_progress_t >= PROGRESS_INTERVAL_S or finished):
                    send("progress", prog)
                    seen_progress, last_progress_t, last_write = pseq, now, now
                if finished and pseq == seen_progress:
                    send("end", summary)
                    self.wfile.flush()
                    return
                if now - last_write >= KEEPALIVE_S:
                    self.wfile.write(b": keepalive\n\n")
                    last_write = now
                self.wfile.flush()
                if pseq != seen_progress:
                    # throttled progress pending: don't spin on the condition
                    time.sleep(max(0.0, PROGRESS_INTERVAL_S - (time.time() - last_progress_t)))
        except (BrokenPipeError, ConnectionResetError, OSError):
            return


# --------------------------------------------------------------------------- main

def free_port(preferred):
    """preferred if it is free on loopback, else any free port (macOS AirPlay holds 5000)."""
    import socket
    for port in (preferred, 0):
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
            try:
                sock.bind(("127.0.0.1", port))
                return sock.getsockname()[1]
            except OSError:
                continue
    raise ApiError(500, "No free port for the visualization")


def load_token():
    try:
        with open(TOKEN_FILE, encoding="utf-8") as f:
            tok = f.read().strip()
        if tok:
            return tok
    except OSError:
        pass
    os.makedirs(os.path.dirname(TOKEN_FILE), mode=0o700, exist_ok=True)
    tok = secrets.token_urlsafe(24)
    fd = os.open(TOKEN_FILE, os.O_WRONLY | os.O_CREAT | os.O_TRUNC, 0o600)
    with os.fdopen(fd, "w", encoding="utf-8") as f:
        f.write(tok + "\n")
    return tok


def main():
    global DOCKER_PLATFORM
    ap = argparse.ArgumentParser(description="Local Indiflight build daemon for the web configurator.")
    ap.add_argument("--host", default="127.0.0.1", help="address to bind (default 127.0.0.1)")
    ap.add_argument("--port", type=int, default=8742, help="port (default 8742)")
    ap.add_argument("--no-token", action="store_true",
                    help="disable token authentication; the origin allowlist still applies, but any "
                         "local process (and any page on an allowed origin) can then start builds and simulations; "
                         "edited simulation scripts are refused")
    ap.add_argument("--allow-origin", action="append", default=[], metavar="ORIGIN",
                    help="also accept requests from this browser origin, e.g. https://my.host:8443 "
                         "(repeatable; always allowed: %s, localhost/127.0.0.1 on any port, Tauri)"
                         % ", ".join(DEFAULT_ORIGINS))
    ap.add_argument("--image", default="indiflight-builder", help="docker image tag (default indiflight-builder)")
    ap.add_argument("--sil-image", default="pyndiflight-sil",
                    help="docker image tag of the SIL simulation runner (default pyndiflight-sil)")
    ap.add_argument("--platform", default=DOCKER_PLATFORM,
                    help="docker platform of the builder (default: native, here %s). "
                         "linux/amd64 on an ARM host runs the x86 toolchain under emulation" % DOCKER_PLATFORM)
    args = ap.parse_args()
    DOCKER_PLATFORM = args.platform

    builder_image = ImageSpec(args.image, "Dockerfile")
    sil_image = ImageSpec(args.sil_image, "support/simulation/sil.Dockerfile", "support/simulation", base=builder_image)
    builder = Builder(builder_image, sil_image)
    Handler.builder = builder
    Handler.token = None if args.no_token else load_token()
    Handler.allowed_origins = set(DEFAULT_ORIGINS) | {o.rstrip("/") for o in args.allow_origin}
    if args.host in ("127.0.0.1", "localhost", "::1"):
        Handler.allowed_hosts = {"127.0.0.1", "localhost", "[::1]"}

    try:
        server = ThreadingHTTPServer((args.host, args.port), Handler)
    except OSError as e:
        print("Cannot listen on %s:%d: %s" % (args.host, args.port, e), file=sys.stderr)
        return 1
    server.daemon_threads = True

    print("Indiflight build daemon")
    print("  repo:  %s" % REPO)
    g = git_info()
    print("  git:   %s (%s)" % (g["describe"], g["branch"] or "detached HEAD"))
    if g["submodulesMissing"]:
        print("  WARNING: uninitialised submodules: %s" % ", ".join(g["submodulesMissing"]))
        print("           run: git submodule update --init --recursive")
    print("  platform: %s" % DOCKER_PLATFORM)
    st = docker_status(builder_image)
    if not st["available"]:
        print("  docker: NOT AVAILABLE (%s)" % st["error"])
    elif not st["exists"]:
        print("  docker: image %s missing, will be built with the first build" % args.image)
    elif not st["upToDate"]:
        print("  docker: image %s outdated, will be rebuilt with the first build" % args.image)
    else:
        print("  docker: image %s up to date" % args.image)
    print("  url:   http://%s:%d" % (args.host if ":" not in args.host else "[%s]" % args.host, args.port))
    if Handler.token:
        print("  token: %s" % Handler.token)
        print("         (paste this into the configurator; stored in %s, delete it to rotate)" % TOKEN_FILE)
    else:
        print("  token: DISABLED (--no-token): any local program and any page on an allowed")
        print("         origin can start builds, which run code with this repo mounted")
    print("  origins: %s, localhost, Tauri" % ", ".join(sorted(Handler.allowed_origins)))
    print("Press Ctrl-C to stop.", flush=True)

    def on_term(signum, frame):
        raise KeyboardInterrupt

    signal.signal(signal.SIGTERM, on_term)
    signal.signal(signal.SIGINT, on_term)  # also when started in the background with SIGINT ignored
    try:
        server.serve_forever(poll_interval=0.5)
    except KeyboardInterrupt:
        print("\nShutting down ...", flush=True)
    finally:
        builder.shutdown()
        server.server_close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
