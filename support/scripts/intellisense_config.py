#!/usr/bin/env python3
"""Generate .vscode/compile_commands.json for C/C++ IntelliSense for a board + profile.

Uses the same flags as `make BOARD=<board> PROFILE=<profile>` (and therefore the
docker build): the `#define` lines from configs/boards/<board>.txt and
configs/profiles/<profile>.txt.

Usage:
    intellisense_config.py              # interactive: pick board, then profile
    intellisense_config.py BOARD PROFILE
    intellisense_config.py --last       # reuse previous selection
    intellisense_config.py --sync       # only update the task dropdown lists

The selection is remembered in .vscode/build_selection.json; the docker build
(entrypoint.sh) uses it when BOARD/PROFILE are not given. In VS Code,
run the task "IntelliSense: select board and profile" (Tasks: Run Task) to pick
both from dropdowns. Every run refreshes the dropdown lists in .vscode/tasks.json
from configs/boards and configs/profiles.
"""
import json, os, re, shlex, subprocess, sys
from pathlib import Path

root = Path(__file__).resolve().parents[2]
sel_file = root / ".vscode" / "build_selection.json"
cc_file = root / ".vscode" / "compile_commands.json"
tasks_file = root / ".vscode" / "tasks.json"

def names(d):
    return sorted(p.stem for p in (root / "configs" / d).glob("*.txt"))

def sync_tasks():
    """Write the current board/profile lists into the pickString inputs."""
    try:
        text = tasks_file.read_text()
    except FileNotFoundError:
        return
    new = text
    for input_id, d in (("intellisenseBoard", "boards"), ("intellisenseProfile", "profiles")):
        opts = "[\n" + ",\n".join(f'                "{n}"' for n in names(d)) + "\n            ]"
        new = re.sub(r'("id"\s*:\s*"' + input_id + r'"[^}]*?"options"\s*:\s*)\[[^\]]*\]',
                     lambda m: m.group(1) + opts, new, count=1)
    if new != text:
        tasks_file.write_text(new)
        print("updated board/profile lists in .vscode/tasks.json")

def target_of(board):
    for line in (root / "configs/boards" / f"{board}.txt").read_text().splitlines():
        m = re.match(r"#define TARGET=(\S+)", line)
        if m:
            return m.group(1)
    sys.exit(f"no '#define TARGET=' in board {board}")

def last():
    try:
        d = json.loads(sel_file.read_text())
        return d["board"], d["profile"]
    except Exception:
        return None, None

def choose(what, options, default):
    print(f"\nSelect {what}:")
    for i, o in enumerate(options, 1):
        print(f"  {i:2d}) {o}" + ("  [last]" if o == default else ""))
    while True:
        s = input(f"{what} (number or name" + (", enter = last" if default else "") + "): ").strip()
        if not s and default:
            return default
        if s in options:
            return s
        if s.isdigit() and 1 <= int(s) <= len(options):
            return options[int(s) - 1]
        print("invalid")

args = sys.argv[1:]
sync_tasks()
if args == ["--sync"]:
    sys.exit(0)
lb, lp = last()
if args == ["--last"]:
    if not lb:
        sys.exit("no previous selection")
    board, profile = lb, lp
elif len(args) == 2:
    board, profile = args
else:
    board = choose("board", names("boards"), lb)
    profile = choose("profile", names("profiles"), lp)

if board not in names("boards") or profile not in names("profiles"):
    sys.exit("unknown board or profile")

print(f"\nBoard {board}, profile {profile}: running make dry-run ...")
out = subprocess.run(
    ["make", "-n", "-B", "hex", f"TARGET={target_of(board)}", f"BOARD={board}",
     f"PROFILE={profile}", "YES=y"],
    cwd=root, capture_output=True, text=True).stdout

entries = []
for line in out.splitlines():
    if "-c -o" not in line or "gcc" not in line:
        continue
    cmd = line.split("&&", 1)[1] if line.lstrip().startswith("echo") else line
    try:
        argv = shlex.split(cmd)
    except ValueError:
        continue
    srcs = [a for a in argv if re.search(r"\.(c|S|s)$", a) and not a.startswith("-")]
    if not srcs:
        continue
    argv[0] = str((root / argv[0]).resolve()) if "/" in argv[0] else argv[0]
    entries.append({"directory": str(root), "file": srcs[-1], "arguments": argv})

if not entries:
    sys.exit("no compile commands found in make output")

cc_file.write_text(json.dumps(entries, indent=1))
sel_file.write_text(json.dumps({"board": board, "profile": profile}))
print(f"wrote .vscode/compile_commands.json ({len(entries)} files). "
      "Reload IntelliSense if needed (C/C++: Reset IntelliSense Database).")
