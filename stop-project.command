#!/bin/sh
set -eu
ROOT="$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)"
exec python3 - "$ROOT" <<'PY'
import os
from pathlib import Path
import re
import signal
import socket
import stat
import subprocess
import sys

root = Path(sys.argv[1]).resolve()
if not (root / 'physics/control_console.py').is_file():
    sys.exit('ERROR: put this script in the project folder')

sys.path.insert(0, str(root / 'scripts'))
from launch import stop_managed
managed = stop_managed(root)

def cwd(pid):
    try:
        if sys.platform.startswith('linux'):
            return Path(os.readlink(f'/proc/{pid}/cwd')).resolve()
        result = subprocess.run(['lsof', '-a', '-p', str(pid), '-d', 'cwd', '-Fn'],
                                capture_output=True, text=True)
        for line in result.stdout.splitlines():
            if line.startswith('n'):
                return Path(line[1:]).resolve()
    except (OSError, ValueError):
        pass
    return None

def processes():
    result = subprocess.run(['ps', '-axo', 'pid=,uid=,command='], capture_output=True, text=True, check=True)
    rows = []
    for line in result.stdout.splitlines():
        fields = line.strip().split(None, 2)
        if len(fields) == 3:
            rows.append((int(fields[0]), int(fields[1]), fields[2]))
    return rows

def is_project_command(command):
    return bool(re.search(r'(?:^|[/\s])(?:START\.command|start-actuator-runtime|start-console\.command)(?:\s|$)', command)
                or re.search(r'(?:^|[/\s])scripts/launch\.py(?:\s|$)', command)
                or re.search(r'(?:^|[/\s])physics/(?:control_console|ws_bridge)\.py(?:\s|$)', command)
                or re.search(r'(?:^|\s)-m\s+http\.server\s+8788(?:\s|$)', command))

try:
    rows = [] if managed else processes()
except (OSError, subprocess.CalledProcessError):
    sys.exit('ERROR: cannot inspect processes')
targets = [(pid, command) for pid, uid, command in rows
           if uid == os.getuid() and pid != os.getpid() and is_project_command(command)
           and cwd(pid) == root]
# Stop launchers before workers so launchers cannot keep their services alive.
targets.sort(key=lambda row: 0 if 'start-' in row[1] else 1)
for pid, command in targets:
    try:
        # Recheck identity and directory immediately before killing.
        current = {(p, c) for p, uid, c in processes() if uid == os.getuid()}
        if (pid, command) in current and cwd(pid) == root:
            os.kill(pid, signal.SIGKILL)
    except ProcessLookupError:
        pass
    except PermissionError:
        sys.exit('ERROR: cannot stop a project process')

# A forced stop bypasses Python cleanup. Remove only a refused, user-owned socket.
path = Path(os.environ.get('HUMANOID_ACTUATOR_SOCKET', f'/tmp/humanoid-actuator-{os.getuid()}.sock'))
try:
    info = path.lstat()
    if stat.S_ISSOCK(info.st_mode) and info.st_uid == os.getuid():
        with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as client:
            client.settimeout(.5)
            try:
                client.connect(str(path))
            except ConnectionRefusedError:
                if path.lstat().st_ino == info.st_ino:
                    path.unlink()
            except (FileNotFoundError, TimeoutError, PermissionError):
                pass
except FileNotFoundError:
    pass
print('Stopped' if managed or targets else 'No project processes found')
PY
