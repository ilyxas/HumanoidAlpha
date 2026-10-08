#!/bin/sh
set -eu
cd "$(CDPATH= cd -- "$(dirname -- "$0")" && pwd -P)"
export PYTHONDONTWRITEBYTECODE=1
case "${1:-}" in
  -h|--help)
    printf '%s\n' './START.command              — choose a mode' './START.command debug        — developer console' './START.command experiment   — observation + actuator CLI (RAW)' './START.command harness      — observation + actuator CLI + physical harness' './START.command animation    — Xandra GLB animations, no physics (WASD / left stick = move, Space/1/2/3 = actions)' './START.command stop         — stop this project'
    exit 0 ;;
esac
if [ "$#" -eq 0 ]; then
  printf '\n%s\n' 'Humanoid Alpha' '  1 — Debug' '  2 — Experiment (RAW)' '  3 — Harness' '  4 — Animation' '  0 — Stop' ''
  printf 'Choose [1/2/3/4/0]: '
  read -r choice
  case "$choice" in
    1) set -- debug ;;
    2) set -- experiment ;;
    3) set -- harness ;;
    4) set -- animation ;;
    0) set -- stop ;;
    *) printf '%s\n' 'No mode selected.'; exit 2 ;;
  esac
fi
case "$1" in
  stop) exec ./stop-project.command ;;
  debug|experiment|harness|assisted|animation) ;;
  *) printf '%s\n' 'Use: ./START.command [debug|experiment|harness|animation|stop]'; exit 2 ;;
esac
if ! .venv/bin/python -c 'import sys; assert (3,11) <= sys.version_info[:2] <= (3,13)' >/dev/null 2>&1; then
  interpreter=''
  for candidate in "${PYTHON:-}" python3.13 python3.12 python3.11 python3; do
    [ -n "$candidate" ] || continue
    if "$candidate" -c 'import sys; assert (3,11) <= sys.version_info[:2] <= (3,13)' >/dev/null 2>&1; then
      interpreter="$candidate"; break
    fi
  done
  if [ -z "$interpreter" ]; then
    printf '%s\n' 'Install Python 3.13, then run START.command again.' >&2; exit 1
  fi
  "$interpreter" -m venv .venv
fi
if ! .venv/bin/python -c 'import mujoco,numpy,websockets' >/dev/null 2>&1; then
  .venv/bin/python -m pip install -r requirements.txt
fi
exec .venv/bin/python scripts/launch.py "$@"
