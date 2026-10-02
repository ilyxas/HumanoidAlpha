#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
# Prefer project .venv, then PY env, then python3
if [[ -z "${PY:-}" ]]; then
  if [[ -x "$ROOT/.venv/bin/python" ]]; then
    PY="$ROOT/.venv/bin/python"
  else
    PY="python3"
  fi
fi
SCENARIO="${1:-fall}"
cd "$ROOT"
$PY -m http.server 8787 --bind 127.0.0.1 &
HTTP_PID=$!
trap 'kill $HTTP_PID 2>/dev/null; kill $WS_PID 2>/dev/null' EXIT
sleep 0.5
$PY physics/ws_bridge.py --scenario "$SCENARIO" --broadcast-hz 30 &
WS_PID=$!
echo "HTTP http://127.0.0.1:8787/viewer/index.html"
echo "WS   ws://127.0.0.1:8765  scenario=$SCENARIO"
wait
