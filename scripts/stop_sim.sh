#!/usr/bin/env bash
# Shut down the zellij sim session cleanly:
#   1. Ctrl+C every pane, so `ros2 launch` stops its nodes AND Gazebo
#      (killing the session outright only hangs up the shells, which leaves
#      `gz sim server` / `gz sim gui` running as orphans)
#   2. wait for every pane to get back to its prompt
#   3. kill and delete the session
# Ctrl+q inside the session runs this (zellij/config.kdl). From outside:
#   scripts/stop_sim.sh [session-name]   (default: the one running uwr-* session)
set -uo pipefail

SESSION="${1:-${ZELLIJ_SESSION_NAME:-}}"
if [[ -z "$SESSION" ]]; then
    mapfile -t running < <(zellij list-sessions -n 2>/dev/null | grep -v EXITED | awk '$1 ~ /^uwr-(sim|real)/ {print $1}')
    if (( ${#running[@]} != 1 )); then
        echo "Found ${#running[@]} running uwr sessions (${running[*]:-none}); name one: $0 <session>" >&2
        exit 1
    fi
    SESSION="${running[0]}"
fi
TIMEOUT=20   # seconds to wait for launches to shut down before killing anyway

if ! zellij list-sessions -n 2>/dev/null | grep -v EXITED | grep -q "^$SESSION "; then
    echo "No running session '$SESSION'." >&2
    zellij delete-session "$SESSION" >/dev/null 2>&1
    exit 0
fi

z() { zellij -s "$SESSION" action "$@"; }

# Terminal panes, minus the one this script is running in (when run via Ctrl+q).
panes=()
while read -r id type _; do
    [[ "$type" == terminal ]] || continue
    [[ -n "${ZELLIJ_PANE_ID:-}" && "$id" == "terminal_$ZELLIJ_PANE_ID" ]] && continue
    panes+=("$id")
done < <(z list-panes 2>/dev/null | tail -n +2)

at_prompt() { z dump-screen -p "$1" 2>/dev/null | grep -v '^\s*$' | tail -1 | grep -q '[$#] *$'; }

echo "Stopping ${#panes[@]} panes in '$SESSION'..."
for id in "${panes[@]}"; do
    z write -p "$id" 3   # Ctrl+C
done

for ((i = 0; i < TIMEOUT * 5; i++)); do
    busy=()
    for id in "${panes[@]}"; do
        at_prompt "$id" || busy+=("$id")
    done
    ((${#busy[@]} == 0)) && break
    sleep 0.2
done
if ((${#busy[@]})); then
    echo "Still busy after ${TIMEOUT}s: ${busy[*]}; killing the session anyway." >&2
    sleep 1
fi

zellij kill-session "$SESSION" >/dev/null 2>&1
zellij delete-session -f "$SESSION" >/dev/null 2>&1
