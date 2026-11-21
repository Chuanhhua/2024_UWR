#!/usr/bin/env bash
# Start (or reattach to) the zellij sim session, tmux send-keys style:
#   - four panes from zellij/sim.kdl, each its own `pixi shell`:
#     sim | kf over debug 1 | debug 2
#   - types the sim and kf commands into their panes WITHOUT pressing Enter
#   - attaches you to the session
# Run it from a normal terminal, not through pixi: zellij sits outside the
# environment and every pane activates pixi itself.
#   scripts/deploy_sim.sh [session-name]   (default: uwr-sim)
set -euo pipefail

SESSION="${1:-uwr-sim}"
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
LAYOUT="$ROOT/zellij/sim.kdl"
CONFIG="$ROOT/zellij/config.kdl"
# The Ctrl+q binding in config.kdl runs $UWR_ROOT/scripts/stop_sim.sh
export UWR_ROOT="$ROOT"

if [[ -n "${PIXI_PROJECT_ROOT:-}" ]]; then
    echo "Run this outside pixi (not via pixi run / pixi shell); each pane starts its own pixi shell." >&2
    exit 1
fi

# zellij is installed globally, not in the project env: pixi global install zellij
if ! command -v zellij >/dev/null; then
    echo "zellij not found. Install it with: pixi global install zellij" >&2
    exit 1
fi

# pane name -> command to type into it
declare -A PREFILL=(
    # Gazebo tank sim: robot, bridge, sonar adapter, foxglove bridge
    [sim]="ros2 launch uwr_gazebo sim.launch.py"
    # kf_3dof on sim time plus kf_to_pool; bag goes to kf_runs/sim_*
    [kf]='ros2 launch uwr_gazebo kf_sim.launch.py csv_path:=$PIXI_PROJECT_ROOT/kf_runs/sim_'
)

if [[ -n "${ZELLIJ:-}" ]]; then
    echo "Already inside zellij. Detach first (Ctrl+o d), then rerun." >&2
    exit 1
fi

# Reattach if the session is still running; never retype into it.
# `list-sessions -s` also lists EXITED sessions, and attaching to one of those
# resurrects it with its old commands suspended, so check the full listing.
if zellij list-sessions -n 2>/dev/null | grep -v EXITED | grep -q "^$SESSION "; then
    exec zellij --config "$CONFIG" attach "$SESSION"
fi
# Discard an exited copy so the session is built fresh from the layout.
zellij delete-session "$SESSION" >/dev/null 2>&1 || true

cd "$ROOT"
zellij --config "$CONFIG" --layout "$LAYOUT" \
    attach --create-background "$SESSION"

# Pane IDs don't follow layout order, so look each one up by name.
pane_id() {
    zellij -s "$SESSION" action list-panes 2>/dev/null \
        | awk -v name="$1" '$2 == "terminal" { id = $1; $1 = $2 = ""; sub(/^ +/, ""); if ($0 == name) print id }'
}

# Wait until the shell in a pane has drawn its prompt, so typed text isn't lost.
wait_for_prompt() {
    local id=$1
    # pixi shell has to activate first, so allow more time than a bare shell
    for _ in $(seq 300); do
        zellij -s "$SESSION" action dump-screen -p "$id" 2>/dev/null | grep -q '[$#] *$' && return 0
        sleep 0.1
    done
    echo "warning: no prompt in pane $id after 30s, typing anyway" >&2
}

# Wait until our terminal has attached, so the panes have their final size.
# Typing earlier means bash redraws a wrapped command badly on that resize.
wait_for_client() {
    for _ in $(seq 100); do
        [[ "$(zellij -s "$SESSION" action list-clients 2>/dev/null | wc -l)" -gt 1 ]] && break
        sleep 0.1
    done
    sleep 0.3   # let the resize reach the shells
}

prefill() {
    wait_for_client
    for name in "${!PREFILL[@]}"; do
        id=""
        for _ in $(seq 50); do
            id="$(pane_id "$name")"
            [[ -n "$id" ]] && break
            sleep 0.1
        done
        if [[ -z "$id" ]]; then
            echo "warning: no pane named '$name' in $LAYOUT, skipping" >&2
            continue
        fi
        wait_for_prompt "$id"
        zellij -s "$SESSION" action write-chars -p "$id" "${PREFILL[$name]}"
    done
}

# Type in the background (quietly, so it can't scribble over the zellij UI)
# while attach takes over this terminal.
prefill >/dev/null 2>&1 &
exec zellij --config "$CONFIG" attach "$SESSION"
