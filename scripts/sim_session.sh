# Shared body of the zellij sim sessions. Not run directly: sourced by
#   scripts/deploy_sim_3sonar.sh   3-sonar robot + kf_3dof
#   scripts/deploy_sim_7sonar.sh   7-sonar robot + planar EKF + zig-zag driver
# after they set SESSION and PREFILL (pane name -> command to type into it).
#
# Starts (or reattaches to) the session, tmux send-keys style:
#   - four panes from zellij/sim.kdl, each its own `pixi shell`:
#     sim | kf over debug 1 | debug 2
#   - types the PREFILL commands into their panes WITHOUT pressing Enter
#   - attaches you to the session
# Run the deploy scripts from a normal terminal, not through pixi: zellij sits
# outside the environment and every pane activates pixi itself.
set -euo pipefail

if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
    echo "Don't run this directly; use scripts/deploy_sim_3sonar.sh or scripts/deploy_sim_7sonar.sh" >&2
    exit 1
fi
: "${SESSION:?set SESSION before sourcing}"
declare -p PREFILL >/dev/null 2>&1 || { echo "set PREFILL before sourcing" >&2; exit 1; }

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
# while zellij takes over this terminal. The session is created here, in the
# foreground: zellij >= 0.46 can't fit a layout into a session created with
# `attach --create-background` (it has no terminal size yet), and --layout now
# means "add a tab to an existing session", hence --new-session-with-layout.
prefill >/dev/null 2>&1 &
exec zellij --config "$CONFIG" --session "$SESSION" --new-session-with-layout "$LAYOUT"
