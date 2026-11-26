#!/bin/zsh
cd -- "$(dirname -- "$0")"
if [[ -z "$TERM" || "$TERM" == dumb ]]; then
  export TERM=xterm-256color
fi
exec ./caster-physics-python/.venv/bin/python ./arche-tui/dashboard.py
