#!/usr/bin/env bash
# FROZEN benchmark (write-protected by the autoresearch hook): estimated-state
# flight, RTK-outage (primary metric) + wind scenario, deterministic seeded noise.
# Runs under WSL bash (plugin) and git-bash alike.
set -e
cd "$(dirname "$0")/.."
# Windows-style project root for the matlab.exe argument (git-bash pwd -W, WSL /mnt/c)
ROOT_W=$(pwd -W 2>/dev/null || pwd | sed 's|^/mnt/c/|C:/|; s|^/c/|C:/|')
if command -v matlab.exe >/dev/null 2>&1; then
    ML=matlab.exe
elif [ -x "/mnt/c/Program Files/MATLAB/R2025b/bin/matlab.exe" ]; then
    ML="/mnt/c/Program Files/MATLAB/R2025b/bin/matlab.exe"
else
    ML="/c/Program Files/MATLAB/R2025b/bin/matlab.exe"
fi
"$ML" -batch "run('$ROOT_W/.auto/bench_ertk.m')"
