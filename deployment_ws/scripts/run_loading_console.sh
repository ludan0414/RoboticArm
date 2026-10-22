#!/usr/bin/env bash
# Real cell: persistent operator console; no motion until explicit initialization.
set -euo pipefail
DEPLOY="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
set +u
source "${DEPLOY}/env_jazzy_real.sh"
set -u
exec ros2 run luggage_planning hardware_loading_console.py "$@"
