#!/usr/bin/env bash
# Compatibility alias for old launch instructions.
exec "$(dirname "${BASH_SOURCE[0]}")/start_bridge.sh" "$@"
