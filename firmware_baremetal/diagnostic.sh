#!/usr/bin/env bash
# Run peripheral I/O diagnostics on a board over RS485.
# Usage: bazelisk run //firmware_baremetal:diagnostic -- [--serial /dev/ttyUSB0] [--id 1] [--sweep_amplitude 0.15] [--sweep_duration 2.0] [--debug]

SERIAL="/dev/ttyUSB0"
BOARD_ID=1
EXTRA=()

while [[ $# -gt 0 ]]; do
    case "$1" in
        --serial) SERIAL="$2"; shift 2 ;;
        --id)     BOARD_ID="$2"; shift 2 ;;
        *)        EXTRA+=("$1"); shift ;;
    esac
done

exec python3 -c "
from bd_tools.bin import diagnostic
diagnostic.action(diagnostic.parser_args())
" "$SERIAL" "$BOARD_ID" "${EXTRA[@]}"
